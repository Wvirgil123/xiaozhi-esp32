#include "watcher_settings_ui.h"
#include "watcher_font.h"
#include "backlight.h"
#include "sscma_camera.h"
#include "face_recognition.h"
#include "face_database.h"
#include "settings.h"
#include "lvgl_theme.h"
#include "material_symbols.h"
#include <wifi_manager.h>
#include <ssid_manager.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cmath>

static const char* TAG = "WatcherSettingsUi";

LV_FONT_DECLARE(font_noto_sans_basic_30_4);
LV_FONT_DECLARE(font_noto_sans_basic_20_4);
LV_FONT_DECLARE(font_noto_sans_basic_16_4);
LV_FONT_DECLARE(font_material_symbols_30_4);
LV_FONT_DECLARE(font_material_symbols_20_4);

// ---------------------------------------------------------------------------
// Geometry
//
// The panel is a 412px circle. A centred rectangle of half-width `hw` only
// stays inside the bezel while its top/bottom edges satisfy |dy| <=
// sqrt(R^2 - hw^2), R = 206. Cards are therefore 276 wide (hw = 138), which
// limits them to y in [53, 359]; the list is inset into that band with extra
// padding at both ends so the first and last cards never reach it exactly.
//
//   412 ............ screen
//    58 ............ header bottom (grab bar / page dots / title)
//    60 .. 374 ..... scrollable content list
//      68 .. 372 ... card band actually occupied by first/last card
// ---------------------------------------------------------------------------
static constexpr int kScreenSize   = 412;
static constexpr int kHeaderH      = 58;
static constexpr int kContentTop   = 60;
static constexpr int kContentH     = 314;
static constexpr int kListW        = 300;
static constexpr int kCardW        = 276;
static constexpr int kCardRadius   = 26;
static constexpr int kCapsule      = 46;   // icon capsule, square
static constexpr int kRowH         = 46;   // capsule-height row
static constexpr int kCardGap      = 10;   // gap between list cards
static constexpr int kSliderH      = 16;   // rail thickness

// Slider knob geometry (lv_slider.c: position_knob):
//   width  = MIN(w + 2*transform_w, h + 2*transform_h) / 2 + 2*transform_w
//   height = h + 2*transform_h
// A square knob therefore needs transform_w = D/4 and transform_h = (D-h)/2.
// Without them the theme's 6px knob pad yields a 20x28 upright oval instead
// of a circle.
static constexpr int kKnobGrowW    = 6;    // -> 24px knob
static constexpr int kKnobGrowH    = 4;
static constexpr int kKnobGrowWPrs = 7;    // -> 28px knob while dragging
static constexpr int kKnobGrowHPrs = 6;

// LVGL 9 declares LV_PART_* and LV_STATE_* as distinct enums, so OR-ing them
// directly trips -Wdeprecated-enum-enum-conversion on this toolchain.
static constexpr lv_style_selector_t Sel(lv_part_t part, lv_state_t state) {
    return static_cast<lv_style_selector_t>(part) | static_cast<lv_style_selector_t>(state);
}

static void anim_x_cb(void* var, int32_t v) {
    lv_obj_set_x(static_cast<lv_obj_t*>(var), v);
}

static void anim_y_cb(void* var, int32_t v) {
    lv_obj_set_y(static_cast<lv_obj_t*>(var), v);
}

// Root, dismiss handle, and dedicated chrome: fixed, non-scrollable
static void ConfigureFixedObject(lv_obj_t* obj) {
    if (obj == nullptr) return;
    lv_obj_remove_flag(obj, static_cast<lv_obj_flag_t>(
        LV_OBJ_FLAG_SCROLLABLE |
        LV_OBJ_FLAG_SCROLL_ELASTIC |
        LV_OBJ_FLAG_SCROLL_MOMENTUM |
        LV_OBJ_FLAG_SCROLL_CHAIN_HOR |
        LV_OBJ_FLAG_SCROLL_CHAIN_VER |
        LV_OBJ_FLAG_GESTURE_BUBBLE |
        LV_OBJ_FLAG_EVENT_BUBBLE));
    lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_opa(obj, LV_OPA_TRANSP, LV_PART_SCROLLBAR);
    lv_obj_set_style_width(obj, 0, LV_PART_SCROLLBAR);
    lv_obj_set_style_pad_all(obj, 0, LV_PART_SCROLLBAR);
}

// Cards, buttons and passive containers inside the list: never scroll on their
// own, but bubble events up so presses still reach the drawer handlers.
static void ConfigureListChild(lv_obj_t* obj) {
    if (obj == nullptr) return;
    lv_obj_remove_flag(obj, static_cast<lv_obj_flag_t>(
        LV_OBJ_FLAG_SCROLLABLE |
        LV_OBJ_FLAG_SCROLL_ELASTIC |
        LV_OBJ_FLAG_SCROLL_MOMENTUM |
        LV_OBJ_FLAG_SCROLL_CHAIN_HOR));
    lv_obj_add_flag(obj, static_cast<lv_obj_flag_t>(
        LV_OBJ_FLAG_SCROLL_CHAIN_VER |
        LV_OBJ_FLAG_GESTURE_BUBBLE |
        LV_OBJ_FLAG_EVENT_BUBBLE));
    lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_opa(obj, LV_OPA_TRANSP, LV_PART_SCROLLBAR);
    lv_obj_set_style_width(obj, 0, LV_PART_SCROLLBAR);
    lv_obj_set_style_pad_all(obj, 0, LV_PART_SCROLLBAR);
}

WatcherSettingsUi::WatcherSettingsUi(LcdDisplay* display, CloseCallback close_cb)
    : display_(display), close_cb_(std::move(close_cb)) {

    // Load initial settings
    Settings display_settings("display", false);
    pending_brightness_ = display_settings.GetInt("brightness", 80);

    Settings face_settings("face", false);
    pending_threshold_ = face_settings.GetInt("threshold", 60) / 100.0f;

    Settings model_settings("model", false);
    pending_interval_ = model_settings.GetInt("interval", 5);

    // Language is automatically determined by compile-time configuration
#if defined(CONFIG_LANGUAGE_ZH_CN) || defined(CONFIG_LANGUAGE_ZH_TW)
    language_ = WatcherUiLanguage::Zh;
#else
    language_ = WatcherUiLanguage::En;
#endif

    esp_timer_create_args_t timer_args = {
        .callback = NvsCommitTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "settings_nvs",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&timer_args, &nvs_timer_);
}

WatcherSettingsUi::~WatcherSettingsUi() {
    if (nvs_timer_ != nullptr) {
        esp_timer_stop(nvs_timer_);
        CommitPendingSettings();
        esp_timer_delete(nvs_timer_);
        nvs_timer_ = nullptr;
    }
    Close(false);
}

bool WatcherSettingsUi::IsOpen() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return drawer_state_ != DrawerState::Closed && root_ != nullptr;
}

WatcherSettingsPage WatcherSettingsUi::GetCurrentPage() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return current_page_;
}

void WatcherSettingsUi::RefreshLocalizedText() {
    if (root_ == nullptr) return;

    if (title_label_ != nullptr) {
        switch (current_page_) {
            case WatcherSettingsPage::QuickSettings:
                lv_label_set_text(title_label_, Tr("设定", "Settings"));
                break;
            case WatcherSettingsPage::VisionMode:
                lv_label_set_text(title_label_, Tr("模式设定", "Vision Modes"));
                break;
            case WatcherSettingsPage::Parameters:
                lv_label_set_text(title_label_, Tr("系统设定", "Parameters"));
                break;
        }
    }

    if (wifi_ssid_label_ != nullptr) {
        auto& wifi = WifiManager::GetInstance();
        if (wifi.IsConnected()) {
            std::string ssid = wifi.GetSsid();
            lv_label_set_text(wifi_ssid_label_, (!ssid.empty()) ? ssid.c_str() : Tr("已连接", "Connected"));
        } else if (wifi.IsConfigMode()) {
            lv_label_set_text(wifi_ssid_label_, Tr("热点配网中", "AP Config Mode"));
        } else {
            lv_label_set_text(wifi_ssid_label_, Tr("无网络连接", "Disconnected"));
        }
    }

    if (bri_title_label_ != nullptr) {
        lv_label_set_text(bri_title_label_, Tr("屏幕亮度", "Brightness"));
    }

    if (theme_title_label_ != nullptr) {
        lv_label_set_text(theme_title_label_, Tr("外观风格", "Theme"));
    }
    if (theme_val_label_ != nullptr) {
        bool is_dark = display_->GetTheme() && display_->GetTheme()->name() == "dark";
        lv_label_set_text(theme_val_label_, is_dark ? Tr("暗黑模式", "Dark") : Tr("明亮模式", "Light"));
    }

    if (close_lbl_ != nullptr) {
        lv_label_set_text(close_lbl_, Tr("关闭", "Close"));
    }

    // NOTE: the 20px/16px subset (watcher_font.c) carries 95 CJK glyphs and
    // falls back to a Latin-only Noto face. Every zh string below is written
    // from that 95-glyph set; adding a character outside it renders as tofu.
    const char* zh_modes[4] = { "不感知", "人形感知", "智能问候", "哨兵模式" };
    const char* en_modes[4] = { "Off", "Human Motion", "Smart Greeting", "Sentry Mode" };
    const char* zh_subs[4] = {
        "关闭视觉，不唤醒",
        "有人即唤醒，不识别",
        "叫名字，主动问候",
        "陌生人警戒"
    };
    const char* en_subs[4] = {
        "No camera wake",
        "Wakes on presence",
        "Greets by name",
        "Alerts on strangers"
    };
    for (int i = 0; i < 4; i++) {
        if (mode_labels_[i] != nullptr) {
            lv_label_set_text(mode_labels_[i], Tr(zh_modes[i], en_modes[i]));
        }
        if (mode_sub_labels_[i] != nullptr) {
            lv_label_set_text(mode_sub_labels_[i], Tr(zh_subs[i], en_subs[i]));
        }
    }

    if (th_title_label_ != nullptr) {
        lv_label_set_text(th_title_label_, Tr("识别相似度", "Face Match"));
    }
    if (th_sub_label_ != nullptr) {
        lv_label_set_text(th_sub_label_, Tr("越高越严格", "Higher is stricter"));
    }
    if (int_title_label_ != nullptr) {
        lv_label_set_text(int_title_label_, Tr("问候冷却", "Cooldown"));
    }
    if (int_sub_label_ != nullptr) {
        lv_label_set_text(int_sub_label_, Tr("问候后的冷却时间", "Wait between greetings"));
    }
    if (interval_label_ != nullptr) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%d%s", pending_interval_, Tr("秒", "s"));
        lv_label_set_text(interval_label_, buf);
    }
    if (lbl_minus_ != nullptr) {
        lv_label_set_text(lbl_minus_, "- 1");
    }
    if (lbl_plus_ != nullptr) {
        lv_label_set_text(lbl_plus_, "+ 1");
    }
    if (face_count_title_label_ != nullptr) {
        lv_label_set_text(face_count_title_label_, Tr("已录入", "Enrolled"));
    }
    if (face_count_sub_label_ != nullptr) {
        lv_label_set_text(face_count_sub_label_, Tr("本地已录人数", "People in local database"));
    }
    if (face_count_label_ != nullptr) {
        int count = FaceDatabase::GetInstance().GetFaceCount();
        char db_buf[16];
        snprintf(db_buf, sizeof(db_buf), "%d%s", count, Tr(" 人", ""));
        lv_label_set_text(face_count_label_, db_buf);
    }
}

const lv_font_t* WatcherSettingsUi::GetTextFont() const {
    return &watcher_chinese_font_20;
}

const lv_font_t* WatcherSettingsUi::GetSecondaryFont() const {
    return &watcher_chinese_font_16;
}

WatcherUiPalette WatcherSettingsUi::GetPalette() const {
    bool is_dark = display_->GetTheme() && display_->GetTheme()->name() == "dark";
    if (is_dark) {
        // Night (AMOLED true black): depth comes from surface steps, never
        // from shadows -- every shadow blur would be re-rasterised on each
        // frame of a full-screen scroll on the ESP32-S3.
        return {
            .bg = lv_color_hex(0x000000),
            .card = lv_color_hex(0x14171A),
            .card_elev = lv_color_hex(0x1A1F24),
            .card_border = lv_color_hex(0x2A323A),
            .card_press = lv_color_hex(0x212830),
            .card_active = lv_color_hex(0x0E2A22),
            .card_active_border = lv_color_hex(0x00E5A3),
            .btn_bg = lv_color_hex(0x1E2429),
            .btn_border = lv_color_hex(0x2F3841),
            .accent = lv_color_hex(0x00E5A3),
            .on_accent = lv_color_hex(0x04231B),
            .text_pri = lv_color_hex(0xFFFFFF),
            .text_sec = lv_color_hex(0x9BA3AC),
            .text_ter = lv_color_hex(0x5C656E),
            .text_active = lv_color_hex(0xFFFFFF),
            .text_active_sec = lv_color_hex(0x6FE9C4),
            .icon_bg = lv_color_hex(0x1E252B),
            .btn_close_bg = lv_color_hex(0x171C21),
            .btn_close_border = lv_color_hex(0x2A323A),
            .track_bg = lv_color_hex(0x262E35),
            .dot_inactive = lv_color_hex(0x4A535C),
        };
    } else {
        // Day: the accent darkens to #00875F so white-on-accent icon fills
        // clear 4.5:1, which the bright mint cannot do on a white surface.
        return {
            .bg = lv_color_hex(0xFFFFFF),
            .card = lv_color_hex(0xF3F5F7),
            .card_elev = lv_color_hex(0xF7F9FA),
            .card_border = lv_color_hex(0xE1E6EA),
            .card_press = lv_color_hex(0xE6EBEF),
            .card_active = lv_color_hex(0xE3F9F1),
            .card_active_border = lv_color_hex(0x00875F),
            .btn_bg = lv_color_hex(0xEDF0F3),
            .btn_border = lv_color_hex(0xD8DEE4),
            .accent = lv_color_hex(0x00875F),
            .on_accent = lv_color_hex(0xFFFFFF),
            .text_pri = lv_color_hex(0x0B0E11),
            .text_sec = lv_color_hex(0x5B6470),
            .text_ter = lv_color_hex(0x9AA3AD),
            .text_active = lv_color_hex(0x05412F),
            .text_active_sec = lv_color_hex(0x0B6B4C),
            .icon_bg = lv_color_hex(0xE7EBEF),
            .btn_close_bg = lv_color_hex(0xF1F4F6),
            .btn_close_border = lv_color_hex(0xDCE2E8),
            .track_bg = lv_color_hex(0xDFE4E9),
            .dot_inactive = lv_color_hex(0xC2CAD2),
        };
    }
}

void WatcherSettingsUi::ApplyThemeToRoot() {
    if (root_ == nullptr) return;
    auto pal = GetPalette();

    lv_obj_set_style_bg_color(root_, pal.bg, 0);
    lv_obj_set_style_bg_opa(root_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(root_, 0, 0);
    StyleHeader();
}

void WatcherSettingsUi::StyleHeader() {
    auto pal = GetPalette();

    if (dismiss_bar_ != nullptr) {
        lv_obj_set_style_bg_color(dismiss_bar_, pal.text_ter, 0);
        lv_obj_set_style_bg_opa(dismiss_bar_, LV_OPA_80, 0);
    }

    // Page indicator: the active page stretches into a short bar so position
    // is readable at a glance without a second text label.
    const int active_page = static_cast<int>(current_page_);
    for (int i = 0; i < 3; i++) {
        if (page_dots_[i] == nullptr) continue;
        const bool is_active = (i == active_page);
        lv_obj_set_width(page_dots_[i], is_active ? 16 : 5);
        lv_obj_set_style_bg_color(page_dots_[i],
                                  is_active ? pal.accent : pal.dot_inactive, 0);
    }

    if (title_label_ != nullptr) {
        lv_obj_set_style_text_color(title_label_, pal.text_pri, 0);
    }
}

void WatcherSettingsUi::ClearWidgetPointers() {
    dismiss_handle_ = nullptr;
    dismiss_bar_ = nullptr;
    for (int i = 0; i < 3; i++) {
        page_dots_[i] = nullptr;
    }
    // title_label_ / content_card_ live in the header and list chrome that
    // survives an in-place page rebuild, so they keep their pointers here.
    wifi_card_ = nullptr;
    wifi_icon_label_ = nullptr;
    wifi_ssid_label_ = nullptr;
    wifi_reconfig_btn_ = nullptr;
    wifi_reconfig_icon_ = nullptr;
    bri_box_ = nullptr;
    bri_title_label_ = nullptr;
    brightness_slider_ = nullptr;
    brightness_label_ = nullptr;
    theme_btn_ = nullptr;
    theme_title_label_ = nullptr;
    theme_val_label_ = nullptr;
    for (int i = 0; i < 4; i++) {
        mode_btns_[i] = nullptr;
        mode_labels_[i] = nullptr;
        mode_sub_labels_[i] = nullptr;
        mode_icons_[i] = nullptr;
        mode_checks_[i] = nullptr;
    }
    th_card_ = nullptr;
    th_title_label_ = nullptr;
    th_sub_label_ = nullptr;
    threshold_slider_ = nullptr;
    threshold_val_label_ = nullptr;
    int_card_ = nullptr;
    int_title_label_ = nullptr;
    int_sub_label_ = nullptr;
    interval_label_ = nullptr;
    btn_minus_ = nullptr;
    lbl_minus_ = nullptr;
    btn_plus_ = nullptr;
    lbl_plus_ = nullptr;
    face_count_card_ = nullptr;
    face_count_icon_ = nullptr;
    face_count_title_label_ = nullptr;
    face_count_sub_label_ = nullptr;
    face_count_label_ = nullptr;
    close_btn_ = nullptr;
    close_icon_ = nullptr;
    close_lbl_ = nullptr;
}

// ---------------------------------------------------------------------------
// Shared construction helpers
// ---------------------------------------------------------------------------

lv_obj_t* WatcherSettingsUi::CreateCard(lv_obj_t* parent, int width, int height,
                                        lv_color_t bg, lv_color_t border) {
    lv_obj_t* card = lv_obj_create(parent);
    lv_obj_set_size(card, width, height);
    lv_obj_set_style_bg_color(card, bg, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(card, kCardRadius, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(card, border, 0);
    lv_obj_set_style_pad_hor(card, 14, 0);
    lv_obj_set_style_pad_ver(card, 10, 0);
    lv_obj_set_style_pad_row(card, 5, 0);
    ConfigureListChild(card);
    return card;
}

void WatcherSettingsUi::ApplyCardPressFeedback(lv_obj_t* obj, const WatcherUiPalette& pal) {
    if (obj == nullptr) return;
    lv_obj_set_style_bg_color(obj, pal.card_press, Sel(LV_PART_MAIN, LV_STATE_PRESSED));
    lv_obj_set_style_border_color(obj, pal.accent, Sel(LV_PART_MAIN, LV_STATE_PRESSED));
}

lv_obj_t* WatcherSettingsUi::CreateIconCapsule(lv_obj_t* parent, const char* symbol,
                                               const WatcherUiPalette& pal) {
    // The capsule is the label's own background: a padded, rounded label needs
    // no wrapper object, so an icon costs one widget instead of two.
    lv_obj_t* icon = lv_label_create(parent);
    lv_label_set_text(icon, symbol);
    lv_obj_set_size(icon, kCapsule, kCapsule);
    lv_obj_set_style_text_font(icon, &font_material_symbols_30_4, 0);
    lv_obj_set_style_text_color(icon, pal.text_pri, 0);
    lv_obj_set_style_text_align(icon, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_bg_color(icon, pal.icon_bg, 0);
    lv_obj_set_style_bg_opa(icon, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(icon, 14, 0);
    lv_obj_set_style_pad_hor(icon, 7, 0);
    lv_obj_set_style_pad_ver(icon, 8, 0);   // 46 - 16 = 30 = icon line height
    lv_obj_remove_flag(icon, LV_OBJ_FLAG_CLICKABLE);
    return icon;
}

lv_obj_t* WatcherSettingsUi::CreateValueRow(lv_obj_t* parent, const char* symbol,
                                            const WatcherUiPalette& pal) {
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_set_size(row, LV_PCT(100), kRowH);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_style_pad_column(row, 12, 0);
    ConfigureListChild(row);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    CreateIconCapsule(row, symbol, pal);
    return row;
}

lv_obj_t* WatcherSettingsUi::CreateSlider(lv_obj_t* parent, const WatcherUiPalette& pal,
                                          int width, int min, int max, int value) {
    lv_obj_t* slider = lv_slider_create(parent);
    lv_obj_set_size(slider, width, kSliderH);
    lv_slider_set_range(slider, min, max);
    lv_slider_set_value(slider, value, LV_ANIM_OFF);

    // EVENT_BUBBLE is NOT a default object flag (lv_obj.c constructor), so
    // without it a press on the rail never reaches the drawer handler: the
    // DismissHandleCallback slider guard would be dead code and an up-swipe
    // started on the rail would be a silent dead zone.
    //
    // GESTURE_BUBBLE, by contrast, IS set by default and must stay removed:
    // it would turn a deliberate leftward slider drag into LV_DIR_LEFT and
    // close the drawer on the Parameters page.
    lv_obj_remove_flag(slider, static_cast<lv_obj_flag_t>(
        LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_CHAIN_HOR));
    lv_obj_add_flag(slider, static_cast<lv_obj_flag_t>(
        LV_OBJ_FLAG_EVENT_BUBBLE | LV_OBJ_FLAG_SCROLL_CHAIN_VER));
    lv_obj_set_scrollbar_mode(slider, LV_SCROLLBAR_MODE_OFF);

    // Rail
    lv_obj_set_style_bg_color(slider, pal.track_bg, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(slider, 0, LV_PART_MAIN);

    // Fill
    lv_obj_set_style_bg_color(slider, pal.accent, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_INDICATOR);

    // Knob. The default theme pads LV_PART_KNOB by 6px, which squashes the
    // knob into a 20x28 oval; zeroing the pad and driving the size through
    // transform_* produces a true circle that overhangs the rail by 4px.
    lv_obj_set_style_pad_all(slider, 0, LV_PART_KNOB);
    lv_obj_set_style_transform_width(slider, kKnobGrowW, LV_PART_KNOB);
    lv_obj_set_style_transform_height(slider, kKnobGrowH, LV_PART_KNOB);
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_KNOB);
    lv_obj_set_style_bg_color(slider, pal.text_pri, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_width(slider, 0, LV_PART_KNOB);

    // Press: grow to 28px. Declared locally so the theme's generic grow style
    // cannot land on a different size.
    lv_obj_set_style_transform_width(slider, kKnobGrowWPrs, Sel(LV_PART_KNOB, LV_STATE_PRESSED));
    lv_obj_set_style_transform_height(slider, kKnobGrowHPrs, Sel(LV_PART_KNOB, LV_STATE_PRESSED));
    return slider;
}

lv_obj_t* WatcherSettingsUi::CreateCloseButton(lv_obj_t* parent, const WatcherUiPalette& pal) {
    close_btn_ = lv_button_create(parent);
    lv_obj_set_size(close_btn_, 152, 42);
    lv_obj_set_style_bg_color(close_btn_, pal.btn_close_bg, 0);
    lv_obj_set_style_radius(close_btn_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(close_btn_, 1, 0);
    lv_obj_set_style_border_color(close_btn_, pal.btn_close_border, 0);
    lv_obj_set_style_pad_all(close_btn_, 0, 0);
    lv_obj_set_style_pad_column(close_btn_, 6, 0);
    ConfigureListChild(close_btn_);
    lv_obj_set_flex_flow(close_btn_, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(close_btn_, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    ApplyCardPressFeedback(close_btn_, pal);
    lv_obj_add_event_cb(close_btn_, CloseButtonCallback, LV_EVENT_CLICKED, this);

    close_icon_ = lv_label_create(close_btn_);
    lv_label_set_text(close_icon_, MATERIAL_SYMBOLS_CLOSE);
    lv_obj_set_style_text_font(close_icon_, &font_material_symbols_20_4, 0);
    lv_obj_set_style_text_color(close_icon_, pal.text_sec, 0);
    lv_obj_remove_flag(close_icon_, LV_OBJ_FLAG_CLICKABLE);

    close_lbl_ = lv_label_create(close_btn_);
    lv_label_set_text(close_lbl_, Tr("关闭", "Close"));
    lv_obj_set_style_text_font(close_lbl_, GetTextFont(), 0);
    lv_obj_set_style_text_color(close_lbl_, pal.text_pri, 0);
    lv_obj_remove_flag(close_lbl_, LV_OBJ_FLAG_CLICKABLE);
    return close_btn_;
}

void WatcherSettingsUi::BuildHeader() {
    auto pal = GetPalette();

    dismiss_handle_ = lv_obj_create(root_);
    lv_obj_set_size(dismiss_handle_, 236, kHeaderH);
    lv_obj_align(dismiss_handle_, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(dismiss_handle_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(dismiss_handle_, 0, 0);
    lv_obj_set_style_pad_all(dismiss_handle_, 0, 0);
    ConfigureFixedObject(dismiss_handle_);
    lv_obj_add_flag(dismiss_handle_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(dismiss_handle_, DismissHandleCallback, LV_EVENT_ALL, this);

    // Grab bar
    dismiss_bar_ = lv_obj_create(dismiss_handle_);
    lv_obj_set_size(dismiss_bar_, 36, 4);
    lv_obj_align(dismiss_bar_, LV_ALIGN_TOP_MID, 0, 4);
    lv_obj_set_style_radius(dismiss_bar_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(dismiss_bar_, 0, 0);
    ConfigureFixedObject(dismiss_bar_);
    lv_obj_remove_flag(dismiss_bar_, LV_OBJ_FLAG_CLICKABLE);

    // Page indicator (position readout for the three drawer pages)
    lv_obj_t* dots = lv_obj_create(dismiss_handle_);
    lv_obj_set_size(dots, 56, 5);
    lv_obj_align(dots, LV_ALIGN_TOP_MID, 0, 15);
    lv_obj_set_style_bg_opa(dots, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(dots, 0, 0);
    lv_obj_set_style_pad_all(dots, 0, 0);
    lv_obj_set_style_pad_column(dots, 5, 0);
    ConfigureFixedObject(dots);
    lv_obj_remove_flag(dots, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_flex_flow(dots, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(dots, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    for (int i = 0; i < 3; i++) {
        page_dots_[i] = lv_obj_create(dots);
        lv_obj_set_height(page_dots_[i], 5);
        lv_obj_set_style_radius(page_dots_[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(page_dots_[i], 0, 0);
        lv_obj_set_style_bg_opa(page_dots_[i], LV_OPA_COVER, 0);
        ConfigureFixedObject(page_dots_[i]);
        lv_obj_remove_flag(page_dots_[i], LV_OBJ_FLAG_CLICKABLE);
    }

    // Title
    title_label_ = lv_label_create(dismiss_handle_);
    lv_obj_set_style_text_font(title_label_, GetTextFont(), 0);
    lv_obj_set_style_text_color(title_label_, pal.text_pri, 0);
    lv_obj_set_style_text_align(title_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(title_label_, LV_ALIGN_TOP_MID, 0, 24);
    lv_obj_remove_flag(title_label_, LV_OBJ_FLAG_CLICKABLE);

    StyleHeader();
}

void WatcherSettingsUi::BuildContentList(bool scrollable) {
    content_card_ = lv_obj_create(root_);
    lv_obj_set_size(content_card_, kListW, kContentH);
    lv_obj_align(content_card_, LV_ALIGN_TOP_MID, 0, kContentTop);
    lv_obj_set_style_bg_opa(content_card_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(content_card_, 0, 0);
    lv_obj_set_style_pad_hor(content_card_, (kListW - kCardW) / 2, 0);
    lv_obj_set_style_pad_row(content_card_, kCardGap, 0);
    lv_obj_set_flex_flow(content_card_, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_flag(content_card_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(content_card_, static_cast<lv_obj_flag_t>(
        LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_EVENT_BUBBLE));
    lv_obj_add_event_cb(content_card_, DismissHandleCallback, LV_EVENT_ALL, this);

    if (scrollable) {
        lv_obj_set_style_pad_top(content_card_, 2, 0);
        lv_obj_set_style_pad_bottom(content_card_, 40, 0);
        lv_obj_set_flex_align(content_card_, LV_FLEX_ALIGN_START,
                              LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_add_flag(content_card_, static_cast<lv_obj_flag_t>(
            LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM));
        lv_obj_set_scroll_dir(content_card_, LV_DIR_VER);
    } else {
        // QuickSettings is sized to sit entirely inside the safe circle band,
        // so it never scrolls. That removes the only competing consumer of the
        // upward swipe and makes "swipe up anywhere" a dependable dismiss.
        lv_obj_set_style_pad_top(content_card_, 0, 0);
        lv_obj_set_style_pad_bottom(content_card_, 0, 0);
        lv_obj_set_flex_align(content_card_, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_remove_flag(content_card_, LV_OBJ_FLAG_SCROLLABLE);
    }
    lv_obj_set_scrollbar_mode(content_card_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_remove_flag(content_card_, static_cast<lv_obj_flag_t>(
        LV_OBJ_FLAG_SCROLL_CHAIN_HOR | LV_OBJ_FLAG_SCROLL_CHAIN_VER));
}

void WatcherSettingsUi::ScheduleNvsCommit() {
    if (nvs_timer_ != nullptr) {
        esp_timer_stop(nvs_timer_);
        // Debounced because dragging a slider fires VALUE_CHANGED on every
        // frame, and each vision-mode step rewrites three NVS keys.
        esp_timer_start_once(nvs_timer_, 800000);
    }
}

void WatcherSettingsUi::NvsCommitTimerCallback(void* arg) {
    auto* self = static_cast<WatcherSettingsUi*>(arg);
    if (self == nullptr) return;
    // Offload NVS commit and camera UART communication to a background task
    // to avoid blocking ESP-IDF's esp_timer system daemon thread.
    xTaskCreate([](void* task_arg) {
        auto* s = static_cast<WatcherSettingsUi*>(task_arg);
        if (s != nullptr) {
            s->CommitPendingSettings();
        }
        vTaskDelete(nullptr);
    }, "ui_nvs_commit", 3072, self, 3, nullptr);
}

void WatcherSettingsUi::CommitPendingSettings() {
    int brightness_to_save = -1;
    int threshold_to_save = -1;
    int interval_to_save = -1;
    int vision_mode_to_apply = -1;

    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (dirty_brightness_) {
            brightness_to_save = pending_brightness_;
            dirty_brightness_ = false;
        }
        if (dirty_threshold_) {
            threshold_to_save = static_cast<int>(std::round(pending_threshold_ * 100.0f));
            dirty_threshold_ = false;
        }
        if (dirty_interval_) {
            interval_to_save = pending_interval_;
            dirty_interval_ = false;
        }
        if (dirty_vision_mode_) {
            vision_mode_to_apply = pending_vision_mode_;
            dirty_vision_mode_ = false;
        }
    }

    if (brightness_to_save >= 0) {
        Settings s("display", true);
        s.SetInt("brightness", brightness_to_save);
        ESP_LOGI(TAG, "Committed brightness to NVS: %d", brightness_to_save);
    }
    if (threshold_to_save >= 0) {
        Settings s("face", true);
        s.SetInt("threshold", threshold_to_save);
        ESP_LOGI(TAG, "Committed face threshold to NVS: %d%%", threshold_to_save);
    }
    if (interval_to_save >= 0) {
        Settings sm("model", true);
        sm.SetInt("interval", interval_to_save);
        ESP_LOGI(TAG, "Committed interval to NVS: %ds", interval_to_save);
    }
    if (vision_mode_to_apply >= 0 && camera_ != nullptr) {
        // Applied outside the lock: SetVisionWakeMode writes NVS and toggles
        // the recognition engines, and the knob changes only the highlight
        // until the user stops turning.
        camera_->SetVisionWakeMode(vision_mode_to_apply);
    }
}

void WatcherSettingsUi::FlushPendingChanges() {
    if (nvs_timer_ != nullptr) {
        esp_timer_stop(nvs_timer_);
    }
    CommitPendingSettings();
}

void WatcherSettingsUi::Open(WatcherSettingsPage page) {
    DisplayLockGuard lock(display_);
    auto pal = GetPalette();

    std::lock_guard<std::recursive_mutex> guard(mutex_);

    if (drawer_state_ == DrawerState::Closing) {
        return;
    }

    if (root_ != nullptr) {
        if (current_page_ == page) {
            return;
        }
        // Switching page in place: drop the old chrome and re-run the entry
        // animation so the new page slides in from its own edge.
        lv_anim_delete(root_, nullptr);
        lv_obj_del(root_);
        root_ = nullptr;
        content_card_ = nullptr;
        title_label_ = nullptr;
        ClearWidgetPointers();
    }

    current_page_ = page;
    drawer_state_ = DrawerState::Opening;

    root_ = lv_obj_create(lv_layer_top());
    lv_obj_set_size(root_, kScreenSize, kScreenSize);
    lv_obj_set_style_bg_color(root_, pal.bg, 0);
    lv_obj_set_style_bg_opa(root_, LV_OPA_COVER, 0); // 100% solid opaque
    lv_obj_set_style_radius(root_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(root_, 0, 0);
    lv_obj_set_style_pad_all(root_, 0, 0);
    ConfigureFixedObject(root_);
    lv_obj_add_flag(root_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(root_, DismissHandleCallback, LV_EVENT_ALL, this);

    // Initial off-screen position based on pull direction
    if (page == WatcherSettingsPage::QuickSettings) {
        lv_obj_set_pos(root_, 0, -kScreenSize);
    } else if (page == WatcherSettingsPage::VisionMode) {
        lv_obj_set_pos(root_, kScreenSize, 0);
    } else if (page == WatcherSettingsPage::Parameters) {
        lv_obj_set_pos(root_, -kScreenSize, 0);
    }

    BuildHeader();
    BuildContentList(page != WatcherSettingsPage::QuickSettings);

    if (backlight_ != nullptr) {
        pending_brightness_ = backlight_->brightness();
    }
    if (camera_ != nullptr) {
        pending_vision_mode_ = camera_->GetVisionWakeMode();
    }
    pending_threshold_ = FaceRecognition::GetInstance().GetMatchThreshold();

    if (current_page_ == WatcherSettingsPage::QuickSettings) {
        BuildQuickSettingsPage();
    } else if (current_page_ == WatcherSettingsPage::VisionMode) {
        BuildVisionModePage();
    } else if (current_page_ == WatcherSettingsPage::Parameters) {
        BuildParametersPage();
    }

    StyleHeader();

    // Slide-in pull animation
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, root_);
    lv_anim_set_duration(&a, 260);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);

    if (page == WatcherSettingsPage::QuickSettings) {
        lv_anim_set_values(&a, -kScreenSize, 0);
        lv_anim_set_exec_cb(&a, anim_y_cb);
    } else if (page == WatcherSettingsPage::VisionMode) {
        lv_anim_set_values(&a, kScreenSize, 0);
        lv_anim_set_exec_cb(&a, anim_x_cb);
    } else if (page == WatcherSettingsPage::Parameters) {
        lv_anim_set_values(&a, -kScreenSize, 0);
        lv_anim_set_exec_cb(&a, anim_x_cb);
    }

    lv_anim_set_completed_cb(&a, [](lv_anim_t* anim) {
        auto* self = static_cast<WatcherSettingsUi*>(anim->user_data);
        if (self != nullptr) {
            self->drawer_state_ = DrawerState::Open;
        }
    });
    lv_anim_set_user_data(&a, this);
    lv_anim_start(&a);
}

void WatcherSettingsUi::Close(bool animated) {
    DisplayLockGuard lock(display_);
    std::lock_guard<std::recursive_mutex> guard(mutex_);

    if (root_ == nullptr || drawer_state_ == DrawerState::Closed) return;

    // Persist before the exit animation so a knob-only change (which is only
    // highlighted, not applied) is never lost on dismissal.
    FlushPendingChanges();

    if (!animated) {
        // Immediate synchronous close: cancel any in-flight exit animation
        // to prevent use-after-free callbacks, then destroy widgets immediately.
        if (root_ != nullptr) {
            lv_anim_delete(root_, nullptr);
        }
        FinishClose();
        return;
    }

    if (drawer_state_ == DrawerState::Closing) return;

    drawer_state_ = DrawerState::Closing;
    lv_anim_delete(root_, nullptr);

    int start_y = lv_obj_get_y(root_);
    int start_x = lv_obj_get_x(root_);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, root_);
    lv_anim_set_duration(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);

    if (current_page_ == WatcherSettingsPage::QuickSettings) {
        lv_anim_set_values(&a, start_y, -kScreenSize);
        lv_anim_set_exec_cb(&a, anim_y_cb);
    } else if (current_page_ == WatcherSettingsPage::VisionMode) {
        lv_anim_set_values(&a, start_x, kScreenSize);
        lv_anim_set_exec_cb(&a, anim_x_cb);
    } else if (current_page_ == WatcherSettingsPage::Parameters) {
        lv_anim_set_values(&a, start_x, -kScreenSize);
        lv_anim_set_exec_cb(&a, anim_x_cb);
    }

    lv_anim_set_completed_cb(&a, [](lv_anim_t* anim) {
        auto* self = static_cast<WatcherSettingsUi*>(anim->user_data);
        if (self != nullptr) {
            self->FinishClose();
        }
    });
    lv_anim_set_user_data(&a, this);
    lv_anim_start(&a);
}

void WatcherSettingsUi::FinishClose() {
    DisplayLockGuard lock(display_);
    std::lock_guard<std::recursive_mutex> guard(mutex_);

    if (root_ != nullptr) {
        lv_anim_delete(root_, nullptr);
        lv_obj_del(root_);
        root_ = nullptr;
        content_card_ = nullptr;
        title_label_ = nullptr;
        ClearWidgetPointers();
    }
    drawer_state_ = DrawerState::Closed;
    dismiss_tracking_ = false;
    if (close_cb_) {
        close_cb_();
    }
}

void WatcherSettingsUi::BuildQuickSettingsPage() {
    auto pal = GetPalette();

    lv_label_set_text(title_label_, Tr("设定", "Settings"));

    // --- Wi-Fi status + reconfiguration -------------------------------------
    wifi_card_ = CreateCard(content_card_, kCardW, 68, pal.card, pal.card_border);
    lv_obj_set_flex_flow(wifi_card_, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(wifi_card_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(wifi_card_, 12, 0);

    wifi_icon_label_ = CreateIconCapsule(wifi_card_, MATERIAL_SYMBOLS_WIFI, pal);

    wifi_ssid_label_ = lv_label_create(wifi_card_);
    lv_obj_set_style_text_font(wifi_ssid_label_, GetTextFont(), 0);
    lv_obj_set_style_text_color(wifi_ssid_label_, pal.text_pri, 0);
    lv_obj_set_flex_grow(wifi_ssid_label_, 1);
    lv_label_set_long_mode(wifi_ssid_label_, LV_LABEL_LONG_DOT);
    lv_obj_remove_flag(wifi_ssid_label_, LV_OBJ_FLAG_CLICKABLE);

    auto& wifi = WifiManager::GetInstance();
    if (wifi.IsConnected()) {
        std::string ssid = wifi.GetSsid();
        lv_label_set_text(wifi_icon_label_, MATERIAL_SYMBOLS_WIFI);
        lv_obj_set_style_text_color(wifi_icon_label_, pal.accent, 0);
        lv_label_set_text(wifi_ssid_label_, (!ssid.empty()) ? ssid.c_str() : Tr("已连接", "Connected"));
    } else if (wifi.IsConfigMode()) {
        lv_label_set_text(wifi_icon_label_, MATERIAL_SYMBOLS_WIFI);
        lv_obj_set_style_text_color(wifi_icon_label_, pal.text_sec, 0);
        lv_label_set_text(wifi_ssid_label_, Tr("热点配网中", "AP Config Mode"));
    } else {
        lv_label_set_text(wifi_icon_label_, MATERIAL_SYMBOLS_WIFI_OFF);
        lv_obj_set_style_text_color(wifi_icon_label_, pal.text_sec, 0);
        lv_label_set_text(wifi_ssid_label_, Tr("无网络连接", "Disconnected"));
    }

    wifi_reconfig_btn_ = lv_button_create(wifi_card_);
    lv_obj_set_size(wifi_reconfig_btn_, 38, 38);
    lv_obj_set_style_radius(wifi_reconfig_btn_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(wifi_reconfig_btn_, pal.btn_bg, 0);
    lv_obj_set_style_border_width(wifi_reconfig_btn_, 1, 0);
    lv_obj_set_style_border_color(wifi_reconfig_btn_, pal.btn_border, 0);
    lv_obj_set_style_pad_all(wifi_reconfig_btn_, 0, 0);
    ConfigureListChild(wifi_reconfig_btn_);
    ApplyCardPressFeedback(wifi_reconfig_btn_, pal);
    lv_obj_add_event_cb(wifi_reconfig_btn_, WifiButtonCallback, LV_EVENT_CLICKED, this);

    wifi_reconfig_icon_ = lv_label_create(wifi_reconfig_btn_);
    lv_label_set_text(wifi_reconfig_icon_, MATERIAL_SYMBOLS_REFRESH);
    lv_obj_set_style_text_font(wifi_reconfig_icon_, &font_material_symbols_20_4, 0);
    lv_obj_set_style_text_color(wifi_reconfig_icon_, pal.text_pri, 0);
    lv_obj_center(wifi_reconfig_icon_);

    // --- Brightness ---------------------------------------------------------
    bri_box_ = CreateCard(content_card_, kCardW, 94, pal.card_elev, pal.card_border);
    lv_obj_set_flex_flow(bri_box_, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(bri_box_, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t* bri_row = CreateValueRow(bri_box_, MATERIAL_SYMBOLS_BRIGHTNESS_6, pal);

    bri_title_label_ = lv_label_create(bri_row);
    lv_label_set_text(bri_title_label_, Tr("屏幕亮度", "Brightness"));
    lv_obj_set_style_text_font(bri_title_label_, GetTextFont(), 0);
    lv_obj_set_style_text_color(bri_title_label_, pal.text_pri, 0);
    lv_obj_remove_flag(bri_title_label_, LV_OBJ_FLAG_CLICKABLE);

    brightness_label_ = lv_label_create(bri_row);
    char bri_buf[16];
    snprintf(bri_buf, sizeof(bri_buf), "%d%%", pending_brightness_);
    lv_label_set_text(brightness_label_, bri_buf);
    lv_obj_set_style_text_font(brightness_label_, GetTextFont(), 0);
    lv_obj_set_style_text_color(brightness_label_, pal.accent, 0);
    lv_obj_set_flex_grow(brightness_label_, 1);
    lv_obj_set_style_text_align(brightness_label_, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_remove_flag(brightness_label_, LV_OBJ_FLAG_CLICKABLE);

    brightness_slider_ = CreateSlider(bri_box_, pal, kCardW - 28, 10, 100, pending_brightness_);
    lv_obj_add_event_cb(brightness_slider_, BrightnessSliderCallback, LV_EVENT_VALUE_CHANGED, this);

    // --- Theme --------------------------------------------------------------
    theme_btn_ = lv_button_create(content_card_);
    lv_obj_set_size(theme_btn_, kCardW, 68);
    lv_obj_set_style_bg_color(theme_btn_, pal.card, 0);
    lv_obj_set_style_bg_opa(theme_btn_, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(theme_btn_, kCardRadius, 0);
    lv_obj_set_style_border_width(theme_btn_, 1, 0);
    lv_obj_set_style_border_color(theme_btn_, pal.card_border, 0);
    lv_obj_set_style_pad_hor(theme_btn_, 14, 0);
    lv_obj_set_style_pad_ver(theme_btn_, 10, 0);
    lv_obj_set_style_pad_column(theme_btn_, 12, 0);
    ConfigureListChild(theme_btn_);
    lv_obj_set_flex_flow(theme_btn_, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(theme_btn_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    ApplyCardPressFeedback(theme_btn_, pal);
    lv_obj_add_event_cb(theme_btn_, ThemeButtonCallback, LV_EVENT_CLICKED, this);

    CreateIconCapsule(theme_btn_, MATERIAL_SYMBOLS_EYEGLASSES, pal);

    theme_title_label_ = lv_label_create(theme_btn_);
    lv_label_set_text(theme_title_label_, Tr("外观风格", "Theme"));
    lv_obj_set_style_text_font(theme_title_label_, GetTextFont(), 0);
    lv_obj_set_style_text_color(theme_title_label_, pal.text_pri, 0);
    lv_obj_remove_flag(theme_title_label_, LV_OBJ_FLAG_CLICKABLE);

    bool is_dark = display_->GetTheme() && display_->GetTheme()->name() == "dark";
    theme_val_label_ = lv_label_create(theme_btn_);
    lv_label_set_text(theme_val_label_, is_dark ? Tr("暗黑模式", "Dark") : Tr("明亮模式", "Light"));
    lv_obj_set_style_text_font(theme_val_label_, GetTextFont(), 0);
    lv_obj_set_style_text_color(theme_val_label_, pal.text_sec, 0);
    lv_obj_set_flex_grow(theme_val_label_, 1);
    lv_obj_set_style_text_align(theme_val_label_, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_remove_flag(theme_val_label_, LV_OBJ_FLAG_CLICKABLE);

    // --- Close --------------------------------------------------------------
    CreateCloseButton(content_card_, pal);
}

void WatcherSettingsUi::OnSwitchWifiClicked() {
    if (wifi_reconfig_btn_ != nullptr) {
        lv_obj_clear_flag(wifi_reconfig_btn_, LV_OBJ_FLAG_CLICKABLE);
    }
    if (wifi_ssid_label_ != nullptr) {
        lv_label_set_text(wifi_ssid_label_, Tr("正在重启...", "Rebooting..."));
    }

    ESP_LOGI(TAG, "User requested switch Wi-Fi: clearing credentials and rebooting into config mode");
    SsidManager::GetInstance().Clear();

    BaseType_t ret = xTaskCreate([](void*) {
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }, "reboot_wifi", 2048, nullptr, 5, nullptr);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to spawn reboot task, restarting synchronously");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }
}

void WatcherSettingsUi::WifiButtonCallback(lv_event_t* e) {
    auto* self = static_cast<WatcherSettingsUi*>(lv_event_get_user_data(e));
    self->OnSwitchWifiClicked();
}

void WatcherSettingsUi::BuildVisionModePage() {
    auto pal = GetPalette();

    lv_label_set_text(title_label_, Tr("模式设定", "Vision Modes"));

    bool has_face_model = (camera_ != nullptr && camera_->IsFaceModelAvailable());
    if (!has_face_model && (pending_vision_mode_ == 2 || pending_vision_mode_ == 3)) {
        pending_vision_mode_ = 1; // Fallback to 人形感知
    }

    const char* zh_titles[4] = { "不感知", "人形感知", "智能问候", "哨兵模式" };
    const char* en_titles[4] = { "Off", "Human Motion", "Smart Greeting", "Sentry Mode" };
    const char* zh_subs[4] = {
        "关闭视觉，不唤醒",
        "有人即唤醒，不识别",
        "叫名字，主动问候",
        "陌生人警戒"
    };
    const char* en_subs[4] = {
        "No camera wake",
        "Wakes on presence",
        "Greets by name",
        "Alerts on strangers"
    };
    const char* icons[4] = {
        MATERIAL_SYMBOLS_CANCEL,
        MATERIAL_SYMBOLS_PERSON,
        MATERIAL_SYMBOLS_ROBOT_2,
        MATERIAL_SYMBOLS_NOTIFICATIONS
    };

    for (int i = 0; i < 4; i++) {
        bool disabled = (!has_face_model && (i == 2 || i == 3));

        // 92px tall: a full-width, thumb-sized row that still leaves the list
        // scrollable, which is what distinguishes this page from QuickSettings.
        mode_btns_[i] = lv_button_create(content_card_);
        lv_obj_set_size(mode_btns_[i], kCardW, 92);
        lv_obj_set_style_radius(mode_btns_[i], kCardRadius, 0);
        lv_obj_set_style_border_width(mode_btns_[i], 1, 0);
        lv_obj_set_style_pad_hor(mode_btns_[i], 14, 0);
        lv_obj_set_style_pad_ver(mode_btns_[i], 10, 0);
        lv_obj_set_style_pad_column(mode_btns_[i], 12, 0);
        ConfigureListChild(mode_btns_[i]);
        lv_obj_set_flex_flow(mode_btns_[i], LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(mode_btns_[i], LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        if (!disabled) {
            ApplyCardPressFeedback(mode_btns_[i], pal);
            lv_obj_add_event_cb(mode_btns_[i], ModeButtonCallback, LV_EVENT_CLICKED, this);
        } else {
            lv_obj_set_style_opa(mode_btns_[i], LV_OPA_40, 0);
            lv_obj_remove_flag(mode_btns_[i], LV_OBJ_FLAG_CLICKABLE);
        }

        mode_icons_[i] = CreateIconCapsule(mode_btns_[i], icons[i], pal);

        // Center column for Title + Subtitle
        lv_obj_t* col = lv_obj_create(mode_btns_[i]);
        lv_obj_set_style_bg_opa(col, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(col, 0, 0);
        lv_obj_set_style_pad_all(col, 0, 0);
        lv_obj_set_style_pad_row(col, 1, 0);
        ConfigureListChild(col);
        lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
        lv_obj_set_flex_grow(col, 1);

        mode_labels_[i] = lv_label_create(col);
        lv_label_set_text(mode_labels_[i], Tr(zh_titles[i], en_titles[i]));
        lv_obj_set_style_text_font(mode_labels_[i], GetTextFont(), 0);
        lv_obj_remove_flag(mode_labels_[i], LV_OBJ_FLAG_CLICKABLE);

        mode_sub_labels_[i] = lv_label_create(col);
        if (disabled) {
            lv_label_set_text(mode_sub_labels_[i], Tr("需烧录人脸固件", "Requires face firmware"));
        } else {
            lv_label_set_text(mode_sub_labels_[i], Tr(zh_subs[i], en_subs[i]));
        }
        // The column is flex-sized; a content-width label would overflow the
        // 152px text column and run under the checkmark. Pin the width and
        // ellipsise instead of wrapping into the 92px card.
        lv_obj_set_width(mode_sub_labels_[i], LV_PCT(100));
        lv_label_set_long_mode(mode_sub_labels_[i], LV_LABEL_LONG_DOT);
        lv_obj_set_style_text_font(mode_sub_labels_[i], GetSecondaryFont(), 0);
        lv_obj_set_style_text_color(mode_sub_labels_[i], pal.text_sec, 0);
        lv_obj_remove_flag(mode_sub_labels_[i], LV_OBJ_FLAG_CLICKABLE);

        // Right checkmark. Kept in the flex flow at all times and toggled by
        // opacity: hiding it would resize the text column on every selection.
        mode_checks_[i] = lv_label_create(mode_btns_[i]);
        lv_label_set_text(mode_checks_[i], MATERIAL_SYMBOLS_CHECK);
        lv_obj_set_size(mode_checks_[i], 24, 24);
        lv_obj_set_style_text_font(mode_checks_[i], &font_material_symbols_20_4, 0);
        lv_obj_set_style_text_align(mode_checks_[i], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_opa(mode_checks_[i], LV_OPA_TRANSP, 0);
        lv_obj_remove_flag(mode_checks_[i], LV_OBJ_FLAG_CLICKABLE);
    }

    CreateCloseButton(content_card_, pal);
    UpdateVisionModeUi(pending_vision_mode_);
}

void WatcherSettingsUi::BuildParametersPage() {
    auto pal = GetPalette();

    lv_label_set_text(title_label_, Tr("系统设定", "Parameters"));

    int th_percent = static_cast<int>(std::round(pending_threshold_ * 100.0f));

    // --- Card 1: face match threshold ---------------------------------------
    th_card_ = CreateCard(content_card_, kCardW, 116, pal.card_elev, pal.card_border);
    lv_obj_set_flex_flow(th_card_, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(th_card_, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t* th_row1 = CreateValueRow(th_card_, MATERIAL_SYMBOLS_PERSON, pal);

    th_title_label_ = lv_label_create(th_row1);
    lv_label_set_text(th_title_label_, Tr("识别相似度", "Face Match"));
    lv_obj_set_style_text_font(th_title_label_, GetTextFont(), 0);
    lv_obj_set_style_text_color(th_title_label_, pal.text_pri, 0);
    lv_obj_remove_flag(th_title_label_, LV_OBJ_FLAG_CLICKABLE);

    threshold_val_label_ = lv_label_create(th_row1);
    char th_buf[16];
    snprintf(th_buf, sizeof(th_buf), "%d%%", th_percent);
    lv_label_set_text(threshold_val_label_, th_buf);
    lv_obj_set_style_text_font(threshold_val_label_, GetTextFont(), 0);
    lv_obj_set_style_text_color(threshold_val_label_, pal.accent, 0);
    lv_obj_set_flex_grow(threshold_val_label_, 1);
    lv_obj_set_style_text_align(threshold_val_label_, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_remove_flag(threshold_val_label_, LV_OBJ_FLAG_CLICKABLE);

    th_sub_label_ = lv_label_create(th_card_);
    lv_label_set_text(th_sub_label_, Tr("越高越严格", "Higher is stricter"));
    lv_obj_set_style_text_font(th_sub_label_, GetSecondaryFont(), 0);
    lv_obj_set_style_text_color(th_sub_label_, pal.text_sec, 0);
    lv_obj_remove_flag(th_sub_label_, LV_OBJ_FLAG_CLICKABLE);

    threshold_slider_ = CreateSlider(th_card_, pal, kCardW - 28, 40, 95, th_percent);
    lv_obj_add_event_cb(threshold_slider_, ThresholdSliderCallback, LV_EVENT_VALUE_CHANGED, this);

    // --- Card 2: greeting cooldown stepper ----------------------------------
    int_card_ = CreateCard(content_card_, kCardW, 136, pal.card, pal.card_border);
    lv_obj_set_flex_flow(int_card_, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(int_card_, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t* int_row1 = CreateValueRow(int_card_, MATERIAL_SYMBOLS_SCHEDULE, pal);

    int_title_label_ = lv_label_create(int_row1);
    lv_label_set_text(int_title_label_, Tr("问候冷却", "Cooldown"));
    lv_obj_set_style_text_font(int_title_label_, GetTextFont(), 0);
    lv_obj_set_style_text_color(int_title_label_, pal.text_pri, 0);
    lv_obj_remove_flag(int_title_label_, LV_OBJ_FLAG_CLICKABLE);

    interval_label_ = lv_label_create(int_row1);
    char int_buf[32];
    snprintf(int_buf, sizeof(int_buf), "%d%s", pending_interval_, Tr("秒", "s"));
    lv_label_set_text(interval_label_, int_buf);
    lv_obj_set_style_text_font(interval_label_, GetTextFont(), 0);
    lv_obj_set_style_text_color(interval_label_, pal.accent, 0);
    lv_obj_set_flex_grow(interval_label_, 1);
    lv_obj_set_style_text_align(interval_label_, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_remove_flag(interval_label_, LV_OBJ_FLAG_CLICKABLE);

    int_sub_label_ = lv_label_create(int_card_);
    lv_label_set_text(int_sub_label_, Tr("问候后的冷却时间", "Wait between greetings"));
    lv_obj_set_style_text_font(int_sub_label_, GetSecondaryFont(), 0);
    lv_obj_set_style_text_color(int_sub_label_, pal.text_sec, 0);
    lv_obj_remove_flag(int_sub_label_, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* int_row2 = lv_obj_create(int_card_);
    lv_obj_set_size(int_row2, LV_PCT(100), 36);
    lv_obj_set_style_bg_opa(int_row2, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(int_row2, 0, 0);
    lv_obj_set_style_pad_all(int_row2, 0, 0);
    lv_obj_set_style_pad_column(int_row2, 12, 0);
    ConfigureListChild(int_row2);
    lv_obj_set_flex_flow(int_row2, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(int_row2, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Number-only labels: the unit already lives on the value label above, so
    // the buttons stay short enough for the Chinese glyph set.
    struct StepperSpec { lv_obj_t** btn; lv_obj_t** lbl; int delta; };
    const StepperSpec steppers[2] = {
        { &btn_minus_, &lbl_minus_, -1 },
        { &btn_plus_,  &lbl_plus_,   1 },
    };
    for (const auto& spec : steppers) {
        lv_obj_t* btn = lv_button_create(int_row2);
        lv_obj_set_size(btn, 118, 36);
        lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(btn, pal.btn_bg, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, pal.btn_border, 0);
        lv_obj_set_style_pad_all(btn, 0, 0);
        ConfigureListChild(btn);
        ApplyCardPressFeedback(btn, pal);

        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text(lbl, spec.delta < 0 ? "- 1" : "+ 1");
        lv_obj_set_style_text_font(lbl, GetTextFont(), 0);
        lv_obj_set_style_text_color(lbl, pal.text_pri, 0);
        lv_obj_center(lbl);
        lv_obj_remove_flag(lbl, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_set_user_data(btn, (void*)(intptr_t)spec.delta);
        lv_obj_add_event_cb(btn, IntervalButtonCallback, LV_EVENT_CLICKED, this);
        *spec.btn = btn;
        *spec.lbl = lbl;
    }

    // --- Card 3: local face database ---------------------------------------
    face_count_card_ = CreateCard(content_card_, kCardW, 96, pal.card, pal.card_border);
    lv_obj_set_flex_flow(face_count_card_, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(face_count_card_, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t* fc_row1 = CreateValueRow(face_count_card_, MATERIAL_SYMBOLS_MEMORY, pal);

    face_count_title_label_ = lv_label_create(fc_row1);
    lv_label_set_text(face_count_title_label_, Tr("已录入", "Enrolled"));
    lv_obj_set_style_text_font(face_count_title_label_, GetTextFont(), 0);
    lv_obj_set_style_text_color(face_count_title_label_, pal.text_pri, 0);
    lv_obj_remove_flag(face_count_title_label_, LV_OBJ_FLAG_CLICKABLE);

    face_count_label_ = lv_label_create(fc_row1);
    int count = FaceDatabase::GetInstance().GetFaceCount();
    char db_buf[16];
    snprintf(db_buf, sizeof(db_buf), "%d%s", count, Tr(" 人", ""));
    lv_label_set_text(face_count_label_, db_buf);
    lv_obj_set_style_text_font(face_count_label_, GetTextFont(), 0);
    lv_obj_set_style_text_color(face_count_label_, pal.accent, 0);
    lv_obj_set_flex_grow(face_count_label_, 1);
    lv_obj_set_style_text_align(face_count_label_, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_remove_flag(face_count_label_, LV_OBJ_FLAG_CLICKABLE);

    face_count_sub_label_ = lv_label_create(face_count_card_);
    lv_label_set_text(face_count_sub_label_, Tr("本地已录人数", "People in local database"));
    lv_obj_set_style_text_font(face_count_sub_label_, GetSecondaryFont(), 0);
    lv_obj_set_style_text_color(face_count_sub_label_, pal.text_sec, 0);
    lv_obj_remove_flag(face_count_sub_label_, LV_OBJ_FLAG_CLICKABLE);

    bool has_face_model = (camera_ != nullptr && camera_->IsFaceModelAvailable());
    if (!has_face_model) {
        lv_obj_set_style_opa(th_card_, LV_OPA_40, 0);
        lv_obj_remove_flag(threshold_slider_, LV_OBJ_FLAG_CLICKABLE);
        lv_label_set_text(th_sub_label_, Tr("需烧录人脸固件", "Requires face firmware"));

        lv_obj_set_style_opa(face_count_card_, LV_OPA_40, 0);
        lv_label_set_text(face_count_sub_label_, Tr("需烧录人脸固件", "Requires face firmware"));
    }

    // --- Card 4: close ------------------------------------------------------
    CreateCloseButton(content_card_, pal);
}

void WatcherSettingsUi::UpdateBrightnessUi(int brightness) {
    if (brightness_label_ != nullptr) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d%%", brightness);
        lv_label_set_text(brightness_label_, buf);
    }
    if (brightness_slider_ != nullptr) {
        lv_slider_set_value(brightness_slider_, brightness, LV_ANIM_OFF);
    }
}

void WatcherSettingsUi::UpdateVisionModeUi(int mode) {
    auto pal = GetPalette();

    for (int i = 0; i < 4; i++) {
        if (mode_btns_[i] == nullptr) continue;
        const bool selected = (i == mode);

        lv_obj_set_style_bg_color(mode_btns_[i], selected ? pal.card_active : pal.card, 0);
        lv_obj_set_style_border_color(mode_btns_[i],
                                      selected ? pal.card_active_border : pal.card_border, 0);
        lv_obj_set_style_border_width(mode_btns_[i], selected ? 2 : 1, 0);

        if (mode_labels_[i] != nullptr) {
            lv_obj_set_style_text_color(mode_labels_[i],
                                        selected ? pal.text_active : pal.text_pri, 0);
        }
        if (mode_sub_labels_[i] != nullptr) {
            lv_obj_set_style_text_color(mode_sub_labels_[i],
                                        selected ? pal.text_active_sec : pal.text_sec, 0);
        }
        if (mode_icons_[i] != nullptr) {
            // Selected icons invert into a solid accent capsule; unselected
            // ones sit in a neutral capsule so the row scans as one shape.
            lv_obj_set_style_bg_color(mode_icons_[i],
                                      selected ? pal.accent : pal.icon_bg, 0);
            lv_obj_set_style_text_color(mode_icons_[i],
                                        selected ? pal.on_accent : pal.text_sec, 0);
        }
        if (mode_checks_[i] != nullptr) {
            lv_obj_set_style_text_color(mode_checks_[i],
                                        selected ? pal.accent : pal.text_sec, 0);
            lv_obj_set_style_opa(mode_checks_[i],
                                 selected ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
        }
    }
}

void WatcherSettingsUi::UpdateThresholdUi(float threshold) {
    int percent = static_cast<int>(std::round(threshold * 100.0f));
    if (threshold_val_label_ != nullptr) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d%%", percent);
        lv_label_set_text(threshold_val_label_, buf);
    }
    if (threshold_slider_ != nullptr) {
        lv_slider_set_value(threshold_slider_, percent, LV_ANIM_OFF);
    }
}

void WatcherSettingsUi::OnKnobRotate(bool clockwise) {
    DisplayLockGuard lock(display_);
    if (!IsOpen()) {
        return;
    }

    if (current_page_ == WatcherSettingsPage::QuickSettings) {
        {
            std::lock_guard<std::recursive_mutex> guard(mutex_);
            pending_brightness_ += (clockwise ? 5 : -5);
            if (pending_brightness_ > 100) pending_brightness_ = 100;
            if (pending_brightness_ < 10) pending_brightness_ = 10;
            dirty_brightness_ = true;
        }
        UpdateBrightnessUi(pending_brightness_);
        if (backlight_ != nullptr) {
            backlight_->SetBrightness(pending_brightness_, false);
        }
        ScheduleNvsCommit();
    } else if (current_page_ == WatcherSettingsPage::VisionMode) {
        bool has_face_model = (camera_ != nullptr && camera_->IsFaceModelAvailable());
        {
            std::lock_guard<std::recursive_mutex> guard(mutex_);
            if (has_face_model) {
                pending_vision_mode_ = (pending_vision_mode_ + (clockwise ? 1 : 3)) % 4;
            } else {
                // If face model is missing, toggle only between Off (0) and Human Motion (1)
                pending_vision_mode_ = (pending_vision_mode_ == 0) ? 1 : 0;
            }
            dirty_vision_mode_ = true;
        }
        // Highlight immediately, apply once the user stops turning: a fast
        // spin through all four detents used to issue four NVS writes plus
        // four recognition-engine toggles.
        UpdateVisionModeUi(pending_vision_mode_);
        ScheduleNvsCommit();
        if (mode_btns_[pending_vision_mode_] != nullptr && content_card_ != nullptr) {
            lv_obj_scroll_to_view(mode_btns_[pending_vision_mode_], LV_ANIM_ON);
        }
    } else if (current_page_ == WatcherSettingsPage::Parameters) {
        bool has_face_model = (camera_ != nullptr && camera_->IsFaceModelAvailable());
        if (!has_face_model) {
            return;
        }
        {
            std::lock_guard<std::recursive_mutex> guard(mutex_);
            pending_threshold_ += (clockwise ? 0.02f : -0.02f);
            if (pending_threshold_ > 0.95f) pending_threshold_ = 0.95f;
            if (pending_threshold_ < 0.40f) pending_threshold_ = 0.40f;
            dirty_threshold_ = true;
        }
        UpdateThresholdUi(pending_threshold_);
        FaceRecognition::GetInstance().SetMatchThreshold(pending_threshold_);
        ScheduleNvsCommit();
    }
}

void WatcherSettingsUi::OnKnobClick() {
    Close(true);
}

bool WatcherSettingsUi::HandleTouchGesture(lv_dir_t dir) {
    if (drawer_state_ != DrawerState::Open) return false;
    if (current_page_ == WatcherSettingsPage::QuickSettings) {
        if (dir == LV_DIR_TOP) {
            Close(true);
            return true;
        }
    } else if (current_page_ == WatcherSettingsPage::VisionMode) {
        if (dir == LV_DIR_RIGHT) {
            Close(true);
            return true;
        }
    } else if (current_page_ == WatcherSettingsPage::Parameters) {
        if (dir == LV_DIR_LEFT) {
            Close(true);
            return true;
        }
    }
    return false;
}

void WatcherSettingsUi::DismissHandleCallback(lv_event_t* e) {
    auto* self = static_cast<WatcherSettingsUi*>(lv_event_get_user_data(e));
    if (self == nullptr) return;

    if (self->drawer_state_ != DrawerState::Open) {
        return;
    }

    const auto code = lv_event_get_code(e);
    auto* indev = lv_indev_active();
    if (!indev) return;

    if (self->activity_cb_) {
        self->activity_cb_();
    }

    // 1. Direct gesture event dispatched by LVGL
    if (code == LV_EVENT_GESTURE) {
        lv_dir_t gesture_dir = lv_indev_get_gesture_dir(indev);
        if (self->HandleTouchGesture(gesture_dir)) {
            lv_indev_wait_release(indev);
            return;
        }
    }

    if (code == LV_EVENT_PRESS_LOST || code == LV_EVENT_INDEV_RESET) {
        self->dismiss_tracking_ = false;
        return;
    }

    // When a scroll takes over the press LVGL does NOT send PRESS_LOST to the
    // act object (lv_indev_scroll.c), so a drag that is scrolling a list would
    // otherwise keep its tracking flag armed and could also be read as a
    // dismiss on the same release.
    if (code == LV_EVENT_SCROLL_BEGIN) {
        self->dismiss_tracking_ = false;
        return;
    }

    if (code == LV_EVENT_PRESSED) {
        // Tracking is armed for slider presses too -- the rail is a legitimate
        // place to start an outward swipe. MatchesDismissGesture() rejects the
        // ones that are actually value drags.
        self->dismiss_press_target_ = static_cast<lv_obj_t*>(lv_event_get_target(e));
        lv_indev_get_point(indev, &self->dismiss_start_pos_);
        self->dismiss_tracking_ = true;
        return;
    }

    // Swipe-to-dismiss is evaluated WHILE THE FINGER IS DOWN.
    //
    // lv_indev_wait_release() is only honoured at the top of the next input
    // read (lv_indev.c), and the release path then emits LV_EVENT_CLICKED
    // unconditionally. Calling wait_release from the RELEASED handler is
    // therefore too late to suppress that click: an upward swipe that starts
    // on the theme row would close the drawer AND flip the theme, and one that
    // starts on the Wi-Fi button would clear credentials and reboot the
    // device. Firing during PRESSING arms the suppression before the release
    // is processed, which is exactly how LVGL's own gesture path stays safe.
    if (code == LV_EVENT_PRESSING) {
        if (!self->dismiss_tracking_) return;
        lv_point_t pos;
        lv_indev_get_point(indev, &pos);
        const int dx = pos.x - self->dismiss_start_pos_.x;
        const int dy = pos.y - self->dismiss_start_pos_.y;
        if (self->MatchesDismissGesture(dx, dy, false)) {
            self->dismiss_tracking_ = false;
            lv_indev_wait_release(indev);
            self->Close(true);
        }
        return;
    }

    // A tap cannot be recognised mid-drag, so it is still resolved on release.
    // It is safe there: tap-to-dismiss only arms when the press started on the
    // drawer chrome, which has no actionable child under it.
    if (code == LV_EVENT_RELEASED && self->dismiss_tracking_) {
        self->dismiss_tracking_ = false;
        lv_point_t end_pos;
        lv_indev_get_point(indev, &end_pos);
        const int dx = end_pos.x - self->dismiss_start_pos_.x;
        const int dy = end_pos.y - self->dismiss_start_pos_.y;
        if (self->MatchesDismissGesture(dx, dy, true)) {
            lv_indev_wait_release(indev);
            self->Close(true);
        }
    }
}

// `allow_tap` is set only for the release-time check: a tap carries no motion,
// so it can never be classified during PRESSING. Axis domination is 1.5
// everywhere so a diagonal drag that scrolls a list is not read as a dismiss.
bool WatcherSettingsUi::MatchesDismissGesture(int dx, int dy, bool allow_tap) const {
    const int start_x = dismiss_start_pos_.x;
    const int start_y = dismiss_start_pos_.y;

    // The rails are horizontal controls, so on a slider only a clearly
    // perpendicular drag may dismiss. This keeps a deliberate slider drag from
    // closing the drawer while still letting an outward swipe that happens to
    // start on the rail work, instead of turning that strip into a dead zone.
    const bool on_slider =
        (dismiss_press_target_ == brightness_slider_) ||
        (dismiss_press_target_ == threshold_slider_);
    if (on_slider && std::abs(dy) <= std::abs(dx) * 1.5) return false;

    // Keyed off the PRESS TARGET, not a y threshold: the content list starts at
    // y=60, so a y-based rule would also fire for a tap on the top edge of the
    // first card and close the drawer while selecting an option.
    const bool tap_on_chrome =
        (dismiss_press_target_ == root_) || (dismiss_press_target_ == dismiss_handle_);
    if (allow_tap && tap_on_chrome && std::abs(dx) < 20 && std::abs(dy) < 20) {
        return true;
    }

    switch (current_page_) {
        case WatcherSettingsPage::QuickSettings:
            // Entered by pulling down, so dismissal is an upward swipe. The
            // page never scrolls, so a vertical swipe is unambiguous.
            if (std::abs(dy) >= 45 && std::abs(dy) > std::abs(dx) * 1.5) return true;
            // Bezel hotzones: the top handle and the bottom curve of the
            // circle, where a full-width swipe is awkward.
            if ((start_y < 85 || start_y > 330) && dy <= -30 && std::abs(dy) > std::abs(dx)) {
                return true;
            }
            break;
        case WatcherSettingsPage::VisionMode:
            // Entered by pulling left, so dismissal is a rightward swipe.
            if (std::abs(dx) >= 52 && std::abs(dx) > std::abs(dy) * 1.5) return true;
            // Left bezel: the classic smartwatch edge-back gesture.
            if (start_x < 85 && dx >= 35 && std::abs(dx) > std::abs(dy)) return true;
            break;
        case WatcherSettingsPage::Parameters:
            // Entered by pulling right, so dismissal is a leftward swipe.
            if (std::abs(dx) >= 52 && std::abs(dx) > std::abs(dy) * 1.5) return true;
            // Right bezel edge-back.
            if (start_x > 327 && dx <= -35 && std::abs(dx) > std::abs(dy)) return true;
            break;
    }
    return false;
}

void WatcherSettingsUi::BrightnessSliderCallback(lv_event_t* e) {
    auto* self = static_cast<WatcherSettingsUi*>(lv_event_get_user_data(e));
    lv_obj_t* slider = static_cast<lv_obj_t*>(lv_event_get_target(e));
    int val = lv_slider_get_value(slider);
    {
        std::lock_guard<std::recursive_mutex> guard(self->mutex_);
        self->pending_brightness_ = val;
        self->dirty_brightness_ = true;
    }
    self->UpdateBrightnessUi(val);
    if (self->backlight_ != nullptr) {
        self->backlight_->SetBrightness(val, false);
    }
    self->ScheduleNvsCommit();
}

void WatcherSettingsUi::ThemeButtonCallback(lv_event_t* e) {
    auto* self = static_cast<WatcherSettingsUi*>(lv_event_get_user_data(e));
    auto& theme_mgr = LvglThemeManager::GetInstance();
    if (self->drawer_state_ != DrawerState::Open) return;
    bool is_dark = self->display_->GetTheme() && self->display_->GetTheme()->name() == "dark";
    auto* next_theme = theme_mgr.GetTheme(is_dark ? "light" : "dark");
    if (next_theme != nullptr) {
        self->display_->SetTheme(next_theme);
        self->ApplyThemeToRoot();
        self->ClearWidgetPointers();
        lv_obj_clean(self->content_card_);
        if (self->current_page_ == WatcherSettingsPage::QuickSettings) {
            self->BuildQuickSettingsPage();
        } else if (self->current_page_ == WatcherSettingsPage::VisionMode) {
            self->BuildVisionModePage();
        } else if (self->current_page_ == WatcherSettingsPage::Parameters) {
            self->BuildParametersPage();
        }
        self->StyleHeader();
    }
}

void WatcherSettingsUi::ModeButtonCallback(lv_event_t* e) {
    auto* self = static_cast<WatcherSettingsUi*>(lv_event_get_user_data(e));
    lv_obj_t* btn = static_cast<lv_obj_t*>(lv_event_get_target(e));
    bool has_face_model = (self->camera_ != nullptr && self->camera_->IsFaceModelAvailable());
    for (int i = 0; i < 4; i++) {
        if (self->drawer_state_ != DrawerState::Open) return;
        if (self->mode_btns_[i] == btn) {
            if (!has_face_model && (i == 2 || i == 3)) {
                return;
            }
            {
                std::lock_guard<std::recursive_mutex> guard(self->mutex_);
                self->pending_vision_mode_ = i;
                self->dirty_vision_mode_ = true;
            }
            self->UpdateVisionModeUi(i);
            self->ScheduleNvsCommit();
            break;
        }
    }
}

void WatcherSettingsUi::ThresholdSliderCallback(lv_event_t* e) {
    auto* self = static_cast<WatcherSettingsUi*>(lv_event_get_user_data(e));
    lv_obj_t* slider = static_cast<lv_obj_t*>(lv_event_get_target(e));
    int val = lv_slider_get_value(slider);
    {
        std::lock_guard<std::recursive_mutex> guard(self->mutex_);
        self->pending_threshold_ = val / 100.0f;
        self->dirty_threshold_ = true;
    }
    self->UpdateThresholdUi(self->pending_threshold_);
    FaceRecognition::GetInstance().SetMatchThreshold(self->pending_threshold_);
    self->ScheduleNvsCommit();
}

void WatcherSettingsUi::IntervalButtonCallback(lv_event_t* e) {
    auto* self = static_cast<WatcherSettingsUi*>(lv_event_get_user_data(e));
    lv_obj_t* btn = static_cast<lv_obj_t*>(lv_event_get_target(e));
    int delta = (int)(intptr_t)lv_obj_get_user_data(btn);
    if (self->drawer_state_ != DrawerState::Open) return;
    {
        std::lock_guard<std::recursive_mutex> guard(self->mutex_);
        self->pending_interval_ += delta;
        if (self->pending_interval_ < 1) self->pending_interval_ = 1;
        if (self->pending_interval_ > 15) self->pending_interval_ = 15;
        self->dirty_interval_ = true;
    }

    if (self->interval_label_ != nullptr) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%d%s", self->pending_interval_, self->Tr("秒", "s"));
        lv_label_set_text(self->interval_label_, buf);
    }
    FaceRecognition::GetInstance().SetCooldownInterval(self->pending_interval_);
    self->ScheduleNvsCommit();
}

void WatcherSettingsUi::CloseButtonCallback(lv_event_t* e) {
    auto* self = static_cast<WatcherSettingsUi*>(lv_event_get_user_data(e));
    self->Close(true);
}
