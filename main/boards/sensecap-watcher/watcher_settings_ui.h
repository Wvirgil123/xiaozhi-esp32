#ifndef WATCHER_SETTINGS_UI_H
#define WATCHER_SETTINGS_UI_H

#include "display/lcd_display.h"
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <esp_timer.h>

class Backlight;
class SscmaCamera;

enum class WatcherSettingsPage {
    QuickSettings,  // Swipe Down: Connected Wi-Fi SSID, Switch Wi-Fi button, Brightness, Theme
    VisionMode,     // Swipe Left: 4 Vision & Perception modes (Smart Greeting, Human Detect, DND, Off)
    Parameters,     // Swipe Right: Face threshold slider, Cooldown interval, DB info
};

enum class WatcherUiLanguage {
    Zh = 0,
    En = 1
};

enum class DrawerState {
    Closed,
    Opening,
    Open,
    Closing
};

// SenseCraft AMOLED palette. Layers, not shadows: depth is expressed with
// surface steps (bg -> card -> card_elev), hairline borders and a single
// brand accent. Box shadows are deliberately avoided: the drawer scrolls
// full-screen on an ESP32-S3 and shadow blur cost lands on every frame.
struct WatcherUiPalette {
    lv_color_t bg;                  // AMOLED true black / pure white
    lv_color_t card;                // resting card surface
    lv_color_t card_elev;           // raised surface for slider cards
    lv_color_t card_border;         // hairline outline
    lv_color_t card_press;          // pressed surface
    lv_color_t card_active;         // selected surface (vision modes)
    lv_color_t card_active_border;  // selected 2px accent outline
    lv_color_t btn_bg;              // pill / icon button surface
    lv_color_t btn_border;
    lv_color_t accent;              // brand mint
    lv_color_t on_accent;           // foreground on a solid accent fill
    lv_color_t text_pri;
    lv_color_t text_sec;
    lv_color_t text_ter;            // grab bar, inactive dots
    lv_color_t text_active;         // primary text on a selected card
    lv_color_t text_active_sec;     // secondary text on a selected card
    lv_color_t icon_bg;             // icon capsule surface
    lv_color_t btn_close_bg;
    lv_color_t btn_close_border;
    lv_color_t track_bg;            // slider rail
    lv_color_t dot_inactive;
};

class WatcherSettingsUi {
public:
    using CloseCallback = std::function<void()>;
    using ActivityCallback = std::function<void()>;

    WatcherSettingsUi(LcdDisplay* display, CloseCallback close_cb);
    ~WatcherSettingsUi();

    void SetBacklight(Backlight* backlight) { backlight_ = backlight; }
    void SetCamera(SscmaCamera* camera) { camera_ = camera; }
    void SetActivityCallback(ActivityCallback cb) { activity_cb_ = std::move(cb); }
    void SetBatteryInfoProvider(std::function<int()> get_battery_percent,
                                std::function<uint16_t()> get_battery_voltage) {
        get_battery_percent_ = std::move(get_battery_percent);
        get_battery_voltage_ = std::move(get_battery_voltage);
    }

    void Open(WatcherSettingsPage page);
    void Close(bool animated = true);
    void FinishClose();
    bool IsOpen() const;
    WatcherSettingsPage GetCurrentPage() const;

    // Language support
    WatcherUiLanguage GetLanguage() const { return language_; }
    const char* Tr(const char* zh, const char* en) const {
        return language_ == WatcherUiLanguage::Zh ? zh : en;
    }
    void RefreshLocalizedText();

    // Knob handlers
    void OnKnobRotate(bool clockwise);
    void OnKnobClick();

    // Process touch gesture: returns true if handled
    bool HandleTouchGesture(lv_dir_t dir);

    // Switch WiFi button action
    void OnSwitchWifiClicked();

    WatcherUiPalette GetPalette() const;
    void ApplyThemeToRoot();
    const lv_font_t* GetTextFont() const;
    const lv_font_t* GetSecondaryFont() const;

private:
    void BuildQuickSettingsPage();
    void BuildVisionModePage();
    void BuildParametersPage();
    void ClearWidgetPointers();

    // Shared construction helpers
    void BuildHeader();
    void StyleHeader();
    void BuildContentList(bool scrollable);
    lv_obj_t* CreateIconCapsule(lv_obj_t* parent, const char* symbol,
                                const WatcherUiPalette& pal);
    lv_obj_t* CreateCloseButton(lv_obj_t* parent, const WatcherUiPalette& pal);
    lv_obj_t* CreateCard(lv_obj_t* parent, int width, int height,
                         lv_color_t bg, lv_color_t border);
    lv_obj_t* CreateValueRow(lv_obj_t* parent, const char* symbol,
                             const WatcherUiPalette& pal);
    lv_obj_t* CreateSlider(lv_obj_t* parent, const WatcherUiPalette& pal,
                           int width, int min, int max, int value);
    void ApplyCardPressFeedback(lv_obj_t* obj, const WatcherUiPalette& pal);
    bool MatchesDismissGesture(int dx, int dy, bool allow_tap) const;

    void UpdateBrightnessUi(int brightness);
    void UpdateVisionModeUi(int mode);
    void UpdateThresholdUi(float threshold);

    void ScheduleNvsCommit();
    static void NvsCommitTimerCallback(void* arg);
    void CommitPendingSettings();
    void FlushPendingChanges();

    mutable std::recursive_mutex mutex_;
    LcdDisplay* display_;
    CloseCallback close_cb_;
    ActivityCallback activity_cb_;
    Backlight* backlight_ = nullptr;
    SscmaCamera* camera_ = nullptr;
    std::function<int()> get_battery_percent_;
    std::function<uint16_t()> get_battery_voltage_;

    DrawerState drawer_state_ = DrawerState::Closed;

    lv_obj_t* root_ = nullptr;
    lv_obj_t* dismiss_handle_ = nullptr;
    lv_obj_t* dismiss_bar_ = nullptr;
    lv_obj_t* page_dots_[3] = {nullptr, nullptr, nullptr};
    lv_obj_t* content_card_ = nullptr;
    lv_obj_t* title_label_ = nullptr;
    lv_obj_t* close_btn_ = nullptr;
    lv_obj_t* close_icon_ = nullptr;
    lv_obj_t* close_lbl_ = nullptr;

    // Quick Settings elements
    lv_obj_t* wifi_card_ = nullptr;
    lv_obj_t* wifi_icon_label_ = nullptr;
    lv_obj_t* wifi_ssid_label_ = nullptr;
    lv_obj_t* wifi_reconfig_btn_ = nullptr;
    lv_obj_t* wifi_reconfig_icon_ = nullptr;

    lv_obj_t* bri_box_ = nullptr;
    lv_obj_t* bri_title_label_ = nullptr;
    lv_obj_t* brightness_slider_ = nullptr;
    lv_obj_t* brightness_label_ = nullptr;

    lv_obj_t* theme_btn_ = nullptr;
    lv_obj_t* theme_title_label_ = nullptr;
    lv_obj_t* theme_val_label_ = nullptr;

    // Vision Mode elements
    lv_obj_t* mode_btns_[4] = {nullptr, nullptr, nullptr, nullptr};
    lv_obj_t* mode_labels_[4] = {nullptr, nullptr, nullptr, nullptr};
    lv_obj_t* mode_sub_labels_[4] = {nullptr, nullptr, nullptr, nullptr};
    lv_obj_t* mode_icons_[4] = {nullptr, nullptr, nullptr, nullptr};
    lv_obj_t* mode_checks_[4] = {nullptr, nullptr, nullptr, nullptr};

    // Parameters elements
    lv_obj_t* th_card_ = nullptr;
    lv_obj_t* th_title_label_ = nullptr;
    lv_obj_t* th_sub_label_ = nullptr;
    lv_obj_t* threshold_slider_ = nullptr;
    lv_obj_t* threshold_val_label_ = nullptr;

    lv_obj_t* int_card_ = nullptr;
    lv_obj_t* int_title_label_ = nullptr;
    lv_obj_t* int_sub_label_ = nullptr;
    lv_obj_t* interval_label_ = nullptr;
    lv_obj_t* btn_minus_ = nullptr;
    lv_obj_t* lbl_minus_ = nullptr;
    lv_obj_t* btn_plus_ = nullptr;
    lv_obj_t* lbl_plus_ = nullptr;

    lv_obj_t* face_count_card_ = nullptr;
    lv_obj_t* face_count_icon_ = nullptr;
    lv_obj_t* face_count_title_label_ = nullptr;
    lv_obj_t* face_count_sub_label_ = nullptr;
    lv_obj_t* face_count_label_ = nullptr;

    WatcherSettingsPage current_page_ = WatcherSettingsPage::QuickSettings;
    WatcherUiLanguage language_ = WatcherUiLanguage::Zh;
    int pending_brightness_ = 80;
    int pending_vision_mode_ = 0;
    float pending_threshold_ = 0.60f;
    int pending_interval_ = 5;

    bool dirty_brightness_ = false;
    bool dirty_threshold_ = false;
    bool dirty_interval_ = false;
    bool dirty_vision_mode_ = false;

    esp_timer_handle_t nvs_timer_ = nullptr;

    // Touch handle tracking for outward dismiss
    lv_point_t dismiss_start_pos_ = {0, 0};
    lv_obj_t* dismiss_press_target_ = nullptr;
    bool dismiss_tracking_ = false;

    static void DismissHandleCallback(lv_event_t* e);
    static void WifiButtonCallback(lv_event_t* e);
    static void BrightnessSliderCallback(lv_event_t* e);
    static void ThemeButtonCallback(lv_event_t* e);
    static void ModeButtonCallback(lv_event_t* e);
    static void ThresholdSliderCallback(lv_event_t* e);
    static void IntervalButtonCallback(lv_event_t* e);
    static void CloseButtonCallback(lv_event_t* e);
};

#endif // WATCHER_SETTINGS_UI_H
