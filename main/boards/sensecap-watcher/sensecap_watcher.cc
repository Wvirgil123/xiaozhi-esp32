#include "display/lv_display.h"
#include "misc/lv_event.h"
#include "wifi_board.h"
#include "sensecap_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "knob.h"
#include "config.h"
#include "led/single_led.h"
#include "power_save_timer.h"
#include "sscma_camera.h"
#include "face_recognition.h"
#include <material_symbols.h>
#include "lvgl_theme.h"
#include "remote_display.h"
#include "remote_display_http_server.h"
#include "face_serial_handler.h"
#include "mcp_server.h"
#include "settings.h"
#include "watcher_settings_ui.h"
#include <wifi_manager.h>
#include <ssid_manager.h>

#include <esp_log.h>
#include <esp_check.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_spd2010.h>
#include <esp_lcd_touch_spd2010.h>
#include <esp_lvgl_port_touch.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <iot_button.h>
#include <iot_knob.h>
#include <esp_io_expander_tca95xx_16bit.h>
#include <esp_sleep.h>
#include <esp_console.h>
#include <esp_mac.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_app_desc.h>

#include "assets/lang_config.h"

#define TAG "sensecap_watcher"

class CustomLcdDisplay : public SpiLcdDisplay {
    public:
        CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle,
                        esp_lcd_panel_handle_t panel_handle,
                        int width,
                        int height,
                        int offset_x,
                        int offset_y,
                        bool mirror_x,
                        bool mirror_y,
                        bool swap_xy)
            : SpiLcdDisplay(io_handle, panel_handle, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
            // Note: UI customization should be done in SetupUI(), not in constructor
            // to ensure lvgl objects are created before accessing them
        }

        virtual void SetupUI() override {
            // Call parent SetupUI() first to create all lvgl objects
            SpiLcdDisplay::SetupUI();

            DisplayLockGuard lock(this);
            auto lvgl_theme = static_cast<LvglTheme*>(current_theme_);
            auto text_font = lvgl_theme->text_font()->font();
            auto icon_font = lvgl_theme->icon_font()->font();

            lv_obj_set_size(top_bar_, LV_HOR_RES, text_font->line_height);
            lv_obj_set_style_layout(top_bar_, LV_LAYOUT_NONE, 0);
            lv_obj_set_style_pad_top(top_bar_, 10, 0);
            lv_obj_set_style_pad_bottom(top_bar_, 1, 0);

            lv_obj_set_size(status_bar_, LV_HOR_RES, text_font->line_height);
            lv_obj_set_style_layout(status_bar_, LV_LAYOUT_NONE, 0);
            lv_obj_set_style_pad_top(status_bar_, 10, 0);
            lv_obj_set_style_pad_bottom(status_bar_, 1, 0);
            lv_obj_set_y(status_bar_, text_font->line_height);
            lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_IGNORE_LAYOUT);

            // Reparent mute and battery labels to top_bar_ to allow absolute positioning
            lv_obj_set_parent(mute_label_, top_bar_);
            lv_obj_set_parent(battery_label_, top_bar_);
            lv_obj_set_style_margin_left(battery_label_, 0, 0);

            // Vision mode indicator (read-only): face recognition vs object detection.
            mode_label_ = lv_label_create(top_bar_);
            lv_label_set_text(mode_label_, "");
            lv_obj_set_style_text_font(mode_label_, icon_font, 0);
            lv_obj_set_style_text_color(mode_label_, lvgl_theme->text_color(), 0);

            // 针对圆形屏幕调整位置：四个图标沿顶部居中均匀铺开（间距 1.5 行高），
            // 避免右侧图标互相挤压。mute 常隐藏，留的槽位空着不影响观感。
            //    network   mute   mode   battery    //
            //                 status                //
            lv_obj_align(network_label_, LV_ALIGN_TOP_MID, -2.25 * icon_font->line_height, 0);
            lv_obj_align(mute_label_, LV_ALIGN_TOP_MID, -0.75 * icon_font->line_height, 0);
            lv_obj_align(mode_label_, LV_ALIGN_TOP_MID, 0.75 * icon_font->line_height, 0);
            lv_obj_align(battery_label_, LV_ALIGN_TOP_MID, 2.25 * icon_font->line_height, 0);

            lv_obj_align(status_label_, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_flex_grow(status_label_, 0);
            lv_obj_set_width(status_label_, LV_HOR_RES * 0.75);
            lv_label_set_long_mode(status_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);

            lv_obj_align(notification_label_, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_width(notification_label_, LV_HOR_RES * 0.75);
            lv_label_set_long_mode(notification_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);

            lv_obj_align(low_battery_popup_, LV_ALIGN_BOTTOM_MID, 0, -20);
            lv_obj_set_style_bg_color(low_battery_popup_, lv_color_hex(0xFF0000), 0);
            lv_obj_set_width(low_battery_label_, LV_HOR_RES * 0.75);
            lv_label_set_long_mode(low_battery_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);

            // 针对圆形屏幕调整底部对话框位置，避免被圆角遮挡
            lv_obj_set_style_pad_bottom(bottom_bar_, 30, 0);
            lv_obj_set_width(chat_message_label_, LV_HOR_RES * 0.75); // 限制宽度，避免文字贴边

            // 监听主屏手势与滑动作操作，调出快捷设置或感知模式
            lv_obj_t* screen = lv_screen_active();
            lv_obj_add_event_cb(screen, [](lv_event_t* e) {
                auto* self = static_cast<CustomLcdDisplay*>(lv_event_get_user_data(e));
                if (!self->gesture_cb_) return;
                lv_event_code_t code = lv_event_get_code(e);
                if (code == LV_EVENT_GESTURE) {
                    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
                    self->screen_touch_tracking_ = false;
                    auto* indev = lv_indev_active();
                    if (indev) lv_indev_wait_release(indev);
                    self->gesture_cb_(dir);
                } else if (code == LV_EVENT_PRESSED) {
                    lv_indev_get_point(lv_indev_active(), &self->screen_touch_start_);
                    self->screen_touch_tracking_ = true;
                } else if (code == LV_EVENT_PRESS_LOST || code == LV_EVENT_INDEV_RESET) {
                    self->screen_touch_tracking_ = false;
                } else if (code == LV_EVENT_RELEASED && self->screen_touch_tracking_) {
                    self->screen_touch_tracking_ = false;
                    lv_point_t end_pos;
                    lv_indev_get_point(lv_indev_active(), &end_pos);
                    int dx = end_pos.x - self->screen_touch_start_.x;
                    int dy = end_pos.y - self->screen_touch_start_.y;
                    if (dy > 45 && std::abs(dx) < 40) {
                        auto* indev = lv_indev_active();
                        if (indev) lv_indev_wait_release(indev);
                        self->gesture_cb_(LV_DIR_BOTTOM);
                    } else if (dx < -50 && std::abs(dy) < 40) {
                        auto* indev = lv_indev_active();
                        if (indev) lv_indev_wait_release(indev);
                        self->gesture_cb_(LV_DIR_LEFT);
                    } else if (dx > 50 && std::abs(dy) < 40) {
                        auto* indev = lv_indev_active();
                        if (indev) lv_indev_wait_release(indev);
                        self->gesture_cb_(LV_DIR_RIGHT);
                    }
                }
            }, LV_EVENT_ALL, this);
        }

        using GestureCallback = std::function<void(lv_dir_t)>;
        void SetGestureCallback(GestureCallback cb) {
            gesture_cb_ = std::move(cb);
        }

        // Refresh the vision mode indicator alongside the standard status bar.
        void UpdateStatusBar(bool update_all = false) override {
            SpiLcdDisplay::UpdateStatusBar(update_all);
            DisplayLockGuard lock(this);
            if (mode_label_ == nullptr) {
                return;
            }
            // 四态：关闭(不唤醒) / 物体检测 / 人脸识别·普通 / 人脸识别·免打扰(DND)
            const char* icon = nullptr;
            auto* cam = static_cast<SscmaCamera*>(Board::GetInstance().GetCamera());
            if (cam != nullptr) {
                switch (cam->GetVisionWakeMode()) {
                    case SscmaCamera::VISION_FACE_DND:
                        icon = MATERIAL_SYMBOLS_VOLUME_OFF;  // 人脸识别·免打扰(熟人 DND)
                        break;
                    case SscmaCamera::VISION_FACE:
                        icon = MATERIAL_SYMBOLS_PERSON;      // 人脸识别·普通(见人打招呼)
                        break;
                    case SscmaCamera::VISION_OBJECT:
                        icon = MATERIAL_SYMBOLS_SEARCH;      // 物体检测
                        break;
                    case SscmaCamera::VISION_OFF:
                    default:
                        icon = MATERIAL_SYMBOLS_CANCEL;      // 完全不唤醒
                        break;
                }
            }
            if (mode_icon_ != icon) {
                mode_icon_ = icon;
                lv_label_set_text(mode_label_, icon ? icon : "");
            }
        }

        // 重写方法以支持远程显示状态同步
        void SetEmotion(const char* emotion) override {
            SpiLcdDisplay::SetEmotion(emotion);
            RemoteDisplay::GetInstance()->SendEmotion(emotion);
        }

        void SetStatus(const char* status) override {
            SpiLcdDisplay::SetStatus(status);
            RemoteDisplay::GetInstance()->SendStatus(status);
        }

        void SetChatMessage(const char* role, const char* content) override {
            SpiLcdDisplay::SetChatMessage(role, content);
            RemoteDisplay::GetInstance()->SendChatMessage(role, content);
        }

        void ClearChatMessages() override {
            SpiLcdDisplay::ClearChatMessages();
            RemoteDisplay::GetInstance()->SendChatMessage("", "");
        }

        void SetTheme(Theme* theme) override {
            SpiLcdDisplay::SetTheme(theme);
            RemoteDisplay::GetInstance()->SendTheme(theme ? theme->name().c_str() : "");
        }

    private:
        lv_obj_t* mode_label_ = nullptr;     // vision mode indicator
        const char* mode_icon_ = nullptr;    // cached icon to avoid redundant redraws
        GestureCallback gesture_cb_;
        lv_point_t screen_touch_start_ = {0, 0};
        bool screen_touch_tracking_ = false;
};

class SensecapWatcher : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    CustomLcdDisplay* display_ = nullptr;
    std::unique_ptr<WatcherSettingsUi> settings_ui_;
    i2c_master_bus_handle_t touch_i2c_bus_ = nullptr;
    esp_lcd_touch_handle_t touch_handle_ = nullptr;
    std::unique_ptr<Knob> knob_;
    esp_io_expander_handle_t io_exp_handle;
    SemaphoreHandle_t io_exp_mutex_ = nullptr;  // I2C 操作互斥锁
    button_handle_t btns;
    PowerSaveTimer* power_save_timer_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    uint32_t long_press_cnt_;
    button_driver_t* btn_driver_ = nullptr;
    static SensecapWatcher* instance_;
    SscmaCamera* camera_ = nullptr;
    RemoteDisplayHttpServer remote_display_http_server_;

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(10);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->OnShutdownRequest([this]() {
            ESP_LOGI(TAG, "Shutting down");
            bool is_charging = (IoExpanderGetLevel(BSP_PWR_VBUS_IN_DET) == 0);
            if (is_charging) {
                ESP_LOGI(TAG, "charging");
                GetBacklight()->SetBrightness(0);
            } else {
                IoExpanderSetLevel(BSP_PWR_SYSTEM, 0);
            }
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = BSP_GENERAL_I2C_SDA,
            .scl_io_num = BSP_GENERAL_I2C_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // pulldown for lcd i2c
        const gpio_config_t io_config = {
            .pin_bit_mask = (1ULL << BSP_TOUCH_I2C_SDA) | (1ULL << BSP_TOUCH_I2C_SCL) | (1ULL << BSP_SPI3_HOST_PCLK) | (1ULL << BSP_SPI3_HOST_DATA0) | (1ULL << BSP_SPI3_HOST_DATA1)
                            | (1ULL << BSP_SPI3_HOST_DATA2) | (1ULL << BSP_SPI3_HOST_DATA3) | (1ULL << BSP_LCD_SPI_CS) | (1UL << DISPLAY_BACKLIGHT_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_config);

        gpio_set_level(BSP_TOUCH_I2C_SDA, 0);
        gpio_set_level(BSP_TOUCH_I2C_SCL, 0);
    
        gpio_set_level(BSP_LCD_SPI_CS, 0);
        gpio_set_level(DISPLAY_BACKLIGHT_PIN, 0);
        gpio_set_level(BSP_SPI3_HOST_PCLK, 0);
        gpio_set_level(BSP_SPI3_HOST_DATA0, 0);
        gpio_set_level(BSP_SPI3_HOST_DATA1, 0);
        gpio_set_level(BSP_SPI3_HOST_DATA2, 0);
        gpio_set_level(BSP_SPI3_HOST_DATA3, 0);

    }

    esp_err_t IoExpanderSetLevel(uint16_t pin_mask, uint8_t level) {
        esp_err_t ret = ESP_ERR_TIMEOUT;
        if (xSemaphoreTake(io_exp_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            ret = esp_io_expander_set_level(io_exp_handle, pin_mask, level);
            xSemaphoreGive(io_exp_mutex_);
        } else {
            ESP_LOGW(TAG, "IoExpanderSetLevel: mutex timeout");
        }
        return ret;
    }

    uint8_t IoExpanderGetLevel(uint16_t pin_mask) {
        uint32_t pin_val = 0;
        if (xSemaphoreTake(io_exp_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            esp_io_expander_get_level(io_exp_handle, DRV_IO_EXP_INPUT_MASK, &pin_val);
            xSemaphoreGive(io_exp_mutex_);
        } else {
            ESP_LOGW(TAG, "IoExpanderGetLevel: mutex timeout");
        }
        pin_mask &= DRV_IO_EXP_INPUT_MASK;
        return (uint8_t)((pin_val & pin_mask) ? 1 : 0);
    }

    void InitializeExpander() {
        esp_err_t ret = ESP_OK;
        esp_io_expander_new_i2c_tca95xx_16bit(i2c_bus_, ESP_IO_EXPANDER_I2C_TCA9555_ADDRESS_001, &io_exp_handle);

        ret |= esp_io_expander_set_dir(io_exp_handle, DRV_IO_EXP_INPUT_MASK, IO_EXPANDER_INPUT);
        ret |= esp_io_expander_set_dir(io_exp_handle, DRV_IO_EXP_OUTPUT_MASK, IO_EXPANDER_OUTPUT);
        ret |= esp_io_expander_set_level(io_exp_handle, DRV_IO_EXP_OUTPUT_MASK, 0);
        ret |= esp_io_expander_set_level(io_exp_handle, BSP_PWR_SYSTEM, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        ret |= esp_io_expander_set_level(io_exp_handle, BSP_PWR_START_UP, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    
        uint32_t pin_val = 0;
        ret |= esp_io_expander_get_level(io_exp_handle, DRV_IO_EXP_INPUT_MASK, &pin_val);
        ESP_LOGI(TAG, "IO expander initialized: %x", DRV_IO_EXP_OUTPUT_MASK | (uint16_t)pin_val);
    
        assert(ret == ESP_OK);
    }

    void OnKnobRotate(bool clockwise) {
        if (display_ == nullptr) {
            return;
        }

        if (settings_ui_ && settings_ui_->IsOpen()) {
            settings_ui_->OnKnobRotate(clockwise);
            power_save_timer_->WakeUp();
            return;
        }

        auto codec = GetAudioCodec();
        int current_volume = codec->output_volume();
        int new_volume = current_volume + (clockwise ? -5 : 5); 

        // 确保音量在有效范围内
        if (new_volume > 100) {
            new_volume = 100;
            ESP_LOGW(TAG, "Volume reached maximum limit: %d", new_volume);
        } else if (new_volume < 0) {
            new_volume = 0;
            ESP_LOGW(TAG, "Volume reached minimum limit: %d", new_volume);
        }

        codec->SetOutputVolume(new_volume);
        ESP_LOGI(TAG, "Volume changed from %d to %d", current_volume, new_volume);
        
        // 显示通知前检查实际变化
        if (new_volume != codec->output_volume()) {
            ESP_LOGE(TAG, "Failed to set volume! Expected:%d Actual:%d", 
                   new_volume, codec->output_volume());
        }
        GetDisplay()->ShowNotification(std::string(Lang::Strings::VOLUME) + ": "+std::to_string(codec->output_volume()));
        power_save_timer_->WakeUp();
    }

    void InitializeKnob() {
        knob_ = std::make_unique<Knob>(BSP_KNOB_A_PIN, BSP_KNOB_B_PIN);
        knob_->OnRotate([this](bool clockwise) {
            ESP_LOGD(TAG, "Knob rotation detected. Clockwise:%s", clockwise ? "true" : "false");
            OnKnobRotate(clockwise);
        });
        ESP_LOGI(TAG, "Knob initialized with pins A:%d B:%d", BSP_KNOB_A_PIN, BSP_KNOB_B_PIN);
    }

    void InitializeButton() {
        // 设置静态实例指针
        instance_ = this;
        
        // watcher 是通过长按滚轮进行开机的, 需要等待滚轮释放, 否则用户开机松手时可能会误触成单击
        ESP_LOGI(TAG, "waiting for knob button release");
        while(IoExpanderGetLevel(BSP_KNOB_BTN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        button_config_t btn_config = {
            .long_press_time = 2000,
            .short_press_time = 0
        };
        btn_driver_ = (button_driver_t*)calloc(1, sizeof(button_driver_t));
        btn_driver_->enable_power_save = false;
        btn_driver_->get_key_level = [](button_driver_t *button_driver) -> uint8_t {
            return !instance_->IoExpanderGetLevel(BSP_KNOB_BTN);
        };
        
        ESP_ERROR_CHECK(iot_button_create(&btn_config, btn_driver_, &btns));
        
        iot_button_register_cb(btns, BUTTON_SINGLE_CLICK, nullptr, [](void* button_handle, void* usr_data) {
            auto self = static_cast<SensecapWatcher*>(usr_data);
            self->power_save_timer_->WakeUp();

            if (self->settings_ui_ && self->settings_ui_->IsOpen()) {
                self->settings_ui_->OnKnobClick();
                return;
            }

            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                self->EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        }, this);
        
        iot_button_register_cb(btns, BUTTON_LONG_PRESS_START, nullptr, [](void* button_handle, void* usr_data) {
            auto self = static_cast<SensecapWatcher*>(usr_data);
            bool is_charging = (self->IoExpanderGetLevel(BSP_PWR_VBUS_IN_DET) == 0);
            self->long_press_cnt_ = 0;
            if (is_charging) {
                ESP_LOGI(TAG, "charging");
            } else {
                self->IoExpanderSetLevel(BSP_PWR_LCD, 0);
                self->IoExpanderSetLevel(BSP_PWR_SYSTEM, 0);
            }
        }, this);

        iot_button_register_cb(btns, BUTTON_LONG_PRESS_HOLD, nullptr, [](void* button_handle, void* usr_data) {
            auto self = static_cast<SensecapWatcher*>(usr_data);
            self->long_press_cnt_++; // 每隔20ms加一
            // 长按10s 恢复出厂设置: 2+0.02*400 = 10
            if (self->long_press_cnt_ > 400) {
                ESP_LOGI(TAG, "Factory reset");
                nvs_flash_erase();
                esp_restart();
            }
        }, this);
    }

    void InitializeSpi() {
        ESP_LOGI(TAG, "Initialize SSCMA SPI bus");
        spi_bus_config_t spi_cfg = {0};

        spi_cfg.mosi_io_num = BSP_SPI2_HOST_MOSI;
        spi_cfg.miso_io_num = BSP_SPI2_HOST_MISO;
        spi_cfg.sclk_io_num = BSP_SPI2_HOST_SCLK;
        spi_cfg.quadwp_io_num = -1;
        spi_cfg.quadhd_io_num = -1;
        spi_cfg.isr_cpu_id = ESP_INTR_CPU_AFFINITY_1;
        spi_cfg.max_transfer_sz = 4095;
   
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO));

        ESP_LOGI(TAG, "Initialize QSPI bus");

        spi_bus_config_t qspi_cfg = {0};
        qspi_cfg.sclk_io_num = BSP_SPI3_HOST_PCLK;
        qspi_cfg.data0_io_num = BSP_SPI3_HOST_DATA0;
        qspi_cfg.data1_io_num = BSP_SPI3_HOST_DATA1;
        qspi_cfg.data2_io_num = BSP_SPI3_HOST_DATA2;
        qspi_cfg.data3_io_num = BSP_SPI3_HOST_DATA3;
        qspi_cfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * DRV_LCD_BITS_PER_PIXEL / 8 / CONFIG_BSP_LCD_SPI_DMA_SIZE_DIV;
    
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &qspi_cfg, SPI_DMA_CH_AUTO));
    }

    void Initializespd2010Display() {
        ESP_LOGI(TAG, "Install panel IO");
        const esp_lcd_panel_io_spi_config_t io_config = {
            .cs_gpio_num = BSP_LCD_SPI_CS,
            .dc_gpio_num = GPIO_NUM_NC,
            .spi_mode = 3,
            .pclk_hz = DRV_LCD_PIXEL_CLK_HZ,
            .trans_queue_depth = 2,
            .lcd_cmd_bits = DRV_LCD_CMD_BITS,
            .lcd_param_bits = DRV_LCD_PARAM_BITS,
            .flags = {
                .quad_mode = true,
            },
        };
        spd2010_vendor_config_t vendor_config = {
            .flags = {
                .use_qspi_interface = 1,
            },
        };
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, &panel_io_);
    
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.rgb_ele_order = DRV_LCD_RGB_ELEMENT_ORDER;
        panel_config.bits_per_pixel = DRV_LCD_BITS_PER_PIXEL;
        panel_config.reset_gpio_num = BSP_LCD_GPIO_RST; // Shared with Touch reset
        panel_config.vendor_config = &vendor_config;
        esp_lcd_new_panel_spd2010(panel_io_, &panel_config, &panel_);

        esp_lcd_panel_reset(panel_);
        esp_lcd_panel_init(panel_);
        esp_lcd_panel_mirror(panel_, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(panel_, true);

        display_ = new CustomLcdDisplay(panel_io_, panel_,
            DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
        
        // 使每次刷新的起始列数索引是4的倍数且列数总数是4的倍数，以满足SPD2010的要求
        lv_display_add_event_cb(lv_display_get_default(), [](lv_event_t *e) {
            lv_area_t *area = (lv_area_t *)lv_event_get_param(e);
            uint16_t x1 = area->x1;
            uint16_t x2 = area->x2;
            // round the start of area down to the nearest 4N number
            area->x1 = (x1 >> 2) << 2;
            // round the end of area up to the nearest 4M+3 number
            area->x2 = ((x2 >> 2) << 2) + 3;
        }, LV_EVENT_INVALIDATE_AREA, NULL);
        
    }

    void InitializeTouch() {
        ESP_LOGI(TAG, "Initializing SPD2010 touch controller");
        gpio_reset_pin(BSP_TOUCH_I2C_SDA);
        gpio_reset_pin(BSP_TOUCH_I2C_SCL);

        i2c_master_bus_config_t touch_i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)BSP_TOUCH_I2C_NUM,
            .sda_io_num = BSP_TOUCH_I2C_SDA,
            .scl_io_num = BSP_TOUCH_I2C_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        esp_err_t ret = i2c_new_master_bus(&touch_i2c_bus_cfg, &touch_i2c_bus_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create touch I2C bus: %s", esp_err_to_name(ret));
            return;
        }

        esp_lcd_panel_io_i2c_config_t tp_io_config = {};
        tp_io_config.dev_addr = ESP_LCD_TOUCH_IO_I2C_SPD2010_ADDRESS;
        tp_io_config.scl_speed_hz = 100000;
        tp_io_config.control_phase_bytes = 1;
        tp_io_config.flags.disable_control_phase = 1;

        esp_lcd_panel_io_handle_t tp_io_handle = nullptr;
        ret = esp_lcd_new_panel_io_i2c(touch_i2c_bus_, &tp_io_config, &tp_io_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create touch panel IO: %s", esp_err_to_name(ret));
            return;
        }

        const esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH,
            .y_max = DISPLAY_HEIGHT,
            .rst_gpio_num = GPIO_NUM_NC,
            .int_gpio_num = GPIO_NUM_NC,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = DISPLAY_SWAP_XY,
                .mirror_x = DISPLAY_MIRROR_X,
                .mirror_y = DISPLAY_MIRROR_Y,
            },
        };
        ret = esp_lcd_touch_new_i2c_spd2010(tp_io_handle, &tp_cfg, &touch_handle_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create touch driver: %s", esp_err_to_name(ret));
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
        esp_lcd_touch_read_data(touch_handle_);
        vTaskDelay(pdMS_TO_TICKS(50));

        const lvgl_port_touch_cfg_t touch_port_cfg = {
            .disp = lv_display_get_default(),
            .handle = touch_handle_,
        };
        lv_indev_t* touch_indev = lvgl_port_add_touch(&touch_port_cfg);
        if (touch_indev == nullptr) {
            ESP_LOGE(TAG, "Failed to add touch device to LVGL");
            return;
        }
        ESP_LOGI(TAG, "Touch panel initialized successfully");
    }

    void InitializeSettingsUi() {
        settings_ui_ = std::make_unique<WatcherSettingsUi>(display_, [this]() {
            ESP_LOGI(TAG, "Settings UI closed");
        });
        settings_ui_->SetBacklight(GetBacklight());
        settings_ui_->SetCamera(camera_);
        settings_ui_->SetActivityCallback([this]() {
            if (power_save_timer_) {
                power_save_timer_->WakeUp();
            }
        });
        settings_ui_->SetBatteryInfoProvider(
            [this]() { return (int)BatterygetPercent(false); },
            [this]() { return BatterygetVoltage(); }
        );

        display_->SetGestureCallback([this](lv_dir_t dir) {
            power_save_timer_->WakeUp();
            if (settings_ui_ == nullptr) return;
            if (settings_ui_->IsOpen()) {
                settings_ui_->HandleTouchGesture(dir);
                return;
            }

            if (dir == LV_DIR_BOTTOM) {
                ESP_LOGI(TAG, "Swipe DOWN detected -> Open QuickSettings");
                settings_ui_->Open(WatcherSettingsPage::QuickSettings);
            } else if (dir == LV_DIR_LEFT) {
                ESP_LOGI(TAG, "Swipe LEFT detected -> Open VisionMode");
                settings_ui_->Open(WatcherSettingsPage::VisionMode);
            } else if (dir == LV_DIR_RIGHT) {
                ESP_LOGI(TAG, "Swipe RIGHT detected -> Open Parameters");
                settings_ui_->Open(WatcherSettingsPage::Parameters);
            }
        });
    }

    uint16_t BatterygetVoltage(void) {
        static bool initialized = false;
        static adc_oneshot_unit_handle_t adc_handle;
        static adc_cali_handle_t cali_handle = NULL;
        if (!initialized) {
            adc_oneshot_unit_init_cfg_t init_config = {
                .unit_id = ADC_UNIT_1,
            };
            adc_oneshot_new_unit(&init_config, &adc_handle);
    
            adc_oneshot_chan_cfg_t ch_config = {
                .atten = BSP_BAT_ADC_ATTEN,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            adc_oneshot_config_channel(adc_handle, BSP_BAT_ADC_CHAN, &ch_config);
    
            adc_cali_curve_fitting_config_t cali_config = {
                .unit_id = ADC_UNIT_1,
                .chan = BSP_BAT_ADC_CHAN,
                .atten = BSP_BAT_ADC_ATTEN,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            if (adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle) == ESP_OK) {
                initialized = true;
            }
        }
        if (initialized) {
            int raw_value = 0;
            int voltage = 0; // mV
            adc_oneshot_read(adc_handle, BSP_BAT_ADC_CHAN, &raw_value);
            adc_cali_raw_to_voltage(cali_handle, raw_value, &voltage);
            voltage = voltage * 82 / 20;
            // ESP_LOGI(TAG, "voltage: %dmV", voltage);
            return (uint16_t)voltage;
        }
        return 0;
    }

    uint8_t BatterygetPercent(bool print = false) {
        int voltage = 0;
        for (uint8_t i = 0; i < 10; i++) {
            voltage += BatterygetVoltage();
        }
        voltage /= 10;
        int percent = (-1 * voltage * voltage + 9016 * voltage - 19189000) / 10000;
        percent = (percent > 100) ? 100 : (percent < 0) ? 0 : percent;
        if (print) {
            printf("voltage: %dmV, percentage: %d%%\r\n", voltage, percent);
        }
        return (uint8_t)percent;
    }

    void InitializeCmd() {
        esp_console_repl_t *repl = NULL;
        esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
        repl_config.max_cmdline_length = 4096;
        repl_config.prompt = "SenseCAP>";
        
        const esp_console_cmd_t cmd1 = {
            .command = "reboot",
            .help = "reboot the device",
            .hint = nullptr,
            .func = [](int argc, char** argv) -> int {
                esp_restart();
                return 0;
            },
            .argtable = nullptr
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd1));

        const esp_console_cmd_t cmd2 = {
            .command = "shutdown",
            .help = "shutdown the device",
            .hint = nullptr,
            .func = NULL,
            .argtable = NULL,
            .func_w_context = [](void *context,int argc, char** argv) -> int {
                auto self = static_cast<SensecapWatcher*>(context);
                self->GetBacklight()->SetBrightness(0);
                self->IoExpanderSetLevel(BSP_PWR_SYSTEM, 0);
                return 0;
            },
            .context =this
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd2));

        const esp_console_cmd_t cmd3 = {
            .command = "battery",
            .help = "get battery percent",
            .hint = NULL,
            .func = NULL,
            .argtable = NULL,
            .func_w_context = [](void *context,int argc, char** argv) -> int {
                auto self = static_cast<SensecapWatcher*>(context);
                self->BatterygetPercent(true);
                return 0;
            },
            .context =this
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd3));

        const esp_console_cmd_t cmd4 = {
            .command = "factory_reset",
            .help = "factory reset and reboot the device",
            .hint = NULL,
            .func = NULL,
            .argtable = NULL,
            .func_w_context = [](void *context,int argc, char** argv) -> int {
                nvs_flash_erase();
                esp_restart();
                return 0;
            },
            .context =this
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd4));

        const esp_console_cmd_t cmd5 = {
            .command = "read_mac",
            .help = "Read mac address",
            .hint = NULL,
            .func = NULL,
            .argtable = NULL,
            .func_w_context = [](void *context,int argc, char** argv) -> int {
                uint8_t mac[6];
                esp_read_mac(mac, ESP_MAC_WIFI_STA);
                printf("wifi_sta_mac: " MACSTR "\n", MAC2STR(mac));
                esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
                printf("wifi_softap_mac: " MACSTR "\n", MAC2STR(mac));
                esp_read_mac(mac, ESP_MAC_BT);
                printf("bt_mac: " MACSTR "\n", MAC2STR(mac));
                return 0;
            },
            .context =this
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd5));

        const esp_console_cmd_t cmd6 = {
            .command = "version",
            .help = "Read version info",
            .hint = NULL,
            .func = NULL,
            .argtable = NULL,
            .func_w_context = [](void *context,int argc, char** argv) -> int {
                auto self = static_cast<SensecapWatcher*>(context);
                auto app_desc = esp_app_get_description();
                const char* region = "UNKNOWN";
                #if defined(CONFIG_LANGUAGE_ZH_CN)
                    region = "CN";
                #elif defined(CONFIG_LANGUAGE_EN_US)
                    region = "US";
                #elif defined(CONFIG_LANGUAGE_JA_JP)
                    region = "JP";
                #elif defined(CONFIG_LANGUAGE_ES_ES)
                    region = "ES";
                #elif defined(CONFIG_LANGUAGE_DE_DE)
                    region = "DE";
                #elif defined(CONFIG_LANGUAGE_FR_FR)
                    region = "FR";
                #elif defined(CONFIG_LANGUAGE_IT_IT)
                    region = "IT";
                #elif defined(CONFIG_LANGUAGE_PT_PT)
                    region = "PT";
                #elif defined(CONFIG_LANGUAGE_RU_RU)
                    region = "RU";
                #elif defined(CONFIG_LANGUAGE_KO_KR)
                    region = "KR";
                #endif
                printf("{\"type\":0,\"name\":\"VER?\",\"code\":0,\"data\":{\"software\":\"%s\",\"hardware\":\"watcher xiaozhi agent\",\"camera\":%d,\"region\":\"%s\"}}\n",
                       app_desc->version,
                       self->GetCamera() == nullptr ? 0 : 1,
                       region);
                return 0;
            },
            .context =this
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd6));

        // remote_display 命令 - 配置远程显示
        const esp_console_cmd_t cmd_remote = {
            .command = "remote_display",
            .help = "Configure remote display. Usage:\n"
                    "  remote_display status     - Show current config\n"
                    "  remote_display enable     - Enable remote display\n"
                    "  remote_display disable    - Disable remote display\n"
                    "  remote_display url <url>  - Set server URL (e.g. ws://192.168.1.100:8765)\n"
                    "  remote_display connect    - Connect now\n"
                    "  remote_display disconnect - Disconnect",
            .hint = NULL,
            .func = NULL,
            .argtable = NULL,
            .func_w_context = [](void *context, int argc, char** argv) -> int {
                if (argc < 2) {
                    printf("Usage: remote_display <status|enable|disable|url|connect|disconnect>\n");
                    return 1;
                }

                auto* remote = RemoteDisplay::GetInstance();
                const char* subcmd = argv[1];

                if (strcmp(subcmd, "status") == 0) {
                    auto config = RemoteDisplay::LoadConfig();
                    printf("Remote Display Config:\n");
                    printf("  enabled: %s\n", config.enabled ? "true" : "false");
                    printf("  url: %s\n", config.server_url.empty() ? "(not set)" : config.server_url.c_str());
                    printf("  timeout: %d ms\n", config.timeout_ms);
                    printf("  running: %s\n", remote->IsRunning() ? "true" : "false");
                } else if (strcmp(subcmd, "enable") == 0) {
                    auto config = RemoteDisplay::LoadConfig();
                    config.enabled = true;
                    RemoteDisplay::SaveConfig(config);
                    printf("Remote display enabled. Use 'remote_display connect' to connect.\n");
                } else if (strcmp(subcmd, "disable") == 0) {
                    auto config = RemoteDisplay::LoadConfig();
                    config.enabled = false;
                    RemoteDisplay::SaveConfig(config);
                    remote->Stop();
                    printf("Remote display disabled.\n");
                } else if (strcmp(subcmd, "url") == 0) {
                    if (argc < 3) {
                        printf("Usage: remote_display url <ws://host:port>\n");
                        return 1;
                    }
                    auto config = RemoteDisplay::LoadConfig();
                    config.server_url = argv[2];
                    RemoteDisplay::SaveConfig(config);
                    printf("Server URL set to: %s\n", argv[2]);
                } else if (strcmp(subcmd, "connect") == 0) {
                    if (remote->IsRunning()) {
                        printf("Already connected.\n");
                        return 0;
                    }
                    if (remote->StartWithConfig()) {
                        printf("Connected to remote display server.\n");
                    } else {
                        printf("Failed to connect. Check URL and network.\n");
                    }
                } else if (strcmp(subcmd, "disconnect") == 0) {
                    auto config = RemoteDisplay::LoadConfig();
                    config.enabled = false;
                    RemoteDisplay::SaveConfig(config);
                    remote->Stop();
                    printf("Disconnected.\n");
                } else {
                    printf("Unknown subcommand: %s\n", subcmd);
                    return 1;
                }
                return 0;
            },
            .context = this
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_remote));

        // Register static IP configuration command
        const esp_console_cmd_t cmd_static_ip = {
            .command = "wifi_static_ip",
            .help = "set/show/clear static IP.\n"
                    "  wifi_static_ip set <ip> <gw> [mask] [dns]\n"
                    "  wifi_static_ip show\n"
                    "  wifi_static_ip clear",
            .hint = NULL,
            .func = [](int argc, char** argv) -> int {
                if (argc < 2) {
                    printf("Usage: wifi_static_ip set|show|clear\n");
                    return 1;
                }

                if (strcmp(argv[1], "show") == 0) {
                    nvs_handle_t nvs;
                    if (nvs_open("wifi", NVS_READONLY, &nvs) != ESP_OK) {
                        printf("{\"enabled\":false}\n");
                        return 0;
                    }
                    uint8_t en = 0;
                    nvs_get_u8(nvs, "static_ip_en", &en);
                    char ip[16] = {}, gw[16] = {}, mask[16] = {}, dns[16] = {};
                    size_t len;
                    len = sizeof(ip);   nvs_get_str(nvs, "static_ip", ip, &len);
                    len = sizeof(gw);   nvs_get_str(nvs, "static_gw", gw, &len);
                    len = sizeof(mask);  nvs_get_str(nvs, "static_nm", mask, &len);
                    len = sizeof(dns);   nvs_get_str(nvs, "static_dns", dns, &len);
                    nvs_close(nvs);
                    printf("{\"enabled\":%s,\"ip\":\"%s\",\"gateway\":\"%s\","
                           "\"netmask\":\"%s\",\"dns\":\"%s\"}\n",
                           en ? "true" : "false", ip, gw, mask, dns);
                    return 0;
                }

                if (strcmp(argv[1], "clear") == 0) {
                    nvs_handle_t nvs;
                    if (nvs_open("wifi", NVS_READWRITE, &nvs) == ESP_OK) {
                        nvs_set_u8(nvs, "static_ip_en", 0);
                        nvs_commit(nvs);
                        nvs_close(nvs);
                    }
                    printf("{\"success\":true,\"msg\":\"static IP cleared, reboot to use DHCP\"}\n");
                    return 0;
                }

                if (strcmp(argv[1], "set") == 0) {
                    if (argc < 4) {
                        printf("{\"success\":false,\"error\":\"Usage: wifi_static_ip set <ip> <gw> [mask] [dns]\"}\n");
                        return 1;
                    }
                    const char *mask_val = (argc >= 5) ? argv[4] : "255.255.255.0";
                    const char *dns_val = (argc >= 6) ? argv[5] : "";
                    nvs_handle_t nvs;
                    if (nvs_open("wifi", NVS_READWRITE, &nvs) != ESP_OK) {
                        printf("{\"success\":false,\"error\":\"NVS open failed\"}\n");
                        return 1;
                    }
                    nvs_set_u8(nvs, "static_ip_en", 1);
                    nvs_set_str(nvs, "static_ip", argv[2]);
                    nvs_set_str(nvs, "static_gw", argv[3]);
                    nvs_set_str(nvs, "static_nm", mask_val);
                    nvs_set_str(nvs, "static_dns", dns_val);
                    nvs_commit(nvs);
                    nvs_close(nvs);
                    printf("{\"success\":true,\"ip\":\"%s\",\"gateway\":\"%s\","
                           "\"netmask\":\"%s\",\"dns\":\"%s\"}\n",
                           argv[2], argv[3], mask_val, dns_val[0] ? dns_val : "(use gateway)");
                    return 0;
                }

                printf("Unknown subcommand: %s\n", argv[1]);
                return 1;
            },
            .argtable = NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_static_ip));

        const esp_console_cmd_t cmd_wifi_set = {
            .command = "wifi_set",
            .help = "Set WiFi credentials and reboot: wifi_set <ssid> [password]",
            .hint = NULL,
            .func = [](int argc, char** argv) -> int {
                if (argc < 2) {
                    printf("Usage: wifi_set <ssid> [password]\n");
                    return 1;
                }
                std::string ssid = argv[1];
                std::string password = (argc >= 3) ? argv[2] : "";
                SsidManager::GetInstance().AddSsid(ssid, password);
                printf("WiFi credentials saved for '%s'. Rebooting...\n", ssid.c_str());
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
                return 0;
            },
            .argtable = NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_wifi_set));

        // Register local-only remote wake configuration command.
        const esp_console_cmd_t cmd_remote_wake = {
            .command = "remote_wake",
            .help = "Configure LAN remote wake. Usage: remote_wake status|enable [token]|disable|clear",
            .hint = NULL,
            .func = NULL,
            .argtable = NULL,
            .func_w_context = [](void *context, int argc, char** argv) -> int {
                constexpr const char* kNamespace = "remote_wake";
                constexpr const char* kTokenKey = "token";
                constexpr const char* kEnabledKey = "enabled";
                constexpr const char* kTokenChars =
                    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._~-";
                auto valid_token = [kTokenChars](const std::string& token) {
                    return token.size() >= 16 && token.size() <= 128 &&
                           token.find_first_not_of(kTokenChars) == std::string::npos;
                };

                if (argc < 2) {
                    printf("Usage: remote_wake status|enable [token]|disable|clear\n");
                    return 1;
                }

                const char* subcmd = argv[1];
                Settings settings(kNamespace, false);
                if (strcmp(subcmd, "status") == 0) {
                    if (argc != 2) {
                        printf("Usage: remote_wake status\n");
                        return 1;
                    }
                    const bool enabled = settings.GetBool(kEnabledKey, false);
                    const bool token_configured = !settings.GetString(kTokenKey, "").empty();
                    printf("{\"enabled\":%s,\"token_configured\":%s}\n",
                           enabled ? "true" : "false",
                           token_configured ? "true" : "false");
                    return 0;
                }

                if (strcmp(subcmd, "enable") == 0) {
                    if (argc > 3) {
                        printf("Usage: remote_wake enable [token]\n");
                        return 1;
                    }
                    const std::string current_token = settings.GetString(kTokenKey, "");
                    const std::string token = argc == 3 ? argv[2] : current_token;
                    if (!valid_token(token)) {
                        printf("remote_wake enable requires a valid token (16..128 ASCII characters)\n");
                        return 1;
                    }
                    Settings writable(kNamespace, true);
                    if (argc == 3) {
                        writable.SetString(kTokenKey, token);
                    }
                    writable.SetBool(kEnabledKey, true);
                    printf("{\"success\":true,\"enabled\":true,\"token_configured\":true}\n");
                    return 0;
                }

                if (strcmp(subcmd, "disable") == 0) {
                    if (argc != 2) {
                        printf("Usage: remote_wake disable\n");
                        return 1;
                    }
                    Settings writable(kNamespace, true);
                    writable.SetBool(kEnabledKey, false);
                    printf("{\"success\":true,\"enabled\":false}\n");
                    return 0;
                }

                if (strcmp(subcmd, "clear") == 0) {
                    if (argc != 2) {
                        printf("Usage: remote_wake clear\n");
                        return 1;
                    }
                    Settings writable(kNamespace, true);
                    writable.SetString(kTokenKey, "");
                    writable.SetBool(kEnabledKey, false);
                    printf("{\"success\":true,\"enabled\":false,\"token_configured\":false}\n");
                    return 0;
                }

                printf("Unknown remote_wake subcommand\n");
                return 1;
            },
            .context = this
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_remote_wake));

        // Register face database CRUD commands
        FaceSerialHandler::GetInstance().RegisterCommands();

        esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
        ESP_ERROR_CHECK(esp_console_start_repl(repl));
    }

    void InitializeCamera() {

        ESP_LOGI(TAG, "Initialize Camera");

        // !!!NOTE: SD Card use same SPI bus as sscma client, so we need to disable SD card CS pin first
        const gpio_config_t io_config = {
            .pin_bit_mask = (1ULL << BSP_SD_SPI_CS),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t ret = gpio_config(&io_config);
        if (ret != ESP_OK)
            return;

        gpio_set_level(BSP_SD_SPI_CS, 1);

        camera_ = new SscmaCamera(io_exp_handle);
    }

    void ApplyStaticIpIfConfigured() {
        nvs_handle_t nvs;
        if (nvs_open("wifi", NVS_READONLY, &nvs) != ESP_OK) return;

        uint8_t enabled = 0;
        nvs_get_u8(nvs, "static_ip_en", &enabled);
        if (!enabled) {
            nvs_close(nvs);
            return;
        }

        char ip[16] = {}, gw[16] = {}, mask[16] = {}, dns[16] = {};
        size_t len;
        len = sizeof(ip);   nvs_get_str(nvs, "static_ip", ip, &len);
        len = sizeof(gw);   nvs_get_str(nvs, "static_gw", gw, &len);
        len = sizeof(mask);  nvs_get_str(nvs, "static_nm", mask, &len);
        len = sizeof(dns);   nvs_get_str(nvs, "static_dns", dns, &len);
        nvs_close(nvs);

        if (ip[0] == '\0') return;

        esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (!sta) {
            ESP_LOGE(TAG, "Static IP: STA netif not found");
            return;
        }

        esp_netif_dhcpc_stop(sta);

        esp_netif_ip_info_t ip_info = {};
        esp_netif_str_to_ip4(ip, &ip_info.ip);
        esp_netif_str_to_ip4(gw[0] ? gw : "0.0.0.0", &ip_info.gw);
        esp_netif_str_to_ip4(mask[0] ? mask : "255.255.255.0", &ip_info.netmask);
        esp_netif_set_ip_info(sta, &ip_info);

        // DNS: 如果未指定则使用阿里公共 DNS
        const char *dns_addr = dns[0] ? dns : "223.5.5.5";
        if (dns_addr[0]) {
            esp_netif_dns_info_t dns_info = {};
            dns_info.ip.type = ESP_IPADDR_TYPE_V4;
            esp_netif_str_to_ip4(dns_addr, &dns_info.ip.u_addr.ip4);
            esp_netif_set_dns_info(sta, ESP_NETIF_DNS_MAIN, &dns_info);
        }

        ESP_LOGI(TAG, "Static IP applied: %s gw=%s mask=%s dns=%s", ip, gw, mask, dns_addr);
    }

    void InitializeRemoteDisplay() {
        // 同步注册投屏控制 MCP 工具，避免异步任务在多核环境下与主任务并发修改 McpServer::tools_
        RegisterScreenCastTool();

        // 启动任务等待联网后启动常驻 HTTP 服务并恢复连接
        xTaskCreate([](void* arg) {
            auto* self = static_cast<SensecapWatcher*>(arg);

            // 注册 Opus 音频转发回调
            Application::GetInstance().SetOpusForwardCallback(
                [](const std::vector<uint8_t>& opus_data, int sample_rate, int frame_duration) {
                    auto* remote = RemoteDisplay::GetInstance();
                    if (remote && remote->IsRunning()) {
                        remote->ForwardOpusAudio(opus_data, sample_rate, frame_duration);
                    }
                });

            // 等待 WiFi 真正连接
            ESP_LOGI(TAG, "Waiting for WiFi connection before starting remote display...");
            auto& wifi = WifiManager::GetInstance();
            int wait_seconds = 0;
            while (!wifi.IsConnected()) {
                if (wait_seconds > 0 && wait_seconds % 10 == 0) {
                    ESP_LOGI(TAG, "Still waiting WiFi connection... %ds", wait_seconds);
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                wait_seconds++;
            }
            ESP_LOGI(TAG, "WiFi connected, IP: %s", wifi.GetIpAddress().c_str());

            // 联网后无条件常驻 HTTP 服务器，承载投屏控制与人脸端点。
            self->remote_display_http_server_.SetCamera(self->camera_);
            self->remote_display_http_server_.Start(80);

            // 如果 NVS 中有保存的配置，尝试自动重连（HTTP 服务器已常驻，无需再启动）
            auto config = RemoteDisplay::LoadConfig();
            if (config.enabled && !config.server_url.empty()) {
                auto* remote = RemoteDisplay::GetInstance();
                if (remote->StartWithConfig()) {
                    ESP_LOGI(TAG, "Remote display auto-reconnected to %s", config.server_url.c_str());
                } else {
                    ESP_LOGW(TAG, "Remote display initial connection failed; background retry scheduled");
                }
            }

            vTaskDelete(nullptr);
        }, "remote_disp_init", 2560, this, 1, nullptr);
    }

    void RegisterScreenCastTool() {
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddUserOnlyTool("self.screen_cast",
            "控制投屏功能的开启与关闭。\n"
            "当用户说'开启投屏'、'打开投屏'时，使用 enable=1 开启。\n"
            "当用户说'关闭投屏'、'停止投屏'时，使用 enable=0 关闭。\n"
            "树莓派需手动输入本设备 IP，通过 HTTP 发起连接。\n"
            "不传参数则查询当前投屏状态。",
            PropertyList({
                Property("enable", kPropertyTypeInteger, 0, 1)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                try {
                    const Property& enable_prop = properties["enable"];
                    int en = enable_prop.value<int>();
                    ESP_LOGI(TAG, "screen_cast tool called: enable=%d", en);
                    if (en == 1) {
                        return StartScreenCast();
                    } else {
                        return StopScreenCast();
                    }
                } catch (const std::runtime_error& e) {
                    // 没传参数 → 查询状态
                    ESP_LOGW(TAG, "screen_cast tool: no enable param, caught: %s", e.what());
                    auto* remote = RemoteDisplay::GetInstance();
                    bool casting = remote && remote->IsRunning();
                    auto config = RemoteDisplay::LoadConfig();
                    return std::string("{\"casting\":") + (casting ? "true" : "false") +
                           ",\"enabled\":" + (config.enabled ? "true" : "false") +
                           ",\"configured\":" + (!config.server_url.empty() ? "true" : "false") + "}";
                }
            });

        // 查询本机 WiFi IP 地址。
        // 使用场景：当用户问"你的IP地址是多少"、"设备IP"、"局域网地址"时调用。
        // 返回：{"ip":"192.168.x.x","connected":true} 或 {"ip":"","connected":false}
        mcp_server.AddUserOnlyTool("self.get_ip_address",
            "获取本设备的 WiFi 局域网 IP 地址。\n"
            "使用场景：当用户问'你的IP地址是多少'、'设备IP'、'局域网地址'、'怎么连你'时调用。\n"
            "返回：JSON，包含 ip(IPv4 地址字符串) 和 connected(是否已联网)。",
            PropertyList(),
            [](const PropertyList&) -> ReturnValue {
                auto& wifi = WifiManager::GetInstance();
                bool connected = wifi.IsConnected();
                std::string ip = connected ? wifi.GetIpAddress() : std::string();
                return std::string("{\"ip\":\"") + ip + "\",\"connected\":" +
                       (connected ? "true" : "false") + "}";
            });
    }

    std::string StartScreenCast() {
        ESP_LOGI(TAG, "StartScreenCast() called");
        // 如果已经在投屏，返回状态
        auto* remote = RemoteDisplay::GetInstance();
        if (remote && remote->IsRunning()) {
            ESP_LOGI(TAG, "Already casting, skip");
            return std::string("{\"success\":true,\"casting\":true,\"enabled\":true,\"configured\":true,\"message\":\"已在投屏中\"}");
        }

        // HTTP server normally starts as soon as Wi-Fi is available. Keep an
        // idempotent fallback in case the voice command races initialization.
        if (!remote_display_http_server_.IsRunning()) {
            ESP_LOGI(TAG, "HTTP server not running yet, starting...");
            remote_display_http_server_.SetCamera(camera_);
            bool ok = remote_display_http_server_.Start(80);
            if (!ok || !remote_display_http_server_.IsRunning()) {
                return std::string("{\"success\":false,\"casting\":false,\"enabled\":false,\"configured\":false,\"message\":\"投屏控制服务启动失败\"}");
            }
        }

        auto config = RemoteDisplay::LoadConfig();
        if (config.server_url.empty()) {
            return std::string("{\"success\":false,\"casting\":false,\"enabled\":false,\"configured\":false,\"message\":\"请在树莓派投屏页面输入本设备 IP 发起连接\"}");
        }

        config.enabled = true;
        RemoteDisplay::SaveConfig(config);
        bool connected = remote->StartWithConfig();
        if (connected) {
            return std::string("{\"success\":true,\"casting\":true,\"enabled\":true,\"configured\":true,\"message\":\"投屏已连接\"}");
        }
        return std::string("{\"success\":true,\"casting\":false,\"enabled\":true,\"configured\":true,\"message\":\"暂时无法连接，已在后台重试\"}");
    }

    std::string StopScreenCast() {
        // Persist disable before stopping so queued retries cannot reconnect.
        auto config = RemoteDisplay::LoadConfig();
        config.enabled = false;
        RemoteDisplay::SaveConfig(config);

        auto* remote = RemoteDisplay::GetInstance();
        if (remote) {
            remote->Stop();
        }

        ESP_LOGI(TAG, "Screen cast stopped");
        return std::string("{\"success\":true,\"casting\":false,\"enabled\":false,\"configured\":") +
               (!config.server_url.empty() ? "true" : "false") +
               ",\"message\":\"投屏已关闭\"}";
    }

public:
    SensecapWatcher() {
        ESP_LOGI(TAG, "Initialize Sensecap Watcher");
        // 创建 I2C 操作互斥锁（必须在 InitializeExpander 之前）
        io_exp_mutex_ = xSemaphoreCreateMutex();
        assert(io_exp_mutex_ != nullptr);

        InitializePowerSaveTimer();
        InitializeI2c();
        InitializeSpi();
        InitializeExpander();
        InitializeCmd();  //工厂生产测试使用
        InitializeButton();
        Initializespd2010Display();
        GetBacklight()->RestoreBrightness();  // 对于不带摄像头的版本，InitializeCamera需要3s, 所以先恢复背光亮度
        InitializeTouch();
        InitializeCamera();
        InitializeSettingsUi();
        InitializeKnob();
        InitializeRemoteDisplay();
    }

    virtual bool SuppressCommonDeviceTools() const override { return true; }

    virtual AudioCodec* GetAudioCodec() override {
        static SensecapAudioCodec audio_codec(
            i2c_bus_, 
            AUDIO_INPUT_SAMPLE_RATE, 
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, 
            AUDIO_I2S_GPIO_BCLK, 
            AUDIO_I2S_GPIO_WS, 
            AUDIO_I2S_GPIO_DOUT, 
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, 
            AUDIO_CODEC_ES8311_ADDR, 
            AUDIO_CODEC_ES7243E_ADDR, 
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    // 根据 https://github.com/Seeed-Studio/OSHW-SenseCAP-Watcher/blob/main/Hardware/SenseCAP_Watcher_v1.0_SCH.pdf
    // RGB LED型号为 ws2813 mini, 连接在GPIO 40，供电电压 3.3v, 没有连接 BIN 双信号线
    // 可以直接兼容SingleLED采用的ws2812
    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    void StartNetwork() override {
        WifiBoard::StartNetwork();
        // 首次启动直接应用
        ApplyStaticIpIfConfigured();
        // 注册事件：配网退出后 StartStation 会再次触发 STA_START，确保静态 IP 重新应用
        // 用 static guard 防止 StartNetwork 被多次调用时重复注册
        static bool sta_handler_registered = false;
        if (!sta_handler_registered) {
            esp_event_handler_instance_t handler;
            esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_START,
                [](void* arg, esp_event_base_t, int32_t, void*) {
                    static_cast<SensecapWatcher*>(arg)->ApplyStaticIpIfConfigured();
                }, this, &handler);
            sta_handler_registered = true;
        }
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }

    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        static int low_battery_streak = 0;
        charging = (IoExpanderGetLevel(BSP_PWR_VBUS_IN_DET) == 0);
        discharging = !charging;

        uint16_t voltage = BatterygetVoltage();
        if (voltage >= 2500) {
            level = (int)BatterygetPercent(false);
            if (level <= 1 && discharging) {
                low_battery_streak++;
                if (low_battery_streak >= 3) {
                    ESP_LOGI(TAG, "Battery level is low (%d mV), shutting down", voltage);
                    IoExpanderSetLevel(BSP_PWR_SYSTEM, 0);
                }
            } else {
                low_battery_streak = 0;
            }
        } else {
            // ADC 未就绪或采样异常时保持默认安全电量，禁止误触发断电
            level = 50;
            low_battery_streak = 0;
        }

        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }
        return true;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(SensecapWatcher);

// 定义静态成员变量
SensecapWatcher* SensecapWatcher::instance_ = nullptr;
