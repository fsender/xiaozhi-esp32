#include "wifi_board.h"
#include "display/lcd_display.h"
#include "esp_lcd_sh8601.h"

#include "codecs/es8311_audio_codec.h"
#include "application.h"
#include "button.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "config.h"
#include "power_save_timer.h"
#include "axp2101.h"
#include "i2c_device.h"
#include "assets.h"
#include "qmi8658.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include "esp_io_expander_tca9554.h"
#include "settings.h"

#include <esp_lcd_touch_ft5x06.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>
#include <esp_timer.h>
#include <cmath>
#include <esp_heap_caps.h>
#include <jpeg_decoder.h>

#define TAG "WaveshareEsp32s3TouchAMOLED1inch8"

class Pmic : public Axp2101 {
public:
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
        WriteReg(0x22, 0b110); // PWRON > OFFLEVEL as POWEROFF Source enable
        WriteReg(0x27, 0x10);  // hold 4s to power off

        // Disable All DCs but DC1
        WriteReg(0x80, 0x01);
        // Disable All LDOs
        WriteReg(0x90, 0x00);
        WriteReg(0x91, 0x00);

        // Set DC1 to 3.3V
        WriteReg(0x82, (3300 - 1500) / 100);

        // Set ALDO1 to 3.3V
        WriteReg(0x92, (3300 - 500) / 100);

        // Enable ALDO1(MIC)
        WriteReg(0x90, 0x01);
    
        WriteReg(0x64, 0x02); // CV charger voltage setting to 4.1V
        
        WriteReg(0x61, 0x02); // set Main battery precharge current to 50mA
        WriteReg(0x62, 0x08); // set Main battery charger current to 400mA ( 0x08-200mA, 0x09-300mA, 0x0A-400mA )
        WriteReg(0x63, 0x01); // set Main battery term charge current to 25mA
    }
};

#define LCD_OPCODE_WRITE_CMD (0x02ULL)
#define LCD_OPCODE_READ_CMD (0x03ULL)
#define LCD_OPCODE_WRITE_COLOR (0x32ULL)

static const sh8601_lcd_init_cmd_t vendor_specific_init[] = {
    {0x11, (uint8_t[]){0x00}, 0, 120},
    {0x44, (uint8_t[]){0x01, 0xD1}, 2, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x53, (uint8_t[]){0x20}, 1, 10},
    {0x2A, (uint8_t[]){0x00, 0x00, 0x01, 0x6F}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xBF}, 4, 0},
    {0x51, (uint8_t[]){0x00}, 1, 10},
    {0x29, (uint8_t[]){0x00}, 0, 10}
};

// Map emotion + device state to GIF asset
static const char* GetGifAssetForEmotion(const char* emotion) {
    if (!emotion) emotion = "neutral";

    // Check device state for more accurate GIF selection
    auto& app = Application::GetInstance();
    auto state = app.GetDeviceState();

    // State-based overrides when emotion is neutral/default
    if (strcmp(emotion, "neutral") == 0 || strcmp(emotion, "idle") == 0) {
        switch (state) {
            case kDeviceStateConnecting:  return "listen_start.gif";
            case kDeviceStateListening:   return "listening.gif";
            case kDeviceStateSpeaking:    return "speaking.gif";
            case kDeviceStateIdle:        return "idle.gif";
            default: break;
        }
    }

    // When server sends "happy" during speaking, show speaking animation
    if (strcmp(emotion, "happy") == 0 && state == kDeviceStateSpeaking) {
        return "speaking.gif";
    }

    // Direct emotion mapping
    struct EmotionMap { const char* name; const char* gif; };
    static const EmotionMap kMap[] = {
        {"neutral",    "idle.gif"},
        {"idle",       "idle.gif"},
        {"listening",  "listening.gif"},
        {"thinking",   "thinking.gif"},
        {"speaking",   "speaking.gif"},
        {"sleepy",     "sleep.gif"},
        {"happy",      "tap.gif"},
        {"surprised",  "shake1.gif"},
        {"angry",      "shake2.gif"},
        {"sad",        "listen_end.gif"},
        {"crying",     "listen_end.gif"},
        {"confused",   "idle_to_think.gif"},
        {"embarrassed","listen_end.gif"},
        {"funny",      "tap.gif"},
        {"laughing",   "tap.gif"},
        {"loving",     "tap.gif"},
        {"kissy",      "tap.gif"},
        {"winking",    "tap.gif"},
        {"cool",       "tap.gif"},
        {"confident",  "tap.gif"},
        {"delicious",  "tap.gif"},
        {"relaxed",    "idle.gif"},
        {"shocked",    "shake1.gif"},
        {"silly",      "tap.gif"},
    };
    for (const auto& entry : kMap) {
        if (strcmp(emotion, entry.name) == 0) {
            return entry.gif;
        }
    }
    return "idle.gif";
}

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
        : SpiLcdDisplay(io_handle, panel_handle,
                    width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
    }

    lv_obj_t* bg_img_ = nullptr;
    uint8_t* bg_pixels_ = nullptr;

    // GIF queue system
    struct GifStep { std::string name; bool loop; };
    std::vector<GifStep> gif_queue_;
    bool gif_busy_ = false;
    lv_timer_t* gif_watchdog_timer_ = nullptr;
    std::string current_state_ = "idle";
    bool gesture_active_ = false;  // Prevent tap during gesture

    virtual void SetupUI() override {
        SpiLcdDisplay::SetupUI();
        DisplayLockGuard lock(this);

        auto screen = lv_screen_active();

        // Make container transparent, non-scrollable, and enable event bubbling
        if (container_) {
            lv_obj_set_style_bg_opa(container_, LV_OPA_TRANSP, 0);
            lv_obj_remove_flag(container_, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_scrollbar_mode(container_, LV_SCROLLBAR_MODE_OFF);
            lv_obj_add_flag(container_, LV_OBJ_FLAG_EVENT_BUBBLE);
        }

        // Load and decode bg_large.jpg using ESP ROM JPEG decoder
        void* bg_ptr = nullptr;
        size_t bg_size = 0;
        if (Assets::GetInstance().GetAssetData("bg_large.jpg", bg_ptr, bg_size)) {
            ESP_LOGW("CustomDisplay", "bg_large.jpg found, size=%u bytes, decoding...", (unsigned)bg_size);

            // Allocate output buffer for RGB565 decoded image (368x448x2)
            bg_pixels_ = (uint8_t*)heap_caps_malloc(368 * 448 * 2, MALLOC_CAP_SPIRAM);
            if (bg_pixels_) {
                esp_jpeg_image_cfg_t cfg = {};
                cfg.indata = static_cast<uint8_t*>(bg_ptr);
                cfg.indata_size = bg_size;
                cfg.outbuf = bg_pixels_;
                cfg.outbuf_size = 368 * 448 * 2;
                cfg.out_format = JPEG_IMAGE_FORMAT_RGB565;
                cfg.out_scale = JPEG_IMAGE_SCALE_0;
                cfg.flags.swap_color_bytes = 0;

                esp_jpeg_image_output_t img_info = {};
                esp_err_t ret = esp_jpeg_decode(&cfg, &img_info);
                if (ret == ESP_OK) {
                    ESP_LOGW("CustomDisplay", "JPEG decoded: %dx%d, %u bytes", img_info.width, img_info.height, (unsigned)img_info.output_len);

                    static lv_img_dsc_t bg_dsc = {};
                    bg_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
                    bg_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
                    bg_dsc.header.w = img_info.width;
                    bg_dsc.header.h = img_info.height;
                    bg_dsc.header.stride = img_info.width * 2;
                    bg_dsc.data = bg_pixels_;
                    bg_dsc.data_size = img_info.output_len;

                    bg_img_ = lv_image_create(screen);
                    lv_image_set_src(bg_img_, &bg_dsc);
                    lv_obj_set_size(bg_img_, LV_HOR_RES, LV_VER_RES);
                    lv_obj_align(bg_img_, LV_ALIGN_CENTER, 0, 0);
                    lv_obj_move_to_index(bg_img_, 0);
                    ESP_LOGW("CustomDisplay", "Background image displayed successfully!");
                } else {
                    ESP_LOGE("CustomDisplay", "JPEG decode failed: %d", ret);
                    lv_obj_set_style_bg_color(screen, lv_color_hex(0x1A3A5C), 0);
                    heap_caps_free(bg_pixels_);
                    bg_pixels_ = nullptr;
                }
            } else {
                ESP_LOGE("CustomDisplay", "Failed to allocate PSRAM for bg image");
                lv_obj_set_style_bg_color(screen, lv_color_hex(0x1A3A5C), 0);
            }
        } else {
            ESP_LOGE("CustomDisplay", "bg_large.jpg NOT found in assets partition!");
            lv_obj_set_style_bg_color(screen, lv_color_hex(0x1A3A5C), 0);
        }

        ApplyWhiteTheme();

        // Initialize GIF watchdog timer for queue system
        InitGifWatchdog();

        // Register touch gesture handlers on the GIF image area
        if (emoji_image_) {
            lv_obj_add_flag(emoji_image_, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(emoji_image_, OnEmojiClicked, LV_EVENT_CLICKED, NULL);
            ESP_LOGW("CustomDisplay", "Tap handler on emoji_image_");
        }
        if (emoji_label_) {
            lv_obj_add_flag(emoji_label_, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(emoji_label_, OnEmojiClicked, LV_EVENT_CLICKED, NULL);
        }
        // Swipe gesture on screen
        lv_obj_add_event_cb(screen, OnGesture, LV_EVENT_GESTURE, NULL);
        ESP_LOGW("CustomDisplay", "All gesture handlers registered");
    }

    static void OnGesture(lv_event_t* e) {
        static uint32_t last_gesture_time = 0;
        uint32_t now = esp_timer_get_time() / 1000;
        if (now - last_gesture_time < 2000) return;
        last_gesture_time = now;

        auto& app = Application::GetInstance();
        auto state = app.GetDeviceState();

        // Only respond to swipe in idle state
        if (state != kDeviceStateIdle) {
            ESP_LOGW("Gesture", "Swipe ignored: state=%d (not idle)", state);
            return;
        }

        lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
        auto* display = dynamic_cast<CustomLcdDisplay*>(Board::GetInstance().GetDisplay());
        if (!display) return;

        display->gesture_active_ = true;

        switch (dir) {
            case LV_DIR_LEFT:
            case LV_DIR_RIGHT:
                ESP_LOGW("Gesture", ">>> SWIPE HORIZONTAL -> shake2 -> listen -> listening");
                display->WakeWithTrigger("swipe_horizontal");
                break;
            case LV_DIR_TOP:
            case LV_DIR_BOTTOM:
                ESP_LOGW("Gesture", ">>> SWIPE VERTICAL -> shake1 -> listen -> listening");
                display->WakeWithTrigger("swipe_vertical");
                break;
            default:
                break;
        }
        app.StartListening();
    }

    static void OnEmojiClicked(lv_event_t* e) {
        auto* display = dynamic_cast<CustomLcdDisplay*>(Board::GetInstance().GetDisplay());
        // Skip tap if gesture just fired (within 500ms)
        if (display && display->gesture_active_) {
            ESP_LOGW("Gesture", "TAP skipped: gesture active");
            display->gesture_active_ = false;
            return;
        }

        static uint32_t last_tap_time = 0;
        uint32_t now = esp_timer_get_time() / 1000;
        if (now - last_tap_time < 2000) return;
        last_tap_time = now;

        auto& app = Application::GetInstance();
        auto state = app.GetDeviceState();

        if (state == kDeviceStateIdle) {
            // Idle: tap to wake with GIF chain
            if (display) {
                ESP_LOGW("Gesture", ">>> TAP (idle) -> tap.gif -> listen_start -> listening");
                display->WakeWithTrigger("tap");
            }
            app.ToggleChatState();
        } else if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
            // Listening/Speaking: tap to interrupt
            ESP_LOGW("Gesture", ">>> TAP (listening/speaking) -> interrupt -> idle");
            if (display && display->gif_busy_) {
                // Currently playing a chain (shake/listen_start) - replace queue with idle
                ESP_LOGW("Gesture", "Chain active, replacing queue with idle.gif");
                display->gif_queue_.clear();
                display->gif_queue_.push_back({"idle.gif", true});
                display->current_state_ = "idle";
            }
            app.ToggleChatState();
        } else {
            ESP_LOGW("Gesture", "TAP ignored: state=%d", state);
        }
    }

    // --- GIF Queue System ---
    void InitGifWatchdog() {
        // Use LVGL timer - runs in LVGL task context, safe for lv_image_set_src
        gif_watchdog_timer_ = lv_timer_create([](lv_timer_t* timer) {
            auto* self = static_cast<CustomLcdDisplay*>(lv_timer_get_user_data(timer));
            if (self->gif_controller_ && !self->gif_controller_->IsPlaying() && !self->gif_queue_.empty()) {
                self->gif_queue_.erase(self->gif_queue_.begin());
                self->PlayNextInQueue();
            }
        }, 200, this);
        lv_timer_pause(gif_watchdog_timer_);  // Start paused
    }

    void PlayNextInQueue() {
        if (gif_queue_.empty()) {
            gif_busy_ = false;
            if (gif_watchdog_timer_) lv_timer_pause(gif_watchdog_timer_);
            ESP_LOGW("GifDisplay", "Queue empty, gif_busy_=false");
            // Apply current state GIF (in case it was skipped during busy)
            ApplyCurrentStateGif();
            return;
        }
        auto& step = gif_queue_.front();
        ESP_LOGW("GifDisplay", "Queue next: %s (%s)", step.name.c_str(), step.loop ? "loop" : "once");

        // If this is the final looping step, clear gif_busy_
        if (step.loop && gif_queue_.size() == 1) {
            gif_busy_ = false;
            ESP_LOGW("GifDisplay", "Final loop step, gif_busy_=false");
            // Don't apply current state here - let the looping GIF play
        }

        LoadAndPlayGif(step.name.c_str(), step.loop);

        if (!step.loop && gif_watchdog_timer_) {
            lv_timer_resume(gif_watchdog_timer_);
            lv_timer_reset(gif_watchdog_timer_);
        } else if (step.loop && gif_watchdog_timer_) {
            lv_timer_pause(gif_watchdog_timer_);
        }
    }

    void ApplyCurrentStateGif() {
        const char* target_gif = GetStateTargetGif();
        std::string target_state;
        if (strstr(target_gif, "idle")) target_state = "idle";
        else if (strstr(target_gif, "listening")) target_state = "listening";
        else if (strstr(target_gif, "speaking")) target_state = "speaking";
        else if (strstr(target_gif, "thinking")) target_state = "thinking";
        else target_state = "idle";

        if (current_state_ != target_state) {
            ESP_LOGW("GifDisplay", "State mismatch after busy: %s -> %s, applying transition",
                     current_state_.c_str(), target_state.c_str());
            const char* trans_gif = GetTransitionGif(current_state_.c_str(), target_state.c_str());
            if (trans_gif) {
                EnqueueGifChain({{trans_gif, false}, {target_gif, true}}, false);
            } else {
                EnqueueGifChain({{target_gif, true}}, false);
            }
            current_state_ = target_state;
        } else if (gif_queue_.empty()) {
            // Same state but queue is empty - just play the target GIF
            ESP_LOGW("GifDisplay", "Applying state GIF: %s", target_gif);
            LoadAndPlayGif(target_gif, true);
        }
    }

    void EnqueueGifChain(std::vector<GifStep> steps, bool high_priority) {
        if (gif_busy_ && !high_priority) {
            ESP_LOGW("GifDisplay", "Busy, ignoring normal priority chain");
            return;
        }
        if (high_priority) {
            if (gif_controller_) {
                gif_controller_->Stop();
                gif_controller_.reset();
            }
            if (gif_watchdog_timer_) lv_timer_pause(gif_watchdog_timer_);
        }
        // ALL chains set gif_busy_ to prevent interruption during playback
        gif_busy_ = true;
        gif_queue_ = steps;
        ESP_LOGW("GifDisplay", "Enqueued %d steps, priority=%s, gif_busy_=true",
                 (int)steps.size(), high_priority ? "HIGH" : "normal");
        PlayNextInQueue();
    }

    // Wake-up trigger: high priority chain
    void WakeWithTrigger(const char* trigger) {
        std::string initial_gif;
        if (strcmp(trigger, "tap") == 0 || strcmp(trigger, "button") == 0) {
            initial_gif = "tap.gif";
        } else if (strcmp(trigger, "swipe_vertical") == 0) {
            initial_gif = "shake1.gif";
        } else {
            initial_gif = "shake2.gif";
        }
        ESP_LOGW("GifDisplay", "WakeWithTrigger('%s') -> %s", trigger, initial_gif.c_str());
        EnqueueGifChain({
            {initial_gif, false},
            {"listen_start.gif", false},
            {"listening.gif", true},
        }, true);
        current_state_ = "listening";
    }

    // --- End GIF Queue System ---

    virtual void SetTheme(Theme* theme) override {
        SpiLcdDisplay::SetTheme(theme);
        // Reapply customizations after parent SetTheme overwrites them
        if (container_) {
            lv_obj_set_style_bg_opa(container_, LV_OPA_TRANSP, 0);
        }
        if (bg_img_) {
            lv_obj_move_to_index(bg_img_, 0);  // Ensure bg stays at back
        }
        ApplyWhiteTheme();
    }

    void ApplyWhiteTheme() {
        if (!setup_ui_called_) return;
        DisplayLockGuard lock(this);

        lv_color_t white = lv_color_hex(0xFFFFFF);
        lv_color_t chat_text = lv_color_hex(0x0F6DB7);

        // Hide top bar (WiFi, battery, mute icons)
        if (top_bar_) lv_obj_add_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);

        // Status bar: white text for time, no background
        if (status_label_) {
            lv_obj_set_style_text_color(status_label_, white, 0);
            lv_obj_set_style_bg_opa(status_bar_, LV_OPA_TRANSP, 0);
        }
        if (notification_label_) lv_obj_set_style_text_color(notification_label_, white, 0);

        // Content area: white rounded rectangle, 85% opacity, blue text
        if (content_) {
            lv_obj_set_style_bg_color(content_, white, 0);
            lv_obj_set_style_bg_opa(content_, 216, 0);  // 85% opacity
            lv_obj_set_style_radius(content_, 16, 0);
            lv_obj_set_style_border_width(content_, 0, 0);
            lv_obj_set_style_text_color(content_, chat_text, 0);
        }
        if (chat_message_label_) {
            lv_obj_set_style_text_color(chat_message_label_, chat_text, 0);
        }
    }

    bool LoadAndPlayGif(const char* gif_name, bool loop) {
        void* ptr = nullptr;
        size_t size = 0;
        if (!Assets::GetInstance().GetAssetData(gif_name, ptr, size)) {
            ESP_LOGE("GifDisplay", "Asset not found: %s", gif_name);
            return false;
        }

        if (gif_controller_) {
            gif_controller_->Stop();
            gif_controller_.reset();
        }

        lv_img_dsc_t tmp_dsc = {};
        tmp_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
        tmp_dsc.header.cf = LV_COLOR_FORMAT_ARGB8888;
        tmp_dsc.data = static_cast<const uint8_t*>(ptr);
        tmp_dsc.data_size = size;
        gif_controller_ = std::make_unique<LvglGif>(&tmp_dsc);

        if (!gif_controller_ || !gif_controller_->IsLoaded()) {
            ESP_LOGE("GifDisplay", "GIF decode failed: %s", gif_name);
            gif_controller_.reset();
            return false;
        }

        if (!loop) {
            gif_controller_->SetLoopCount(1);
        }

        gif_controller_->SetFrameCallback(
            [this]() { lv_image_set_src(emoji_image_, gif_controller_->image_dsc()); });

        // Don't call lv_image_set_src here - let the GIF timer handle all frame updates
        // to avoid blocking the main task with LVGL rendering
        gif_controller_->Start();

        if (emoji_label_) lv_obj_add_flag(emoji_label_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);

        ESP_LOGW("GifDisplay", ">>> PLAYING: %s (%s, %dx%d)", gif_name,
                 loop ? "loop" : "once", gif_controller_->width(), gif_controller_->height());
        return true;
    }

    // Get state-based target GIF
    const char* GetStateTargetGif() {
        auto state = Application::GetInstance().GetDeviceState();
        switch (state) {
            case kDeviceStateConnecting:  return "listen_start.gif";
            case kDeviceStateListening:   return "listening.gif";
            case kDeviceStateSpeaking:    return "speaking.gif";
            default:                      return "idle.gif";
        }
    }

    // Get transition GIF for state change (only during conversation)
    const char* GetTransitionGif(const char* from_state, const char* to_state) {
        if (!from_state || !to_state) return nullptr;
        std::string from(from_state), to(to_state);

        if (from == "listening" && to == "speaking") return "listen_to_speak.gif";
        if (from == "speaking" && to == "listening") return "listen_start.gif";
        if (from == "listening" && to == "thinking") return "listen_to_think.gif";
        if (from == "listening" && to == "idle")      return "listen_end.gif";
        return nullptr;
    }

    virtual void SetEmotion(const char* emotion) override {
        if (!emotion) emotion = "neutral";
        if (!emoji_image_) {
            SpiLcdDisplay::SetEmotion(emotion);
            return;
        }

        const char* target_gif = GetStateTargetGif();
        std::string prev_state = current_state_;
        std::string target_state;
        if (strstr(target_gif, "idle")) target_state = "idle";
        else if (strstr(target_gif, "listening")) target_state = "listening";
        else if (strstr(target_gif, "speaking")) target_state = "speaking";
        else if (strstr(target_gif, "thinking")) target_state = "thinking";
        else target_state = "idle";

        ESP_LOGW("GifDisplay", "SetEmotion('%s') state: %s -> %s, gif: %s, busy=%d",
                 emotion, prev_state.c_str(), target_state.c_str(), target_gif, gif_busy_);

        // Always update state tracking
        current_state_ = target_state;

        // Skip GIF changes if high-priority chain is active
        if (gif_busy_) {
            ESP_LOGW("GifDisplay", "gif_busy_, will apply when chain finishes");
            return;
        }

        // Check for state transition (only during conversation)
        const char* trans_gif = nullptr;
        if (prev_state != target_state) {
            trans_gif = GetTransitionGif(prev_state.c_str(), target_state.c_str());
        }

        DisplayLockGuard lock(this);

        if (trans_gif) {
            ESP_LOGW("GifDisplay", "State transition: %s -> %s -> %s",
                     prev_state.c_str(), trans_gif, target_gif);
            EnqueueGifChain({
                {trans_gif, false},
                {target_gif, true},
            }, false);
        } else {
            EnqueueGifChain({{target_gif, true}}, false);
        }
    }
};

class CustomBacklight : public Backlight {
public:
    CustomBacklight(esp_lcd_panel_io_handle_t panel_io) : Backlight(), panel_io_(panel_io) {}

protected:
    esp_lcd_panel_io_handle_t panel_io_;

    virtual void SetBrightnessImpl(uint8_t brightness) override {
        auto display = Board::GetInstance().GetDisplay();
        DisplayLockGuard lock(display);
        uint8_t data[1] = {((uint8_t)((255 * brightness) / 100))};
        int lcd_cmd = 0x51;
        lcd_cmd &= 0xff;
        lcd_cmd <<= 8;
        lcd_cmd |= LCD_OPCODE_WRITE_CMD << 24;
        esp_lcd_panel_io_tx_param(panel_io_, lcd_cmd, &data, sizeof(data));
    }
};

class WaveshareEsp32s3TouchAMOLED1inch8 : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Pmic* pmic_ = nullptr;
    Button boot_button_;
    CustomLcdDisplay* display_;
    CustomBacklight* backlight_;
    esp_io_expander_handle_t io_expander = NULL;
    PowerSaveTimer* power_save_timer_;
    int last_discharging_ = -1;  // -1 = unknown, 0 = charging (always-on), 1 = discharging

    // Standalone dim-only timer used while charging. PowerSaveTimer is
    // disabled on the charger so wake word detection isn't disturbed by its
    // sleep callbacks; this lightweight timer applies just the visual dim
    // (low brightness + sleepy emoji) on its own.
    esp_timer_handle_t dim_timer_ = nullptr;
    int dim_ticks_ = 0;
    bool dimmed_ = false;
    bool dim_timer_running_ = false;
    static constexpr int kDimSeconds = 60;

    // QMI8658 IMU
    Qmi8658* imu_ = nullptr;
    TaskHandle_t imu_task_handle_ = nullptr;
    volatile bool shake_pending_ = false;

    // Touch gesture state - handled by LVGL
    // esp_lcd_touch_handle_t touch_handle_ = nullptr;

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(20);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->OnShutdownRequest([this]() {
            pmic_->PowerOff();
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeDimTimer() {
        esp_timer_create_args_t args = {
            .callback = [](void* arg) {
                auto self = static_cast<WaveshareEsp32s3TouchAMOLED1inch8*>(arg);
                self->DimTick();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "dim_tick",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&args, &dim_timer_));
    }

    void ApplyDim(bool on) {
        Application::GetInstance().Schedule([this, on]() {
            if (on) {
                ESP_LOGI(TAG, "Dim mode (always-on)");
                GetDisplay()->SetPowerSaveMode(true);
                GetBacklight()->SetBrightness(20);
            } else {
                GetDisplay()->SetPowerSaveMode(false);
                GetBacklight()->RestoreBrightness();
            }
        });
    }

    void DimTick() {
        if (!Application::GetInstance().CanEnterSleepMode()) {
            // Activity (conversation, button) — restore display and reset.
            dim_ticks_ = 0;
            if (dimmed_) {
                dimmed_ = false;
                ApplyDim(false);
            }
            return;
        }
        dim_ticks_++;
        if (dim_ticks_ >= kDimSeconds && !dimmed_) {
            dimmed_ = true;
            ApplyDim(true);
        }
    }

    void StartDimTimer() {
        if (dim_timer_running_) return;
        dim_ticks_ = 0;
        esp_timer_start_periodic(dim_timer_, 1000000);
        dim_timer_running_ = true;
    }

    void StopDimTimer() {
        if (!dim_timer_running_) return;
        esp_timer_stop(dim_timer_);
        dim_timer_running_ = false;
        if (dimmed_) {
            dimmed_ = false;
            ApplyDim(false);
        }
    }

    void InitializeCodecI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
    }

    void InitializeTca9554(void) {
        esp_err_t ret = esp_io_expander_new_i2c_tca9554(codec_i2c_bus_, I2C_ADDRESS, &io_expander);
        if(ret != ESP_OK)
            ESP_LOGE(TAG, "TCA9554 create returned error");
        ret = esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1 |IO_EXPANDER_PIN_NUM_2, IO_EXPANDER_OUTPUT);
        ret |= esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_4, IO_EXPANDER_INPUT);
        ESP_ERROR_CHECK(ret);
        ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1|IO_EXPANDER_PIN_NUM_2, 1);
        ESP_ERROR_CHECK(ret);
        vTaskDelay(pdMS_TO_TICKS(100));
        ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1|IO_EXPANDER_PIN_NUM_2, 0);
        ESP_ERROR_CHECK(ret);
        vTaskDelay(pdMS_TO_TICKS(300));
        ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1|IO_EXPANDER_PIN_NUM_2, 1);
        ESP_ERROR_CHECK(ret);
    }

    void InitializeAxp2101() {
        ESP_LOGI(TAG, "Init AXP2101");
        pmic_ = new Pmic(codec_i2c_bus_, 0x34);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.sclk_io_num = GPIO_NUM_11;
        buscfg.data0_io_num = GPIO_NUM_4;
        buscfg.data1_io_num = GPIO_NUM_5;
        buscfg.data2_io_num = GPIO_NUM_6;
        buscfg.data3_io_num = GPIO_NUM_7;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        buscfg.flags = SPICOMMON_BUSFLAG_QUAD;
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeSH8601Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS;
        io_config.dc_gpio_num = GPIO_NUM_NC;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 32;
        io_config.lcd_param_bits = 8;
        io_config.flags.quad_mode = true;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        const sh8601_vendor_config_t vendor_config = {
            .init_cmds = &vendor_specific_init[0],
            .init_cmds_size = sizeof(vendor_specific_init) / sizeof(sh8601_lcd_init_cmd_t),
            .flags ={
                .use_qspi_interface = 1,
            }
        };

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.flags.reset_active_high = 1,
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        panel_config.vendor_config = (void *)&vendor_config;
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, false);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(panel, true);
        display_ = new CustomLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
        backlight_ = new CustomBacklight(panel_io);
        backlight_->RestoreBrightness();
    }

    void InitializeTouch()
    {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH,
            .y_max = DISPLAY_HEIGHT,
            .rst_gpio_num = GPIO_NUM_NC,
            .int_gpio_num = GPIO_NUM_21,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 0,
                .mirror_x = 0,
                .mirror_y = 0,
            },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = {
            .dev_addr = ESP_LCD_TOUCH_IO_I2C_FT5x06_ADDRESS,
            .control_phase_bytes = 1,
            .dc_bit_offset = 0,
            .lcd_cmd_bits = 8,
            .flags =
            {
                .disable_control_phase = 1,
            }
        };
        tp_io_config.scl_speed_hz = 400 * 1000;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(codec_i2c_bus_, &tp_io_config, &tp_io_handle));
        ESP_LOGI(TAG, "Initialize touch controller");
        ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp));
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(), 
            .handle = tp,
        };
        lvgl_port_add_touch(&touch_cfg);
        ESP_LOGI(TAG, "Touch panel initialized successfully");
    }

    void InitializeQmi8658() {
        // QMI8658 shares I2C bus with codec
        // Try both possible addresses: 0x6A (SA0=LOW) and 0x6B (SA0=HIGH)
        ESP_LOGI(TAG, "Initializing QMI8658 IMU on shared I2C bus");

        // Try 0x6B first (LCD-1.85B uses this)
        imu_ = new Qmi8658(codec_i2c_bus_, 0x6B);
        if (imu_->Init()) {
            ESP_LOGI(TAG, "QMI8658 IMU initialized at address 0x6B");
            StartImuTask();
            return;
        }
        delete imu_;
        imu_ = nullptr;

        // Try 0x6A (default)
        imu_ = new Qmi8658(codec_i2c_bus_, 0x6A);
        if (imu_->Init()) {
            ESP_LOGI(TAG, "QMI8658 IMU initialized at address 0x6A");
            StartImuTask();
            return;
        }
        delete imu_;
        imu_ = nullptr;

        ESP_LOGW(TAG, "QMI8658 init failed at both addresses (0x6A and 0x6B)");
    }

    void StartImuTask() {
        xTaskCreatePinnedToCore(ImuTask, "imu_task", 4 * 1024, this, 5, &imu_task_handle_, 1);
        ESP_LOGI(TAG, "IMU motion detection task started");
    }

    static void ImuTask(void* arg) {
        auto* self = static_cast<WaveshareEsp32s3TouchAMOLED1inch8*>(arg);
        if (!self || !self->imu_) {
            vTaskDelete(NULL);
            return;
        }

        // Shake detection parameters
        constexpr float kShakeAccelThreshold = 2.5f;  // g-force delta for shake
        constexpr int64_t kShakeCooldownMs = 2000;     // Cooldown between shakes

        // Tilt detection parameters
        constexpr float kTiltThreshold = 0.4f;         // g-force for tilt detection
        constexpr int64_t kTiltCooldownMs = 1500;      // Cooldown between tilts

        float prev_ax = 0, prev_ay = 0, prev_az = 0;
        bool has_prev = false;
        int64_t last_shake_ms = 0;
        int64_t last_tilt_ms = 0;

        while (true) {
            float ax, ay, az;
            if (self->imu_->ReadAccel(ax, ay, az)) {
                int64_t now_ms = esp_timer_get_time() / 1000;

                if (has_prev) {
                    // Shake detection: rapid acceleration change
                    float dx = fabsf(ax - prev_ax);
                    float dy = fabsf(ay - prev_ay);
                    float dz = fabsf(az - prev_az);
                    float shake_magnitude = dx + dy + dz;

                    if (shake_magnitude > kShakeAccelThreshold &&
                        (now_ms - last_shake_ms) > kShakeCooldownMs) {
                        last_shake_ms = now_ms;
                        self->OnShakeDetected();
                    }

                    // Tilt detection: sustained orientation change
                    float tilt_x = fabsf(ax);
                    float tilt_y = fabsf(ay);

                    if ((tilt_x > kTiltThreshold || tilt_y > kTiltThreshold) &&
                        (now_ms - last_tilt_ms) > kTiltCooldownMs) {
                        last_tilt_ms = now_ms;
                        self->OnTiltDetected(ax, ay);
                    }
                }
                prev_ax = ax;
                prev_ay = ay;
                prev_az = az;
                has_prev = true;
            }
            vTaskDelay(pdMS_TO_TICKS(80));
        }
    }

    void OnShakeDetected() {
        auto& app = Application::GetInstance();
        auto state = app.GetDeviceState();

        if (state != kDeviceStateIdle || shake_pending_) {
            return;
        }
        shake_pending_ = true;

        app.Schedule([this]() {
            shake_pending_ = false;
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() != kDeviceStateIdle) {
                return;
            }
            app.StartListening();
        });
    }

    void OnTiltDetected(float ax, float ay) {
        auto& app = Application::GetInstance();
        auto state = app.GetDeviceState();

        if (state != kDeviceStateIdle) {
            return;
        }

        // Could trigger different animations based on tilt direction
        // For now, no action - implement specific animations as needed
    }

    void InitializeGestureHandlers() {
        // Gesture handlers registered in CustomLcdDisplay::SetupUI()
        // where emoji_image_ and emoji_label_ are accessible
        ESP_LOGW(TAG, "Gesture handlers deferred to display SetupUI");
    }

    // 初始化工具
    void InitializeTools() {
        auto &mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.system.reconfigure_wifi",
            "End this conversation and enter WiFi configuration mode.\n"
            "**CAUTION** You must ask the user to confirm this action.",
            PropertyList(), [this](const PropertyList& properties) {
                EnterWifiConfigMode();
                return true;
            });
    }

public:
    WaveshareEsp32s3TouchAMOLED1inch8() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializePowerSaveTimer();
        InitializeDimTimer();
        InitializeCodecI2c();
        InitializeTca9554();
        InitializeAxp2101();
        InitializeQmi8658();
        InitializeSpi();
        InitializeSH8601Display();
        InitializeTouch();
        InitializeButtons();
        InitializeTools();
        InitializeGestureHandlers();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        return backlight_;
    }

    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override {
        charging = pmic_->IsCharging();
        discharging = pmic_->IsDischarging();
        int discharging_state = discharging ? 1 : 0;
        if (discharging_state != last_discharging_) {
            // On charger: disable PowerSaveTimer so its sleep-mode callbacks
            // (which on this board appear to disrupt the AFE / audio pipeline)
            // never fire. Run the standalone dim timer instead for the
            // brightness + sleepy-emoji UX. On battery: reverse — disable dim
            // timer, re-enable PowerSaveTimer (60s sleep, 5min auto-shutdown).
            power_save_timer_->SetEnabled(discharging);
            if (discharging) {
                StopDimTimer();
                ESP_LOGI(TAG, "Always-on disabled (battery)");
            } else {
                StartDimTimer();
                ESP_LOGI(TAG, "Always-on enabled (charging)");
            }
            last_discharging_ = discharging_state;
        }

        level = pmic_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(WaveshareEsp32s3TouchAMOLED1inch8);
