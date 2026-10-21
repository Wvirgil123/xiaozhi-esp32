#include "remote_display.h"
#include "board.h"
#include "settings.h"

#include <esp_log.h>
#include <web_socket.h>
#include <cstring>
#include <arpa/inet.h>
#include <cJSON.h>
#include <mbedtls/base64.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <algorithm>
#include <limits>

#define TAG "RemoteDisplay"

// Parse ws://host:port/path → host, port
static bool ParseWsUrl(const std::string& url, std::string& host, uint16_t& port) {
    auto pos = url.find("://");
    if (pos == std::string::npos) return false;
    std::string rest = url.substr(pos + 3);
    auto slash = rest.find('/');
    if (slash != std::string::npos) rest = rest.substr(0, slash);
    auto colon = rest.rfind(':');
    if (colon != std::string::npos) {
        host = rest.substr(0, colon);
        port = (uint16_t)atoi(rest.substr(colon + 1).c_str());
    } else {
        host = rest;
        port = 80;
    }
    return !host.empty();
}

// Quick non-blocking TCP connect to check reachability (avoids 18s LWIP SYN timeout)
static bool IsTcpReachable(const char* host, uint16_t port, int timeout_ms) {
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) return false;

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) return false;

    // Set non-blocking
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    int ret = connect(sock, (struct sockaddr*)&addr, sizeof(addr));
    if (ret == 0) { close(sock); return true; }
    if (errno != EINPROGRESS) { close(sock); return false; }

    // Wait with select()
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(sock, &wfds);
    struct timeval tv = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
    ret = select(sock + 1, nullptr, &wfds, nullptr, &tv);

    bool ok = false;
    if (ret > 0) {
        int err = 0;
        socklen_t len = sizeof(err);
        getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &len);
        ok = (err == 0);
    }
    close(sock);
    return ok;
}

// NVS 存储 key
#define NVS_NAMESPACE "remote_disp"
#define NVS_KEY_ENABLED "enabled"
#define NVS_KEY_URL "url"
#define NVS_KEY_TIMEOUT "timeout"

// Opus 音频帧头结构 (little-endian)
struct __attribute__((packed)) OpusAudioHeader {
    uint8_t type;            // MSG_TYPE_AUDIO_FRAME (0x02)
    uint16_t sample_rate;    // 采样率
    uint8_t frame_duration;  // 帧时长 (ms)
    // 后跟 Opus 编码数据
};

RemoteDisplay* RemoteDisplay::GetInstance() {
    static RemoteDisplay instance;
    return &instance;
}

RemoteDisplayConfig RemoteDisplay::LoadConfig() {
    RemoteDisplayConfig config;
    Settings settings(NVS_NAMESPACE, false);
    config.enabled = settings.GetBool(NVS_KEY_ENABLED, false);
    config.server_url = settings.GetString(NVS_KEY_URL, "");
    config.timeout_ms = settings.GetInt(NVS_KEY_TIMEOUT, 4000);
    return config;
}

void RemoteDisplay::SaveConfig(const RemoteDisplayConfig& config) {
    Settings settings(NVS_NAMESPACE, true);
    settings.SetBool(NVS_KEY_ENABLED, config.enabled);
    settings.SetString(NVS_KEY_URL, config.server_url);
    settings.SetInt(NVS_KEY_TIMEOUT, config.timeout_ms);
    ESP_LOGI(TAG, "Config saved: enabled=%d, url=%s, timeout=%d",
             config.enabled, config.server_url.c_str(), config.timeout_ms);
}

bool RemoteDisplay::StartWithConfig() {
    auto config = LoadConfig();
    if (!config.enabled) {
        ESP_LOGI(TAG, "Remote display disabled in config");
        return false;
    }
    if (config.server_url.empty()) {
        ESP_LOGW(TAG, "Remote display URL not configured");
        return false;
    }
    return Start(config.server_url, config.timeout_ms);
}

RemoteDisplay::~RemoteDisplay() {
    Stop();
    audio_shutdown_ = true;
    if (audio_task_) {
        xTaskNotifyGive(audio_task_);
        if (audio_task_stopped_) {
            xSemaphoreTake(audio_task_stopped_, portMAX_DELAY);
        }
        audio_task_ = nullptr;
    }
    if (audio_task_stopped_) {
        vSemaphoreDelete(audio_task_stopped_);
        audio_task_stopped_ = nullptr;
    }
    if (reconnect_task_) {
        vTaskDelete(reconnect_task_);
        reconnect_task_ = nullptr;
    }
    if (cleanup_timer_) {
        xTimerDelete(cleanup_timer_, portMAX_DELAY);
        cleanup_timer_ = nullptr;
    }
}

void RemoteDisplay::OnCleanupTimer(TimerHandle_t timer) {
    auto* self = static_cast<RemoteDisplay*>(pvTimerGetTimerID(timer));
    // Timer service callbacks must never run blocking network cleanup. Wake the
    // dedicated worker, which serializes destruction with reconnect attempts.
    self->cleanup_due_ = true;
    self->ScheduleReconnect(self->lifecycle_epoch_.load());
}

void RemoteDisplay::RetireWebSocketLocked(bool schedule_cleanup) {
    if (!websocket_) {
        return;
    }

    // Never destroy a WebSocket here: this can race an OnDisconnected callback
    // that is still unwinding through WebSocket::OnTcpData(). Invalidate its
    // callbacks, close it, and keep the object alive until the delayed worker
    // cleanup runs.
    connect_generation_.fetch_add(1);
    {
        std::lock_guard<std::mutex> send_lock(send_mutex_);
        websocket_->Close();
        retired_websockets_.push_back({std::move(websocket_), esp_timer_get_time()});
    }

    if (schedule_cleanup && cleanup_timer_) {
        // Keep the timer anchored to the oldest retired socket. Resetting it on
        // every reconnect can postpone cleanup forever during a reconnect storm.
        if (!xTimerIsTimerActive(cleanup_timer_) &&
            xTimerStart(cleanup_timer_, 0) != pdPASS) {
            ESP_LOGW(TAG, "Failed to schedule delayed WebSocket cleanup");
            // Keep the socket alive rather than risk freeing it while a callback
            // is unwinding. A later reconnect/cleanup timer can reclaim it.
            ScheduleReconnect(lifecycle_epoch_.load());
        }
    } else if (schedule_cleanup) {
        // Without the safety delay, retaining the socket is preferable to a
        // callback use-after-free. Wake retry handling without marking it due.
        ScheduleReconnect(lifecycle_epoch_.load());
    }
}

void RemoteDisplay::CleanupDisconnectedSocketLocked() {
    if (!cleanup_due_.exchange(false)) {
        return;
    }

    if (!running_ && !connected_ && websocket_) {
        // The timer delay guarantees the disconnect callback chain has returned.
        RetireWebSocketLocked(false);
        current_server_url_.clear();
    }

    if (!retired_websockets_.empty()) {
        constexpr int64_t kCleanupDelayUs = 1000 * 1000;
        const int64_t now_us = esp_timer_get_time();
        size_t freed_count = 0;
        int64_t earliest_remaining_us = std::numeric_limits<int64_t>::max();
        std::lock_guard<std::mutex> send_lock(send_mutex_);
        auto keep_from = std::remove_if(
            retired_websockets_.begin(), retired_websockets_.end(),
            [now_us, &freed_count](const RetiredWebSocket& retired) {
                if (now_us - retired.retired_at_us >= kCleanupDelayUs) {
                    ++freed_count;
                    return true;
                }
                return false;
            });
        retired_websockets_.erase(keep_from, retired_websockets_.end());
        for (const auto& retired : retired_websockets_) {
            earliest_remaining_us = std::min(earliest_remaining_us, retired.retired_at_us);
        }

        if (freed_count > 0) {
            ESP_LOGI(TAG, "%u WebSocket resources freed by worker, free internal: %lu",
                (unsigned)freed_count,
                (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        }

        // Newer sockets may not yet have completed their own safety delay.
        // Re-arm for the earliest remaining deadline rather than freeing them
        // together with an older socket.
        if (earliest_remaining_us != std::numeric_limits<int64_t>::max() && cleanup_timer_) {
            int64_t remaining_us = kCleanupDelayUs - (now_us - earliest_remaining_us);
            uint32_t remaining_ms = static_cast<uint32_t>(std::max<int64_t>(1, (remaining_us + 999) / 1000));
            TickType_t remaining_ticks = pdMS_TO_TICKS(remaining_ms);
            if (remaining_ticks == 0) {
                remaining_ticks = 1;
            }
            if (xTimerChangePeriod(cleanup_timer_, remaining_ticks, 0) != pdPASS) {
                ESP_LOGW(TAG, "Failed to re-arm delayed WebSocket cleanup");
            }
        }
    }
}

bool RemoteDisplay::Start(const std::string& server_url, int timeout_ms) {
    uint32_t lifecycle_epoch;
    bool ok;
    {
        std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

        if (running_ && connected_ && current_server_url_ == server_url) {
            ESP_LOGI(TAG, "Already connected to %s", server_url.c_str());
            return true;
        }

        EnsureReconnectTaskLocked();
        EnsureAudioTaskLocked();
        lifecycle_epoch = lifecycle_epoch_.fetch_add(1) + 1;
        session_ready_ = false;
        running_ = false;
        connected_ = false;
        ClearAudioQueueAndNotify();

        // An explicit start supersedes any existing connection and retry target.
        RetireWebSocketLocked();
        current_server_url_.clear();
        ok = StartLocked(server_url, timeout_ms, lifecycle_epoch);
    }

    if (!ok) {
        ScheduleReconnect(lifecycle_epoch);
    }
    return ok;
}

bool RemoteDisplay::StartLocked(const std::string& server_url, int timeout_ms,
                                uint32_t lifecycle_epoch) {
    if (lifecycle_epoch != lifecycle_epoch_.load()) {
        return false;
    }

    // A disconnected socket is normally released by the delayed cleanup timer.
    // A retry starts after that delay, but clean up defensively before replacing it.
    if (websocket_) {
        RetireWebSocketLocked();
        current_server_url_.clear();
    }

    // 获取网络接口并创建 WebSocket
    auto network = Board::GetInstance().GetNetwork();
    if (!network) {
        ESP_LOGE(TAG, "Network not available");
        return false;
    }

    // Create cleanup timer once (one-shot, 1s delay)
    if (!cleanup_timer_) {
        cleanup_timer_ = xTimerCreate("ws_clean", pdMS_TO_TICKS(1000), pdFALSE, this, OnCleanupTimer);
    }

    connected_ = false;
    running_ = false;
    session_ready_ = false;
    uint32_t generation = connect_generation_.fetch_add(1) + 1;

    ESP_LOGI(TAG, "Free internal RAM before WebSocket create: %lu",
        (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    websocket_ = network->CreateWebSocket(2);  // 使用 connect_id 2
    if (!websocket_) {
        ESP_LOGE(TAG, "Failed to create WebSocket");
        return false;
    }

    // 设置回调
    websocket_->OnConnected([this, generation]() {
        if (generation != connect_generation_.load()) {
            return;
        }
        ESP_LOGI(TAG, "Connected to remote display server");
        connected_ = true;
    });

    websocket_->OnDisconnected([this, generation]() {
        if (generation != connect_generation_.load()) {
            return;
        }
        uint32_t expected = generation;
        if (!connect_generation_.compare_exchange_strong(expected, generation + 1)) {
            return;
        }
        ESP_LOGW(TAG, "Disconnected, free internal: %lu",
            (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        session_ready_ = false;
        connected_ = false;
        running_ = false;
        if (audio_task_) {
            xTaskNotifyGive(audio_task_);
        }
        // Defer WebSocket cleanup to free internal RAM
        // Can't destroy from within its own callback chain (TCP receive task UAF)
        if (!cleanup_timer_ ||
            (!xTimerIsTimerActive(cleanup_timer_) &&
             xTimerStart(cleanup_timer_, 0) != pdPASS)) {
            // Keep the socket alive; the reconnect worker can retire it before
            // creating the replacement even if the cleanup timer is unavailable.
            ScheduleReconnect(lifecycle_epoch_.load());
        }
    });

    websocket_->OnError([](int error) {
        ESP_LOGE(TAG, "WebSocket error: %d", error);
    });

    // Quick TCP reachability check to avoid 18s LWIP SYN retry timeout
    std::string host;
    uint16_t port;
    if (ParseWsUrl(server_url, host, port)) {
        int check_ms = std::min(timeout_ms, 2000);
        if (!IsTcpReachable(host.c_str(), port, check_ms)) {
            ESP_LOGW(TAG, "Server not reachable: %s:%d (checked in %dms)", host.c_str(), port, check_ms);
            RetireWebSocketLocked();
            return false;
        }
    }

    // 尝试连接
    ESP_LOGI(TAG, "Connecting to %s (timeout %dms)...", server_url.c_str(), timeout_ms);

    if (!websocket_->Connect(server_url.c_str())) {
        ESP_LOGE(TAG, "Failed to connect to remote display server");
        RetireWebSocketLocked();
        return false;
    }

    // 等待连接完成
    int wait_time = 0;
    while (!connected_ && wait_time < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_time += 100;
    }

    if (!connected_ || generation != connect_generation_.load() ||
        lifecycle_epoch != lifecycle_epoch_.load()) {
        ESP_LOGE(TAG, "Connection timeout");
        // If a disconnect callback ran, delayed cleanup already owns the socket.
        // Otherwise retire it now; actual destruction is still delayed.
        if (generation == connect_generation_.load()) {
            RetireWebSocketLocked();
        }
        return false;
    }

    current_server_url_ = server_url;
    running_ = true;

    // A rapid disconnect may race with Connect() returning. Commit first, then
    // validate the generations so Start can never resurrect a dead session.
    if (generation != connect_generation_.load() ||
        lifecycle_epoch != lifecycle_epoch_.load() || !connected_) {
        running_ = false;
        return false;
    }

    // No normal UI/audio send is allowed before this ordered initial sequence.
    SendHello();
    SendUIState(true);
    if (generation != connect_generation_.load() ||
        lifecycle_epoch != lifecycle_epoch_.load() || !connected_) {
        running_ = false;
        return false;
    }
    session_ready_ = true;
    if (generation != connect_generation_.load() ||
        lifecycle_epoch != lifecycle_epoch_.load() || !connected_) {
        session_ready_ = false;
        running_ = false;
        return false;
    }
    if (ui_state_dirty_) {
        ScheduleUIFlush();
    }

    ESP_LOGI(TAG, "Remote display started, free internal: %lu",
        (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    return true;
}

void RemoteDisplay::Stop() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    lifecycle_epoch_.fetch_add(1);
    reconnect_epoch_ = lifecycle_epoch_.load();
    session_ready_ = false;
    running_ = false;
    ClearAudioQueueAndNotify();

    // Cancel an old timer; retiring the active socket below starts a fresh one.
    if (cleanup_timer_) {
        xTimerStop(cleanup_timer_, 0);
    }

    RetireWebSocketLocked();

    connected_ = false;
    current_server_url_.clear();
    if (reconnect_task_) {
        xTaskNotifyGive(reconnect_task_);
    }
    ESP_LOGI(TAG, "Remote display stopped");
}

void RemoteDisplay::EnsureReconnectTaskLocked() {
    if (reconnect_task_) {
        return;
    }
    if (xTaskCreate(ReconnectTask, "remote_disp_retry", 3072, this, 1,
                    &reconnect_task_) != pdPASS) {
        reconnect_task_ = nullptr;
        ESP_LOGE(TAG, "Failed to create reconnect task");
    }
}

bool RemoteDisplay::EnsureAudioTaskLocked() {
    if (audio_task_) {
        return true;
    }
    if (audio_shutdown_) {
        return false;
    }
    if (!audio_task_stopped_) {
        audio_task_stopped_ = xSemaphoreCreateBinary();
        if (!audio_task_stopped_) {
            if (!audio_task_create_failed_.exchange(true)) {
                ESP_LOGE(TAG, "Failed to create audio task stop semaphore");
            }
            return false;
        }
    }
    if (xTaskCreate(AudioTask, "remote_audio", 3072, this, 2, &audio_task_) != pdPASS) {
        audio_task_ = nullptr;
        if (!audio_task_create_failed_.exchange(true)) {
            ESP_LOGE(TAG, "Failed to create remote audio task");
        }
        return false;
    }
    return true;
}

void RemoteDisplay::ClearAudioQueueAndNotify() {
    {
        std::lock_guard<std::mutex> queue_lock(audio_queue_mutex_);
        audio_queue_.clear();
        audio_queued_duration_ms_ = 0;
    }
    if (audio_task_) {
        xTaskNotifyGive(audio_task_);
    }
}

void RemoteDisplay::ScheduleReconnect(uint32_t lifecycle_epoch) {
    reconnect_epoch_ = lifecycle_epoch;
    TaskHandle_t task = reconnect_task_;
    if (task) {
        xTaskNotifyGive(task);
    }
}

void RemoteDisplay::ScheduleUIFlush() {
    TaskHandle_t task = reconnect_task_;
    if (task) {
        xTaskNotifyGive(task);
    }
}

void RemoteDisplay::RequestReconnect() {
    uint32_t lifecycle_epoch;
    {
        std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
        EnsureReconnectTaskLocked();
        lifecycle_epoch = lifecycle_epoch_.load();
    }
    ScheduleReconnect(lifecycle_epoch);
}

void RemoteDisplay::ReconnectTask(void* arg) {
    auto* self = static_cast<RemoteDisplay*>(arg);
    static constexpr int kRetryDelayMs[] = {3000, 5000, 10000, 30000};

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        {
            std::lock_guard<std::mutex> lifecycle_lock(self->lifecycle_mutex_);
            self->CleanupDisconnectedSocketLocked();
        }
        if (self->running_ && self->connected_) {
            if (self->session_ready_ && self->ui_state_dirty_) {
                self->SendUIState(true);
            }
            continue;
        }
        uint32_t epoch = self->reconnect_epoch_.load();
        size_t delay_index = 0;

        while (true) {
            auto config = LoadConfig();
            if (epoch != self->lifecycle_epoch_.load() || !config.enabled ||
                config.server_url.empty()) {
                break;
            }

            TickType_t delay = pdMS_TO_TICKS(kRetryDelayMs[delay_index]);
            if (ulTaskNotifyTake(pdTRUE, delay) > 0) {
                {
                    std::lock_guard<std::mutex> lifecycle_lock(self->lifecycle_mutex_);
                    self->CleanupDisconnectedSocketLocked();
                }
                if (self->running_ && self->connected_) {
                    if (self->session_ready_ && self->ui_state_dirty_) {
                        self->SendUIState(true);
                    }
                    break;
                }
                epoch = self->reconnect_epoch_.load();
                delay_index = 0;
                continue;
            }

            bool ok = false;
            {
                std::lock_guard<std::mutex> lifecycle_lock(self->lifecycle_mutex_);
                // Re-read persistent state only after entering the lifecycle
                // lock, immediately before the blocking connection attempt.
                config = LoadConfig();
                if (epoch != self->lifecycle_epoch_.load() || !config.enabled ||
                    config.server_url.empty()) {
                    break;
                }
                if (self->running_ && self->connected_) {
                    ok = true;
                } else {
                    ok = self->StartLocked(config.server_url, config.timeout_ms, epoch);
                }
            }

            if (ok) {
                break;
            }
            if (delay_index + 1 < sizeof(kRetryDelayMs) / sizeof(kRetryDelayMs[0])) {
                ++delay_index;
            }
        }
    }
}

void RemoteDisplay::SendEmotion(const char* emotion) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const char* value = emotion ? emotion : "";
        if (current_emotion_ == value) return;  // 无变化
        current_emotion_ = value;
        ui_state_version_.fetch_add(1);
        ui_state_dirty_ = true;
    }

    if (session_ready_) ScheduleUIFlush();
}

void RemoteDisplay::SendStatus(const char* status) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const char* value = status ? status : "";
        if (current_status_ == value) return;  // 无变化
        current_status_ = value;
        ui_state_version_.fetch_add(1);
        ui_state_dirty_ = true;
    }

    if (session_ready_) ScheduleUIFlush();
}

void RemoteDisplay::SendChatMessage(const char* role, const char* content) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_chat_role_ = role ? role : "";
        current_chat_text_ = content ? content : "";
        ui_state_version_.fetch_add(1);
        ui_state_dirty_ = true;
    }

    if (session_ready_) ScheduleUIFlush();
}

void RemoteDisplay::SendTheme(const char* theme_name) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_theme_ = theme_name ? theme_name : "";
        ui_state_version_.fetch_add(1);
        ui_state_dirty_ = true;
    }

    if (session_ready_) ScheduleUIFlush();
}

void RemoteDisplay::SendVolume(int volume, bool muted) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (current_volume_ == volume && current_muted_ == muted) return;
        current_volume_ = volume;
        current_muted_ = muted;
        ui_state_version_.fetch_add(1);
        ui_state_dirty_ = true;
    }

    if (session_ready_) ScheduleUIFlush();
}

void RemoteDisplay::SendUIState(bool force) {
    if (!running_ || !connected_ || (!force && !session_ready_)) return;

    // 构造 JSON
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "ui_state");
    uint32_t snapshot_version;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        cJSON_AddStringToObject(root, "emotion", current_emotion_.c_str());
        cJSON_AddStringToObject(root, "status", current_status_.c_str());

        cJSON* chat = cJSON_CreateObject();
        cJSON_AddStringToObject(chat, "role", current_chat_role_.c_str());
        cJSON_AddStringToObject(chat, "text", current_chat_text_.c_str());
        cJSON_AddItemToObject(root, "chat", chat);

        cJSON_AddStringToObject(root, "theme", current_theme_.c_str());
        cJSON_AddNumberToObject(root, "volume", current_volume_);
        cJSON_AddBoolToObject(root, "muted", current_muted_);
        snapshot_version = ui_state_version_.load();
    }

    char* json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json_str) {
        std::unique_lock<std::mutex> lock(send_mutex_, std::defer_lock);
        if (force) {
            lock.lock();
        } else if (!lock.try_lock()) {
            free(json_str);
            ScheduleUIFlush();
            return;
        }
        bool sent = false;
        if (running_ && connected_ && websocket_) {
            sent = websocket_->Send(json_str);
        }
        free(json_str);
        if (sent) {
            ui_state_dirty_ = false;
            if (snapshot_version != ui_state_version_.load()) {
                ui_state_dirty_ = true;
                if (session_ready_) {
                    ScheduleUIFlush();
                }
            }
        } else {
            ui_state_dirty_ = true;
            if (!force && session_ready_) {
                ScheduleUIFlush();
            }
        }
    }
}

void RemoteDisplay::AudioTask(void* arg) {
    auto* self = static_cast<RemoteDisplay*>(arg);
    uint32_t total_drops = 0;
    uint32_t unreported_drops = 0;
    TickType_t last_report_tick = 0;
    bool reported_once = false;

    auto report_drops = [&]() {
        unreported_drops += self->audio_drop_count_.exchange(0);
        if (unreported_drops == 0) {
            return;
        }
        TickType_t now = xTaskGetTickCount();
        if (!reported_once || now - last_report_tick >= pdMS_TO_TICKS(5000) ||
            self->audio_shutdown_) {
            total_drops += unreported_drops;
            ESP_LOGW(TAG, "Remote audio dropped %lu frame(s), total=%lu",
                (unsigned long)unreported_drops, (unsigned long)total_drops);
            unreported_drops = 0;
            last_report_tick = now;
            reported_once = true;
        }
    };

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        report_drops();
        if (self->audio_shutdown_) {
            break;
        }

        while (true) {
            AudioFrame frame;
            {
                std::lock_guard<std::mutex> queue_lock(self->audio_queue_mutex_);
                if (self->audio_queue_.empty()) {
                    break;
                }
                frame = std::move(self->audio_queue_.front());
                self->audio_queue_.pop_front();
                self->audio_queued_duration_ms_ -= frame.frame_duration;
            }

            size_t total_size = sizeof(OpusAudioHeader) + frame.payload.size();
            std::vector<uint8_t> buffer(total_size);
            auto* header = reinterpret_cast<OpusAudioHeader*>(buffer.data());
            header->type = MSG_TYPE_AUDIO_FRAME;
            header->sample_rate = static_cast<uint16_t>(frame.sample_rate);
            header->frame_duration = static_cast<uint8_t>(frame.frame_duration);
            memcpy(buffer.data() + sizeof(OpusAudioHeader), frame.payload.data(), frame.payload.size());

            // Queue and send locks are never held together. Stop or a connection
            // switch invalidates the generation before this final send check.
            std::lock_guard<std::mutex> send_lock(self->send_mutex_);
            if (!self->running_ || !self->connected_ || !self->session_ready_ ||
                frame.connect_generation != self->connect_generation_.load() ||
                !self->websocket_) {
                self->audio_drop_count_.fetch_add(1);
                continue;
            }
            if (!self->websocket_->Send(buffer.data(), buffer.size(), true)) {
                self->audio_drop_count_.fetch_add(1);
            }
        }
        report_drops();
    }

    {
        std::lock_guard<std::mutex> queue_lock(self->audio_queue_mutex_);
        self->audio_queue_.clear();
        self->audio_queued_duration_ms_ = 0;
    }
    report_drops();
    if (self->audio_task_stopped_) {
        xSemaphoreGive(self->audio_task_stopped_);
    }
    vTaskDelete(nullptr);
}

void RemoteDisplay::ForwardOpusAudio(const std::vector<uint8_t>& opus_data, int sample_rate, int frame_duration) {
    TaskHandle_t task = audio_task_;
    if (!running_ || !connected_ || !session_ready_ || !task) {
        return;
    }

    if (opus_data.empty() || opus_data.size() > 4096 ||
        frame_duration <= 0 || frame_duration > 120) {
        audio_drop_count_.fetch_add(1);
        xTaskNotifyGive(task);
        return;
    }

    std::unique_lock<std::mutex> queue_lock(audio_queue_mutex_, std::try_to_lock);
    if (!queue_lock.owns_lock()) {
        audio_drop_count_.fetch_add(1);
        xTaskNotifyGive(task);
        return;
    }

    static constexpr size_t kMaxAudioFrames = 5;
    static constexpr int kMaxAudioDurationMs = 300;
    while (!audio_queue_.empty() &&
           (audio_queue_.size() >= kMaxAudioFrames ||
            audio_queued_duration_ms_ + frame_duration > kMaxAudioDurationMs)) {
        audio_queued_duration_ms_ -= audio_queue_.front().frame_duration;
        audio_queue_.pop_front();
        audio_drop_count_.fetch_add(1);
    }

    AudioFrame frame;
    frame.payload = opus_data;
    frame.sample_rate = sample_rate;
    frame.frame_duration = frame_duration;
    frame.connect_generation = connect_generation_.load();
    audio_queued_duration_ms_ += frame_duration;
    audio_queue_.push_back(std::move(frame));
    queue_lock.unlock();
    xTaskNotifyGive(task);
}

void RemoteDisplay::SendHello() {
    // 发送 hello 消息
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "hello");
    cJSON_AddStringToObject(root, "client", "sensecap-watcher");
    cJSON_AddStringToObject(root, "mode", "ui_state");

    cJSON* audio = cJSON_CreateObject();
    cJSON_AddStringToObject(audio, "format", "opus");
    cJSON_AddItemToObject(root, "audio", audio);

    char* json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json_str) {
        std::lock_guard<std::mutex> lock(send_mutex_);
        if (connected_ && websocket_) {
            websocket_->Send(json_str);
            ESP_LOGI(TAG, "Sent hello: %s", json_str);
        }
        free(json_str);
    }
}

void RemoteDisplay::SendPreviewImage(const uint8_t* jpeg_data, size_t size) {
    if (!running_ || !connected_ || !session_ready_) return;
    if (jpeg_data == nullptr || size == 0) return;

    // 计算 Base64 编码后的大小
    size_t base64_len = 0;
    mbedtls_base64_encode(nullptr, 0, &base64_len, jpeg_data, size);

    // 分配 Base64 缓冲区（使用 PSRAM 避免占用内部 RAM）
    char* base64_buf = (char*)heap_caps_malloc(base64_len + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (base64_buf == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate memory for base64 encoding");
        return;
    }

    // 执行 Base64 编码
    size_t written = 0;
    int ret = mbedtls_base64_encode((unsigned char*)base64_buf, base64_len + 1, &written, jpeg_data, size);
    if (ret != 0) {
        ESP_LOGE(TAG, "Base64 encoding failed: %d", ret);
        free(base64_buf);
        return;
    }
    base64_buf[written] = '\0';

    // 构造 JSON 消息
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "preview_image");
    cJSON_AddStringToObject(root, "format", "jpeg");
    cJSON_AddNumberToObject(root, "size", size);
    cJSON_AddStringToObject(root, "data", base64_buf);

    char* json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    free(base64_buf);

    if (json_str) {
        std::unique_lock<std::mutex> lock(send_mutex_, std::try_to_lock);
        if (lock.owns_lock() && running_ && connected_ && websocket_) {
            websocket_->Send(json_str);
            ESP_LOGI(TAG, "Sent preview image: %zu bytes (base64: %zu)", size, written);
        }
        free(json_str);
    }
}
