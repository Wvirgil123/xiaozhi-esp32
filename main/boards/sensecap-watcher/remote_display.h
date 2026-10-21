#ifndef REMOTE_DISPLAY_H
#define REMOTE_DISPLAY_H

#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <vector>
#include <deque>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/semphr.h>

class WebSocket;

// 远程显示消息类型
enum RemoteDisplayMessageType : uint8_t {
    MSG_TYPE_UI_STATE     = 0x10,   // UI 状态 (JSON)
    MSG_TYPE_AUDIO_FRAME  = 0x02,   // 音频帧 (Opus)
    MSG_TYPE_AUDIO_PCM    = 0x03,   // 兼容旧版 PCM 数据
    MSG_TYPE_HEARTBEAT    = 0x04,   // 心跳
};

// 远程显示配置
struct RemoteDisplayConfig {
    bool enabled = false;
    std::string server_url;
    int timeout_ms = 1000;  // 默认 1 秒超时，避免长时间阻塞
};

// 远程显示服务 - 将 UI 状态同步到树莓派
class RemoteDisplay {
public:
    static RemoteDisplay* GetInstance();

    // 配置管理 (NVS 存储)
    static RemoteDisplayConfig LoadConfig();
    static void SaveConfig(const RemoteDisplayConfig& config);

    // 使用存储的配置启动
    bool StartWithConfig();

    // 尝试连接到远程服务器，失败返回 false
    // timeout_ms: 连接超时时间，默认 1000ms（减少阻塞时间）
    bool Start(const std::string& server_url, int timeout_ms = 1000);
    void Stop();
    bool IsRunning() const { return running_ && connected_; }

    // Wake the background reconnect worker. The worker re-checks the persisted
    // enabled flag and URL before doing any blocking network work.
    void RequestReconnect();

    // UI 状态发送接口
    void SendEmotion(const char* emotion);
    void SendStatus(const char* status);
    void SendChatMessage(const char* role, const char* content);
    void SendTheme(const char* theme_name);
    void SendVolume(int volume, bool muted);

    // 图像发送接口
    void SendPreviewImage(const uint8_t* jpeg_data, size_t size);

    // Opus 音频转发 - 由 Application::OnIncomingAudio 回调调用
    void ForwardOpusAudio(const std::vector<uint8_t>& opus_data, int sample_rate, int frame_duration);

private:
    struct RetiredWebSocket {
        std::unique_ptr<WebSocket> socket;
        int64_t retired_at_us;
    };

    struct AudioFrame {
        std::vector<uint8_t> payload;
        int sample_rate;
        int frame_duration;
        uint32_t connect_generation;
    };

    RemoteDisplay() = default;
    ~RemoteDisplay();

    void SendUIState(bool force = false);
    void SendHello();
    bool StartLocked(const std::string& server_url, int timeout_ms, uint32_t lifecycle_epoch);
    void RetireWebSocketLocked(bool schedule_cleanup = true);
    void CleanupDisconnectedSocketLocked();
    void EnsureReconnectTaskLocked();
    void ScheduleReconnect(uint32_t lifecycle_epoch);
    void ScheduleUIFlush();
    bool EnsureAudioTaskLocked();
    void ClearAudioQueueAndNotify();
    static void ReconnectTask(void* arg);
    static void AudioTask(void* arg);
    static void OnCleanupTimer(TimerHandle_t timer);

    std::unique_ptr<WebSocket> websocket_;
    // WebSocket callbacks can originate inside WebSocket::OnTcpData(). Moving
    // an old socket here keeps it alive until the delayed worker cleanup runs.
    std::vector<RetiredWebSocket> retired_websockets_;
    TimerHandle_t cleanup_timer_ = nullptr;

    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> session_ready_{false};
    std::atomic<bool> cleanup_due_{false};
    std::atomic<bool> ui_state_dirty_{false};
    std::atomic<uint32_t> ui_state_version_{0};
    std::atomic<uint32_t> connect_generation_{0};
    std::atomic<uint32_t> lifecycle_epoch_{0};
    std::atomic<uint32_t> reconnect_epoch_{0};
    TaskHandle_t reconnect_task_ = nullptr;
    TaskHandle_t audio_task_ = nullptr;
    SemaphoreHandle_t audio_task_stopped_ = nullptr;
    std::atomic<bool> audio_shutdown_{false};
    std::atomic<bool> audio_task_create_failed_{false};
    std::atomic<uint32_t> audio_drop_count_{0};
    std::string current_server_url_;  // 当前连接的服务器 URL

    std::deque<AudioFrame> audio_queue_;
    int audio_queued_duration_ms_ = 0;

    // UI 状态缓存
    std::string current_emotion_;
    std::string current_status_;
    std::string current_chat_role_;
    std::string current_chat_text_;
    std::string current_theme_;
    int current_volume_ = 70;
    bool current_muted_ = false;

    std::mutex state_mutex_;
    std::mutex send_mutex_;
    std::mutex audio_queue_mutex_;
    std::mutex lifecycle_mutex_;
};

#endif // REMOTE_DISPLAY_H
