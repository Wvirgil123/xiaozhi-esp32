#include "face_recognition.h"
#include "application.h"
#include "sscma_camera.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <cstring>

static const char* TAG = "FaceRecognition";

// Default cooldown: 5 seconds (matches object detection default interval)
#define DEFAULT_COOLDOWN_US 5000000

std::string TruncateUtf8(const std::string& s, size_t max_bytes) {
    if (s.size() <= max_bytes) {
        return s;
    }
    // s[end] is the first byte we drop; if it continues a multi-byte sequence
    // (0b10xxxxxx) walk back until we land on a lead byte.
    size_t end = max_bytes;
    while (end > 0 && (static_cast<unsigned char>(s[end]) & 0xC0) == 0x80) {
        --end;
    }
    return s.substr(0, end);
}

// ============== FaceVotingBuffer Implementation ==============

FaceVotingBuffer::FaceVotingBuffer()
    : write_index_(0), valid_count_(0) {
    mutex_ = xSemaphoreCreateMutex();
    Clear();
}

FaceVotingBuffer::~FaceVotingBuffer() {
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

void FaceVotingBuffer::Clear() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    for (int i = 0; i < FACE_VOTING_BUFFER_SIZE; i++) {
        buffer_[i].valid = false;
    }
    write_index_ = 0;
    valid_count_ = 0;
    xSemaphoreGive(mutex_);
}

void FaceVotingBuffer::PruneOldEntries(int64_t current_time_us) {
    for (int i = 0; i < FACE_VOTING_BUFFER_SIZE; i++) {
        if (buffer_[i].valid) {
            if (current_time_us - buffer_[i].timestamp_us > window_us_) {
                buffer_[i].valid = false;
                valid_count_--;
            }
        }
    }
}

void FaceVotingBuffer::AddEmbedding(const float* embedding, int64_t timestamp_us) {
    if (timestamp_us == 0) {
        timestamp_us = esp_timer_get_time();
    }

    xSemaphoreTake(mutex_, portMAX_DELAY);

    // Prune old entries first
    PruneOldEntries(timestamp_us);

    // Add new entry
    if (buffer_[write_index_].valid) {
        valid_count_--;  // We're overwriting a valid entry
    }

    memcpy(buffer_[write_index_].embedding, embedding, FACE_EMBEDDING_SIZE);
    buffer_[write_index_].timestamp_us = timestamp_us;
    buffer_[write_index_].valid = true;
    valid_count_++;

    write_index_ = (write_index_ + 1) % FACE_VOTING_BUFFER_SIZE;

    xSemaphoreGive(mutex_);

    ESP_LOGD(TAG, "Added embedding to voting buffer (valid: %d)", valid_count_);
}

bool FaceVotingBuffer::GetConsensusEmbedding(float* out_embedding) {
    xSemaphoreTake(mutex_, portMAX_DELAY);

    int64_t current_time = esp_timer_get_time();
    PruneOldEntries(current_time);

    if (valid_count_ < FACE_VOTING_MIN_VOTES) {
        xSemaphoreGive(mutex_);
        ESP_LOGD(TAG, "Not enough votes (%d < %d)", valid_count_, FACE_VOTING_MIN_VOTES);
        return false;
    }

    // Calculate average embedding
    memset(out_embedding, 0, FACE_EMBEDDING_SIZE);

    int count = 0;
    for (int i = 0; i < FACE_VOTING_BUFFER_SIZE; i++) {
        if (buffer_[i].valid) {
            for (int j = 0; j < FACE_EMBEDDING_DIM; j++) {
                out_embedding[j] += buffer_[i].embedding[j];
            }
            count++;
        }
    }

    if (count > 0) {
        for (int j = 0; j < FACE_EMBEDDING_DIM; j++) {
            out_embedding[j] /= count;
        }
    }

    // Normalize the result
    FaceDatabase::NormalizeEmbedding(out_embedding);

    xSemaphoreGive(mutex_);

    ESP_LOGD(TAG, "Got consensus embedding from %d votes", count);
    return true;
}

int FaceVotingBuffer::GetVoteCount() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    PruneOldEntries(esp_timer_get_time());
    int count = valid_count_;
    xSemaphoreGive(mutex_);
    return count;
}

void FaceVotingBuffer::SetWindow(int64_t window_us) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    window_us_ = window_us;
    xSemaphoreGive(mutex_);
}

// ============== FaceRecognition Implementation ==============

FaceRecognition& FaceRecognition::GetInstance() {
    static FaceRecognition instance;
    return instance;
}

FaceRecognition::FaceRecognition()
    : enabled_(false)
    , familiar_mode_(false)
    , match_threshold_(0.6f)
    , cooldown_duration_us_(DEFAULT_COOLDOWN_US)
    , has_pending_notification_(false) {
    notification_mutex_ = xSemaphoreCreateMutex();
    last_match_.matched = false;
    last_match_.subject_id = 0;
    last_match_.similarity = 0.0f;
    last_match_.index = -1;
}

bool FaceRecognition::IsEnabled() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return enabled_;
}

void FaceRecognition::SetFamiliarMode(bool enabled) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    familiar_mode_ = enabled;
    ESP_LOGI(TAG, "Familiar DND mode %s", enabled ? "enabled" : "disabled");
}

bool FaceRecognition::IsFamiliarMode() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return familiar_mode_;
}

void FaceRecognition::SetEnabled(bool enabled) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    enabled_ = enabled;
    if (enabled) {
        // Starting face mode: if we were in cooldown and deferred the timer start,
        // begin the cooldown timer now (same as object detection's need_start_cooldown)
        if (state_ == FaceDetectionState::COOLDOWN && need_start_cooldown_) {
            cooldown_start_time_ = esp_timer_get_time();
            need_start_cooldown_ = false;
            ESP_LOGI(TAG, "Face cooldown timer started");
        }
    } else {
        voting_buffer_.Clear();
    }
    ESP_LOGI(TAG, "Face recognition %s", enabled ? "enabled" : "disabled");
}

void FaceRecognition::SetValidationDuration(int seconds) {
    int64_t window_us = (int64_t)seconds * 1000000LL;
    voting_buffer_.SetWindow(window_us);
    ESP_LOGI(TAG, "Validation duration set to %ds", seconds);
}

void FaceRecognition::SetCooldownInterval(int seconds) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cooldown_duration_us_ = (int64_t)seconds * 1000000LL;
    ESP_LOGI(TAG, "Cooldown interval set to %ds", seconds);
}

void FaceRecognition::NotifyFacePresent() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_face_seen_us_ = esp_timer_get_time();
    // Throttled INFO (every ~2s) so serial isn't flooded at 14fps.
    static int64_t last_present_log_us = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_present_log_us > 2000000) {
        last_present_log_us = now;
        ESP_LOGI(TAG, "NotifyFacePresent (state=%d)", (int)state_);
    }
}

void FaceRecognition::NotifyNoFace() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Throttled INFO (every ~2s) so serial isn't flooded at 14fps.
    static int64_t last_noface_log_us = 0;
    int64_t log_now = esp_timer_get_time();
    if (log_now - last_noface_log_us > 2000000) {
        last_noface_log_us = log_now;
        ESP_LOGI(TAG, "NotifyNoFace (state=%d)", (int)state_);
    }

    int64_t now = esp_timer_get_time();
    // Face gone for the debounce period → this person is done. Reset the
    // unknown-grace streak so the NEXT person to appear gets a fresh grace
    // window (otherwise a stale grace start would prematurely declare them
    // "person"). Applies in any state (a person may leave mid-grace, before
    // any greeting / cooldown).
    if ((now - last_face_seen_us_) >= kFaceGoneDebounceUs) {
        unknown_grace_start_us_ = 0;
    }

    // Check if we can exit cooldown: time elapsed AND face gone for debounce period
    if (state_ != FaceDetectionState::COOLDOWN) {
        return;
    }

    bool time_passed = (now - cooldown_start_time_) >= cooldown_duration_us_;
    bool face_gone = (now - last_face_seen_us_) >= kFaceGoneDebounceUs;

    if (time_passed && face_gone) {
        state_ = FaceDetectionState::DETECTING;
        voting_buffer_.Clear();
        last_notified_name_.clear();
        unknown_grace_start_us_ = 0;
        ESP_LOGI(TAG, "Cooldown complete and face left, back to detecting");
    }
}

void FaceRecognition::ProcessFaceData(const HimaxFaceData& face_data) {
    ESP_LOGD(TAG, "ProcessFaceData: has_embedding=%d", face_data.has_embedding);
    if (!face_data.has_embedding) {
        ESP_LOGD(TAG, "No embedding in face data");
        return;
    }

    float threshold = 0.6f;
    bool familiar_mode = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!enabled_) {
            ESP_LOGD(TAG, "ProcessFaceData: not enabled, skip");
            return;
        }
        if (state_ == FaceDetectionState::COOLDOWN) {
            ESP_LOGD(TAG, "ProcessFaceData: in COOLDOWN, skip");
            return;
        }
        threshold = match_threshold_;
        familiar_mode = familiar_mode_;
    }

    // Add to voting buffer
    voting_buffer_.AddEmbedding(face_data.embedding);
    ESP_LOGD(TAG, "ProcessFaceData: after AddEmbedding votes=%d (need %d)",
             voting_buffer_.GetVoteCount(), FACE_VOTING_MIN_VOTES);

    // Try to get consensus embedding
    float consensus_emb[FACE_EMBEDDING_DIM];
    bool have_consensus = voting_buffer_.GetConsensusEmbedding(consensus_emb);
    ESP_LOGD(TAG, "ProcessFaceData: consensus=%d", have_consensus);
    if (have_consensus) {
        // Match against database
        auto& db = FaceDatabase::GetInstance();
        FaceMatchResult match = db.Match(consensus_emb, threshold);

        ESP_LOGI(TAG, "Match result: matched=%d, name=%s, similarity=%.3f, threshold=%.2f, familiar_dnd=%d",
                 match.matched, match.matched ? match.name.c_str() : "N/A",
                 match.similarity, threshold, familiar_mode);

        if (match.matched) {
            unknown_grace_start_us_ = 0;  // resolved to a name → end any unknown streak
            // Familiar DND: ignore familiar faces (don't wake)
            if (familiar_mode) {
                ESP_LOGI(TAG, "ProcessFaceData: -> SuppressCurrentFace (familiar DND): %s (sim=%.3f)", match.name.c_str(), match.similarity);
                SuppressCurrentFace();
            } else {
                // Normal mode: notify for familiar faces
                ESP_LOGI(TAG, "ProcessFaceData: -> HandleRecognitionResult (matched)");
                HandleRecognitionResult(match);
            }
        } else {
            // Unknown: hold before declaring person/stranger. A known face still
            // approaching (far/angled → low similarity) gets a grace window to
            // resolve to a name; only after kUnknownGraceUs of continuous no-match
            // do we greet person/stranger. Avoids the premature "person detected"
            // flash for a familiar person walking up.
            int64_t now = esp_timer_get_time();
            if (unknown_grace_start_us_ == 0) {
                unknown_grace_start_us_ = now;
                ESP_LOGI(TAG, "ProcessFaceData: unmatched face, starting %llds grace before declaring unknown",
                         (long long)(kUnknownGraceUs / 1000000));
            } else if (now - unknown_grace_start_us_ >= kUnknownGraceUs) {
                ESP_LOGI(TAG, "ProcessFaceData: grace elapsed, -> HandleUnknownPersonDetected (stranger_alert=%d)", familiar_mode);
                HandleUnknownPersonDetected(familiar_mode);
            } else {
                ESP_LOGD(TAG, "ProcessFaceData: unmatched, still within grace (%.1fs), waiting",
                         (now - unknown_grace_start_us_) / 1e6);
            }
        }
    }
}

void FaceRecognition::TriggerNotification(const std::string& wake_word) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_ = FaceDetectionState::COOLDOWN;
        need_start_cooldown_ = true;  // Actual timer starts when face mode resumes after conversation
    }
    voting_buffer_.Clear();
    // Wake window: drop further sscma event frames (each parses into an ~8KB
    // internal-SRAM cJSON tree) until the camera main loop tears face mode
    // down — protects internal SRAM during the wake TLS handshake.
    SscmaCamera::SetDropEvents(true);
    Application::GetInstance().Schedule([wake_word]() {
        Application::GetInstance().WakeWordInvoke(wake_word);
    });
}

void FaceRecognition::SuppressCurrentFace() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_ = FaceDetectionState::COOLDOWN;
        cooldown_start_time_ = esp_timer_get_time();
        need_start_cooldown_ = false;
    }
    voting_buffer_.Clear();
}

void FaceRecognition::HandleRecognitionResult(const FaceMatchResult& result) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_match_ = result;
        last_match_time_us_ = esp_timer_get_time();
        last_notified_name_ = result.name;
    }

    // Notify via wake word (decode hex-encoded UTF-8 names from SBC)
    std::string display_name = FaceDatabase::DecodeName(result.name);
    ESP_LOGI(TAG, "Recognized: %s (similarity: %.2f)", display_name.c_str(), result.similarity);

    // Short tag + name only: the cloud rejects anything past ~25 bytes (see
    // kWakeWordMaxBytes). <f></f> costs 7 bytes, so the name gets 18 — about
    // 6 CJK characters, truncated on a character boundary beyond that.
    std::string wake_word =
        "<f>" + TruncateUtf8(display_name, kWakeWordPayloadMaxBytes) + "</f>";
    TriggerNotification(wake_word);

}

void FaceRecognition::HandleUnknownPersonDetected(bool is_stranger_alert) {
    const char* person_id = is_stranger_alert ? "stranger" : "person";

    ESP_LOGI(TAG, "%s detected", person_id);
    // "stranger" / "person" are 8 / 6 bytes — comfortably inside the budget.
    std::string wake_word = std::string("<f>") + person_id + "</f>";
    TriggerNotification(wake_word);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_notified_name_ = person_id;
    }
}

bool FaceRecognition::IsInCooldown() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_ == FaceDetectionState::COOLDOWN;
}

void FaceRecognition::StartCooldown() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cooldown_start_time_ = esp_timer_get_time();
}

FaceMatchResult FaceRecognition::GetLastMatch() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return last_match_;
}

int64_t FaceRecognition::GetLastMatchTimeUs() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return last_match_time_us_;
}

void FaceRecognition::CaptureCurrentSpeaker() {
    SpeakerIdentity speaker;  // defaults to invalid
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (last_match_.matched && last_match_time_us_ != 0) {
            int64_t age_us = esp_timer_get_time() - last_match_time_us_;
            if (age_us >= 0 && age_us <= kSpeakerMaxAgeUs) {
                speaker.valid = true;
                speaker.name = FaceDatabase::DecodeName(last_match_.name);
                speaker.subject_id = last_match_.subject_id;
                speaker.similarity = last_match_.similarity;
            }
        }
        current_speaker_ = speaker;
        conversation_seq_++;  // new conversation rising edge → bump seq (仅首次 key)
    }
    if (speaker.valid) {
        ESP_LOGI(TAG, "current_speaker = %s (subject_id=%d, %.2f)",
                 speaker.name.c_str(), speaker.subject_id, speaker.similarity);
    } else {
        ESP_LOGI(TAG, "current_speaker = <unknown>");
    }
}

void FaceRecognition::ClearCurrentSpeaker() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_speaker_ = SpeakerIdentity{};
}

uint32_t FaceRecognition::GetConversationSeq() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return conversation_seq_;
}

void FaceRecognition::SetCurrentSpeaker(const SpeakerIdentity& speaker) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_speaker_ = speaker;
    }
    ESP_LOGI(TAG, "current_speaker (on-demand) = %s (subject_id=%d, %.2f)",
             speaker.valid ? speaker.name.c_str() : "<unknown>",
             speaker.subject_id, speaker.similarity);
}

FaceRecognition::SpeakerIdentity FaceRecognition::GetCurrentSpeaker() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_speaker_;
}

void FaceRecognition::SetMatchThreshold(float threshold) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    match_threshold_ = threshold;
}

float FaceRecognition::GetMatchThreshold() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return match_threshold_;
}

void FaceRecognition::SetPendingNotification(const std::string& wake_word) {
    xSemaphoreTake(notification_mutex_, portMAX_DELAY);
    pending_wake_word_ = wake_word;
    has_pending_notification_ = true;
    xSemaphoreGive(notification_mutex_);
    ESP_LOGI(TAG, "Pending notification queued: %s", wake_word.c_str());
}

bool FaceRecognition::DeliverPendingNotification() {
    xSemaphoreTake(notification_mutex_, portMAX_DELAY);
    if (!has_pending_notification_) {
        xSemaphoreGive(notification_mutex_);
        return false;
    }

    DeviceState dev_state = Application::GetInstance().GetDeviceState();
    if (dev_state != kDeviceStateIdle) {
        xSemaphoreGive(notification_mutex_);
        return false;
    }

    std::string wake_word = std::move(pending_wake_word_);
    has_pending_notification_ = false;
    pending_wake_word_.clear();
    xSemaphoreGive(notification_mutex_);

    ESP_LOGI(TAG, "Delivering deferred notification: %s", wake_word.c_str());
    // Same wake-window guard as TriggerNotification: this deferred delivery
    // also opens a conversation while face mode is still streaming events.
    SscmaCamera::SetDropEvents(true);
    Application::GetInstance().Schedule([wake_word]() {
        Application::GetInstance().WakeWordInvoke(wake_word);
    });
    return true;
}
