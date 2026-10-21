#ifndef FACE_RECOGNITION_H
#define FACE_RECOGNITION_H

#include "face_database.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <mutex>
#include <string>

// Multi-frame voting configuration
#define FACE_VOTING_BUFFER_SIZE 5
#define FACE_VOTING_MIN_VOTES 3
#define FACE_VOTING_WINDOW_US_DEFAULT 2000000  // 2 seconds default (maps to detect_duration_sec)

// Parsed face data from Himax
struct HimaxFaceData {
    int box_x;
    int box_y;
    int box_w;
    int box_h;
    int score;           // Detection confidence (0-100)
    float quality;       // Quality score based on pose angles
    float embedding[FACE_EMBEDDING_DIM];
    bool has_embedding;
};

// Multi-frame voting buffer for stable recognition
class FaceVotingBuffer {
public:
    FaceVotingBuffer();
    ~FaceVotingBuffer();

    void AddEmbedding(const float* embedding, int64_t timestamp_us = 0);
    bool GetConsensusEmbedding(float* out_embedding);
    void Clear();
    int GetVoteCount();
    void SetWindow(int64_t window_us);

private:
    struct VoteEntry {
        float embedding[FACE_EMBEDDING_DIM];
        int64_t timestamp_us;
        bool valid;
    };

    VoteEntry buffer_[FACE_VOTING_BUFFER_SIZE];
    int write_index_;
    int valid_count_;
    int64_t window_us_ = FACE_VOTING_WINDOW_US_DEFAULT;
    SemaphoreHandle_t mutex_;

    void PruneOldEntries(int64_t current_time_us);
};

// Wake-word size budget. The official cloud rejects listen/state=detect
// payloads above a black-box size heuristic, answering with
// {"type":"alert","status":"ERROR","message":"Detect is only for wake words,
//  do not send long texts."} — which application.cc renders as an ERROR screen.
// The metric is neither character count nor byte count nor BPE tokens (measured:
// "a"x40 passes while 40 distinct chars are rejected; a 48-byte CJK string passes
// while a 26-byte tagged one is rejected), so no formula is reproducible. What is
// reproducible is the envelope: across 60 probes every payload <=25 UTF-8 bytes
// passed, and no rejection occurred below 26 bytes. Budget in BYTES, not chars.
// The <f></f> / <d></d> wrappers cost 7 bytes, leaving 18 for the payload.
constexpr size_t kWakeWordMaxBytes = 25;
constexpr size_t kWakeWordPayloadMaxBytes = kWakeWordMaxBytes - 7;

// Truncate to at most max_bytes, backing off to a UTF-8 character boundary so a
// long name can never emit a half sequence (which would break the server parse).
std::string TruncateUtf8(const std::string& s, size_t max_bytes);

// Face recognition state machine (mirrors object detection)
enum class FaceDetectionState {
    DETECTING,  // Normal: accumulating votes, matching
    COOLDOWN,   // After trigger: waiting for interval + face to leave
};

// Main face recognition controller
class FaceRecognition {
public:
    static FaceRecognition& GetInstance();

    // Face detection mode
    bool IsEnabled() const;
    void SetEnabled(bool enabled);

    // Process face data from Himax (ignored during COOLDOWN)
    void ProcessFaceData(const HimaxFaceData& face_data);

    // Face presence tracking (called from on_event callback every frame)
    void NotifyFacePresent();
    void NotifyNoFace();

    // Get last recognition result
    FaceMatchResult GetLastMatch() const;
    // Monotonic time (esp_timer_get_time, us) when last_match_ was last set.
    // 0 if no match has ever been recorded. Use to gate on freshness.
    int64_t GetLastMatchTimeUs() const;

    // Conversation speaker identity. The camera task freezes who we are talking
    // to when a conversation starts (CaptureCurrentSpeaker) and clears it when it
    // ends (ClearCurrentSpeaker). Queried by the self.conversation.speaker MCP
    // tool so permission-gated commands can check "who is talking".
    struct SpeakerIdentity {
        bool valid = false;       // false = unknown (stranger / no recent face)
        std::string name;         // decoded display name
        int subject_id = 0;       // warehouse subject id; 0 = unknown / not set
        float similarity = 0.0f;  // face match cosine similarity
    };
    // Freeze the speaker from the most recent fresh, confident match (<=15s old).
    void CaptureCurrentSpeaker();
    void ClearCurrentSpeaker();
    SpeakerIdentity GetCurrentSpeaker() const;
    // Monotonic conversation counter, bumped every time CaptureCurrentSpeaker
    // runs (i.e. each new conversation's rising edge). The backend uses it to
    // implement verify-once-per-conversation (仅首次): a verify whose conv_seq
    // matches the last verified one is served from cache (免验), a new conv_seq
    // triggers a fresh device verify. Never 0 after the first conversation.
    uint32_t GetConversationSeq() const;
    // Overwrite the frozen speaker with an on-demand match (self.face.identify),
    // so later tools in the same conversation see the refreshed identity.
    void SetCurrentSpeaker(const SpeakerIdentity& speaker);

    // Deferred notification: stores wake word and delivers when device returns to idle.
    void SetPendingNotification(const std::string& wake_word);
    bool DeliverPendingNotification();

    // Configuration - timing parameters (shared with object detection)
    void SetValidationDuration(int seconds);
    void SetCooldownInterval(int seconds);

    // Configuration - face-specific
    void SetMatchThreshold(float threshold);
    float GetMatchThreshold() const;

    // Familiar DND mode (熟人免打扰)
    void SetFamiliarMode(bool enabled);
    bool IsFamiliarMode() const;

    // Cooldown management (avoid repeated notifications)
    bool IsInCooldown() const;
    void StartCooldown();

private:
    FaceRecognition();
    ~FaceRecognition() = default;

    FaceRecognition(const FaceRecognition&) = delete;
    FaceRecognition& operator=(const FaceRecognition&) = delete;

    void TriggerNotification(const std::string& wake_word);
    void SuppressCurrentFace();
    void HandleRecognitionResult(const FaceMatchResult& result);
    void HandleUnknownPersonDetected(bool is_stranger_alert);

    bool enabled_;
    bool familiar_mode_;

    FaceVotingBuffer voting_buffer_;
    FaceMatchResult last_match_;
    int64_t last_match_time_us_ = 0;
    SpeakerIdentity current_speaker_;  // frozen for the active conversation
    uint32_t conversation_seq_ = 0;    // bumped on each CaptureCurrentSpeaker
    static constexpr int64_t kSpeakerMaxAgeUs = 15LL * 1000000;  // 15s freshness

    float match_threshold_;

    // State machine (mirrors object detection IDLE/VALIDATING/COOLDOWN)
    FaceDetectionState state_ = FaceDetectionState::DETECTING;
    int64_t cooldown_start_time_ = 0;
    int64_t cooldown_duration_us_;
    int64_t last_face_seen_us_ = 0;
    bool need_start_cooldown_ = false;  // Defer cooldown start until face mode resumes
    static constexpr int64_t kFaceGoneDebounceUs = 2000000;  // 2s: face must be gone this long

    // Unknown-face grace: a detected-but-unmatched face (approaching / far / angled)
    // does NOT immediately greet "person" — we hold for kUnknownGraceUs measured from
    // the first unmatched consensus, giving an approaching known face time to resolve
    // to a name. A match within the window greets by name (grace reset); only if the
    // window elapses with no match do we declare person/stranger. Best of both:
    // familiar → name (no "person" flash), true stranger → still detected (after grace).
    int64_t unknown_grace_start_us_ = 0;  // 0 = not in an unknown streak
    static constexpr int64_t kUnknownGraceUs = 2000000;  // 2s

    std::string last_notified_name_;

    // Deferred notification state (accessed from SSCMA process task + camera task)
    SemaphoreHandle_t notification_mutex_;
    std::string pending_wake_word_;
    bool has_pending_notification_;
    mutable std::mutex state_mutex_;
};

#endif // FACE_RECOGNITION_H
