#include "remote_display_http_server.h"
#include "remote_display.h"
#include "sscma_camera.h"
#include "face_database.h"
#include "face_recognition.h"
#include "settings.h"
#include "application.h"

#include <vector>
#include <cmath>

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cJSON.h>
#include <cstring>
#include <cstdlib>
#include <string>
#include <mbedtls/base64.h>
#include <sys/socket.h>

namespace {
constexpr size_t kRemoteWakeMaxBody = 2048;
constexpr size_t kRemoteWakeContextMax = 25;
constexpr size_t kRemoteWakeSpeakMax = 512;
constexpr int64_t kRemoteWakeDedupeUs = 15000000;
std::string RemoteWakeToken() {
    Settings s("remote_wake", false);
    return s.GetString("token", "");
}
bool RemoteWakeEnabled() {
    Settings s("remote_wake", false);
    return s.GetBool("enabled", false) && !s.GetString("token", "").empty();
}
bool HasRemoteWakeAuth(httpd_req_t* req) {
    std::string expected = RemoteWakeToken();
    size_t len = httpd_req_get_hdr_value_len(req, "Authorization");
    if (expected.empty() || len < 7 || len > 255)
        return false;
    std::string got(len + 1, '\0');
    if (httpd_req_get_hdr_value_str(req, "Authorization", got.data(), got.size()) != ESP_OK)
        return false;
    got.resize(len);
    return got.compare(0, 7, "Bearer ") == 0 && got.substr(7) == expected;
}
bool ValidUtf8(const std::string& s) {
    for (size_t i = 0; i < s.size();) {
        const uint8_t c = static_cast<uint8_t>(s[i++]);
        if (c == 0)
            return false;
        if (c <= 0x7f)
            continue;
        size_t n = c >= 0xc2 && c <= 0xdf   ? 1
                   : c >= 0xe0 && c <= 0xef ? 2
                   : c >= 0xf0 && c <= 0xf4 ? 3
                                            : 0;
        if (n == 0 || i + n > s.size())
            return false;
        uint32_t cp = c & ((1u << (6 - n)) - 1u);
        for (size_t j = 0; j < n; ++j) {
            const uint8_t d = static_cast<uint8_t>(s[i++]);
            if ((d & 0xc0) != 0x80)
                return false;
            cp = (cp << 6) | (d & 0x3f);
        }
        if ((n == 2 && cp < 0x800) || (n == 3 && cp < 0x10000) || (cp >= 0xd800 && cp <= 0xdfff) ||
            cp > 0x10ffff)
            return false;
    }
    return true;
}
bool ContainsJsonNulEscape(const std::string& body) {
    for (size_t pos = 0; (pos = body.find("u0000", pos)) != std::string::npos; ++pos) {
        size_t slashes = 0;
        for (size_t p = pos; p > 0 && body[p - 1] == '\\'; --p)
            ++slashes;
        if ((slashes & 1u) != 0)
            return true;
    }
    return false;
}
const char* RemoteWakeState(DeviceState state) {
    switch (state) {
        case kDeviceStateIdle:
            return "idle";
        case kDeviceStateSpeaking:
            return "speaking";
        case kDeviceStateListening:
            return "listening";
        case kDeviceStateConnecting:
            return "connecting";
        default:
            return "busy";
    }
}
}

static const char* TAG = "RemoteDisplayHttp";

RemoteDisplayHttpServer* RemoteDisplayHttpServer::instance_ = nullptr;

// Max request body size (256 bytes is plenty for {"ws_url":"ws://..."})
#define MAX_REQ_BODY 256

// ---- /api/face/batch-update limits ----------------------------------------
// The on-device face/NPU model identity. The batch-update full-sync MUST carry
// a matching model_tag, otherwise the device would ingest embeddings from a
// different model (incompatible vector space) and silently mis-match faces.
// MobileFaceNet, 128-D. Wire embeddings are fp16 (256B/face) by default; a
// float32 (512B) legacy path is still accepted. Bump the suffix if the embedder
// (vector space) ever changes — the byte ENCODING is carried separately in the
// `embedding_format` field, not the model_tag.
#define DEVICE_FACE_MODEL_TAG "we2-mfnr6-128-v1"
// Upper bound on how many faces a single batch may carry (matches FACE_MAX_COUNT).
#define FACE_BATCH_MAX_FACES FACE_MAX_COUNT
// Dedicated body cap for the batch handler — MAX_REQ_BODY=256 is far too small.
// fp16 path: 20 faces * (~344 b64 chars for 256B + name + JSON envelope) ≈ 8KB.
// 16KB ceiling keeps margin (incl. the legacy float32 512B/face path at ~16KB)
// while bounding the SPIRAM allocation to prevent OOM heaping.
#define FACE_BATCH_MAX_BODY (16 * 1024)

// Rate limit for /api/face/embed: minimum interval between *accepted* calls.
// Caps hammering of Himax / internal SRAM. Tune as needed; verify-once callers
// never hit this (their spacing is far above 1s).
#define FACE_EMBED_MIN_INTERVAL_US (1000 * 1000)  // 1s

RemoteDisplayHttpServer::~RemoteDisplayHttpServer() {
    Stop();
}

// ---------- HTTP handlers (static) ----------

esp_err_t RemoteDisplayHttpServer::HandleStartCast(httpd_req_t* req) {
    char buf[MAX_REQ_BODY];
    int received = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    buf[received] = '\0';

    cJSON* root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON* ws_url = cJSON_GetObjectItem(root, "ws_url");
    if (!ws_url || !cJSON_IsString(ws_url) || strlen(ws_url->valuestring) == 0) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing ws_url");
        return ESP_FAIL;
    }

    const char* requested_url = ws_url->valuestring;
    if (strncmp(requested_url, "ws://", 5) != 0 &&
        strncmp(requested_url, "wss://", 6) != 0) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid ws_url");
        return ESP_FAIL;
    }

    auto* remote = RemoteDisplay::GetInstance();
    std::string server_url = requested_url;
    cJSON_Delete(root);

    // Persist the new target as disabled before cancelling the old lifecycle.
    // This guarantees a stale retry can neither reconnect the previous target
    // nor survive a failed attempt to the new one.
    auto config = RemoteDisplay::LoadConfig();
    config.server_url = server_url;
    config.enabled = false;
    RemoteDisplay::SaveConfig(config);
    remote->Stop();

    bool ok = remote->Start(server_url, 3000);

    if (ok) {
        config = RemoteDisplay::LoadConfig();
        config.server_url = server_url;
        config.enabled = true;
        RemoteDisplay::SaveConfig(config);
        if (!remote->IsRunning()) {
            remote->RequestReconnect();
        }
    }

    cJSON* resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "success", ok);
    cJSON_AddBoolToObject(resp, "casting", remote->IsRunning());
    cJSON_AddBoolToObject(resp, "enabled", ok);
    if (!ok) {
        cJSON_AddStringToObject(resp, "error", "Failed to connect");
    }
    char* resp_str = cJSON_PrintUnformatted(resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);
    cJSON_Delete(resp);
    return ESP_OK;
}

esp_err_t RemoteDisplayHttpServer::HandleStopCast(httpd_req_t* req) {
    auto config = RemoteDisplay::LoadConfig();
    config.enabled = false;
    RemoteDisplay::SaveConfig(config);

    auto* remote = RemoteDisplay::GetInstance();
    remote->Stop();

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true}");
    return ESP_OK;
}

esp_err_t RemoteDisplayHttpServer::HandleStatus(httpd_req_t* req) {
    auto* remote = RemoteDisplay::GetInstance();
    auto config = RemoteDisplay::LoadConfig();

    cJSON* resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "casting", remote->IsRunning());
    cJSON_AddBoolToObject(resp, "enabled", config.enabled);
    cJSON_AddBoolToObject(resp, "configured", !config.server_url.empty());
    cJSON_AddStringToObject(resp, "server_url", config.server_url.c_str());

    char* resp_str = cJSON_PrintUnformatted(resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);
    cJSON_Delete(resp);
    return ESP_OK;
}

esp_err_t RemoteDisplayHttpServer::HandleWakeStatus(httpd_req_t* req) {
    const bool enabled = RemoteWakeEnabled();
    const bool authenticated = !enabled || HasRemoteWakeAuth(req);
    if (!authenticated) {
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"error\":\"unauthorized\",\"authenticated\":false}");
    }
    cJSON* r = cJSON_CreateObject();
    cJSON_AddStringToObject(r, "state",
                            RemoteWakeState(Application::GetInstance().GetDeviceState()));
    cJSON_AddBoolToObject(r, "enabled", enabled);
    cJSON_AddBoolToObject(r, "authenticated", authenticated);
    if (!enabled)
        cJSON_AddStringToObject(r, "reason", "not_configured");
    cJSON* c = cJSON_AddObjectToObject(r, "capabilities");
    cJSON_AddNumberToObject(c, "max_context_bytes", kRemoteWakeContextMax);
    cJSON_AddNumberToObject(c, "max_speak_bytes", kRemoteWakeSpeakMax);
#if CONFIG_SEND_WAKE_WORD_DATA
    cJSON_AddBoolToObject(c, "context", enabled);
    cJSON_AddBoolToObject(c, "speak", enabled);
    cJSON_AddStringToObject(c, "speak_backend", "local_server_device_call_tts");
#else
    cJSON_AddBoolToObject(c, "context", false);
    cJSON_AddBoolToObject(c, "speak", false);
    cJSON_AddStringToObject(c, "reason", "send_wake_word_data_disabled");
#endif
    char* out = cJSON_PrintUnformatted(r);
    httpd_resp_set_type(req, "application/json");
    esp_err_t e = httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
    cJSON_free(out);
    cJSON_Delete(r);
    return e;
}

esp_err_t RemoteDisplayHttpServer::HandleWake(httpd_req_t* req) {
    if (!RemoteWakeEnabled()) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "remote wake disabled");
        return ESP_FAIL;
    }
    if (!HasRemoteWakeAuth(req)) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "unauthorized");
        return ESP_FAIL;
    }
#if !CONFIG_SEND_WAKE_WORD_DATA
    httpd_resp_set_status(req, "501 Not Implemented");
    httpd_resp_sendstr(req, "wake word data is disabled");
    return ESP_FAIL;
#else
    if (req->content_len == 0 || req->content_len > kRemoteWakeMaxBody) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
        return ESP_FAIL;
    }
    std::string body(req->content_len, '\0');
    size_t off = 0;
    while (off < body.size()) {
        int n = httpd_req_recv(req, body.data() + off, body.size() - off);
        if (n <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
            return ESP_FAIL;
        }
        off += n;
    }
    const char* end = nullptr;
    cJSON* root = cJSON_ParseWithLengthOpts(body.data(), body.size(), &end, 0);
    cJSON* id = root ? cJSON_GetObjectItem(root, "request_id") : nullptr;
    cJSON* mode = root ? cJSON_GetObjectItem(root, "mode") : nullptr;
    cJSON* text = root ? cJSON_GetObjectItem(root, "text") : nullptr;
    cJSON* ttl = root ? cJSON_GetObjectItem(root, "ttl_ms") : nullptr;
    if (ContainsJsonNulEscape(body) || !root || !cJSON_IsObject(root) ||
        end != body.data() + body.size() || !cJSON_IsString(id) || !cJSON_IsString(mode) ||
        !cJSON_IsString(text) || !cJSON_IsNumber(ttl) || ttl->valuedouble != ttl->valueint ||
        ttl->valueint < 1 || ttl->valueint > 15000 || strlen(id->valuestring) == 0 ||
        strlen(id->valuestring) > 96 ||
        (strcmp(mode->valuestring, "context") && strcmp(mode->valuestring, "speak")) ||
        strlen(text->valuestring) == 0) {
        if (root) {
            cJSON_Delete(root);
        }
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid wake request");
        return ESP_FAIL;
    }
    int ttl_ms = ttl->valueint;
    std::string rid = id->valuestring, m = mode->valuestring, payload = text->valuestring;
    cJSON_Delete(root);
    if (!ValidUtf8(payload) ||
        payload.size() > (m == "context" ? kRemoteWakeContextMax : kRemoteWakeSpeakMax)) {
        httpd_resp_set_status(req, "413 Content Too Large");
        httpd_resp_sendstr(req, "text too long or invalid UTF-8");
        return ESP_FAIL;
    }
    int64_t now = esp_timer_get_time();
    const std::string fingerprint = m + "\n" + payload;
    if (instance_ != nullptr) {
        std::lock_guard<std::mutex> lock(instance_->wake_request_mutex_);
        while (!instance_->wake_request_history_.empty() &&
               now - instance_->wake_request_history_.front().accepted_us >= kRemoteWakeDedupeUs)
            instance_->wake_request_history_.pop_front();
        for (const auto& record : instance_->wake_request_history_)
            if (record.id == rid) {
                httpd_resp_set_type(req, "application/json");
                if (record.fingerprint == fingerprint) {
                    httpd_resp_set_status(req, "202 Accepted");
                    return httpd_resp_sendstr(req, "{\"status\":\"duplicate\"}");
                }
                httpd_resp_set_status(req, "409 Conflict");
                return httpd_resp_sendstr(req, "{\"error\":\"request_id_conflict\"}");
            }
    }
    if (Application::GetInstance().GetDeviceState() != kDeviceStateIdle ||
        !Application::GetInstance().RequestRemoteWake(
            rid, m == "speak" ? "[device_call] " + payload : payload, now + ttl_ms * 1000LL)) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"error\":\"device_busy\"}");
    }
    if (instance_ != nullptr) {
        std::lock_guard<std::mutex> lock(instance_->wake_request_mutex_);
        instance_->wake_request_history_.push_back({rid, fingerprint, now});
        while (instance_->wake_request_history_.size() > 32)
            instance_->wake_request_history_.pop_front();
    }
    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"accepted\"}");
#endif
}

// ---------- Face embedding (on-device, for cloud callers) ----------

esp_err_t RemoteDisplayHttpServer::HandleFaceEmbed(httpd_req_t* req) {
    SscmaCamera* camera = instance_ ? instance_->camera_ : nullptr;
    if (camera == nullptr) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"camera_unavailable\"}");
        return ESP_OK;
    }

    // State gate: reject during OTA ("upgrading"), while passive greeting is
    // actively running ("greeting_active"), or when Himax is otherwise busy
    // ("busy"). Allowed during a voice conversation (intended use → warm path).
    const char* block = camera->FaceEmbedBlockReason();
    if (block != nullptr) {
        // upgrading / greeting_active are state conflicts (409); busy is transient (503).
        const char* status = (strcmp(block, "busy") == 0)
                                 ? "503 Service Unavailable"
                                 : "409 Conflict";
        httpd_resp_set_status(req, status);
        httpd_resp_set_type(req, "application/json");
        char body[64];
        snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"%s\"}", block);
        httpd_resp_sendstr(req, body);
        return ESP_OK;
    }

    // Rate limit: cap call frequency. Only *accepted* calls advance the
    // timestamp, so a fast poller is throttled but never permanently starved.
    int64_t now_us = esp_timer_get_time();
    int64_t last_us = instance_->last_face_call_us_.load();
    if (last_us != 0 && (now_us - last_us) < FACE_EMBED_MIN_INTERVAL_US) {
        httpd_resp_set_status(req, "429 Too Many Requests");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"rate_limited\"}");
        return ESP_OK;
    }
    instance_->last_face_call_us_.store(now_us);

    // SRAM instrumentation: this endpoint runs on the (small) httpd worker and
    // drives the deep sscma_client + SPI chain. Log internal-SRAM headroom so we
    // can size the stack / socket budget from real data instead of guessing.
    size_t sram_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

    float embedding[FACE_EMBEDDING_DIM];
    SscmaCamera::SingleShotTiming t;
    bool ok = camera->BenchSingleShotFaceEmbedding(embedding, &t);

    size_t sram_after = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t sram_min = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    UBaseType_t stack_left = uxTaskGetStackHighWaterMark(nullptr);
    ESP_LOGI(TAG,
        "[FaceEmbed] ok=%d total=%ldus SRAM before=%u after=%u min=%u stack_hwm=%u",
        ok, (long)t.total_us, (unsigned)sram_before, (unsigned)sram_after,
        (unsigned)sram_min, (unsigned)stack_left);

    httpd_resp_set_type(req, "application/json");

    if (!ok) {
        // No face detected / timeout — structured error, HTTP 200 (same as the
        // MCP tool) so the caller can deny rather than retry blindly.
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"no_face_or_timeout\"}");
        return ESP_OK;
    }

    // Encode the 128 float32 LE (512 bytes) as base64 — identical wire format to
    // the warehouse face library so the server side needs zero conversion.
    const size_t raw_len = sizeof(float) * FACE_EMBEDDING_DIM;  // 512
    const size_t b64_buf = ((raw_len + 2) / 3) * 4 + 16;
    uint8_t* b64 = static_cast<uint8_t*>(heap_caps_malloc(b64_buf, MALLOC_CAP_SPIRAM));
    if (b64 == nullptr) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"oom\"}");
        return ESP_OK;
    }
    size_t b64_len = 0;
    int rc = mbedtls_base64_encode(b64, b64_buf, &b64_len,
                                   reinterpret_cast<const uint8_t*>(embedding), raw_len);
    if (rc != 0) {
        heap_caps_free(b64);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"b64_encode_failed\"}");
        return ESP_OK;
    }

    std::string resp;
    resp.reserve(b64_len + 96);
    resp.append("{\"ok\":true,\"format\":\"float32_le_b64\",\"dim\":");
    resp.append(std::to_string(FACE_EMBEDDING_DIM));
    resp.append(",\"score\":");
    resp.append(std::to_string(t.face_score));
    resp.append(",\"embedding_b64\":\"");
    resp.append(reinterpret_cast<const char*>(b64), b64_len);
    resp.append("\"}");
    heap_caps_free(b64);

    httpd_resp_sendstr(req, resp.c_str());
    return ESP_OK;
}

// ---------- Face DB full sync (server library push) ----------

esp_err_t RemoteDisplayHttpServer::HandleFaceBatchUpdate(httpd_req_t* req) {
    httpd_resp_set_type(req, "application/json");

    // 1) Bounded read into a SPIRAM buffer (MAX_REQ_BODY=256 is far too small).
    int total = req->content_len;
    if (total <= 0 || total > FACE_BATCH_MAX_BODY) {
        httpd_resp_set_status(req, "413 Payload Too Large");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"body_too_large\"}");
        return ESP_OK;
    }
    char* body = static_cast<char*>(heap_caps_malloc(total + 1, MALLOC_CAP_SPIRAM));
    if (body == nullptr) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"oom\"}");
        return ESP_OK;
    }
    int off = 0;
    while (off < total) {
        int r = httpd_req_recv(req, body + off, total - off);
        if (r <= 0) {
            heap_caps_free(body);
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"recv_failed\"}");
            return ESP_OK;
        }
        off += r;
    }
    body[total] = '\0';

    cJSON* root = cJSON_Parse(body);
    heap_caps_free(body);
    if (root == nullptr) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid_json\"}");
        return ESP_OK;
    }

    // 2) model_tag validation — reject the WHOLE batch (4xx) on mismatch so we
    //    never mix vector spaces from different models.
    cJSON* mt = cJSON_GetObjectItem(root, "model_tag");
    if (!cJSON_IsString(mt) || strcmp(mt->valuestring, DEVICE_FACE_MODEL_TAG) != 0) {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"model_tag_mismatch\"}");
        return ESP_OK;
    }

    // 2b) embedding_format dispatch. The decoded byte width per face depends on
    //     the encoding; we never reinterpret bytes as the wrong width.
    //       "fp16"    -> 256 bytes = 128 IEEE-754 binary16 LE  (current pinned contract)
    //       "float32" -> 512 bytes = 128 float32 LE            (legacy compat)
    //       (absent)  -> "float32" (old clients predate the field)
    //       unknown   -> 409 Conflict
    //     FUTURE int8 extension point: add an `else if "int8"` branch here that
    //     sets expected_bytes = 128 (+ a per-vector scale read elsewhere) and a
    //     FMT_INT8 mode; the per-face loop below would dequantize int8 -> float32
    //     exactly like fp16 does. faces_ / NVS-save paths need no change because
    //     ReplaceAll receives canonical float32 regardless of wire encoding.
    enum EmbFmt { FMT_FP16, FMT_FLOAT32 };
    EmbFmt emb_fmt = FMT_FLOAT32;
    size_t expected = FACE_EMBEDDING_DIM * sizeof(float);  // float32 default = 512
    {
        cJSON* jfmt = cJSON_GetObjectItem(root, "embedding_format");
        const char* fmt = (cJSON_IsString(jfmt) && jfmt->valuestring) ? jfmt->valuestring : "float32";
        if (strcmp(fmt, "fp16") == 0) {
            emb_fmt = FMT_FP16;
            expected = FACE_EMBEDDING_DIM * sizeof(uint16_t);  // 256
        } else if (strcmp(fmt, "float32") == 0) {
            emb_fmt = FMT_FLOAT32;
            expected = FACE_EMBEDDING_DIM * sizeof(float);     // 512
        // } else if (strcmp(fmt, "int8") == 0) { /* FUTURE: 128B + scale -> float32 */ }
        } else {
            cJSON_Delete(root);
            httpd_resp_set_status(req, "409 Conflict");
            httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"unknown_embedding_format\"}");
            return ESP_OK;
        }
    }

    cJSON* faces = cJSON_GetObjectItem(root, "faces");
    if (!cJSON_IsArray(faces)) {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing_faces\"}");
        return ESP_OK;
    }
    int n = cJSON_GetArraySize(faces);
    if (n > FACE_BATCH_MAX_FACES) {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"too_many_faces\"}");
        return ESP_OK;
    }

    // 3) Decode each face → FaceEntry list. The decoded byte width is `expected`
    //    (256 fp16 / 512 float32, set by the dispatch above). Either way the
    //    FaceEntry embedding is filled with CANONICAL float32 before ReplaceAll.
    //    Validate name length + embedding width up front so ReplaceAll never has
    //    to truncate/skip.
    std::vector<FaceEntry> entries;
    entries.reserve(n);
    const char* perr = nullptr;
    for (int i = 0; i < n; i++) {
        cJSON* f = cJSON_GetArrayItem(faces, i);
        cJSON* jname = cJSON_GetObjectItem(f, "name");
        cJSON* jsid  = cJSON_GetObjectItem(f, "subject_id");
        cJSON* jemb  = cJSON_GetObjectItem(f, "embedding_b64");
        if (!cJSON_IsString(jname) || !cJSON_IsString(jemb)) { perr = "bad_face_fields"; break; }
        size_t nlen = strlen(jname->valuestring);
        if (nlen == 0 || nlen >= FACE_NAME_MAX_LEN) { perr = "bad_name_len"; break; }

        FaceEntry e;
        e.name = jname->valuestring;
        e.subject_id = cJSON_IsNumber(jsid) ? jsid->valueint : 0;

        // 512B buffer covers the larger (float32) format; fp16 uses the first 256B.
        uint8_t decoded[FACE_EMBEDDING_DIM * sizeof(float)];
        size_t dlen = 0;
        int rc = mbedtls_base64_decode(decoded, sizeof(decoded), &dlen,
                    reinterpret_cast<const unsigned char*>(jemb->valuestring),
                    strlen(jemb->valuestring));
        if (rc != 0 || dlen != expected) { perr = "bad_embedding"; break; }

        if (emb_fmt == FMT_FP16) {
            // 128 binary16 LE -> float32 (byte-wise unpack: alignment-safe).
            for (int j = 0; j < FACE_EMBEDDING_DIM; j++) {
                uint16_t half = (uint16_t)decoded[2 * j] |
                                ((uint16_t)decoded[2 * j + 1] << 8);
                e.embedding[j] = FaceDatabase::HalfToFloat32(half);
            }
        } else {  // FMT_FLOAT32
            memcpy(e.embedding, decoded, FACE_EMBEDDING_DIM * sizeof(float));
        }
        entries.push_back(std::move(e));
    }

    // 3b) Optional device-side face config pushed alongside the library (the
    //     warehouse platform's face config page is the source of truth).
    //     Parsed here while root is alive; applied only after ReplaceAll
    //     succeeds so a rejected batch never half-applies config.
    //       match_threshold   int 0-100 -> NVS face.threshold (applies live)
    //       identify_mode     "local"|"lan" -> NVS face.id_mode
    //       identify_endpoint string       -> NVS face.id_url
    //       identify_token    string       -> NVS face.id_token
    int cfg_threshold = -1;
    std::string cfg_id_mode, cfg_id_url, cfg_id_token, cfg_pull_token;
    bool has_id_cfg = false;
    bool has_pull_token = false;
    {
        cJSON* jt = cJSON_GetObjectItem(root, "match_threshold");
        if (cJSON_IsNumber(jt) && jt->valueint >= 0 && jt->valueint <= 100) {
            cfg_threshold = jt->valueint;
        }
        cJSON* jm = cJSON_GetObjectItem(root, "identify_mode");
        if (cJSON_IsString(jm) && (strcmp(jm->valuestring, "local") == 0 ||
                                   strcmp(jm->valuestring, "lan") == 0)) {
            has_id_cfg = true;
            cfg_id_mode = jm->valuestring;
            cJSON* ju = cJSON_GetObjectItem(root, "identify_endpoint");
            cJSON* jk = cJSON_GetObjectItem(root, "identify_token");
            if (cJSON_IsString(ju) && strlen(ju->valuestring) < 160) cfg_id_url = ju->valuestring;
            if (cJSON_IsString(jk) && strlen(jk->valuestring) < 128) cfg_id_token = jk->valuestring;
        }
        // pull_token (§8d): independent of identify_mode — both local and lan
        // modes must receive it so the backend-direct current-speaker pull
        // authenticates regardless of whether a remote endpoint is configured.
        cJSON* jp = cJSON_GetObjectItem(root, "pull_token");
        if (cJSON_IsString(jp) && strlen(jp->valuestring) < 128) {
            has_pull_token = true;
            cfg_pull_token = jp->valuestring;
        }
    }
    cJSON_Delete(root);

    if (perr != nullptr) {
        httpd_resp_set_status(req, "400 Bad Request");
        char resp[64];
        snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"%s\"}", perr);
        httpd_resp_sendstr(req, resp);
        return ESP_OK;
    }

    // 4) Apply under an inference pause so the write window can't race the
    //    passive-greeting / Match path. Resume runs on EVERY exit path below
    //    (finally-equivalent), even when ReplaceAll fails.
    SscmaCamera* camera = instance_ ? instance_->camera_ : nullptr;
    if (camera != nullptr) camera->PauseInference();
    bool applied = FaceDatabase::GetInstance().ReplaceAll(entries);
    if (camera != nullptr) camera->ResumeInference();

    if (!applied) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"persist_failed\"}");
        return ESP_OK;
    }

    // 5) Persist pushed config; threshold applies live. A re-push overwrites
    //    NVS wholesale (platform config is authoritative, no merge).
    if (cfg_threshold >= 0 || has_id_cfg || has_pull_token) {
        Settings settings("face", true);
        if (cfg_threshold >= 0) {
            settings.SetInt("threshold", cfg_threshold);
            FaceRecognition::GetInstance().SetMatchThreshold(cfg_threshold / 100.0f);
            ESP_LOGI(TAG, "face threshold <- %d (pushed)", cfg_threshold);
        }
        if (has_id_cfg) {
            settings.SetString("id_mode", cfg_id_mode);
            settings.SetString("id_url", cfg_id_url);
            settings.SetString("id_token", cfg_id_token);
            ESP_LOGI(TAG, "face identify mode <- %s url=%s (pushed)",
                     cfg_id_mode.c_str(), cfg_id_url.c_str());
        }
        if (has_pull_token) {
            // §8d: independent per-device pull_token for current-speaker auth.
            settings.SetString("pull_token", cfg_pull_token);
            ESP_LOGI(TAG, "face pull_token <- (pushed, len=%d)",
                     (int)cfg_pull_token.size());
        }
    }

    std::string resp = "{\"ok\":true,\"applied_count\":";
    resp += std::to_string(entries.size());
    resp += "}";
    httpd_resp_sendstr(req, resp.c_str());
    return ESP_OK;
}

// ---------- Backend-direct identity pull (§8c/§9) ----------

// Build the {valid,name,subject_id,similarity,mode,age_ms} response body with
// cJSON so names and mode values containing control characters stay valid JSON.
static std::string BuildSpeakerJson(const FaceRecognition::SpeakerIdentity& s,
                                    const std::string& mode, int64_t age_ms) {
    static constexpr const char* kFallback =
        "{\"valid\":false,\"name\":\"\",\"subject_id\":0,"
        "\"similarity\":0,\"mode\":\"\",\"age_ms\":0,\"conv_seq\":0}";
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr) {
        return kFallback;
    }

    // Round to the same four-decimal precision used by the previous formatter.
    double similarity = std::round(static_cast<double>(s.similarity) * 10000.0) / 10000.0;
    bool ok =
        cJSON_AddBoolToObject(root, "valid", s.valid) != nullptr &&
        cJSON_AddStringToObject(root, "name", s.name.c_str()) != nullptr &&
        cJSON_AddNumberToObject(root, "subject_id", s.subject_id) != nullptr &&
        cJSON_AddNumberToObject(root, "similarity", similarity) != nullptr &&
        cJSON_AddStringToObject(root, "mode", mode.c_str()) != nullptr &&
        cJSON_AddNumberToObject(root, "age_ms", age_ms) != nullptr &&
        cJSON_AddNumberToObject(root, "conv_seq",
            FaceRecognition::GetInstance().GetConversationSeq()) != nullptr;
    if (!ok) {
        cJSON_Delete(root);
        return kFallback;
    }

    char* encoded = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (encoded == nullptr) {
        return kFallback;
    }
    std::string result(encoded);
    cJSON_free(encoded);
    return result;
}

esp_err_t RemoteDisplayHttpServer::HandleFaceCurrentSpeaker(httpd_req_t* req) {
    httpd_resp_set_type(req, "application/json");

    // --- Auth: X-Face-Token must equal the NVS-pushed pull_token (batch-update
    //     pull_token, §8d). Independent of id_token/identify_token so it works in
    //     both local and lan modes. Fail-closed: missing header, or an
    //     unprovisioned (empty) device token, both -> 401. Prevents same-LAN
    //     hosts from impersonating the backend to pull an identity.
    std::string expected;
    {
        Settings settings("face", false);
        expected = settings.GetString("pull_token", "");
    }
    std::string got;
    size_t tlen = httpd_req_get_hdr_value_len(req, "X-Face-Token");
    if (tlen > 0) {
        got.resize(tlen + 1);
        if (httpd_req_get_hdr_value_str(req, "X-Face-Token", &got[0], tlen + 1) == ESP_OK) {
            got.resize(tlen);
        } else {
            got.clear();
        }
    }
    if (expected.empty() || got != expected) {
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_sendstr(req, "{\"valid\":false,\"error\":\"unauthorized\"}");
        return ESP_OK;
    }

    // --- Parse fresh=0|1 from the query string (default 0).
    int fresh = 0;
    {
        char query[64];
        if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
            char val[8];
            if (httpd_query_key_value(query, "fresh", val, sizeof(val)) == ESP_OK) {
                fresh = atoi(val);
            }
        }
    }

    auto& rec = FaceRecognition::GetInstance();
    std::string mode;
    {
        Settings settings("face", false);
        mode = settings.GetString("id_mode", "local");
    }

    // --- fresh=0: return the in-memory frozen speaker (zero hardware action).
    //     age_ms = now - GetLastMatchTimeUs (age of the frozen identity); -1 if
    //     no match has ever been recorded.
    if (fresh == 0) {
        FaceRecognition::SpeakerIdentity s = rec.GetCurrentSpeaker();
        int64_t last_us = rec.GetLastMatchTimeUs();
        int64_t age_ms = (last_us > 0)
                             ? (esp_timer_get_time() - last_us) / 1000
                             : -1;
        httpd_resp_sendstr(req, BuildSpeakerJson(s, mode, age_ms).c_str());
        return ESP_OK;
    }

    // --- fresh=1: drive one live identify via the shared IdentifyOnce path.
    SscmaCamera* camera = instance_ ? instance_->camera_ : nullptr;
    if (camera == nullptr) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "{\"valid\":false,\"error\":\"camera_unavailable\"}");
        return ESP_OK;
    }

    // Rate limit: reuse the /api/face/embed cadence (only accepted calls advance
    // the timestamp) so a fast poller is throttled, not permanently starved.
    int64_t now_us = esp_timer_get_time();
    int64_t last_call = instance_->last_face_call_us_.load();
    if (last_call != 0 && (now_us - last_call) < FACE_EMBED_MIN_INTERVAL_US) {
        httpd_resp_set_status(req, "429 Too Many Requests");
        httpd_resp_sendstr(req, "{\"valid\":false,\"error\":\"rate_limited\"}");
        return ESP_OK;
    }
    instance_->last_face_call_us_.store(now_us);

    // §8e F4: allow_preview=true — the backend-direct pull should show the live
    // photo on screen. The httpd worker no longer touches LVGL directly; the
    // preview upshow is marshaled to the main loop via Application::Schedule
    // inside CaptureImpl/BenchSingleShot, so this is thread-safe.
    SscmaCamera::IdentifyStatus st = SscmaCamera::IdentifyStatus::kOk;
    const char* reason = nullptr;
    FaceRecognition::SpeakerIdentity s =
        camera->IdentifyOnce(/*allow_preview=*/true, &st, &reason);

    if (st == SscmaCamera::IdentifyStatus::kBusy) {
        // F3: op lock held -> transient busy.
        httpd_resp_set_status(req, "503 Service Unavailable");
        char body[64];
        snprintf(body, sizeof(body), "{\"valid\":false,\"error\":\"%s\"}",
                 reason ? reason : "busy");
        httpd_resp_sendstr(req, body);
        return ESP_OK;
    }
    if (st == SscmaCamera::IdentifyStatus::kBlocked) {
        // Same mapping as /api/face/embed: busy -> 503 (transient), other
        // reasons (upgrading / greeting_active) -> 409 (state conflict).
        const char* status = (reason && strcmp(reason, "busy") == 0)
                                 ? "503 Service Unavailable"
                                 : "409 Conflict";
        httpd_resp_set_status(req, status);
        char body[64];
        snprintf(body, sizeof(body), "{\"valid\":false,\"error\":\"%s\"}",
                 reason ? reason : "conflict");
        httpd_resp_sendstr(req, body);
        return ESP_OK;
    }

    // kOk: identify ran (s.valid may be false = no face / no match). Just-captured,
    // so age_ms = 0.
    httpd_resp_sendstr(req, BuildSpeakerJson(s, mode, 0).c_str());
    return ESP_OK;
}

static bool FaceCaptureJpegSink(void* ctx, const uint8_t* buf, size_t len) {
    httpd_req_t* req = static_cast<httpd_req_t*>(ctx);
    httpd_resp_set_type(req, "image/jpeg");
    return httpd_resp_send(req, reinterpret_cast<const char*>(buf), len) == ESP_OK;
}

esp_err_t RemoteDisplayHttpServer::HandleFaceCapture(httpd_req_t* req) {
    // Auth：与 current-speaker 同源的 X-Face-Token / NVS face.pull_token，fail-closed。
    std::string expected;
    {
        Settings settings("face", false);
        expected = settings.GetString("pull_token", "");
    }
    std::string got;
    size_t tlen = httpd_req_get_hdr_value_len(req, "X-Face-Token");
    if (tlen > 0) {
        got.resize(tlen + 1);
        if (httpd_req_get_hdr_value_str(req, "X-Face-Token", &got[0], tlen + 1) == ESP_OK) {
            got.resize(tlen);
        } else {
            got.clear();
        }
    }
    if (expected.empty() || got != expected) {
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"unauthorized\"}");
        return ESP_OK;
    }

    SscmaCamera* camera = instance_ ? instance_->camera_ : nullptr;
    if (camera == nullptr) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"camera_unavailable\"}");
        return ESP_OK;
    }

    // 限流：复用 current-speaker/embed 的节奏（instance_->last_face_call_us_）。
    int64_t now_us = esp_timer_get_time();
    int64_t last_call = instance_->last_face_call_us_.load();
    if (last_call != 0 && (now_us - last_call) < FACE_EMBED_MIN_INTERVAL_US) {
        httpd_resp_set_status(req, "429 Too Many Requests");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"rate_limited\"}");
        return ESP_OK;
    }
    instance_->last_face_call_us_.store(now_us);

    int status = camera->CaptureJpegLocked(FaceCaptureJpegSink, req);
    if (status == 200) {
        return ESP_OK;  // JPEG 已在 sink 内发出
    }
    // 非 200：sink 未发送任何 body，返回 JSON 错误。
    if (status == 503) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"busy\"}");
    } else {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"capture_failed\"}");
    }
    return ESP_OK;
}

// ---------- Start / Stop ----------

bool RemoteDisplayHttpServer::Start(int port) {
    if (server_) {
        ESP_LOGW(TAG, "HTTP server already running");
        return true;
    }

    instance_ = this;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    // SRAM-frugal: internal SRAM is tight (~32KB free observed). Cap concurrent
    // sockets low and enable LRU purge so an idle/stuck client can't pin a slot.
    // Stack: measured on-device — the face endpoint drives the deep sscma_client
    // + SPI chain on this worker and left only stack_hwm=340 bytes free at 3072
    // (peak ~2732B). 4608 gives a comfortable ~1.9KB margin (incl. the success
    // path's base64/JSON build) while costing only ~1.5KB more of the SRAM floor.
    config.stack_size = 4608;
    config.max_uri_handlers = 10;
    config.max_open_sockets = 3;
    config.lru_purge_enable = true;
    config.open_fn = [](httpd_handle_t hd, int sockfd) -> esp_err_t {
        // Reduce LWIP socket buffers to save ~6KB internal SRAM per connection
        int sndbuf = 2920;
        int rcvbuf = 2920;
        setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
        setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
        return ESP_OK;
    };

    esp_err_t err = httpd_start(&server_, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(err));
        return false;
    }

    // POST /api/start_cast
    httpd_uri_t start_uri = {
        .uri = "/api/start_cast",
        .method = HTTP_POST,
        .handler = HandleStartCast,
        .user_ctx = nullptr,
    };
    httpd_register_uri_handler(server_, &start_uri);

    // POST /api/stop_cast
    httpd_uri_t stop_uri = {
        .uri = "/api/stop_cast",
        .method = HTTP_POST,
        .handler = HandleStopCast,
        .user_ctx = nullptr,
    };
    httpd_register_uri_handler(server_, &stop_uri);

    // GET /api/status
    httpd_uri_t status_uri = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = HandleStatus,
        .user_ctx = nullptr,
    };
    httpd_register_uri_handler(server_, &status_uri);

    httpd_uri_t wake_status_uri = {.uri = "/api/wake/status",
                                   .method = HTTP_GET,
                                   .handler = HandleWakeStatus,
                                   .user_ctx = nullptr};
    err = httpd_register_uri_handler(server_, &wake_status_uri);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register route %s: %s", wake_status_uri.uri, esp_err_to_name(err));
        httpd_stop(server_);
        server_ = nullptr;
        return false;
    }
    httpd_uri_t wake_uri = {
        .uri = "/api/wake", .method = HTTP_POST, .handler = HandleWake, .user_ctx = nullptr};
    err = httpd_register_uri_handler(server_, &wake_uri);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register route %s: %s", wake_uri.uri, esp_err_to_name(err));
        httpd_stop(server_);
        server_ = nullptr;
        return false;
    }

    // POST /api/face/embed — on-device single-shot face embedding for cloud
    // callers. LAN-only; auth is expected at the cloud wrapper layer.
    httpd_uri_t face_embed_uri = {
        .uri = "/api/face/embed",
        .method = HTTP_POST,
        .handler = HandleFaceEmbed,
        .user_ctx = nullptr,
    };
    httpd_register_uri_handler(server_, &face_embed_uri);

    // POST /api/face/batch-update — full face-DB sync from the server library.
    // LAN-only; model_tag gated; no token auth this round.
    httpd_uri_t face_batch_uri = {
        .uri = "/api/face/batch-update",
        .method = HTTP_POST,
        .handler = HandleFaceBatchUpdate,
        .user_ctx = nullptr,
    };
    httpd_register_uri_handler(server_, &face_batch_uri);

    // GET /api/face/current-speaker?fresh=0|1 — backend-direct identity pull.
    // Header X-Face-Token gated (must match NVS face.id_token). See §8c/§9.
    httpd_uri_t face_speaker_uri = {
        .uri = "/api/face/current-speaker",
        .method = HTTP_GET,
        .handler = HandleFaceCurrentSpeaker,
        .user_ctx = nullptr,
    };
    httpd_register_uri_handler(server_, &face_speaker_uri);

    // GET /api/face/capture — lan option 3 后端拉图，X-Face-Token(pull_token) 鉴权。
    httpd_uri_t face_capture_uri = {
        .uri = "/api/face/capture",
        .method = HTTP_GET,
        .handler = HandleFaceCapture,
        .user_ctx = nullptr,
    };
    httpd_register_uri_handler(server_, &face_capture_uri);

    ESP_LOGI(TAG, "HTTP server started on port %d", port);

    return true;
}

void RemoteDisplayHttpServer::Stop() {
    if (server_) {
        httpd_stop(server_);
        server_ = nullptr;
        ESP_LOGI(TAG, "HTTP server stopped");
    }
    // Avoid a dangling static pointer: handlers resolve the camera via instance_.
    if (instance_ == this) {
        instance_ = nullptr;
    }
}
