#ifndef REMOTE_DISPLAY_HTTP_SERVER_H
#define REMOTE_DISPLAY_HTTP_SERVER_H

#include <esp_http_server.h>
#include <atomic>
#include <cstdint>
#include <string>
#include <deque>
#include <mutex>

class SscmaCamera;  // forward decl — avoids pulling the heavy camera header here

// Lightweight HTTP server for remote display control and on-device face
// embedding for external (cloud) callers.
// Endpoints:
//   POST /api/start_cast  {"ws_url":"ws://rpi:8765"}
//   POST /api/stop_cast
//   GET  /api/status
//   POST /api/face/embed  -> {"ok":true,"format":"float32_le_b64","dim":128,...}
//   POST /api/face/batch-update {"model_tag":"...","faces":[{name,subject_id,embedding_b64}]}
class RemoteDisplayHttpServer {
public:
    RemoteDisplayHttpServer() = default;
    ~RemoteDisplayHttpServer();

    // Start the always-on HTTP server on the given port.
    bool Start(int port = 80);
    void Stop();

    // Inject the camera used by the face-embedding endpoint. Must be set before
    // /api/face/embed is called; if null, the endpoint returns 503.
    void SetCamera(SscmaCamera* cam) { camera_ = cam; }

    bool IsRunning() const { return server_ != nullptr; }

private:
    static esp_err_t HandleStartCast(httpd_req_t* req);
    static esp_err_t HandleStopCast(httpd_req_t* req);
    static esp_err_t HandleStatus(httpd_req_t* req);
    static esp_err_t HandleWakeStatus(httpd_req_t* req);
    static esp_err_t HandleWake(httpd_req_t* req);
    static esp_err_t HandleFaceEmbed(httpd_req_t* req);
    static esp_err_t HandleFaceBatchUpdate(httpd_req_t* req);
    // GET /api/face/current-speaker?fresh=0|1 — backend-direct identity pull.
    // Header X-Face-Token must match NVS face.id_token, else 401. See §9 contract.
    static esp_err_t HandleFaceCurrentSpeaker(httpd_req_t* req);
    // GET /api/face/capture — lan option 3 后端拉图。X-Face-Token(pull_token) 鉴权。
    static esp_err_t HandleFaceCapture(httpd_req_t* req);

    SscmaCamera* camera_ = nullptr;
    // Rate limit for /api/face/embed: timestamp (us) of the last *accepted* call.
    std::atomic<int64_t> last_face_call_us_{0};
    httpd_handle_t server_ = nullptr;
    struct WakeRequestRecord {
        std::string id;
        std::string fingerprint;
        int64_t accepted_us;
    };
    std::deque<WakeRequestRecord> wake_request_history_;
    std::mutex wake_request_mutex_;
    static RemoteDisplayHttpServer* instance_;
};

#endif // REMOTE_DISPLAY_HTTP_SERVER_H
