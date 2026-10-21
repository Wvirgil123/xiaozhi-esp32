#include "sscma_camera.h"
#include "face_database.h"
#include "face_recognition.h"
#include "mcp_server.h"
#include "lvgl_display.h"
#include "lvgl_image.h"
#include "board.h"
#include "system_info.h"
#include "config.h"
#include "settings.h"
#include "remote_display.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <mbedtls/base64.h>
#include "application.h"
#include "sscma_client_commands.h"

#define TAG "SscmaCamera"

#define IMG_JPEG_BUF_SIZE   48 * 1024

// Object-detection wake word, kept inside the cloud's detect size budget
// (see kWakeWordMaxBytes in face_recognition.h). <d></d> costs 7 bytes; the
// count prefix eats a few more, and the class name takes whatever is left.
static std::string MakeDetectWakeWord(int count, const std::string& class_name) {
    std::string prefix = std::to_string(count) + " ";
    size_t budget = kWakeWordPayloadMaxBytes > prefix.size()
                        ? kWakeWordPayloadMaxBytes - prefix.size()
                        : 0;
    return "<d>" + prefix + TruncateUtf8(class_name, budget) + "</d>";
}

static bool __himax_keepalive_check(sscma_client_handle_t client)
{
    esp_err_t ret = ESP_OK;
    sscma_client_reply_t reply = {0};
    int retry = 3;
    while(retry--) {
        ret = sscma_client_request(client, CMD_PREFIX CMD_AT_ID CMD_QUERY CMD_SUFFIX, &reply, true, pdMS_TO_TICKS(2000));
        if (reply.payload != NULL) {
            sscma_client_reply_clear(&reply);
        }
        if( ret != ESP_OK ) {
            ESP_LOGE(TAG, "Himax keepalive check failed: %d", ret);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            return true;
        }
    }
    return false;
}

SscmaCamera* SscmaCamera::instance_ = nullptr;

void SscmaCamera::SetDropEvents(bool drop) {
    if (instance_ != nullptr && instance_->sscma_client_handle_ != nullptr) {
        sscma_client_set_drop_events(instance_->sscma_client_handle_, drop);
        ESP_LOGI(TAG, "sscma drop_events=%d", drop ? 1 : 0);
    }
}

SscmaCamera::SscmaCamera(esp_io_expander_handle_t io_exp_handle) {
    instance_ = this;
    sscma_client_io_spi_config_t spi_io_config = {0};
    spi_io_config.sync_gpio_num = BSP_SSCMA_CLIENT_SPI_SYNC;
    spi_io_config.cs_gpio_num = BSP_SSCMA_CLIENT_SPI_CS;
    spi_io_config.pclk_hz = BSP_SSCMA_CLIENT_SPI_CLK;
    spi_io_config.spi_mode = 0;
    spi_io_config.wait_delay = 10; //两个transfer之间至少延时4ms,但当前 FREERTOS_HZ=100, 延时精度只能达到10ms, 
    spi_io_config.user_ctx = NULL;
    spi_io_config.io_expander = io_exp_handle;
    spi_io_config.flags.sync_use_expander = BSP_SSCMA_CLIENT_RST_USE_EXPANDER;

    sscma_client_new_io_spi_bus((sscma_client_spi_bus_handle_t)BSP_SSCMA_CLIENT_SPI_NUM, &spi_io_config, &sscma_client_io_handle_);

    sscma_client_config_t sscma_client_config = SSCMA_CLIENT_CONFIG_DEFAULT();
    sscma_client_config.event_queue_size = CONFIG_SSCMA_EVENT_QUEUE_SIZE;
    sscma_client_config.tx_buffer_size = CONFIG_SSCMA_TX_BUFFER_SIZE;
    sscma_client_config.rx_buffer_size = CONFIG_SSCMA_RX_BUFFER_SIZE;
    sscma_client_config.process_task_stack = CONFIG_SSCMA_PROCESS_TASK_STACK_SIZE;
    sscma_client_config.process_task_affinity = CONFIG_SSCMA_PROCESS_TASK_AFFINITY;
    sscma_client_config.process_task_priority = CONFIG_SSCMA_PROCESS_TASK_PRIORITY;
    sscma_client_config.monitor_task_stack = CONFIG_SSCMA_MONITOR_TASK_STACK_SIZE;
    sscma_client_config.monitor_task_affinity = CONFIG_SSCMA_MONITOR_TASK_AFFINITY;
    sscma_client_config.monitor_task_priority = CONFIG_SSCMA_MONITOR_TASK_PRIORITY;
    sscma_client_config.reset_gpio_num = BSP_SSCMA_CLIENT_RST;
    sscma_client_config.io_expander = io_exp_handle;
    sscma_client_config.flags.reset_use_expander = BSP_SSCMA_CLIENT_RST_USE_EXPANDER;

    sscma_client_new(sscma_client_io_handle_, &sscma_client_config, &sscma_client_handle_);

    // Suppress noisy SSCMA logs that flood the USB Serial JTAG console
    // (Invalid reply, rx buffer full, spi transmit failed) — keeps serial CRUD usable
    esp_log_level_set("sscma_client", ESP_LOG_ERROR);
    esp_log_level_set("sscma_client.io.spi", ESP_LOG_ERROR);

    sscma_data_queue_ = xQueueCreate(1, sizeof(SscmaData));

    sscma_client_callback_t callback = {0};

    detection_state = SscmaCamera::IDLE;
    state_start_time = 0;
    need_start_cooldown = false;
    callback.on_event = [](sscma_client_handle_t client, const sscma_client_reply_t *reply, void *user_ctx) {
        SscmaCamera* self = static_cast<SscmaCamera*>(user_ctx);
        if (!self) return;

        ESP_LOGD(TAG, "on_event callback triggered");

        char *img = NULL;
        int img_size = 0;
        int box_count = 0;
        sscma_client_box_t     *boxes = NULL;
        int class_count = 0;
        sscma_client_class_t *classes = NULL;
        int point_count = 0;
        sscma_client_point_t  *points = NULL;
        int model_type = 0;
        int obj_cnt = 0;

        int width = 0, height = 0;
        cJSON *data = cJSON_GetObjectItem(reply->payload, "data");
        if (data != NULL && cJSON_IsObject(data)) {
            cJSON *resolution = cJSON_GetObjectItem(data, "resolution");
            if (data != NULL && cJSON_IsArray(resolution) && cJSON_GetArraySize(resolution) == 2) {
                width = cJSON_GetArrayItem(resolution, 0)->valueint;
                height = cJSON_GetArrayItem(resolution, 1)->valueint;
            }

            // Check for face recognition mode data
            cJSON *mode = cJSON_GetObjectItem(data, "mode");
            if (mode != NULL && cJSON_IsString(mode) && strcmp(mode->valuestring, "face") == 0) {
                ESP_LOGD(TAG, "Face mode event received (res=%dx%d)", width, height);
                auto& face_rec = FaceRecognition::GetInstance();

                // Parse face recognition data
                cJSON *faces = cJSON_GetObjectItem(data, "faces");
                if (faces != NULL && cJSON_IsArray(faces)) {
                    int face_count = cJSON_GetArraySize(faces);

                    if (face_count > 0) {
                        face_rec.NotifyFacePresent();
                        for (int i = 0; i < face_count; i++) {
                            cJSON *face = cJSON_GetArrayItem(faces, i);
                            if (face == NULL) continue;

                            HimaxFaceData face_data;
                            memset(&face_data, 0, sizeof(face_data));

                            // Parse bounding box
                            cJSON *box = cJSON_GetObjectItem(face, "box");
                            if (box != NULL && cJSON_IsArray(box) && cJSON_GetArraySize(box) == 4) {
                                face_data.box_x = cJSON_GetArrayItem(box, 0)->valueint;
                                face_data.box_y = cJSON_GetArrayItem(box, 1)->valueint;
                                face_data.box_w = cJSON_GetArrayItem(box, 2)->valueint;
                                face_data.box_h = cJSON_GetArrayItem(box, 3)->valueint;
                            }

                            // Parse score
                            cJSON *score = cJSON_GetObjectItem(face, "score");
                            if (score != NULL && cJSON_IsNumber(score)) {
                                face_data.score = score->valueint;
                            }

                            // Parse quality
                            cJSON *quality = cJSON_GetObjectItem(face, "quality");
                            if (quality != NULL && cJSON_IsNumber(quality)) {
                                face_data.quality = (float)quality->valuedouble;
                            }

                            // Parse embedding
                            cJSON *embedding = cJSON_GetObjectItem(face, "embedding");
                            if (embedding != NULL && cJSON_IsArray(embedding) &&
                                cJSON_GetArraySize(embedding) == FACE_EMBEDDING_DIM) {
                                for (int j = 0; j < FACE_EMBEDDING_DIM; j++) {
                                    cJSON *val = cJSON_GetArrayItem(embedding, j);
                                    if (val != NULL && cJSON_IsNumber(val)) {
                                        face_data.embedding[j] = (float)val->valuedouble;
                                    }
                                }
                                face_data.has_embedding = true;
                            }

                            ESP_LOGD(TAG, "[face %d]: box=[%d,%d,%d,%d], score=%d, quality=%.2f, has_emb=%d",
                                     i, face_data.box_x, face_data.box_y, face_data.box_w, face_data.box_h,
                                     face_data.score, face_data.quality, face_data.has_embedding);

                            // Bench hook (path Z): if a single-shot request is in flight and
                            // we have an embedding, hand it to the waiter and skip the
                            // voting / cooldown state machine. Only the first face in the
                            // frame is captured.
                            if (self->single_shot_pending_.load() &&
                                !self->single_shot_valid_.load() &&
                                face_data.has_embedding) {
                                // 做法 B: grab the JPEG of THIS frame (the one that
                                // produced the matched embedding) for on-screen
                                // preview. Best-effort — failure never blocks the
                                // embedding. fetch_image returns a base64 string
                                // (__strdup'd, freed with heap_caps_free like the
                                // 640x480 path); decode it to raw JPEG in SPIRAM.
                                if (self->single_shot_want_image_.load() &&
                                    !self->single_shot_jpeg_ready_.load()) {
                                    char* b64img = NULL;
                                    int b64len = 0;
                                    if (sscma_utils_fetch_image_from_reply(reply, &b64img, &b64len) == ESP_OK &&
                                        b64img != NULL && b64len > 0) {
                                        uint8_t* raw = (uint8_t*)heap_caps_malloc(b64len, MALLOC_CAP_SPIRAM);
                                        if (raw != NULL) {
                                            size_t raw_len = 0;
                                            int drc = mbedtls_base64_decode(
                                                raw, b64len, &raw_len,
                                                (const unsigned char*)b64img, b64len);
                                            if (drc == 0 && raw_len > 0) {
                                                self->single_shot_jpeg_ = raw;
                                                self->single_shot_jpeg_len_ = raw_len;
                                                self->single_shot_jpeg_ready_.store(true);
                                                ESP_LOGI(TAG, "[BENCH] single-shot preview JPEG captured (%zu bytes)", raw_len);
                                            } else {
                                                heap_caps_free(raw);
                                            }
                                        }
                                    }
                                    if (b64img != NULL) {
                                        heap_caps_free(b64img);
                                    }
                                }
                                memcpy(self->single_shot_embedding_,
                                       face_data.embedding,
                                       sizeof(float) * FACE_EMBEDDING_DIM);
                                self->single_shot_face_score_.store(face_data.score);
                                self->single_shot_face_quality_x1000_.store(
                                    (int)(face_data.quality * 1000.0f));
                                self->single_shot_valid_.store(true);
                                ESP_LOGI(TAG, "[BENCH] embedding captured (score=%d quality=%.2f)",
                                         face_data.score, face_data.quality);
                                continue;
                            }

                            // Process face data
                            face_rec.ProcessFaceData(face_data);
                        }
                    } else {
                        // No face in this frame - notify for cooldown state machine
                        face_rec.NotifyNoFace();
                    }
                }

                return;  // Don't process as regular detection data
            }
        }

        switch ((width+height)) {
            case (416+416): 
            {
                bool is_object_detected = false;
                bool is_need_wake = false;
                const int detect_target = self->detect_target.load();
                const int detect_threshold = self->detect_threshold.load();
                const int detect_duration_sec = self->detect_duration_sec.load();
                const int detect_invoke_interval_sec = self->detect_invoke_interval_sec.load();
                
                // 定期更新检测配置参数，避免频繁NVS访问
                int64_t cur_tm = esp_timer_get_time();

                // 尝试获取检测框数据（目标检测模型）
                if (sscma_utils_fetch_boxes_from_reply(reply, &boxes, &box_count) == ESP_OK && box_count > 0) {
                    for (int i = 0; i < box_count; i++) {
                        ESP_LOGI(TAG, "[box %d]: x=%d, y=%d, w=%d, h=%d, score=%d, target=%d", i,  \
                                boxes[i].x, boxes[i].y, boxes[i].w, boxes[i].h, boxes[i].score, boxes[i].target);
                        if (boxes[i].target == detect_target && boxes[i].score > detect_threshold) {
                           is_object_detected = true;
                           model_type = 0;
                           obj_cnt++;
                           break;
                        }
                    }
                    free(boxes);
                } else if (sscma_utils_fetch_classes_from_reply(reply, &classes, &class_count) == ESP_OK && class_count > 0) {
                    // 尝试获取分类数据（分类模型）
                    for (int i = 0; i < class_count; i++) {
                        ESP_LOGI(TAG, "[class %d]: target=%d, score=%d", i,
                                classes[i].target, classes[i].score);
                        if (classes[i].target == detect_target && classes[i].score > detect_threshold) {
                           is_object_detected = true;
                           model_type = 1;
                           obj_cnt++;
                        }
                    }
                    free(classes);
                } else if (sscma_utils_fetch_points_from_reply(reply, &points, &point_count) == ESP_OK && point_count > 0) {
                     // 尝试获取关键点数据（姿态估计模型）
                    for (int i = 0; i < point_count; i++) {
                        ESP_LOGI(TAG, "[point %d]: x=%d, y=%d, z=%d, score=%d, target=%d", i, 
                                points[i].x, points[i].y, points[i].z, points[i].score, points[i].target);
                        if (points[i].target == detect_target && points[i].score > detect_threshold) {
                           is_object_detected = true;
                           model_type = 2;
                           obj_cnt++;
                        }
                    }
                    free(points);
                }

                // 如果需要开始冷却期，现在开始计时
                if (self->need_start_cooldown) { // 回调暂停，标志保持，等待回调恢复后开始计时
                    self->state_start_time = cur_tm;
                    self->need_start_cooldown = false;
                    ESP_LOGI(TAG, "Starting cooldown timer");
                }
                
                // 状态机驱动的检测逻辑 - 只在人员出现时触发
                switch (self->detection_state) {
                    case SscmaCamera::IDLE:
                        if (is_object_detected) {
                            // 人员出现，开始验证（这是从无到有的转换）
                            self->detection_state = SscmaCamera::VALIDATING;
                            self->state_start_time = cur_tm; // 记录物体出现时间
                            self->last_detected_time = cur_tm; // 初始化最后检测时间
                            ESP_LOGI(TAG, "object appeared, starting validation");
                        }
                        break;
                        
                    case SscmaCamera::VALIDATING:
                        if (is_object_detected) {
                            // 更新最后检测到的时间
                            self->last_detected_time = cur_tm;
                            // 检查是否验证足够时间
                            if ((cur_tm - self->state_start_time) >= (detect_duration_sec * 1000000LL)) {
                                is_need_wake = true;
                            }
                        } else {
                            // 验证期间人员离开，检查去抖动时间
                            if (self->last_detected_time > 0 && 
                                (cur_tm - self->last_detected_time) >= self->detect_debounce_sec * 1000000LL) {
                                // 去抖动时间已过，确认人员已离开，回到空闲
                                self->detection_state = SscmaCamera::IDLE;
                                self->last_detected_time = 0;
                                ESP_LOGI(TAG, "object left during validation (debounced), back to idle");
                            }
                        }
                        break;
                        
                    case SscmaCamera::COOLDOWN:
                        // 冷却期，需要满足两个条件：1)object离开 2)过了15秒
                        if (!is_object_detected && 
                            (cur_tm - self->state_start_time) >= (detect_invoke_interval_sec * 1000000LL)) {
                            // object离开且冷却时间到，回到空闲状态
                            self->detection_state = SscmaCamera::IDLE;
                            ESP_LOGI(TAG, "Cooldown complete and object left, back to idle - ready for next appearance");
                        }
                        // 其他情况继续保持冷却状态
                        break;
                }


                if( is_need_wake ) {
                    ESP_LOGI(TAG, "Validation complete, triggering conversation (type=%d, res=%dx%d)",
                             detect_target, width, height);
                    
                    // 触发对话
                    std::string wake_word;
                    if ( model_type  == 0 ) {
                        std::string cached_target_name = "object";
                        if( self->model != NULL && self->model->classes[detect_target] != NULL ) {
                            cached_target_name = self->model->classes[detect_target];
                        }
                        wake_word = MakeDetectWakeWord(obj_cnt, cached_target_name);
                    } else if ( model_type  == 1 ) {
                        std::string cached_target_name = "object";
                        if( self->model != NULL && self->model->classes[detect_target] != NULL ) {
                            cached_target_name = self->model->classes[detect_target];
                        }
                        wake_word = MakeDetectWakeWord(obj_cnt, cached_target_name);
                    } else if ( model_type  == 2 ) {
                        std::string cached_target_name = "object";
                        if( self->model != NULL && self->model->classes[detect_target] != NULL ) {
                            cached_target_name = self->model->classes[detect_target];
                        }
                        wake_word = MakeDetectWakeWord(obj_cnt, cached_target_name);
                    }
                    printf("wake_word:%s\n", wake_word.c_str());
                    // Wake window: drop further event frames until the camera main
                    // loop reconfigures the mode (protects internal SRAM during the
                    // wake TLS handshake).
                    SscmaCamera::SetDropEvents(true);
                    Application::GetInstance().Schedule([wake_word]() {
                        Application::GetInstance().WakeWordInvoke(wake_word);
                    });
                    
                    // 进入冷却状态，标记需要开始冷却期；如下变量将在会话结束后被使用，等待回调恢复后开始计时
                    self->detection_state = SscmaCamera::COOLDOWN;
                    self->need_start_cooldown = true;
                }
            }
                break;
            case (640+480):

                if (sscma_utils_fetch_image_from_reply(reply, &img, &img_size) == ESP_OK)
                {
                    ESP_LOGD(TAG, "image_size: %d", img_size);
                    // 将数据通过队列发送出去
                    SscmaData data;
                    data.img = (uint8_t*)img;
                    data.len = img_size;

                    // 清空队列，保证只保存最新的数据
                    SscmaData dummy;
                    while (xQueueReceive(self->sscma_data_queue_, &dummy, 0) == pdPASS) {
                        if (dummy.img) {
                            heap_caps_free(dummy.img);
                        }
                    }
                    if (xQueueSend(self->sscma_data_queue_, &data, 0) != pdPASS) {
                        if (data.img) {
                            heap_caps_free(data.img);
                        }
                    }
                    // 注意：img 的释放由接收方负责
                }
                break;
            default:
                ESP_LOGD(TAG, "unknown resolution");
                break;
        }
    };
    callback.on_connect = [](sscma_client_handle_t client, const sscma_client_reply_t *reply, void *user_ctx) {
        ESP_LOGI(TAG, "SSCMA client connected");
        SscmaCamera* self = static_cast<SscmaCamera*>(user_ctx);
        if (self) {
            self->sscma_restarted_ = true;
        }
    };

    callback.on_log = [](sscma_client_handle_t client, const sscma_client_reply_t *reply, void *user_ctx) {
        ESP_LOGD(TAG, "log: %s", reply->data);
    };

    sscma_client_register_callback(sscma_client_handle_, &callback, this);

    sscma_client_init(sscma_client_handle_);

    ESP_LOGI(TAG, "SSCMA client initialized");
    // 设置分辨率
    // 3 = 640x480
    {
        int sensor_retry = 5;
        bool sensor_ok = false;
        while (sensor_retry-- > 0) {
            if (sscma_client_set_sensor(sscma_client_handle_, 1, 3, true) == ESP_OK) {
                sensor_ok = true;
                break;
            }
            ESP_LOGW(TAG, "Failed to set sensor, retrying... (%d left)", sensor_retry);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        if (!sensor_ok) {
            ESP_LOGE(TAG, "Failed to set sensor after retries");
            sscma_client_del(sscma_client_handle_);
            sscma_client_handle_ = NULL;
            return;
        }
    }

    // 获取设备信息
    sscma_client_info_t *info;
    if (sscma_client_get_info(sscma_client_handle_, &info, true) == ESP_OK) {
        ESP_LOGI(TAG, "Device Info - ID: %s, Name: %s", 
            info->id ? info->id : "NULL", 
            info->name ? info->name : "NULL");
    }
    // 初始化JPEG数据的内存
    jpeg_data_.len = 0;
    jpeg_data_.buf = (uint8_t*)heap_caps_malloc(IMG_JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM);;
    if ( jpeg_data_.buf == nullptr ) {
        ESP_LOGE(TAG, "Failed to allocate memory for JPEG buffer");
        return;
    }

    //初始化JPEG解码
    jpeg_error_t err;
    jpeg_dec_config_t config = { .output_type = JPEG_PIXEL_FORMAT_RGB565_LE, .rotate = JPEG_ROTATE_0D };
    err = jpeg_dec_open(&config, &jpeg_dec_);
    if ( err != JPEG_ERR_OK ) {
        ESP_LOGE(TAG, "Failed to open JPEG decoder");
        return;
    }
    jpeg_io_ = (jpeg_dec_io_t*)heap_caps_malloc(sizeof(jpeg_dec_io_t), MALLOC_CAP_SPIRAM);
    if (!jpeg_io_) {
        ESP_LOGE(TAG, "Failed to allocate memory for JPEG IO");
        jpeg_dec_close(jpeg_dec_);
        return;
    }
    memset(jpeg_io_, 0, sizeof(jpeg_dec_io_t));

    jpeg_out_ = (jpeg_dec_header_info_t*)heap_caps_aligned_alloc(16, sizeof(jpeg_dec_header_info_t), MALLOC_CAP_SPIRAM);
    if (!jpeg_out_) {
        ESP_LOGE(TAG, "Failed to allocate memory for JPEG output header");
        heap_caps_free(jpeg_io_);
        jpeg_dec_close(jpeg_dec_);
        return;
    }
    memset(jpeg_out_, 0, sizeof(jpeg_dec_header_info_t));

    // 初始化预览图片描述符，解码缓冲在 Capture() 时临时申请，避免常驻占用 640x480x2 PSRAM。
    memset(&preview_image_, 0, sizeof(preview_image_));
    preview_image_.header.magic = LV_IMAGE_HEADER_MAGIC;
    preview_image_.header.cf = LV_COLOR_FORMAT_RGB565;
    preview_image_.header.flags = LV_IMAGE_FLAGS_ALLOCATED | LV_IMAGE_FLAGS_MODIFIABLE;
    preview_image_.header.w = 640;
    preview_image_.header.h = 480;

    preview_image_.header.stride = preview_image_.header.w * 2;
    preview_image_.data_size = preview_image_.header.w * preview_image_.header.h * 2;
    preview_image_.data = nullptr;

    sscma_client_set_model(sscma_client_handle_, 4);
    model_class_cnt = 0;
    if (sscma_client_get_model(sscma_client_handle_, &model, true) == ESP_OK) {
        printf("ID: %d\n", model->id ? model->id : -1);
        printf("UUID: %s\n", model->uuid ? model->uuid : "N/A");
        printf("Name: %s\n", model->name ? model->name : "N/A");
        printf("Version: %s\n", model->ver ? model->ver : "N/A");
        printf("URL: %s\n", model->url ? model->url : "N/A");
        printf("Checksum: %s\n", model->checksum ? model->checksum : "N/A");
        printf("Classes:\n");
        if (model->classes[0] != NULL)
        {
            for (int i = 0; model->classes[i] != NULL; i++)
            {
                printf("  - %s\n", model->classes[i]);
                model_class_cnt++;
            }
        } else {
            printf("  N/A\n");
        }
    } else {
        printf("get model failed\n");
    }

    ESP_LOGI(TAG, "initialize mcp tools");
    face_model_available_ = ProbeFaceModelAvailable();
    InitializeMcpTools();
    InitializeFaceMcpTools();

    xTaskCreate([](void* arg) {
        auto this_ = (SscmaCamera*)arg;
        bool is_inference = false;
        bool is_face_mode = false;
        int64_t last_keepalive_time = esp_timer_get_time();
        // Voice-busy watchdog: track how long device stays in speaking/listening
        DeviceState prev_dev_state = kDeviceStateUnknown;
        int64_t voice_busy_since = 0;
        bool watchdog_abort_sent = false;
        const int64_t kVoiceBusyTimeoutUs = 60 * 1000000LL;  // 60s
        const int64_t kVoiceBusyForceIdleUs = 70 * 1000000LL;  // 70s: force idle if abort didn't help

        while (true)
        {
            if (this_->capture_in_progress_) {
                // Auto-reset if capture hangs
                int64_t now_sec = esp_timer_get_time() / 1000000;
                if (this_->capture_started_at_.load() > 0 &&
                    now_sec - this_->capture_started_at_.load() > CAPTURE_TIMEOUT_SEC) {
                    ESP_LOGW(TAG, "Capture timeout (%ds), auto-resetting", CAPTURE_TIMEOUT_SEC);
                    this_->capture_in_progress_ = false;
                    this_->capture_started_at_ = 0;
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            if (this_->sscma_restarted_) {
                ESP_LOGI(TAG, "SSCMA restarted detected");
                this_->sscma_restarted_ = false;
                is_inference = false;
                is_face_mode = false;
                SscmaCamera::SetDropEvents(false);  // fresh state → stop dropping events
                // Himax state was reset/resynced — any single-shot warm state is
                // no longer valid (note: a single-shot's idle teardown sets
                // sscma_restarted_ itself, so the warm flag it cleared stays clear).
                this_->himax_face_warm_.store(false);
            }

            // Auto-resume inference if paused too long (browser didn't close properly)
            if (this_->inference_paused_.load() && this_->inference_paused_at_.load() > 0) {
                int64_t now_sec = esp_timer_get_time() / 1000000;
                if (now_sec - this_->inference_paused_at_.load() > INFERENCE_PAUSE_TIMEOUT_SEC) {
                    ESP_LOGW(TAG, "Inference pause timeout (%ds), auto-resuming", INFERENCE_PAUSE_TIMEOUT_SEC);
                    this_->ResumeInference();
                }
            }

            // When inference is paused (external UART enrollment in progress),
            // skip all SPI interactions to avoid conflicts with UART session.
            if (this_->inference_paused_.load()) {
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }

            if (esp_timer_get_time() - last_keepalive_time > 10 * 1000000) {
                last_keepalive_time = esp_timer_get_time();
                bool keepalive_ok = false;
                {
                    std::lock_guard<std::mutex> lock(this_->sscma_mutex_);
                    keepalive_ok = __himax_keepalive_check(this_->sscma_client_handle_);
                }
                if (!keepalive_ok) {
                    ESP_LOGE(TAG, "restart himax");
                    {
                        std::lock_guard<std::mutex> lock(this_->sscma_mutex_);
                        sscma_client_reset(this_->sscma_client_handle_);
                    }
                    // Himax was reset out from under us → drop warm state.
                    this_->himax_face_warm_.store(false);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }

            DeviceState dev_state = Application::GetInstance().GetDeviceState();
            bool is_idle = (dev_state == kDeviceStateIdle);

            // Voice-busy watchdog
            if (dev_state != prev_dev_state) {
                prev_dev_state = dev_state;
                if (dev_state == kDeviceStateSpeaking || dev_state == kDeviceStateListening) {
                    voice_busy_since = esp_timer_get_time();
                } else {
                    voice_busy_since = 0;
                    watchdog_abort_sent = false;
                }
            }
            if (voice_busy_since > 0) {
                int64_t elapsed = esp_timer_get_time() - voice_busy_since;
                if (!watchdog_abort_sent && elapsed > kVoiceBusyTimeoutUs) {
                    // First stage: try graceful abort via ToggleChatState
                    ESP_LOGW(TAG, "Voice busy watchdog: stuck in state %d for >60s, trying abort", dev_state);
                    Application::GetInstance().ToggleChatState();
                    watchdog_abort_sent = true;
                } else if (watchdog_abort_sent && elapsed > kVoiceBusyForceIdleUs) {
                    // Second stage: abort didn't help, force idle and close audio channel
                    ESP_LOGW(TAG, "Voice busy watchdog: still stuck in state %d after abort, forcing idle", dev_state);
                    Application::GetInstance().Schedule([]() {
                        auto& app = Application::GetInstance();
                        auto display = Board::GetInstance().GetDisplay();
                        display->SetChatMessage("system", "");
                        app.SetDeviceState(kDeviceStateIdle);
                    });
                    // Re-arm watchdog in case Schedule doesn't execute (main loop blocked)
                    voice_busy_since = esp_timer_get_time();
                    watchdog_abort_sent = false;
                }
            }
            // Face mode can run anytime except during active voice conversations
            bool is_voice_busy = (dev_state == kDeviceStateConnecting ||
                                  dev_state == kDeviceStateListening ||
                                  dev_state == kDeviceStateSpeaking);
            auto& face_rec = FaceRecognition::GetInstance();
            int face_count = FaceDatabase::GetInstance().GetFaceCount();

            // Conversation speaker identity (board-local). Freeze who we're talking
            // to on the rising edge of a conversation, clear it when it ends. Read
            // by the self.conversation.speaker MCP tool for permission-gated cmds.
            static bool was_voice_busy = false;
            if (is_voice_busy && !was_voice_busy) {
                face_rec.CaptureCurrentSpeaker();
            } else if (!is_voice_busy && was_voice_busy) {
                face_rec.ClearCurrentSpeaker();
            }
            was_voice_busy = is_voice_busy;

            // Debug: print face recognition status every 60 seconds
            static int64_t last_debug_time = 0;
            if (esp_timer_get_time() - last_debug_time > 60000000) {
                last_debug_time = esp_timer_get_time();
                ESP_LOGI(TAG, "[FaceDebug] face_en=%d, inference_en=%d, state=%d, face_count=%d, is_face_mode=%d",
                         this_->face_recognition_en_.load(), this_->inference_en.load(), dev_state, face_count, is_face_mode);
            }

            // Deliver pending face notification when device is idle
            if (is_idle) {
                face_rec.DeliverPendingNotification();
            }

            // Cut4 test: mode 2/3 face recognition re-enabled to validate 263
            // watchdog / PHY crash after PSRAM budget cuts.
            bool want_face_mode = !is_voice_busy && this_->face_model_available_.load() && this_->face_recognition_en_.load();

            // Check if object detection mode should be active
            bool want_object_mode = this_->inference_en.load() && is_idle && !want_face_mode;

            // Handle face recognition mode
            if (want_face_mode) {
                if (!is_face_mode) {
                    ESP_LOGI(TAG, "Start face recognition mode");
                    // Idempotent guard: face frames must be parsed in this mode.
                    SscmaCamera::SetDropEvents(false);
                    {
                        std::lock_guard<std::mutex> lock(this_->sscma_mutex_);
                        sscma_client_break(this_->sscma_client_handle_);
                    }

                    // Send AT+FACE=1 to enable face mode on Himax
                    if (this_->SendFaceModeCommand(true)) {
                        this_->camera_mode_ = SscmaCamera::MODE_FACE_RECOGNITION;
                        face_rec.SetEnabled(true);
                        is_face_mode = true;
                        is_inference = false;

                        // Set sensor to 240x240 (mode 0) for face mode - fits in Himax RGB buffer
                        {
                            std::lock_guard<std::mutex> lock(this_->sscma_mutex_);
                            sscma_client_set_sensor(this_->sscma_client_handle_, 1, 0, true);
                            // Main task now owns Himax (continuous face invoke); any
                            // single-shot warm state from a prior conversation is stale.
                            this_->himax_face_warm_.store(false);
                        }

                        // Wait for Himax to process face mode command
                        vTaskDelay(pdMS_TO_TICKS(200));

                        // Start face inference (show=false → result_only=1, matches "INVOKE" response name)
                        esp_err_t ret = ESP_FAIL;
                        {
                            std::lock_guard<std::mutex> lock(this_->sscma_mutex_);
                            ret = sscma_client_invoke(this_->sscma_client_handle_, -1, false, false);
                        }
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to start face inference: %d; falling back to object detection", ret);
                            this_->SendFaceModeCommand(false);
                            this_->face_recognition_en_ = false;
                            is_face_mode = false;
                            this_->camera_mode_ = SscmaCamera::MODE_OBJECT_DETECT;
                            this_->inference_en.store(1);
                            face_rec.SetEnabled(false);
                        } else {
                            ESP_LOGI(TAG, "Face inference started");
                            // Face mode is actually running now — record the
                            // applied mode (2 vs 3 distinguished by DND flag).
                            this_->applied_vision_mode_.store(
                                face_rec.IsFamiliarMode() ? SscmaCamera::VISION_FACE_DND
                                                          : SscmaCamera::VISION_FACE);
                        }
                    } else {
                        ESP_LOGE(TAG, "Failed to start face mode on WE2; falling back to object detection");
                        this_->face_recognition_en_ = false;
                        is_face_mode = false;
                        this_->camera_mode_ = SscmaCamera::MODE_OBJECT_DETECT;
                        this_->inference_en.store(1);
                        face_rec.SetEnabled(false);
                    }
                }
            } else if (is_face_mode) {
                ESP_LOGI(TAG, "Stop face recognition mode");
                {
                    std::lock_guard<std::mutex> lock(this_->sscma_mutex_);
                    sscma_client_break(this_->sscma_client_handle_);
                }

                // Send AT+FACE=0 to disable face mode on Himax
                this_->SendFaceModeCommand(false);
                this_->camera_mode_ = SscmaCamera::MODE_OBJECT_DETECT;
                face_rec.SetEnabled(false);
                is_face_mode = false;

                // Restore sensor to 416x416 (mode 1) for object detection
                {
                    std::lock_guard<std::mutex> lock(this_->sscma_mutex_);
                    sscma_client_set_sensor(this_->sscma_client_handle_, 1, 1, true);
                    // Sensor left face mode → invalidate any single-shot warm state.
                    this_->himax_face_warm_.store(false);
                }

                // Face-mode teardown complete — the event flood is over, stop
                // dropping event frames (end of the wake-window guard).
                SscmaCamera::SetDropEvents(false);
                // Nothing running right now; object mode (if wanted) stores 1
                // when it actually starts below.
                this_->applied_vision_mode_.store(SscmaCamera::VISION_OFF);
            }

            // Handle object detection mode (only if not in face mode)
            if (want_object_mode && !is_face_mode) {
                if (!is_inference) {
                    ESP_LOGI(TAG, "Start inference (enable=1)");
                    // Idempotent guard: object events (type:1) must be parsed here —
                    // a stuck drop flag would kill object wake.
                    SscmaCamera::SetDropEvents(false);
                    std::lock_guard<std::mutex> lock(this_->sscma_mutex_);
                    sscma_client_break(this_->sscma_client_handle_);
                    sscma_client_set_model(this_->sscma_client_handle_, 4);
                    sscma_client_set_sensor(this_->sscma_client_handle_, 1, 1, true); // 设置分辨率 416X416
                    sscma_client_invoke(this_->sscma_client_handle_, -1, false, true);
                    this_->himax_face_warm_.store(false);  // object-detect mode → warm state invalid
                    is_inference = true;
                    this_->applied_vision_mode_.store(SscmaCamera::VISION_OBJECT);
                }
            } else if (is_inference && !want_object_mode) {
                ESP_LOGI(TAG, "Stop inference (enable=%d state=%d)", this_->inference_en.load(), Application::GetInstance().GetDeviceState());
                is_inference = false;
                {
                    std::lock_guard<std::mutex> lock(this_->sscma_mutex_);
                    sscma_client_break(this_->sscma_client_handle_);
                }
                this_->applied_vision_mode_.store(SscmaCamera::VISION_OFF);
            }

            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }, "sscma_camera", 4096, this, 1, nullptr);

}

SscmaCamera::~SscmaCamera() {
    if (preview_image_.data) {
        heap_caps_free((void*)preview_image_.data);
        preview_image_.data = nullptr;
    }
    if (sscma_client_handle_) {
        sscma_client_del(sscma_client_handle_);
    }
    if (sscma_data_queue_) {
        vQueueDelete(sscma_data_queue_);
    }
    if (jpeg_data_.buf) {
        heap_caps_free(jpeg_data_.buf);
        jpeg_data_.buf = nullptr;
    }
    if (jpeg_dec_) {
        jpeg_dec_close(jpeg_dec_);
        jpeg_dec_ = nullptr;
    }
    if (jpeg_io_) {
        heap_caps_free(jpeg_io_);
        jpeg_io_ = nullptr;
    }
    if (jpeg_out_) {
        heap_caps_free(jpeg_out_);
        jpeg_out_ = nullptr;
    }
}

bool SscmaCamera::ProbeFaceModelAvailable() {
    himax_face_warm_.store(false);

    bool ok = false;
    for (int retry = 0; retry < 3; ++retry) {
        sscma_client_reply_t reply = {0};
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "AT+FACE=0\r\n");

        esp_err_t ret = ESP_FAIL;
        {
            std::lock_guard<std::mutex> lock(sscma_mutex_);
            ret = sscma_client_request(sscma_client_handle_, cmd, &reply, true, pdMS_TO_TICKS(500));
        }
        if (ret == ESP_OK && reply.payload != nullptr) {
            cJSON* code = cJSON_GetObjectItem(reply.payload, "code");
            if (code != nullptr && cJSON_IsNumber(code) && code->valueint == 0) {
                ok = true;
            }
            sscma_client_reply_clear(&reply);
        }
        if (ok) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Himax WE2 face model probe: %s",
             ok ? "AVAILABLE" : "NOT SUPPORTED (fallback to object detection)");
    return ok;
}

bool SscmaCamera::SendFaceModeCommand(bool enable) {
    // Any explicit AT+FACE mode change (main loop, SetCameraMode, etc.) puts
    // Himax into a state the single-shot warm path can no longer assume.
    // BenchSingleShotFaceEmbedding does NOT route through here (it issues raw
    // AT+FACE requests), so this never clobbers its own warm bookkeeping.
    himax_face_warm_.store(false);

    sscma_client_reply_t reply = {0};
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+FACE=%d\r\n", enable ? 1 : 0);

    esp_err_t ret = ESP_FAIL;
    {
        std::lock_guard<std::mutex> lock(sscma_mutex_);
        ret = sscma_client_request(sscma_client_handle_, cmd, &reply, true, pdMS_TO_TICKS(2000));
    }
    bool success = false;
    if (ret == ESP_OK && reply.payload != nullptr) {
        cJSON* code = cJSON_GetObjectItem(reply.payload, "code");
        if (code != nullptr && cJSON_IsNumber(code) && code->valueint == 0) {
            success = true;
        }
        sscma_client_reply_clear(&reply);
    }

    if (!success) {
        ESP_LOGE(TAG, "Failed or rejected to set face mode %d (ret=%d)", enable ? 1 : 0, ret);
        return false;
    }

    ESP_LOGI(TAG, "Face mode %s", enable ? "enabled" : "disabled");
    return true;
}

// Decide whether a single-shot face embed is allowed right now. Returns nullptr
// when allowed, else a machine-readable reason for the HTTP/MCP caller to surface.
const char* SscmaCamera::FaceEmbedBlockReason() const {
    DeviceState st = Application::GetInstance().GetDeviceState();

    // OTA / firmware upgrade: flash is being written and SRAM is tight — never
    // drive Himax or churn heap during this window.
    if (st == kDeviceStateUpgrading) {
        return "upgrading";
    }

    bool voice_busy = (st == kDeviceStateConnecting ||
                       st == kDeviceStateListening ||
                       st == kDeviceStateSpeaking);

    // Passive greeting actively running (idle + enabled). External face calls
    // are not supported in that mode — they'd interrupt the greeting loop.
    // During a conversation greeting is suspended (voice_busy), so allow that.
    if (this->face_recognition_en_.load() && !voice_busy) {
        return "greeting_active";
    }

    // Transient Himax contention.
    if (this->inference_paused_.load() ||
        this->capture_in_progress_.load() ||
        this->single_shot_pending_.load()) {
        return "busy";
    }

    return nullptr;
}

// Bench (path Z): drive Himax through a single inference and capture the
// resulting embedding. Two paths:
//   - COLD: break + AT+FACE=1 + set_sensor(240) + 200ms settle, then invoke.
//     Used whenever Himax is not known to be warm (~820ms total).
//   - WARM (热待命): skip all setup and invoke directly. Only taken when
//     himax_face_warm_ is set AND the device is voice-busy (conversation),
//     where the main camera task is guaranteed not to touch Himax. ~150ms.
// We hold sscma_mutex_ throughout the trigger + wait so the main loop's
// mode-switch logic blocks and resumes cleanly. The on_event callback does
// not take sscma_mutex_, so the wait won't deadlock.
//
// Teardown also depends on voice-busy:
//   - voice-busy: leave Himax resident in face mode (warm) for the next call.
//   - idle: send AT+FACE=0 to restore a clean state and set sscma_restarted_
//     so the main loop re-establishes passive face / object-detection mode
//     (otherwise passive greeting would silently stall — pre-existing bug).
bool SscmaCamera::BenchSingleShotFaceEmbedding(float* out_embedding,
                                               SingleShotTiming* out_timing,
                                               bool want_image) {
    if (out_embedding == nullptr) {
        return false;
    }
    // Refuse concurrent calls.
    bool expected = false;
    if (!single_shot_pending_.compare_exchange_strong(expected, true)) {
        ESP_LOGW(TAG, "[BENCH] single-shot already in progress");
        return false;
    }
    single_shot_valid_.store(false);

    // 做法 B preview state (owned by this call only — guarded by single_shot_pending_).
    // Drop any stale JPEG from a prior aborted run before arming the hook.
    single_shot_jpeg_ready_.store(false);
    if (single_shot_jpeg_ != nullptr) {
        heap_caps_free(single_shot_jpeg_);
        single_shot_jpeg_ = nullptr;
    }
    single_shot_jpeg_len_ = 0;
    single_shot_want_image_.store(want_image);

    // Capture voice-busy once and use it consistently for both the warm-path
    // decision and the teardown decision. The warm flag is only trusted during
    // a conversation, where the main camera task leaves Himax alone.
    DeviceState dev_state = Application::GetInstance().GetDeviceState();
    bool voice_busy = (dev_state == kDeviceStateConnecting ||
                       dev_state == kDeviceStateListening ||
                       dev_state == kDeviceStateSpeaking);
    bool warm = voice_busy && himax_face_warm_.load();

    SingleShotTiming t;
    int64_t t0 = esp_timer_get_time();
    bool got = false;

    {
        // Held throughout: the main loop's mode-switch logic will block
        // on this mutex and resume cleanly once we're done.
        std::lock_guard<std::mutex> lock(sscma_mutex_);

        // Unconditionally stop dropping event frames: this single-shot needs its
        // own type:1 event to deliver the embedding. Covers the hot-standby embed
        // during a conversation, where the wake trigger set the drop flag and the
        // main loop's face-mode teardown may not have cleared it yet.
        SscmaCamera::SetDropEvents(false);

        // Stage 1: setup. Skipped on the warm path (Himax already in face mode,
        // sensor warm at 240x240).
        if (!warm) {
            sscma_client_break(sscma_client_handle_);
            {
                sscma_client_reply_t reply = {0};
                const char* cmd = "AT+FACE=1\r\n";
                esp_err_t ret = sscma_client_request(sscma_client_handle_, cmd,
                                                     &reply, true,
                                                     pdMS_TO_TICKS(2000));
                bool at_face_ok = false;
                if (ret == ESP_OK && reply.payload != NULL) {
                    cJSON* code = cJSON_GetObjectItem(reply.payload, "code");
                    if (code != nullptr && cJSON_IsNumber(code) && code->valueint == 0) {
                        at_face_ok = true;
                    }
                    sscma_client_reply_clear(&reply);
                }
                if (!at_face_ok) {
                    ESP_LOGE(TAG, "[BENCH] AT+FACE=1 failed or rejected (ret=%d)", ret);
                    himax_face_warm_.store(false);
                    single_shot_want_image_.store(false);
                    single_shot_pending_.store(false);
                    return false;
                }
            }
            sscma_client_set_sensor(sscma_client_handle_, 1, 0, true);
            vTaskDelay(pdMS_TO_TICKS(200));  // settle for sensor re-init
        }

        int64_t t_at_face = esp_timer_get_time();
        t.at_face_us = t_at_face - t0;

        // Stage 2: invoke continuously and break out when on_event delivers
        // an embedding. times=1 would only process the *first* frame after
        // the switch — which is usually blank/transient (camera hasn't
        // stabilized), so we never recover. times=-1 keeps Himax processing
        // every frame; the hook in on_event populates single_shot_embedding_
        // on the first face it sees, and the teardown sscma_client_break()
        // below stops Himax. End result is functionally the same as
        // "single shot" from the caller's perspective.
        //
        // 做法 B: show=want_image. When a preview is wanted we ask Himax to attach
        // the JPEG to the face event so on_event can grab the SAME frame that
        // produced the embedding (invoke show=true). Note the historical SRAM
        // caveat of show=true on invoke(-1): the on_event hook breaks out on the
        // first face, and teardown's sscma_client_break() below stops the stream
        // promptly, so only one image-bearing frame is parsed.
        esp_err_t ret = sscma_client_invoke(sscma_client_handle_, -1, false, want_image);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[BENCH] invoke failed: %d", ret);
            sscma_client_break(sscma_client_handle_);
        } else {
            // Stage 3: wait for on_event hook to populate the buffer.
            const int64_t kTimeoutUs = 3000000;  // 3 s
            int64_t deadline = esp_timer_get_time() + kTimeoutUs;
            while (!single_shot_valid_.load() && esp_timer_get_time() < deadline) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        int64_t t_result = esp_timer_get_time();
        t.invoke_to_result_us = t_result - t_at_face;

        got = single_shot_valid_.load();
        if (got) {
            memcpy(out_embedding, single_shot_embedding_,
                   sizeof(float) * FACE_EMBEDDING_DIM);
            t.face_score = single_shot_face_score_.load();
            t.face_quality = single_shot_face_quality_x1000_.load() / 1000.0f;
        } else {
            ESP_LOGW(TAG, "[BENCH] timeout waiting for embedding");
        }

        // Stage 4: teardown — stop the invoke loop either way.
        sscma_client_break(sscma_client_handle_);

        // Re-read voice state at teardown: the entry snapshot (`voice_busy`) may
        // be stale if the conversation ended mid-call. Decide the teardown on the
        // *current* state so we never leave Himax stuck warm after voice ends —
        // an idle device must get the clean teardown that restarts passive modes.
        DeviceState end_state = Application::GetInstance().GetDeviceState();
        bool end_voice_busy = (end_state == kDeviceStateConnecting ||
                               end_state == kDeviceStateListening ||
                               end_state == kDeviceStateSpeaking);
        if (end_voice_busy) {
            // Leave Himax resident in face mode for the next call in this
            // conversation (sensor stays warm at 240x240) — but only if this
            // shot actually produced a face. If not, distrust the warm state so
            // the next call does a full cold re-setup to self-heal.
            himax_face_warm_.store(got);
        } else {
            // Idle: restore a clean state and let the main loop reclaim Himax
            // (passive greeting / object detection) on its next iteration.
            sscma_client_reply_t reply = {0};
            const char* cmd = "AT+FACE=0\r\n";
            sscma_client_request(sscma_client_handle_, cmd, &reply, true,
                                 pdMS_TO_TICKS(2000));
            if (reply.payload != NULL) {
                sscma_client_reply_clear(&reply);
            }
            himax_face_warm_.store(false);
            sscma_restarted_.store(true);
        }

        int64_t t_end = esp_timer_get_time();
        t.teardown_us = t_end - t_result;
        t.total_us = t_end - t0;
    }

    ESP_LOGI(TAG,
             "[BENCH] %s at_face=%ld invoke_to_result=%ld teardown=%ld total=%ld us  got=%d score=%d quality=%.2f",
             warm ? "WARM" : "COLD",
             (long)t.at_face_us,
             (long)t.invoke_to_result_us,
             (long)t.teardown_us,
             (long)t.total_us,
             got ? 1 : 0,
             t.face_score,
             t.face_quality);

    if (out_timing) {
        *out_timing = t;
    }

    // 做法 B preview: decode the single-shot's OWN JPEG (the exact frame that
    // matched, captured in on_event) to RGB565 and hand it to the shared
    // MarshalPreviewFrame — so preview frame == match frame. Done outside the
    // SPI lock (JPEG decode is not SPI), mirroring CaptureImpl. Best-effort: any
    // failure just skips the preview; `got` is already decided. The 240x240 face
    // JPEG's real dimensions come from the decoded header (not the 640x480
    // preview_image_ descriptor, which is only for the object/photo path).
    if (want_image && single_shot_jpeg_ready_.load() &&
        single_shot_jpeg_ != nullptr &&
        jpeg_dec_ && jpeg_io_ && jpeg_out_) {
        memset(jpeg_io_, 0, sizeof(jpeg_dec_io_t));
        memset(jpeg_out_, 0, sizeof(jpeg_dec_header_info_t));
        jpeg_io_->inbuf = single_shot_jpeg_;
        jpeg_io_->inbuf_len = single_shot_jpeg_len_;
        int jret = jpeg_dec_parse_header(jpeg_dec_, jpeg_io_, jpeg_out_);
        if (jret >= 0 && jpeg_out_->width > 0 && jpeg_out_->height > 0) {
            uint16_t jw = jpeg_out_->width;
            uint16_t jh = jpeg_out_->height;
            size_t rgb_size = (size_t)jw * (size_t)jh * 2;
            uint8_t* rgb = (uint8_t*)jpeg_calloc_align(rgb_size, 16);
            if (rgb != nullptr) {
                jpeg_io_->outbuf = rgb;
                int consumed = jpeg_io_->inbuf_len - jpeg_io_->inbuf_remain;
                jpeg_io_->inbuf = single_shot_jpeg_ + consumed;
                jpeg_io_->inbuf_len = jpeg_io_->inbuf_remain;
                if (jpeg_dec_process(jpeg_dec_, jpeg_io_) == JPEG_ERR_OK) {
                    MarshalPreviewFrame(rgb, jw, jh);
                } else {
                    ESP_LOGW(TAG, "[BENCH] preview JPEG decode failed");
                }
                heap_caps_free(rgb);
            }
        } else {
            ESP_LOGW(TAG, "[BENCH] preview JPEG header parse failed (ret=%d)", jret);
        }
    }

    // Unified single-shot image cleanup — sole owner frees the JPEG exactly once.
    single_shot_want_image_.store(false);
    single_shot_jpeg_ready_.store(false);
    if (single_shot_jpeg_ != nullptr) {
        heap_caps_free(single_shot_jpeg_);
        single_shot_jpeg_ = nullptr;
    }
    single_shot_jpeg_len_ = 0;

    single_shot_pending_.store(false);
    return got;
}

void SscmaCamera::InitializeMcpTools() {
    
    Settings settings("model", false);
    detect_threshold = settings.GetInt("threshold", 75);
    detect_invoke_interval_sec = settings.GetInt("interval", 5);
    detect_duration_sec = settings.GetInt("duration", 3);
    detect_target = settings.GetInt("target", 0);
    inference_en = settings.GetInt("enable", 0);

    // Sync timing parameters to face recognition (shared duration/interval)
    auto& face_rec = FaceRecognition::GetInstance();
    face_rec.SetValidationDuration(detect_duration_sec.load());
    face_rec.SetCooldownInterval(detect_invoke_interval_sec.load());

    auto& mcp_server = McpServer::GetInstance();
    // 设置/查询模型参数配置（不带参数=查询，与 self.vision.mode 同款模式）
    mcp_server.AddUserOnlyTool("self.model.param_set",
        "配置或查询视觉模型检测参数。不带任何参数调用=查询当前配置。\n"
        "参数(均为可选，未提供的保持不变)：\n"
        "  `threshold`: 置信度阈值 (0-100)。提高可减少误报，但可能漏检；\n"
        "  `interval`: 冷却时间(秒)。对话结束后多久内不再触发检测；\n"
        "  `duration`: 持续检测确认时间(秒)。\n"
        "  `target`: 检测目标的索引 ID。\n"
        "返回：应用后的完整配置 {threshold, interval, duration, target}。",
        PropertyList({
            Property("threshold", kPropertyTypeInteger, -1, -1, 100),
            Property("interval", kPropertyTypeInteger, -1, -1, 60),
            Property("duration", kPropertyTypeInteger, -1, -1, 60),
            Property("target", kPropertyTypeInteger, -1, -1, this->model_class_cnt > 0 ? this->model_class_cnt - 1 : 255)
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            Settings settings("model", true);
            try {
                const Property& threshold_prop = properties["threshold"];
                int threshold = threshold_prop.value<int>();
                if (threshold != -1) {
                    settings.SetInt("threshold", threshold);
                    this->detect_threshold = threshold;
                    ESP_LOGI(TAG, "Set detection threshold to %d", threshold);
                }
            } catch (const std::runtime_error&) {
                // threshold parameter not provided, skip
            }
            
            try {
                const Property& interval_prop = properties["interval"];
                int interval = interval_prop.value<int>();
                if (interval != -1) {
                    settings.SetInt("interval", interval);
                    this->detect_invoke_interval_sec = interval;
                    FaceRecognition::GetInstance().SetCooldownInterval(interval);
                    ESP_LOGI(TAG, "Set detection interval to %d", interval);
                }
            } catch (const std::runtime_error&) {
                // interval parameter not provided, skip
            }

            try {
                const Property& duration_prop = properties["duration"];
                int duration = duration_prop.value<int>();
                if (duration != -1) {
                    settings.SetInt("duration", duration);
                    this->detect_duration_sec = duration;
                    FaceRecognition::GetInstance().SetValidationDuration(duration);
                }
            } catch (const std::runtime_error&) {
                // duration parameter not provided, skip
            }
            
            try {
                const Property& target_prop = properties["target"];
                int target = target_prop.value<int>();
                if (target != -1) {
                    settings.SetInt("target", target);
                    this->detect_target = target;
                    ESP_LOGI(TAG, "Set detection target to %d", target);
                }
            } catch (const std::runtime_error&) {
                // target_type parameter not provided, skip
            }

            // Always echo the resulting config — this is also the query path
            // (parameterless call sets nothing and just reads back), which is
            // why the old self.model.param_get tool is gone.
            // `target_type` is emitted alongside `target` for wire compat with
            // param_get's old response shape.
            int cur_threshold = settings.GetInt("threshold", 75);
            int cur_interval = settings.GetInt("interval", 5);
            int cur_duration = settings.GetInt("duration", 3);
            int cur_target = settings.GetInt("target", 0);
            return std::string(
                "{\"threshold\":" + std::to_string(cur_threshold) +
                ",\"interval\":" + std::to_string(cur_interval) +
                ",\"duration\":" + std::to_string(cur_duration) +
                ",\"target\":" + std::to_string(cur_target) +
                ",\"target_type\":" + std::to_string(cur_target) + "}");
        });

    // 推理开关获取
    // Unified proactive-wake switch: one tool covering all four mutually
    // exclusive modes (replaces the old self.model.enable / self.face.enable /
    // self.face.familiar_mode trio).
    mcp_server.AddUserOnlyTool("self.vision.mode",
        "设置或查询摄像头主动唤醒模式（四种互斥；不带 mode 参数=查询）。\n"
        "  0=关闭  1=人脸检测（只判断有没有人，不识别身份）\n"
        "  2=人脸识别（熟人报名字，陌生人报 person detected）  3=熟人免打扰（仅陌生人唤醒）\n"
        "【重要区分】'检测到人/看到人'一律用 1；只有用户明确要求'识别身份/认人/报名字'才用 2。\n"
        "常见说法：'别盯着我/关掉'→0；'物体检测/有人来提醒我'→1；'认人'→2；'免打扰'→3。\n"
        "返回 mode=目标模式，applied=实际生效模式，status=active/switching。switching 表示"
        "对话中的切换会在对话结束后几秒生效，属正常，告知用户已设置即可，不要重试。",
        PropertyList({
            // -1 sentinel = "mode not provided" (query). A real default here
            // would be snapshotted at registration time (before NVS state is
            // final) and silently rewrite the mode on every parameterless call.
            Property("mode", kPropertyTypeInteger, -1, -1, 3)
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            int mode = properties["mode"].value<int>();
            if (mode >= 0) {
                this->SetVisionWakeMode(mode);
            }
            // Report both the configured intent and what is actually running.
            int intent = this->GetVisionWakeMode();
            int applied = this->applied_vision_mode_.load();
            std::string status = (applied == intent) ? "active" : "switching";
            std::string ret = "{\"mode\":" + std::to_string(intent) +
                ",\"applied\":" + std::to_string(applied) +
                ",\"status\":\"" + status + "\"";
            if (applied != intent) {
                ret += ",\"note\":\"switch applies within a few seconds after the conversation ends\"";
            }
            ret += "}";
            return ret;
        });
}

void SscmaCamera::SetExplainUrl(const std::string& url, const std::string& token) {
    explain_url_ = url;
    explain_token_ = token;
}

bool SscmaCamera::Capture() {
    return CaptureImpl(true);
}

int SscmaCamera::CaptureJpegLocked(bool (*sink)(void*, const uint8_t*, size_t), void* ctx) {
    uint8_t* jpeg_copy = nullptr;
    size_t jpeg_len = 0;
    {
        // 与 IdentifyOnce/current-speaker 拉取串行化，避免并发抢帧/覆盖 jpeg_data_。
        std::unique_lock<std::mutex> lk(identify_op_mutex_, std::try_to_lock);
        if (!lk.owns_lock()) {
            return 503;  // 另一路 identify/capture 正在进行
        }
        // TOCTOU：拿锁后复查阻断原因（问候识别 active 等），与 IdentifyOnce 一致。
        const char* block = FaceEmbedBlockReason();
        if (block != nullptr) {
            ESP_LOGW(TAG, "CaptureJpegLocked blocked: %s", block);
            return 503;
        }
        TaskPriorityReset priority_reset(1);  // 抓拍期间降优先级（take_photo 同款）
        if (!Capture() || jpeg_data_.buf == nullptr || jpeg_data_.len == 0) {
            ESP_LOGE(TAG, "CaptureJpegLocked: Capture failed / empty JPEG");
            return 500;
        }
        jpeg_len = jpeg_data_.len;
        jpeg_copy = static_cast<uint8_t*>(
            heap_caps_malloc(jpeg_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (jpeg_copy == nullptr) {
            ESP_LOGE(TAG, "CaptureJpegLocked: OOM copying %zu-byte JPEG", jpeg_len);
            return 500;
        }
        memcpy(jpeg_copy, jpeg_data_.buf, jpeg_len);
    }

    // 网络发送可能阻塞；使用独立副本，避免长时间占用 identify_op_mutex_。
    bool sent = sink != nullptr && sink(ctx, jpeg_copy, jpeg_len);
    heap_caps_free(jpeg_copy);
    return sent ? 200 : 500;
}

// Single owner of the preview marshaling (§8e F4). Copies the decoded RGB565
// frame into an independent SPIRAM buffer and schedules the LVGL upload onto the
// main loop; LvglAllocatedImage frees the copy exactly once in its destructor
// (freed manually here only when no display is present). The `rgb565` source
// stays owned by the caller. Used by BOTH CaptureImpl (640x480 photo) and the
// single-shot preview path (240x240 face frame) — see the header note.
void SscmaCamera::MarshalPreviewFrame(const uint8_t* rgb565, uint16_t w, uint16_t h) {
    if (rgb565 == nullptr || w == 0 || h == 0) {
        return;
    }
    size_t image_size = (size_t)w * (size_t)h * 2;
    size_t stride = (size_t)w * 2;

    uint8_t* data = (uint8_t*)heap_caps_malloc(image_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (data == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate memory for display image");
        return;
    }
    memcpy(data, rgb565, image_size);
    Application::GetInstance().Schedule([data, image_size, w, h, stride]() {
        auto display = dynamic_cast<LvglDisplay*>(Board::GetInstance().GetDisplay());
        if (display == nullptr) {
            heap_caps_free(data);  // no display: avoid leaking the copy
            return;
        }
        auto image = std::make_unique<LvglAllocatedImage>(
            data, image_size, w, h, stride, LV_COLOR_FORMAT_RGB565);
        display->SetPreviewImage(std::move(image));
    });
}

bool SscmaCamera::CaptureImpl(bool drive_preview) {

    SscmaData data;
    int ret = 0;

    if (sscma_client_handle_ == nullptr) {
        ESP_LOGE(TAG, "SSCMA client handle is not initialized");
        return false;
    }

    capture_in_progress_ = true;
    capture_started_at_ = esp_timer_get_time() / 1000000;
    // Capture reconfigures the sensor (mode 3) → any single-shot warm state is stale.
    himax_face_warm_.store(false);

    // 清空队列中的残留数据
    SscmaData stale;
    while (xQueueReceive(sscma_data_queue_, &stale, 0) == pdPASS) {
        if (stale.img) {
            heap_caps_free(stale.img);
        }
    }

    {
        std::lock_guard<std::mutex> lock(sscma_mutex_);
        // 停止可能残留的推理，确保拍照不冲突
        sscma_client_break(sscma_client_handle_);
    }

    if ([this]() {
            std::lock_guard<std::mutex> lock(sscma_mutex_);
            return sscma_client_set_sensor(sscma_client_handle_, 1, 3, true);
        }()) {
        ESP_LOGE(TAG, "Failed to set sensor");
        capture_in_progress_ = false;
        return false;
    }
    ESP_LOGI(TAG, "Capturing image...");
    // himax 可能有缓存数据, 拍两张取最新的.
    if ([this]() {
            std::lock_guard<std::mutex> lock(sscma_mutex_);
            return sscma_client_sample(sscma_client_handle_, 2);
        }()) {
        ESP_LOGE(TAG, "Failed to capture image from SSCMA client");
        capture_in_progress_ = false;
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(500)); // 等待SSCMA客户端处理数据
    if (xQueueReceive(sscma_data_queue_, &data, pdMS_TO_TICKS(2000)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to receive JPEG data from SSCMA client");
        capture_in_progress_ = false;
        return false;
    }

    if (jpeg_data_.buf == nullptr) {
        heap_caps_free(data.img);
        capture_in_progress_ = false;
        return false;
    }

    ret = mbedtls_base64_decode(jpeg_data_.buf, IMG_JPEG_BUF_SIZE, &jpeg_data_.len, data.img, data.len);
    if (ret != 0 || jpeg_data_.len == 0) {
        ESP_LOGE(TAG, "Failed to decode base64 image data, ret: %d, output_len: %zu", ret, jpeg_data_.len);
        heap_caps_free(data.img);
        capture_in_progress_ = false;
        return false;
    }
    heap_caps_free(data.img);

    // drive_preview now means "should this capture show an on-screen preview".
    // When false, jpeg_data_ is still populated for RemoteRecognize; we just skip
    // the remote-cast upload and the local decode/display. When true, the display
    // step below is marshaled to the main loop (§8e F4), so this path is now
    // thread-safe from an httpd worker too.
    if (!drive_preview) {
        capture_in_progress_ = false;
        return true;
    }

    // 发送到远程显示
    auto* remote = RemoteDisplay::GetInstance();
    if (remote->IsRunning()) {
        remote->SendPreviewImage(jpeg_data_.buf, jpeg_data_.len);
    }

    //DECODE JPEG
    if (!jpeg_dec_ || !jpeg_io_ || !jpeg_out_) {
        capture_in_progress_ = false;
        return true;
    }
    if (!preview_image_.data) {
        preview_image_.data = (uint8_t*)jpeg_calloc_align(preview_image_.data_size, 16);
        if (preview_image_.data == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate memory for preview image");
            capture_in_progress_ = false;
            return true;
        }
    }
    jpeg_io_->inbuf = jpeg_data_.buf;
    jpeg_io_->inbuf_len = jpeg_data_.len;
    ret = jpeg_dec_parse_header(jpeg_dec_, jpeg_io_, jpeg_out_);
    if (ret < 0) {
        ESP_LOGE(TAG, "Failed to parse JPEG header, ret: %d", ret);
        heap_caps_free((void*)preview_image_.data);
        preview_image_.data = nullptr;
        capture_in_progress_ = false;
        return true;
    }
    jpeg_io_->outbuf = (unsigned char*)preview_image_.data;
    int inbuf_consumed = jpeg_io_->inbuf_len - jpeg_io_->inbuf_remain;
    jpeg_io_->inbuf =  jpeg_data_.buf + inbuf_consumed;
    jpeg_io_->inbuf_len = jpeg_io_->inbuf_remain;

    ret = jpeg_dec_process(jpeg_dec_, jpeg_io_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to decode JPEG image, ret: %d", ret);
        heap_caps_free((void*)preview_image_.data);
        preview_image_.data = nullptr;
        capture_in_progress_ = false;
        return true;
    }

    // 显示预览图片 — thread-safe marshaling (§8e F4) via the shared helper. The
    // decoded RGB565 frame (whatever thread ran the capture — possibly an httpd
    // worker for the backend-direct /current-speaker?fresh=1 pull) is copied and
    // its LVGL upload deferred to the main loop inside MarshalPreviewFrame.
    MarshalPreviewFrame((const uint8_t*)preview_image_.data,
                        preview_image_.header.w, preview_image_.header.h);
    heap_caps_free((void*)preview_image_.data);
    preview_image_.data = nullptr;
    capture_in_progress_ = false;
    return true;
}
bool SscmaCamera::SetHMirror(bool enabled) {
    return false;
}

bool SscmaCamera::SetVFlip(bool enabled) {
    return false;
}

/**
 * @brief 将摄像头捕获的图像发送到远程服务器进行AI分析和解释
 * 
 * 该函数将当前摄像头缓冲区中的图像编码为JPEG格式，并通过HTTP POST请求
 * 以multipart/form-data的形式发送到指定的解释服务器。服务器将根据提供的
 * 问题对图像进行AI分析并返回结果。
 * 
 * @param question 要向AI提出的关于图像的问题，将作为表单字段发送
 * @return std::string 服务器返回的JSON格式响应字符串
 *         成功时包含AI分析结果，失败时包含错误信息
 *         格式示例：{"success": true, "result": "分析结果"}
 *                  {"success": false, "message": "错误信息"}
 * 
 * @note 调用此函数前必须先调用SetExplainUrl()设置服务器URL
 * @note 函数会等待之前的编码线程完成后再开始新的处理
 * @warning 如果摄像头缓冲区为空或网络连接失败，将返回错误信息
 */
std::string SscmaCamera::Explain(const std::string& question) {
    if (explain_url_.empty()) {
        return "{\"success\": false, \"message\": \"Image explain URL or token is not set\"}";
    }

    auto network = Board::GetInstance().GetNetwork();
    if (!network) {
        return "{\"success\": false, \"message\": \"Network not available\"}";
    }
    auto http = network->CreateHttp(3);
    if (!http) {
        return "{\"success\": false, \"message\": \"Failed to create HTTP client\"}";
    }
    // 构造multipart/form-data请求体
    std::string boundary = "----ESP32_CAMERA_BOUNDARY";
    
    // 构造question字段
    std::string question_field;
    question_field += "--" + boundary + "\r\n";
    question_field += "Content-Disposition: form-data; name=\"question\"\r\n";
    question_field += "\r\n";
    question_field += question + "\r\n";
    
    // 构造文件字段头部
    std::string file_header;
    file_header += "--" + boundary + "\r\n";
    file_header += "Content-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\"\r\n";
    file_header += "Content-Type: image/jpeg\r\n";
    file_header += "\r\n";
    
    // 构造尾部
    std::string multipart_footer;
    multipart_footer += "\r\n--" + boundary + "--\r\n";

    // 配置HTTP客户端，使用分块传输编码
    http->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    http->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());
    if (!explain_token_.empty()) {
        http->SetHeader("Authorization", "Bearer " + explain_token_);
    }
    http->SetHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    http->SetHeader("Transfer-Encoding", "chunked");
    if (!http->Open("POST", explain_url_)) {
        ESP_LOGE(TAG, "Failed to connect to explain URL");
        return "{\"success\": false, \"message\": \"Failed to connect to explain URL\"}";
    }
    
    // 第一块：question字段
    http->Write(question_field.c_str(), question_field.size());
    
    // 第二块：文件字段头部
    http->Write(file_header.c_str(), file_header.size());
    
    // 第三块：JPEG数据
    http->Write((const char*)jpeg_data_.buf, jpeg_data_.len);

    // 第四块：multipart尾部
    http->Write(multipart_footer.c_str(), multipart_footer.size());
    
    // 结束块
    http->Write("", 0);

    if (http->GetStatusCode() != 200) {
        ESP_LOGE(TAG, "Failed to upload photo, status code: %d", http->GetStatusCode());
        return "{\"success\": false, \"message\": \"Failed to upload photo\"}";
    }

    std::string result = http->ReadAll();
    http->Close();

    ESP_LOGI(TAG, "Explain image size=%d, question=%s\n%s", jpeg_data_.len, question.c_str(), result.c_str());
    return result;
}

int SscmaCamera::RemoteRecognize(const std::string& base_url, const std::string& token,
                                 std::string* out_name, float* out_confidence) {
    // Mirrors the legacy FaceRecognition() HTTP path (proven memory pattern),
    // but URL/token come from NVS-pushed warehouse config instead of a
    // hardcoded IP, and the result is returned structured instead of XML.
    if (jpeg_data_.len == 0 || jpeg_data_.buf == nullptr) {
        ESP_LOGE(TAG, "RemoteRecognize: no captured image");
        return -1;
    }

    size_t base64_len = 0;
    mbedtls_base64_encode(NULL, 0, &base64_len, jpeg_data_.buf, jpeg_data_.len);
    uint8_t* base64_buf = (uint8_t*)heap_caps_aligned_alloc(16, base64_len + 1, MALLOC_CAP_SPIRAM);
    if (!base64_buf) {
        ESP_LOGE(TAG, "RemoteRecognize: OOM base64 buffer (%zu bytes)", base64_len + 1);
        return -2;
    }
    if (mbedtls_base64_encode(base64_buf, base64_len + 1, &base64_len,
                              jpeg_data_.buf, jpeg_data_.len) != 0) {
        heap_caps_free(base64_buf);
        return -2;
    }
    base64_buf[base64_len] = '\0';

    cJSON* json = cJSON_CreateObject();
    if (!json) {
        heap_caps_free(base64_buf);
        return -2;
    }
    cJSON_AddStringToObject(json, "image_base64", (const char*)base64_buf);
    char* json_str = cJSON_PrintUnformatted(json);
    if (!json_str) {
        heap_caps_free(base64_buf);
        cJSON_Delete(json);
        return -2;
    }

    auto network = Board::GetInstance().GetNetwork();
    if (!network) {
        ESP_LOGE(TAG, "RemoteRecognize: network unavailable");
        heap_caps_free(base64_buf);
        cJSON_Delete(json);
        free(json_str);
        return -3;
    }
    auto http = network->CreateHttp(3);
    if (!http) {
        ESP_LOGE(TAG, "RemoteRecognize: failed to create HTTP client");
        heap_caps_free(base64_buf);
        cJSON_Delete(json);
        free(json_str);
        return -3;
    }
    http->SetHeader("Content-Type", "application/json");
    if (!token.empty()) {
        http->SetHeader("Authorization", ("Bearer " + token).c_str());
    }
    http->SetContent(std::string(json_str));

    std::string url = base_url;
    while (!url.empty() && url.back() == '/') url.pop_back();
    url += "/recognize";
    ESP_LOGI(TAG, "RemoteRecognize -> %s (image %zu bytes)", url.c_str(), jpeg_data_.len);

    if (!http->Open("POST", url)) {
        ESP_LOGE(TAG, "RemoteRecognize: failed to connect %s", url.c_str());
        heap_caps_free(base64_buf);
        cJSON_Delete(json);
        free(json_str);
        return -3;
    }
    int status_code = http->GetStatusCode();
    if (status_code != 200) {
        ESP_LOGE(TAG, "RemoteRecognize: HTTP %d", status_code);
        heap_caps_free(base64_buf);
        cJSON_Delete(json);
        free(json_str);
        http->Close();
        return -3;
    }
    std::string response_str = http->ReadAll();
    http->Close();
    heap_caps_free(base64_buf);
    cJSON_Delete(json);
    free(json_str);

    cJSON* response = cJSON_Parse(response_str.c_str());
    if (!response) {
        ESP_LOGE(TAG, "RemoteRecognize: bad JSON: %.120s", response_str.c_str());
        return -4;
    }
    cJSON* matched = cJSON_GetObjectItem(response, "matched");
    cJSON* name = cJSON_GetObjectItem(response, "name");
    cJSON* confidence = cJSON_GetObjectItem(response, "confidence");
    if (!cJSON_IsBool(matched)) {
        cJSON_Delete(response);
        return -4;
    }
    int rc = 1;  // authoritative no-match
    if (cJSON_IsTrue(matched) && cJSON_IsString(name)) {
        if (out_name) *out_name = name->valuestring;
        if (out_confidence) {
            *out_confidence = cJSON_IsNumber(confidence) ? (float)confidence->valuedouble : 0.0f;
        }
        rc = 0;
    }
    cJSON_Delete(response);
    return rc;
}

bool SscmaCamera::SetCameraMode(CameraMode mode) {
    if (mode == camera_mode_) {
        return true;
    }

    ESP_LOGI(TAG, "Switching camera mode to %s",
             mode == MODE_OBJECT_DETECT ? "OBJECT_DETECT" : "FACE_RECOGNITION");

    // Stop current inference
    {
        std::lock_guard<std::mutex> lock(sscma_mutex_);
        sscma_client_break(sscma_client_handle_);
    }

    if (mode == MODE_FACE_RECOGNITION) {
        // Switch to face recognition mode
        if (!SendFaceModeCommand(true)) {
            return false;
        }
        camera_mode_ = MODE_FACE_RECOGNITION;
    } else {
        // Switch back to object detection mode
        SendFaceModeCommand(false);
        camera_mode_ = MODE_OBJECT_DETECT;
        std::lock_guard<std::mutex> lock(sscma_mutex_);
        sscma_client_set_model(sscma_client_handle_, 4);
    }

    return true;
}

void SscmaCamera::SetVisionWakeMode(int mode) {
    if (mode < VISION_OFF || mode > VISION_FACE_DND) {
        ESP_LOGW(TAG, "Invalid vision wake mode %d, ignoring", mode);
        return;
    }
    // Zero-crash Fallback: if face model is not available on Himax WE2,
    // downgrade face recognition modes (VISION_FACE / VISION_FACE_DND)
    // to object detection (VISION_OBJECT).
    if (!face_model_available_.load() && (mode == VISION_FACE || mode == VISION_FACE_DND)) {
        ESP_LOGW(TAG, "Face recognition mode %d requested but WE2 has only object detection; falling back to VISION_OBJECT", mode);
        mode = VISION_OBJECT;
    }

    const bool want_inference = (mode == VISION_OBJECT);
    const bool want_face      = (mode == VISION_FACE || mode == VISION_FACE_DND);
    const bool want_dnd       = (mode == VISION_FACE_DND);

    // Persist across the existing NVS namespaces (no migration needed).
    {
        Settings settings("model", true);
        settings.SetInt("enable", want_inference ? 1 : 0);
    }
    {
        Settings settings("face", true);
        settings.SetInt("enable", want_face ? 1 : 0);
        settings.SetInt("familiar_mode", want_dnd ? 1 : 0);
    }

    // Apply at runtime. The modes are mutually exclusive; the main camera task
    // already gives face mode priority over object detection, but we keep the
    // underlying switches consistent so GetVisionWakeMode() is unambiguous.
    inference_en = want_inference ? 1 : 0;
    face_recognition_en_ = want_face;
    auto& face_rec = FaceRecognition::GetInstance();
    face_rec.SetEnabled(want_face);
    face_rec.SetFamiliarMode(want_dnd);

    ESP_LOGI(TAG, "Vision wake mode set to %d (inference=%d face=%d dnd=%d)",
             mode, want_inference, want_face, want_dnd);
}

int SscmaCamera::GetVisionWakeMode() const {
    if (face_recognition_en_.load()) {
        return FaceRecognition::GetInstance().IsFamiliarMode()
                   ? VISION_FACE_DND : VISION_FACE;
    }
    if (inference_en.load()) {
        return VISION_OBJECT;
    }
    return VISION_OFF;
}

void SscmaCamera::InitializeFaceMcpTools() {
    auto& mcp_server = McpServer::GetInstance();
    auto& face_db = FaceDatabase::GetInstance();
    auto& face_rec = FaceRecognition::GetInstance();

    // Load face recognition settings
    Settings settings("face", false);
    if (!face_model_available_.load()) {
        if (settings.GetInt("enable", 0) != 0) {
            ESP_LOGW(TAG, "Face model not supported on WE2, demoting face enable to 0 in memory (NVS unchanged)");
        }
        face_recognition_en_ = false;
    } else {
        face_recognition_en_ = settings.GetInt("enable", 0);
    }
    // Default 0.40 (was 0.60): the local DB stores server-provisioned embeddings
    // computed from enrollment photos (simulator domain), while greeting matches
    // them against live device-NPU captures — cross-domain cosine peaks ~0.5 for
    // same person, ~0.34 for different. 0.40 separates them; 0.60 would miss
    // the same person. Calibrated against measured cross-domain distribution.
    float threshold = (float)settings.GetInt("threshold", 40) / 100.0f;
    face_rec.SetMatchThreshold(threshold);

    // Load familiar DND mode setting (熟人免打扰)
    bool familiar_mode = settings.GetInt("familiar_mode", 0) != 0;
    face_rec.SetFamiliarMode(familiar_mode);
    ESP_LOGI(TAG, "Familiar DND mode: %s", familiar_mode ? "ON" : "OFF");

    // Cut4 test: boot-time reset to object detection removed so the persisted
    // face-recognition mode (mode 2/3) survives reboot.

    // Tool: List all faces
    mcp_server.AddUserOnlyTool("self.face.list",
        "列出本地数据库中所有已录入的人脸。\n"
        "使用场景：当用户问'有哪些人脸'、'你认识谁'时调用。\n"
        "返回：已录入人脸的名称列表",
        PropertyList(),
        [&face_db](const PropertyList& properties) -> ReturnValue {
            auto faces = face_db.ListFaces();
            if (faces.empty()) {
                return std::string("当前没有已录入的人脸。");
            }
            std::string result = "已录入 " + std::to_string(faces.size()) + " 张人脸: ";
            for (size_t i = 0; i < faces.size(); i++) {
                if (i > 0) result += ", ";
                result += FaceDatabase::DecodeName(faces[i]);
            }
            return result;
        });

    // Tool: query the current conversation speaker (for permission-gated commands).
    // State is owned board-locally by FaceRecognition; the camera task freezes it
    // at conversation start and clears it at conversation end.
    mcp_server.AddUserOnlyTool("self.conversation.speaker",
        "返回当前对话的说话人身份（基于人脸识别）。仅用于闲聊类问题：用户问"
        "「你知道在跟谁说话吗」时调用。\n"
        "禁止：不要在出库/入库/移库等任何仓库操作前调用本工具——那些操作的身份"
        "校验由后端自动完成，与本工具无关，你无需也不应参与验证。\n"
        "返回：{\"valid\":bool,\"name\":string,\"subject_id\":int,\"similarity\":float}。\n"
        "valid=false 表示未知说话人或当前不在对话中。",
        PropertyList(),
        [&face_rec](const PropertyList&) -> ReturnValue {
            FaceRecognition::SpeakerIdentity s = face_rec.GetCurrentSpeaker();
            std::string name;  // minimal JSON string escaping
            for (char c : s.name) {
                if (c == '"' || c == '\\') name += '\\';
                name += c;
            }
            char sim[16];
            snprintf(sim, sizeof(sim), "%.4f", s.similarity);
            std::string result = "{\"valid\":";
            result += (s.valid ? "true" : "false");
            result += ",\"name\":\"" + name + "\",\"subject_id\":";
            result += std::to_string(s.subject_id);
            result += ",\"similarity\":";
            result += sim;
            result += "}";
            return result;
        });

    // Conversational identify tool (experience only — NOT an authorization path).
    // Stock-op security is enforced backend-direct via GET /api/face/current-
    // speaker?fresh=1, which never trusts LLM-forwarded identity. This tool exists
    // purely so the model can answer "认出我是谁 / who am I": a prompt injection
    // could at worst make it say a wrong name in chat — it cannot move inventory,
    // because the warehouse gate does not consult this tool's result. Shares
    // IdentifyOnce()/identify_op_mutex_ with the backend pull, so the two never
    // race a frame (one gets kBusy). allow_preview=true: runs on the main loop,
    // where SetPreviewImage is safe, so the user sees their photo.
    mcp_server.AddUserOnlyTool("self.face.identify",
        "现场拍一张照做人脸识别，仅用于回答「你是谁」这类问题（体验用途，不用于任何鉴权）。\n"
        "使用场景：仅当用户明确问「你能认出我吗 / 我是谁 / 看看这是谁」时调用。\n"
        "禁止：不要在出库/入库/移库等任何仓库操作前调用本工具做身份确认——那些操作"
        "的身份校验由后端自动完成，你无需也不应提前验证，直接调用仓库工具即可。\n"
        "返回：{\"valid\":bool,\"name\":string,\"subject_id\":int,\"similarity\":float}。\n"
        "valid=false 表示没拍到人脸或不认识（陌生人）；忙时返回 error=busy。",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue {
            IdentifyStatus st = IdentifyStatus::kOk;
            const char* reason = nullptr;
            FaceRecognition::SpeakerIdentity s =
                this->IdentifyOnce(/*allow_preview=*/true, &st, &reason);
            if (st != IdentifyStatus::kOk) {
                std::string e = "{\"valid\":false,\"error\":\"";
                e += (reason ? reason : "busy");
                e += "\"}";
                return e;
            }
            std::string name;  // minimal JSON string escaping
            for (char c : s.name) {
                if (c == '"' || c == '\\') name += '\\';
                name += c;
            }
            char sim[16];
            snprintf(sim, sizeof(sim), "%.4f", s.similarity);
            std::string result = "{\"valid\":";
            result += (s.valid ? "true" : "false");
            result += ",\"name\":\"" + name + "\",\"subject_id\":";
            result += std::to_string(s.subject_id);
            result += ",\"similarity\":";
            result += sim;
            result += "}";
            return result;
        });

    ESP_LOGI(TAG, "Face recognition MCP tools initialized");
}

// Shared on-demand identify — see the header for the F1..F4 contract. Called by
// the self.face.identify MCP tool (allow_preview=true, main loop) and the HTTP
// /api/face/current-speaker?fresh=1 endpoint (allow_preview=false, httpd worker).
FaceRecognition::SpeakerIdentity SscmaCamera::IdentifyOnce(
        bool allow_preview, IdentifyStatus* out_status, const char** out_reason) {
    FaceRecognition::SpeakerIdentity s;
    if (out_status) *out_status = IdentifyStatus::kOk;
    if (out_reason) *out_reason = nullptr;

    // F3: forbidden to queue. If another identify (MCP or HTTP) already holds the
    // op lock, bail out immediately so the caller can answer 503 instead of
    // blocking Himax behind a serialized backlog.
    std::unique_lock<std::mutex> op(identify_op_mutex_, std::try_to_lock);
    if (!op.owns_lock()) {
        if (out_status) *out_status = IdentifyStatus::kBusy;
        if (out_reason) *out_reason = "busy";
        return s;  // valid=false
    }

    // F2: TOCTOU re-check. The caller may have checked FaceEmbedBlockReason()
    // before entering, but state (upgrading / greeting / capture / single-shot)
    // can flip between that check and acquiring the op lock. Re-check now that we
    // hold the lock and own the operation; deny if not allowed.
    const char* block = FaceEmbedBlockReason();
    if (block != nullptr) {
        if (out_status) *out_status = IdentifyStatus::kBlocked;
        if (out_reason) *out_reason = block;
        return s;  // valid=false
    }

    // ---- dispatch (moved verbatim from the self.face.identify lambda) ----
    // Mode + endpoint come from the warehouse platform's face config, pushed down
    // with the library (batch-update -> NVS face.id_*):
    //   local (default) — on-device NPU embedding + local DB match
    //   lan             — fresh JPEG POSTed to <id_url>/recognize (name-only).
    // A lan transport error falls back to local; a lan "no match" is authoritative.
    Settings settings("face", false);
    std::string id_mode = settings.GetString("id_mode", "local");
    std::string id_url = settings.GetString("id_url", "");
    auto& rec = FaceRecognition::GetInstance();
    bool remote_answered = false;
    if (id_mode == "lan" && !id_url.empty()) {
        TaskPriorityReset priority_reset(1);
        // F4: httpd-worker path passes allow_preview=false -> CaptureImpl skips
        // preview upload; MCP main-loop path keeps the on-screen preview.
        if (this->CaptureImpl(allow_preview)) {
            std::string rname;
            float rconf = 0.0f;
            int rc = this->RemoteRecognize(
                id_url, settings.GetString("id_token", ""), &rname, &rconf);
            if (rc == 0) {
                s.valid = true;
                s.name = rname;
                s.subject_id = 0;  // remote library is name-keyed
                s.similarity = rconf;
                rec.SetCurrentSpeaker(s);
                remote_answered = true;
            } else if (rc == 1) {
                remote_answered = true;
            }
            // rc < 0: transport/protocol error -> local fallback below
        } else {
            ESP_LOGW(TAG, "identify(lan): capture failed, local fallback");
        }
    }
    if (!remote_answered) {
        // 做法 A（与 self.camera.take_photo 同款机制）：先用 Capture() 拍一张 JPEG 上屏。
        // 人脸推理的 sensor mode 0 不编码 JPEG（cisdp 拿不到图，做法 B 在 Watcher 上物理
        // 不可行，已实测确认）；只有 Capture 的 mode 3 才出 JPEG，就是 take_photo 好用的
        // 原因。代价：这张预览与喂推理的匹配帧不是同一张（差 1~2s、两个 sensor mode），
        // 但可见且稳定。best-effort：拍照失败不阻断识别。
        if (allow_preview) {
            this->CaptureImpl(/*drive_preview=*/true);
        }
        float embedding[FACE_EMBEDDING_DIM];
        SingleShotTiming t;
        if (!this->BenchSingleShotFaceEmbedding(embedding, &t, /*want_image=*/false)) {
            return s;  // valid=false (no face / timeout); status stays kOk
        }
        FaceMatchResult m = FaceDatabase::GetInstance().Match(
            embedding, rec.GetMatchThreshold());
        if (m.matched) {
            s.valid = true;
            s.name = FaceDatabase::DecodeName(m.name);
            s.subject_id = m.subject_id;
            s.similarity = m.similarity;
            rec.SetCurrentSpeaker(s);
        }
    }
    return s;
}

void SscmaCamera::PauseInference() {
    std::lock_guard<std::mutex> lock(pause_state_mutex_);
    if (inference_paused_.load()) return;

    // Save current state
    paused_inference_en_ = inference_en.load();
    paused_face_recognition_en_ = face_recognition_en_.load();

    // Disable both inference modes so the monitor task stops sending SPI commands.
    // Don't call sscma_client_break() here — it blocks on SPI and would freeze
    // the console REPL. The monitor task will detect the flag change and call
    // sscma_client_break itself on the next iteration.
    inference_en = 0;
    face_recognition_en_ = false;

    // External UART enrollment is about to take over Himax → warm state invalid.
    himax_face_warm_.store(false);

    inference_paused_ = true;
    inference_paused_at_ = esp_timer_get_time() / 1000000;  // seconds
    ESP_LOGI(TAG, "Inference paused (was: inference=%d, face=%d), auto-resume in %ds",
             paused_inference_en_, paused_face_recognition_en_, INFERENCE_PAUSE_TIMEOUT_SEC);
}

void SscmaCamera::ResumeInference() {
    std::lock_guard<std::mutex> lock(pause_state_mutex_);
    if (!inference_paused_.load()) return;

    // Restore saved state
    inference_en = paused_inference_en_;
    face_recognition_en_ = paused_face_recognition_en_;

    // Mark SSCMA as restarted so monitor task resets its local is_face_mode/is_inference
    // flags — the external UART session may have left Himax in an unknown state.
    // Clear warm before lifting the pause so no single-shot can slip in (after
    // inference_paused_ flips false) and trust a stale warm flag.
    himax_face_warm_.store(false);
    sscma_restarted_ = true;

    inference_paused_ = false;
    ESP_LOGI(TAG, "Inference resumed (inference=%d, face=%d)",
             inference_en.load(), face_recognition_en_.load());
}
