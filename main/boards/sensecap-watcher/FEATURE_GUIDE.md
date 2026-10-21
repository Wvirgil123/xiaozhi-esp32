# SenseCAP Watcher 人脸识别功能说明

## 功能概述

SenseCAP Watcher 实现了完整的人脸识别功能，通过 ESP32-S3 和 Himax WE2 双芯片协作完成：

- **Himax WE2**: 运行 AI 模型，完成人脸检测和特征提取
- **ESP32-S3**: 管理人脸数据库，执行识别匹配，处理用户交互

## 系统架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          SenseCAP Watcher 人脸识别系统                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                        ESP32-S3 (主控制器)                            │   │
│  │                                                                      │   │
│  │  ┌─────────────────┐   ┌─────────────────┐   ┌─────────────────────┐ │   │
│  │  │ FaceRecognition │   │  FaceDatabase   │   │   RemoteDisplay     │ │   │
│  │  │ Controller      │   │                 │   │                     │ │   │
│  │  │                 │   │ - NVS 存储      │   │ - WebSocket 服务    │ │   │
│  │  │ - 投票缓冲      │◄──│ - 最多 20 张脸  │   │ - UI 状态同步       │ │   │
│  │  │ - 质量评估      │   │ - 余弦相似度    │   │ - 远程预览          │ │   │
│  │  │ - 结果稳定      │   │                 │   │                     │ │   │
│  │  └────────┬────────┘   └─────────────────┘   └─────────────────────┘ │   │
│  │           │                                                          │   │
│  └───────────┼──────────────────────────────────────────────────────────┘   │
│              │ UART (AT 命令)                                               │
│  ┌───────────┼──────────────────────────────────────────────────────────┐   │
│  │           ▼                        Himax WE2 (AI 处理器)              │   │
│  │                                                                      │   │
│  │  ┌─────────────────┐   ┌─────────────────┐   ┌─────────────────────┐ │   │
│  │  │  AT 命令解析    │──►│  SCRFD 人脸检测 │──►│  人脸对齐           │ │   │
│  │  │                 │   │  (160x160)      │   │  (5 点关键点)       │ │   │
│  │  │  AT+FACE=1      │   │  ~600KB         │   │  相似性变换         │ │   │
│  │  └─────────────────┘   └─────────────────┘   └──────────┬──────────┘ │   │
│  │                                                         │            │   │
│  │                        ┌─────────────────┐              │            │   │
│  │                        │  MobileFaceNet  │◄─────────────┘            │   │
│  │                        │  (112x112)      │                           │   │
│  │                        │  128D 特征向量  │                           │   │
│  │                        │  ~1.2MB         │                           │   │
│  │                        └─────────────────┘                           │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 核心组件

### 1. 人脸检测 (Himax WE2)

使用 **SCRFD_500M_KPS** 模型：
- 输入分辨率: 160×160
- 输出: 人脸边界框 + 5 点关键点 (双眼、鼻尖、嘴角)
- Flash 地址: 0x200000
- 模型大小: ~600KB

### 2. 人脸特征提取 (Himax WE2)

使用 **MobileFaceNet** 模型：
- 输入分辨率: 112×112 (对齐后的人脸)
- 输出: 128 维特征向量 (embedding)
- Flash 地址: 0x400000
- 模型大小: ~1.2MB

### 3. 人脸对齐

基于 5 点关键点进行相似性变换：
1. 计算仿射变换矩阵
2. 将检测到的人脸对齐到标准模板
3. 输出 112×112 的对齐人脸图像

### 4. 人脸数据库 (ESP32-S3)

```cpp
// face_database.h
class FaceDatabase {
    static constexpr int MAX_FACES = 20;           // 最多存储 20 张人脸
    static constexpr int EMBEDDING_DIM = 128;      // 特征向量维度
    static constexpr float DEFAULT_THRESHOLD = 0.5f; // 默认匹配阈值

    // 核心方法
    bool AddFace(const std::string& name, const float* embedding);
    bool DeleteFace(const std::string& name);
    MatchResult Match(const float* embedding, float threshold);
    float CosineSimilarity(const float* a, const float* b);
};
```

存储结构：
- 使用 NVS (Non-Volatile Storage) 持久化存储
- 每个人脸记录包含: 名称 + 128 维特征向量
- 支持动态添加/删除

### 5. 人脸识别控制器 (ESP32-S3)

```cpp
// face_recognition.h
class FaceRecognitionController {
    static constexpr int VOTING_BUFFER_SIZE = 5;   // 投票缓冲大小
    static constexpr int VOTING_MIN_VOTES = 3;     // 最小投票数
    static constexpr float MIN_CONFIDENCE = 0.7f;  // 最小置信度

    // 投票机制确保识别结果稳定
    struct FaceVotingBuffer {
        std::string candidates[VOTING_BUFFER_SIZE];
        float scores[VOTING_BUFFER_SIZE];
        int index;
        int count;
    };
};
```

投票机制：
1. 连续采集多帧人脸数据
2. 统计每个候选人的出现次数
3. 只有当某人出现次数 >= 3 次且置信度 >= 0.7 时才确认识别

### 6. 摄像头接口 (ESP32-S3)

```cpp
// sscma_camera.h
class SscmaCamera {
    enum class Mode {
        MODE_OBJECT_DETECT,    // 目标检测模式
        MODE_FACE_RECOGNITION  // 人脸识别模式
    };

    enum class DetectionState {
        IDLE,       // 空闲
        VALIDATING, // 验证中 (投票)
        COOLDOWN    // 冷却期 (避免重复触发)
    };

    // 核心方法
    bool Capture();              // 捕获图像
    bool FaceRecognition();      // 执行人脸识别
    std::string Explain();       // 获取场景描述
};
```

## AT 命令接口

ESP32-S3 通过 UART 发送 AT 命令控制 Himax WE2：

### 标准 SSCMA 命令
| 命令 | 说明 |
|------|------|
| `AT+ID?` | 获取设备 ID |
| `AT+NAME?` | 获取设备名称 |
| `AT+MODEL` | 设置模型 |
| `AT+INVOKE=-1,1` | 开始连续推理 |

### 人脸模式命令
| 命令 | 说明 |
|------|------|
| `AT+FACE=1` | 启用人脸识别模式 |
| `AT+FACE=0` | 禁用人脸模式 (返回目标检测) |
| `AT+FACE?` | 查询当前模式状态 |

### 人脸模式输出格式

```json
{
  "type": 1,
  "name": "INVOKE",
  "code": 0,
  "data": {
    "count": 1,
    "image": "<base64 JPEG>",
    "faces": [{
      "bbox": [100, 50, 80, 100],
      "score": 95,
      "quality": 0.85,
      "landmarks": [[120, 70], [160, 70], [140, 95], [125, 120], [155, 120]],
      "embedding": [0.0123, -0.0456, ..., 0.0789]
    }],
    "width": 320,
    "height": 240
  }
}
```

## 工作流程

### 人脸注册

```
用户请求注册 "张三"
        │
        ▼
┌───────────────────┐
│ 切换到人脸识别模式 │  AT+FACE=1
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ 检测人脸 + 提取特征 │  AT+INVOKE
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ 质量评估          │  检查人脸大小、角度、清晰度
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ 保存到数据库      │  FaceDatabase::AddFace("张三", embedding)
└────────┬──────────┘
         │
         ▼
      完成注册
```

### 人脸识别

```
检测到人脸
    │
    ▼
┌───────────────────┐
│ 提取 128D 特征    │  Himax 返回 embedding
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ 数据库匹配        │  遍历所有已注册人脸，计算余弦相似度
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ 投票缓冲更新      │  记录本次匹配结果
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ 检查投票结果      │  某人出现 >= 3 次？
└────────┬──────────┘
         │
    ┌────┴────┐
    │ 是      │ 否
    ▼         ▼
确认识别   继续采集
    │
    ▼
触发语音: "检测到张三"
```

## 远程显示功能

RemoteDisplay 组件将设备 UI、音频和预览图同步到局域网内的树莓派显示服务。设备联网后常驻 HTTP 控制接口；用户在树莓派页面输入设备 IP 后，由树莓派请求设备反向建立 WebSocket：

```cpp
// remote_display.h
class RemoteDisplay {
    // 连接/断开树莓派显示服务
    bool Start(const std::string& server_url, int timeout_ms = 1000);
    void Stop();

    // 发送预览图像
    void SendPreviewImage(const uint8_t* jpeg_data, size_t size);

    // 缓存并同步 UI 状态
    void SendEmotion(const char* emotion);
    void SendStatus(const char* status);
    void SendChatMessage(const char* role, const char* content);
};
```

功能：
- HTTP 控制接口：`POST /api/start_cast`、`POST /api/stop_cast`、`GET /api/status`
- WebSocket 客户端用于实时通信，断线后按配置自动重试
- 预览图像传输 (JPEG)
- UI 状态同步 (表情、状态、聊天、主题、音量)
- Opus 音频转发
- 手动输入设备 IP，不依赖 UDP/mDNS 自动发现

音频实现约束：
- 上游每个 Opus 包代表 60 ms 音频，不能因 WebSocket 发送锁短暂繁忙就在协议回调里直接丢弃；应先进入有界队列，由独立任务发送。
- 浏览器端需预缓冲并按 `AudioContext` 连续时间轴排程。不要在每段 `onended` 后才以当前时间启动下一段，否则主线程调度会在分片之间引入可听间隙。
- 固件和浏览器缓冲都必须限制总时长；过载时宁可记录并丢弃旧数据，也不能让投屏延迟持续增长。

## 配置参数

### 人脸识别参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `VOTING_BUFFER_SIZE` | 5 | 投票缓冲区大小 |
| `VOTING_MIN_VOTES` | 3 | 确认识别所需最小票数 |
| `MIN_CONFIDENCE` | 0.7 | 最小置信度阈值 |
| `MATCH_THRESHOLD` | 0.5 | 余弦相似度匹配阈值 |
| `COOLDOWN_MS` | 3000 | 识别冷却时间 (毫秒) |

### 人脸数据库参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `MAX_FACES` | 20 | 最大存储人脸数 |
| `EMBEDDING_DIM` | 128 | 特征向量维度 |
| `NVS_NAMESPACE` | "face_db" | NVS 存储命名空间 |

### 模型内存使用

| 模型 | Arena 大小 |
|------|-----------|
| SCRFD | 220 KB |
| MobileFaceNet | 700 KB |
| **总计** | ~1 MB |

## 使用示例

### 1. 启动人脸识别模式

```cpp
// ESP32 端代码
camera_->SetMode(SscmaCamera::Mode::MODE_FACE_RECOGNITION);
```

### 2. 注册新人脸

```cpp
// 获取人脸特征
auto face_data = camera_->GetLastFaceData();
if (face_data.quality >= 0.8f) {
    face_db_->AddFace("用户名", face_data.embedding.data());
}
```

### 3. 识别人脸

```cpp
// 匹配数据库
auto result = face_db_->Match(face_data.embedding.data(), 0.5f);
if (result.matched) {
    printf("识别到: %s (相似度: %.2f)\n", result.name.c_str(), result.similarity);
}
```

## 语音交互集成

### MCP 工具

系统通过 MCP (Model Context Protocol) 工具实现语音控制：

| 工具名称 | 功能 | 触发语句示例 |
|---------|------|-------------|
| `self.face.list` | 列出人脸 | "有哪些人脸"、"你认识谁" |
| `self.face.identify` | 现场识别一次 | "你能认出我吗"、"我是谁" |
| `self.conversation.speaker` | 查当前说话人 | "你知道在跟谁说话吗" |

> 识别阈值不再有 MCP 工具，改为只读 NVS `face/threshold`（默认 40，见
> `sscma_camera.cc` 启动时的 `SetMatchThreshold`）。调阈值需要改 NVS 后重启。

> `self.face.identify` 与 `self.conversation.speaker` **仅供闲聊体验，不是鉴权路径**。
> 出入库等操作的身份校验由后端直连 `GET /api/face/current-speaker?fresh=1` 完成，
> 从不采信经 LLM 转述的身份，因此提示词注入最多让它在对话里说错名字，动不了库存。

### 人脸入库流程（服务端权威）

人脸库的 source of truth 在 `warehouse_system`，**设备端不提供语音注册/删除**。
增删都在服务端完成，再全量下发覆盖设备本地库：

```
服务端录入/删除 (warehouse_system face_subjects + face_enrollments)
         │
         ▼ build_face_library() 按 model_tag 过滤 + fp16 量化
    POST http://<device-ip>/api/face/batch-update
         │
         ▼
FaceDatabase::ReplaceAll()  ← 全量替换，不在 payload 里的条目即被删除
```

之所以不做语音注册/删除：`ReplaceAll` 是全量替换语义，设备端单独增删的条目会在
下一次同步时被服务端原样覆盖回去——用户会看到"删了又活了"。增删只走服务端一条路。

### 识别成功通知

识别成功后，系统会触发语音交互：

```cpp
// 触发唤醒词
WakeWordInvoke("<f>张三</f>");
```

这会让设备说出识别结果，实现自然的人机交互。

> **唤醒词长度预算**：官方云对 `listen/state=detect` 的 `text` 有体量上限，超限会回
> `{"type":"alert","status":"ERROR","message":"Detect is only for wake words, do not send long texts."}`，
> 设备端直接弹成 ERROR 屏。实测安全线为 **UTF-8 ≤ 25 字节**（按字节，不是按字符）。
> 因此标签压到 `<f></f>`（人脸）/ `<d></d>`（物体），各占 7 字节，正文余 18 字节
> ≈ 6 个汉字，超出部分由 `TruncateUtf8()` 按字符边界截断。
> 该上限是闭源云的黑盒启发式，改动前请重跑
> `warehouse_system/e2e_voice_mcp/test_detect_limit_official.py` 复测。

### 有效的语音命令示例

**现场识别**：
- "你能认出我吗"
- "看看我是谁"
- "你知道在跟谁说话吗"

**查询人脸**：
- "你认识谁"
- "人脸库里有哪些人"
- "列出所有已录入的人脸"

## 已知限制

1. **最大人脸数**: 当前限制为 20 张，受 NVS 存储空间限制
2. **光线要求**: 需要充足光线才能保证识别准确率
3. **角度限制**: 正脸效果最佳，侧脸可能影响识别率
4. **处理速度**: 完整识别流程约需 200-300ms

## 相关文件

```
xiaozhi-esp32/main/boards/sensecap-watcher/
├── face_recognition.cc/h     # 人脸识别控制器
├── face_database.cc/h        # 人脸数据库
├── sscma_camera.cc/h         # 摄像头接口
├── remote_display.cc/h       # 远程显示
└── FLASH_GUIDE.md            # 烧录指南

grove_vision_2/sscma-example-we2/EPII_CM55M_APP_S/
└── app/scenario_app/sscma_face/  # Himax 人脸识别应用
    ├── sscma_face_app.cc         # 主应用
    ├── face_detector.cc/h        # SCRFD 封装
    ├── face_embedder.cc/h        # MobileFaceNet 封装
    └── README.md                 # Himax 端说明
```
