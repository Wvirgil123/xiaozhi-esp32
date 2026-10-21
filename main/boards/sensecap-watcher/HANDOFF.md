# SenseCAP Watcher 设置面板重构与视觉模式交互 Handoff

> **对话追溯链接 (Conversation Link)**: [Watcher UI 重构与视觉模式交互对话 (Conversation Link)](conversation://1022cd92-21a3-475a-9154-0e562f4950b5)  
> **Conversation ID**: `1022cd92-21a3-475a-9154-0e562f4950b5`  
> **目标硬件**: SenseCAP Watcher (1.4-inch 412x412 圆形 AMOLED 屏幕, ESP32-S3 + Himax WE2 视觉协处理器 + 物理旋转旋钮)  
> **代码仓库**: `xiaozhi-esp32` (`main/boards/sensecap-watcher/`)  
> **日期**: 2026-10-16  

---

## 1. 改动背景与原始痛点 (Context & Pain Points)

在原有 SenseCAP Watcher 交互与设置实现中，用户在 1.4 英寸高像素密度（326 PPI）圆形屏幕上遇到了多处严重的体验问题：

1. **文字过小、字库残缺、显字不全**：
   - 原始界面文本字号偏小，用户无法在臂长距离内看清；
   - 缺少定制中文字库点阵，设置项名称与说明出现大量缺字、断裂或乱码（"显字显得不全"）；
   - 尝试双语上下混排时，原本有限的圆形屏幕被严重挤占，文字截断。
2. **手势冲突与抽屉交互缺失**：
   - 滑动切换页面时缺乏“中心锚定”机制：拉出面板或回退时无法平滑回到居中位置，缺少向内拉起再顺滑回去的物理阻尼；
   - **内部滚动与外部手势严重冲突**：表单内部纵向滚动时，容易误触发外层页面切换，导致表单整体漂移；
   - 界面底部出现了不必要的横向微型滚动条，与水平滑杆操作冲突；
   - 关闭面板必须瞄准点击极小的“Close”按钮，无法通过顺手向外滑动甩动（Flick/Swipe to Dismiss）快速退出。
3. **模式语义晦涩、认知负担重**：
   - 视觉模式原本命名模糊（如用户无法理解“关闭”和“静音模式”到底有何本质区别）；
   - 参数界面充斥“phase 切换机制及延迟检测”等底层工程用语，非技术用户完全无法理解其作用；
   - Wi-Fi 状态独占多行，布局稀疏低效。

---

## 2. 架构设计与核心改造 (Architecture & Key Solutions)

### 2.1 严格的板级隔离原则 (Strict Board-Level Isolation)
- **绝对零侵入**：所有新增逻辑、UI 状态机、交互手势处理、字库点阵均严格封装在 `main/boards/sensecap-watcher/` 目录下；
- 严禁修改通用应用层 `main/application.*` 或抽象基类 `main/boards/common/board.h`，确保多板卡固件主干代码不受污染。

---

### 2.2 4 个视觉模式语义重构 (Vision Modes Semantic Redesign)
对原本模糊的开关组合进行了系统级语义重构，定义为 4 个贴合用户生活场景的卡片，并附带通俗易懂的 16px 副标题解释：

| 模式名称 | 英文名称 | 底部副标题解释 | 底层状态组合 (`camera_`) | 行为表现 |
|---|---|---|---|---|
| **不感知** | Off | 关闭视觉，不主动唤醒 | `inference_en = 0, face_en = 0` | 摄像头关闭视觉推理，低功耗待机 |
| **人形感知** | Human Motion | 有人即唤醒，不认人脸 | `inference_en = 1, face_en = 0` | 仅做人体活动检测，有人即亮屏唤醒，不识别人脸 |
| **智能问候** | Smart Greeting | 熟人叫名字，主动问候 | `inference_en = 1, face_en = 1, familiar_dnd = 0` | 人脸识别已录入熟人并主动语音打招呼，陌生人唤醒提示 |
| **哨兵模式** | Sentry Mode | 熟人免打扰，陌生人警戒 | `inference_en = 1, face_en = 1, familiar_dnd = 1` | 熟人经过保持静音不打扰；陌生人进入触发警戒唤醒 |

---

### 2.3 Watcher 定制高对比度中文字库点阵 (Custom Font Subsetting)
- 提取了包含 95 个高频 UI 汉字点阵子集，生成独立的点阵源文件：
  - `watcher_font.h` / `watcher_font.c`
  - `watcher_chinese_font_20`：20px 粗体点阵，用于卡片主标题、模式名；
  - `watcher_chinese_font_16`：16px 常规点阵，用于辅助说明与副标题；
  - **自动 ASCII Fallback**：两组字体均挂载 `.fallback = &font_noto_sans_basic_XX_4`，无缝支持英文字母、数字和符号；
  - 彻底解决了中文在 1.4 寸圆屏上缺失、变形、截断的问题。

---

### 2.4 大触控卡片与 SenseCraft 现代暗色美学 (Tactile Cards & Styling)
- **76px 高度大触控卡片**：彻底摒弃紧凑文字列表，改为上下双行（20px 标题 + 16px 副标题）、左侧 30px 状态图标、右侧 30px Checkmark 的圆角卡片；
- **高对比度 AMOLED 配色 (`WatcherUiPalette`)**：
  - 选中高亮状态具有明亮主题色边框（Cyan/Green），未选中状态保持暗色半透明质感；
  - 选中项文本自动提亮至高对比度，副标题同步匹配高可见度；
- **紧凑型 Wi-Fi 栏目**：
  - 改变原本独占多行的做法，改为同一行展示：Wi-Fi 图标 + SSID 文本 + 右侧紧凑切换配网按钮；
- **语言策略**：
  - 直接遵循编译时设定的系统语言（`WatcherUiLanguage::Zh` / `WatcherUiLanguage::En`），单语呈现，彻底避免双语同时渲染导致的界面挤压。

---

### 2.5 手势引擎与抽屉回弹交互 (Gesture Engine & Drawer Dismiss)
- **居中阻尼拉出/拉回**：
  - 采用半透明抽屉面板交互，从屏幕边缘下拉或滑入，释放时带有弹性回弹到正中；
  - 再次向外滑动或反向划过阈值时，自动触发向外收回动画，顺滑回到主表盘中心；
- **滚动条与手势解耦**：
  - 强制隐藏所有不必要的水平滚动条，防止与滑杆或左右手势打架；
  - 区分列表内部上下滑动与外部关闭手势，解决了“一往下滑动整个表单跟着跑”的穿透问题；
- **物理旋钮协同**：
  - 顺时针/逆时针旋转旋钮平滑增减亮度与阈值，按下旋钮执行确认。

---

## 3. 代码文件清单与索引 (File Map)

| 文件路径 | 变更类型 | 说明 |
|---|---|---|
| `main/boards/sensecap-watcher/watcher_settings_ui.h` | 新增/修改 | 设置面板控制器声明、`WatcherSettingsPage` 枚举、`WatcherUiPalette` 配色结构、手势与旋钮回调 |
| `main/boards/sensecap-watcher/watcher_settings_ui.cc` | 新增/修改 | 设置面板全套 UI 构建（QuickSettings, VisionMode, Parameters）、大触控卡片、手势分发、中英文文案刷新 |
| `main/boards/sensecap-watcher/watcher_font.h` | 新增 | 20px / 16px 中文字体外部声明 |
| `main/boards/sensecap-watcher/watcher_font.c` | 新增 | 95 个定制中文点阵数据与 LVGL 8 字体描述符结构 |
| `main/boards/sensecap-watcher/sensecap_watcher.cc` | 修改 | 板级初始化挂载设置面板、手势事件捕获与旋钮物理事件派发 |
| `main/CMakeLists.txt` | 修改 | 将 `watcher_font.c` 加入编译源文件列表 |

---

## 4. 编译、烧录与验证指南 (Build & Flash Guide)

### 4.1 环境准备
```bash
source /Users/harvest/esp/esp-idf/export.sh
```

### 4.2 编译与增量构建
```bash
# 增量编译 app 镜像
idf.py -C /Users/harvest/project/xiaozhi-esp32 build
```
> **产物**：`/Users/harvest/project/xiaozhi-esp32/build/xiaozhi.bin`（约 3.4MB，分区剩余空间约 15% / 0x9c4d0 字节）。

### 4.3 快速烧录 (App-Flash)
SenseCAP Watcher 通常连接在 `/dev/cu.wchusbserial*` 串口（通道 3 为 ESP32-S3 串口）：
```bash
idf.py -C /Users/harvest/project/xiaozhi-esp32 -p /dev/cu.wchusbserial5AF91643623 -b 921600 app-flash
```

### 4.4 交互验证重点 (Checklist)
1. **下拉/左滑呼出**：在主表盘下滑出快捷设置，向左滑呼出模式设定，确认面板平滑居中；
2. **模式卡片显示**：进入模式设定，确认 4 张卡片（不感知、人形感知、智能问候、哨兵模式）汉字清晰饱满，无乱码，无缺字；
3. **触控与高亮**：点击任意模式，卡片左侧图标、右侧 Checkmark 与文字变亮，副标题同步变为高亮色；
4. **单手下滑收起**：无需费力寻找关闭按键，从顶部直接向外滑动，面板应平滑退回主屏；
5. **旋钮调节**：旋转外圈物理旋钮，确认亮度/参数滑块能够顺畅跟手响应。
