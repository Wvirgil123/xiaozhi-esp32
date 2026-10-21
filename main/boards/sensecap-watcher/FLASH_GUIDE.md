# SenseCAP Watcher 固件烧录指南

SenseCAP Watcher 是一款双芯片设备，包含 **ESP32-S3** 和 **Himax WE2 (WiseEye2)** 两个处理器，需要分别烧录各自的固件。

## 设备架构

```
┌─────────────────────────────────────────────────────┐
│                 SenseCAP Watcher                     │
│                                                      │
│  ┌──────────────┐          ┌──────────────────────┐ │
│  │   ESP32-S3   │◄────────►│   Himax WE2 (AI)     │ │
│  │              │  UART    │                      │ │
│  │  - 主控制器  │          │  - AI 视觉处理       │ │
│  │  - WiFi/BLE  │          │  - 摄像头 (OV5647)   │ │
│  │  - 音频      │          │  - 人脸识别          │ │
│  │  - 显示屏    │          │  - 目标检测          │ │
│  └──────────────┘          └──────────────────────┘ │
│         │                            │               │
│         └────────────────────────────┘               │
│                      USB                             │
└─────────────────────────────────────────────────────┘
```

## 串口识别

连接设备后，会出现 **两组串口**（共 4 个端口）：

### macOS
```bash
ls /dev/cu.usbmodem* /dev/cu.wchusbserial*
```

典型输出：
```
/dev/cu.usbmodem5AF91659651      # Himax WE2 串口
/dev/cu.usbmodem5AF91659653      # Himax WE2 串口 (备用)
/dev/cu.wchusbserial5AF91659651  # ESP32-S3 串口 (烧录用)
/dev/cu.wchusbserial5AF91659653  # ESP32-S3 串口
```

### 识别规则
| 端口类型 | 芯片 | 用途 |
|---------|------|------|
| `wchusbserial*` | ESP32-S3 | 固件烧录、日志输出 |
| `usbmodem*` | Himax WE2 | 固件烧录、AI 数据通信 |

### Linux
```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

### Windows
在设备管理器中查看 COM 端口，通常会显示两个串口。

---

## ESP32-S3 固件烧录

### 编译

```bash
cd /path/to/xiaozhi-esp32

# 设置 Python 环境 (macOS)
export IDF_PYTHON_ENV_PATH=/Users/harvest/.espressif/python_env/idf5.5_py3.14_env
source ~/esp/esp-idf/export.sh

# 编译
idf.py build
```

### 烧录

```bash
# macOS - 使用 wchusbserial 端口
idf.py -p /dev/cu.wchusbserial5AF91659653 flash

# Linux
idf.py -p /dev/ttyUSB0 flash

# Windows
idf.py -p COM3 flash
```

### 监控日志

```bash
idf.py -p /dev/cu.wchusbserial5AF91659653 monitor
```

### 备份工厂信息（重要！）

在首次烧录前，备份 SenseCraft 服务器凭证：

```bash
esptool.py --chip esp32s3 --baud 2000000 --before default_reset --after hard_reset --no-stub read_flash 0x9000 204800 nvsfactory.bin
```

---

## Himax WE2 固件烧录

### 编译 (macOS)

```bash
cd /path/to/sscma-example-we2/EPII_CM55M_APP_S

# 重要：macOS 必须使用 gmake (GNU Make)，不能使用 BSD make
gmake clean
gmake APP_TYPE=sscma_face TARGET=SENSECAP_WATCHER
```

输出文件：`obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf`

### 生成固件镜像

```bash
cd ../we2_image_gen_local

# 复制 ELF 文件
cp ../EPII_CM55M_APP_S/obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf input_case1_secboot/

# 生成镜像 (macOS)
./we2_local_image_gen_macOS_arm64 project_case1_blp_wlcsp.json

# Linux
./we2_local_image_gen project_case1_blp_wlcsp.json
```

输出文件：`output_case1_sec_wlcsp/output.img`

### 烧录

> **重要**: SenseCAP Watcher 的 ESP32 固件会监控 Himax 状态，检测到异常时会复位 Himax。
> 这会导致烧录过程中断！必须在烧录 Himax 时**保持 ESP32 复位**。

#### 推荐方法：使用安全烧录脚本

```bash
cd /path/to/sscma-example-we2

# 使用安全烧录脚本（自动保持 ESP32 复位）
python3 flash_himax_safe.py
```

该脚本会：
1. 打开 ESP32 串口并保持复位状态 (DTR=False)
2. 烧录 Himax 固件（此时 ESP32 不会干扰）
3. 烧录完成后释放 ESP32 复位

#### 手动方法（需要两个终端）

**终端 1 - 保持 ESP32 复位：**
```python
import serial
ser = serial.Serial('/dev/cu.wchusbserial5AF91659653', 115200)
ser.dtr = False  # 保持 ESP32 复位
# 保持此终端开启直到烧录完成
```

**终端 2 - 烧录 Himax：**
```bash
sscma.cli flasher -p /dev/cu.usbmodem5AF91659651 -f output.img
# 按 Himax Reset 按钮
```

**烧录完成后**，关闭终端 1 释放 ESP32。

#### 普通 Grove Vision AI V2 烧录

如果是独立的 Grove Vision AI V2（不是 SenseCAP Watcher），可以直接烧录：

```bash
# 安装
pip install python-sscma

# 烧录（按 Reset 后执行）
sscma.cli flasher -p /dev/cu.usbmodem* -b 921600 -f output.img
```

#### 使用 xmodem 脚本

```bash
cd xmodem

# 安装依赖
pip install -r requirements.txt

# 烧录（需要先保持 ESP32 复位！）
python3 xmodem_send.py \
    --port=/dev/cu.usbmodem5AF91659651 \
    --baudrate=921600 \
    --protocol=xmodem \
    --file=../we2_image_gen_local/output_case1_sec_wlcsp/output.img
```

### 进入下载模式

Himax WE2 需要在启动时进入下载模式：

1. **运行烧录命令**（命令会等待设备）
2. **按下设备的 Reset 按钮**
3. 设备会在 100ms 内自动进入下载模式

设备启动时会显示：
```
1st BL Modem Build DATE=Mar 27 2024, Version: 2.12
Please input any key to enter X-Modem mode in 100 ms
```

### 验证固件版本

```bash
python3 -c "
import serial
import time
ser = serial.Serial('/dev/cu.usbmodem5AF91659651', 921600, timeout=3)
time.sleep(2)
data = ser.read(2000)
print(data.decode('utf-8', errors='ignore'))
ser.close()
"
```

查看 `Build date:` 确认固件版本。

---

## 官方固件下载

如需恢复官方固件：

- **ESP32-S3**: https://github.com/Seeed-Studio/SenseCAP-Watcher-Firmware
- **Himax WE2**: https://github.com/Seeed-Studio/OSHW-SenseCAP-Watcher/tree/main/Firmware/Himax

---

## 常见问题

### macOS 上编译 Himax 固件报错 "cannot find @objs.in"

原因：使用了 BSD make 而不是 GNU make

解决：
```bash
# 安装 GNU make
brew install make

# 使用 gmake 代替 make
gmake clean && gmake APP_TYPE=sscma_face TARGET=SENSECAP_WATCHER
```

### Himax 烧录时设备不断重启（SenseCAP Watcher 特有问题）

**最可能的原因：ESP32 固件在复位 Himax**

SenseCAP Watcher 的 ESP32 固件会监控 Himax 芯片的状态。当 Himax 进入下载模式时，ESP32 可能会检测到"异常"并尝试复位 Himax，导致烧录中断。

**解决方案：烧录时保持 ESP32 复位**

```bash
# 使用安全烧录脚本
cd /path/to/sscma-example-we2
python3 flash_himax_safe.py
```

或者手动保持 ESP32 复位：
```python
import serial
ser = serial.Serial('/dev/cu.wchusbserial5AF91659653', 115200)
ser.dtr = False  # EN 引脚拉低，保持复位
# 在另一个终端烧录 Himax
```

**其他可能原因：**
1. USB 连接不稳定 - 尝试直接连接 Mac，不使用 Hub
2. USB 线缆质量问题 - 更换线缆
3. 平台兼容性问题 - 尝试在 Windows/Linux 上烧录

> **注意**: 这个问题只影响 SenseCAP Watcher，不影响独立的 Grove Vision AI V2。

### ESP32 烧录时找不到设备

确保使用 `wchusbserial` 端口，而不是 `usbmodem` 端口。

---

## 开发相关项目路径

```
xiaozhi-esp32/                           # ESP32-S3 主项目
└── main/boards/sensecap-watcher/        # Watcher 板级代码
    ├── face_recognition.cc/h            # 人脸识别
    ├── face_database.cc/h               # 人脸数据库
    ├── sscma_camera.cc/h                # 摄像头接口
    └── remote_display.cc/h              # 远程显示

grove_vision_2/sscma-example-we2/        # Himax WE2 项目
└── EPII_CM55M_APP_S/
    ├── app/scenario_app/sscma_face/     # 人脸识别应用
    └── library/sscma_micro/             # SSCMA 库
```
