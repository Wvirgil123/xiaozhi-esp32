# Build Instructions

## One-click Build

```bash
python scripts/build.py sensecap-watcher \
  --language en-US \
  --wake-word wn9_jarvis_tts
```

## Manual Configuration and Build

```bash
idf.py set-target esp32s3
```

**Configuration**

```bash
idf.py menuconfig
```

Select the board:

```
Xiaozhi Assistant -> Board Type -> SenseCAP Watcher
```

There are some additional configuration options for the watcher. Please select them in menuconfig:

```
CONFIG_BOARD_TYPE_SEEED_STUDIO_SENSECAP_WATCHER=y
CONFIG_ESPTOOLPY_FLASHSIZE_32MB=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions/v2/32m.csv"
CONFIG_BOOTLOADER_CACHE_32BIT_ADDR_QUAD_FLASH=y
CONFIG_ESPTOOLPY_FLASH_MODE_AUTO_DETECT=n
CONFIG_IDF_EXPERIMENTAL_FEATURES=y
CONFIG_LANGUAGE_EN_US=y
CONFIG_SR_WN_WN9_JARVIS_TTS=y
CONFIG_SEND_WAKE_WORD_DATA=y
```

## Configure remote wake locally

HTTP remote wake is disabled by default on the Watcher. Connect the serial console and run these commands locally. The command echo and terminal logs may contain the token you enter; configure it physically and disable recording:

```text
remote_wake status
remote_wake enable <16-128 character safe token>
remote_wake enable                 # enable with the saved token
remote_wake disable                # disable but keep the token
remote_wake clear                  # clear the token and disable
```

`status` only reports `enabled` and `token_configured`; it never prints the token. Tokens may contain only ASCII letters, digits, `.`, `_`, `~`, and `-`. Enabling requires either an existing valid token or an explicitly supplied new token.

## Build and Flash

The current repository builds the Watcher with ESP-IDF 5.5.4. With ESP-IDF 6.0.2, the upstream `sscma_client` still declares the legacy component name `json`, so CMake stops during dependency resolution. Do not bypass this compatibility issue by modifying managed components.

```bash
idf.py -DBOARD_NAME=sensecap-watcher build flash
```

Note: If your device was previously shipped with the SenseCAP firmware (not the Xiaozhi version), please be very careful with the flash partition addresses to avoid accidentally erasing the device information (such as EUI) of the SenseCAP Watcher. Otherwise, even if you restore the SenseCAP firmware, the device may not be able to connect to the SenseCraft server correctly! Therefore, before flashing the firmware, be sure to record the necessary device information to ensure you have a way to recover it!

You can use the following command to back up the factory information:

```bash
# Firstly backup the factory information partition which contains the credentials for connecting the SenseCraft server
esptool.py --chip esp32s3 --baud 2000000 --before default_reset --after hard_reset --no-stub read_flash 0x9000 204800 nvsfactory.bin
```
