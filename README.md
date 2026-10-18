# LTS Respooler 控制板固件（ESP32-C3）

合宙 CORE-ESP32-C3 核心板的 LTS Respooler 控制固件：步进电机收线、TMC2209 驱动、
Web Bluetooth（网页直连）控制、SSD1306 OLED 状态屏、旋转编码器调速。

- 原工程基于 ESP32-S3，已完整移植到 ESP32-C3（引脚、BLE 广播包、内存限制都改过）
- 手机/电脑用 Chrome 或 Edge 打开控制页，通过 Web Bluetooth 连接（iOS Safari 不支持）

---

## 硬件引脚分配（合宙 CORE-ESP32-C3）

| GPIO | 功能 | 备注 |
|---|---|---|
| 2 | 断丝检测 | 上电勿外部下拉（strap） |
| 4 | 步进使能 EN | 低有效 |
| 5 | 连接状态灯 | PWM |
| 6 / 7 | STEP / DIR | |
| 8 | 舵机 | |
| 9 | NeoPixel | BOOT 键同脚，注意 strap |
| 10 | 按钮 | |
| 12 / 13 | TMC2209 UART RX / TX | DIO 模式空闲（= 板载 D4/D5） |
| 19 | 风扇 | 原 USB D+，本板串口走 CH343，故可用 |
| 3 / 1 | **OLED SDA / SCL** | ⚠️ 实物是 SDA→GPIO3、SCL→GPIO1，**不要改成 1/3** |
| 0 / 18 | **编码器 A(CLK) / B(DT)** | v1.3.0 新增，空闲脚 |
| 20 / 21 | UART0（CH343 调试口） | 不要占用 |

**绝对不能用**：GPIO11（VDD_SPI）、GPIO14（Flash CS）、GPIO15（Flash CLK）、
GPIO16/17（Flash D0/D1）——一配置就无限重启。

---

## 旋转编码器调速（v1.3.0）

### 接线

| 编码器 | ESP32-C3 |
|---|---|
| CLK (A) | GPIO0 |
| DT (B) | GPIO18 |
| + / VCC | 3.3V |
| GND | GND |
| SW（按键） | 不接（`ENC_SW_PIN -1`；板载按钮在 GPIO10） |

也可用 VCC=GND 接法（编码器无源），库参数 `-1` 表示不占用引脚供电。

### 行为

| 操作 | 结果 |
|---|---|
| 慢慢转 | 每格 ±1%，SPD 限制在 50–100 |
| 快速猛转 | 库加速度生效，单次最多跳 ±10%（`ENC_MAX_JUMP`） |
| 转完 150 ms | 自动 `sendStatus()` 推给手机，App 滑条同步 |
| 转完 2 秒内 | 手机发来的 `SPD` 命令**被忽略**（编码器优先），串口打印 `[BLE] SPD ignored (encoder has priority)` |
| 手停 800 ms | 才写一次 NVS（保护 flash，断电不丢） |
| 电机运转中 | 随时可调，立刻生效 |

速度换算与 BLE 的 `SPD` 命令完全一致：`间隔(µs) = 170 − (SPD−50) × 77 / 50`。

### 可调参数（`src/main.cpp` 顶部）

```cpp
#define ENC_A_PIN 0
#define ENC_B_PIN 18
#define ENC_SW_PIN -1
#define ENC_STEPS 4            // 转一格加 2% 时改成 2
#define ENC_ACCEL 90           // 0=关闭加速；越大猛转跳得越狠
#define ENC_MAX_JUMP 10        // 单次事件最大 ±10%
#define ENC_BLE_LOCKOUT_MS 2000
#define ENC_NVS_SETTLE_MS 800
```

方向反了 → 交换 `ENC_A_PIN` / `ENC_B_PIN`。

---

## WiFi 局域网 OTA 烧录（v1.4.0）

板子自带刷机网页，不用插 USB、不用装驱动、不需要电脑端服务。

### 怎么进

| 方式 | 操作 |
|---|---|
| **自动**（推荐） | 上电/烧录后 **6 秒**没连上路由器 → 自动开热点 `LTS-CB-OTA2` |
| 串口 | `{"CMD":"OTA_AP"}` 开热点；`{"CMD":"OTA_WEB"}` 用已连的局域网；`{"CMD":"OTA_AP_OFF"}` 退出 |
| 上电按住按钮 | 按住板载按钮 >1.2 秒再上电，直接进 OTA 模式 |

### 怎么刷

1. 电脑/手机连 `LTS-CB-OTA2`，密码 `12345678`
2. 浏览器打开 **`192.168.4.1`**
   - 手机会弹「Sign in to LTS-CB-OTA2」窗口（顶部地址是 `www.googleapis.cn`）——
     那是系统的 Captive Portal 窗口，**不能选文件**，点右上角 `⋮` → 「在 Chrome 中打开」
3. 选 `.pio/build/esp32c3/firmware.bin`（约 1.2 MB）
   - 手机刷机需先把 bin 传到手机（微信文件传输助手 → 保存到手机）
4. 屏幕显示 `OTA UPLOAD xx%` → `FLASHED` → 自动重启

### 固件校验（拒绝错文件，绝不写坏 flash）

| 关卡 | 检查 |
|---|---|
| 网页端 | 文件 > 1.25 MB 直接拦 |
| 板子 1 | 首字节必须 `0xE9`（ESP32 镜像魔数） |
| 板子 2 | 偏移 `0x20` 必须是 app descriptor `0xABCD5432`（排除 bootloader / merged.bin） |
| 板子 3 | 长度 ≤ 1,310,720 字节（app 分区大小） |
| 最后 | `Update.end(true)` 做 ESP 自带完整性校验 |

不通过时屏幕 `FAILED`，串口打印具体原因，flash 一个字节都不会写。

### 细节

- 热点名 `LTS-CB-OTA2`（WPA2）、IP `192.168.4.1`、信道 6（改过信道要同步改名，否则手机会因缓存拒绝重连）
- 已连路由器时不会开热点，屏幕左下角显示 `WIFI <IP>`
- 热点模式会临时释放 BLE（C3 单射频，否则 AP 扫不到），30 分钟无操作自动关闭并重启
- 屏幕左下角 WiFi 状态：`WIFI --` / `WIFI Connected` / `WIFI <IP>`

---

## 构建与烧录

**只有一个构建环境**，VSCode 里直接点 Upload 即可（`platformio.ini` 里已设
`default_envs = esp32c3`）。命令行：

```bash
pio run -e esp32c3 -t upload        # 编译 + 烧录
pio device monitor -b 115200        # 串口日志
```

> 工程里曾有一个 `[env:oled_diag]` 诊断固件（屏幕画棋盘格 + `I2C OK!`），
> 因容易被误烧、看起来像"主固件坏了"，已整段注释。需要测屏时去掉 `platformio.ini`
> 末尾的 `;` 恢复。

### 开机串口日志（正常）

```
[BOOT] 1 setup start
[BOOT] 2 gpio done
[BOOT] 2c rotary encoder ok        ← 编码器初始化
[OLED] scan 0x3C ACK               ← 屏在线（自动扫地址，0x3C/0x3D 自适应）
[BOOT] 2b oled ok (pre-BLE draw)
[BOOT] 6 TMC driver configured
[BOOT] NimBLEDevice::init ok
[BLE] name=LTS CB advStarted=1
[LOOP] alive state=I heap=17xxxx   ← 主循环心跳，每 5 秒
```

---

## 变更记录

### v1.4.0 — 2026-10-19
- **新增 WiFi 局域网 OTA 烧录**：板内刷机网页（SoftAP / 局域网双模式）、Captive Portal
  自动弹页、上传进度上屏、**三层固件校验**（0xE9 + app descriptor + 分区大小）
- 上电 6 秒没联网自动开热点；热点模式释放 BLE 解决单射频共存问题
- OLED 左下角新增 WiFi 状态（`WIFI --` / `WIFI Connected` / `WIFI <IP>`）
- 新增 `tools/ota_server.py`：电脑端固件分发页（可选用，拖拽上传 + SHA256 展示）

### v1.3.0 — 2026-10-19
- **新增旋转编码器调速**（`AiEsp32RotaryEncoder` 1.7）：慢转 ±1%、快转加速放大，
  编码器优先级高于蓝牙（2 秒锁定），改动实时回传手机，NVS 写入做 800 ms 防抖
- `platformio.ini`：新增库依赖；设 `default_envs = esp32c3`；注释掉 `[env:oled_diag]`
- 速度换算统一收口到 `applySpeedPercent()`，BLE `SPD` 与编码器共用同一入口

### v1.2.1 — 2026-10-18
- **修复 OLED 黑屏**：SSD1306 的 SDA/SCL 实物接反（固件按 1/3、实物是 3/1），
  已改为 `OLED_SDA 3` / `OLED_SCL 1`；启动时改为 I2C 全地址扫描、地址自适应
- 移除调试探针（每秒 re-begin、t+8s 全总线扫描等）
- 详细排障过程见 [ESP32-C3移植与BLE调试经验总结.md](ESP32-C3移植与BLE调试经验总结.md)

### v1.2.0 — 2026-10-13
- ESP32-C3 移植完成（引脚重排、避开 Flash 脚）
- BLE 广播包重排（31 字节上限）：名字 + 服务 UUID 放广播包，厂商数据放扫描响应
- 删除 C3 上会导致协议栈半初始化的 `setMTU(512)`
