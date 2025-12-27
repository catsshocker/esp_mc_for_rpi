# ESP32 Motor Controller (簡短說明)

簡介
 - ESP32-S2 馬達控制器，透過 USB(UART) 接收指令並回傳編碼器數值。
主要指令（以換行結束）
 - `sta`：啟動（回傳 `sta`，開始回傳編碼器數值）
 - `end`：停止控制並停馬
 - `res`：重置編碼器
 - `sms <speed1>,<speed2>`：直接設定馬達速度（範圍 -100..100），範例 `sms 50,-30`

序列輸出
 - 編碼器：`<count1>,<count2>\n`（由 `encoder_read_task` 每 10 ms 輸出）
 - 速度回饋（選用）：`Motor1 Speed: %d, Motor2 Speed: %d\n`

快速使用（Python）
1. 安裝： `pip install pyserial`
2. 範例：
```python
from serialControl import SerialControl
c = SerialControl('COM5')
try:
	c.send_motor_speeds(50, -30)   # 會以 sms 50,-30\n 發送
finally:
	c.close()
```

硬體與程式備註
 - 預設腳位：Motor1(pwm=4,A=17,B=16)、Motor2(pwm=21,A=18,B=19)、enablePin=2，編碼器使用 GPIO13/14 與 GPIO22/23。
 - `Motor::setSpeed` 接受 -100..100，內部線性映射到 PWM magnitude 0..255；方向由兩個方向腳位控制。
 - 範例含 `encoder_read_task`、`set_motor_speed_task`；`motor_contorl_task`（PID）與 `feedback_motorSpeed` 為選用，可在 `setup()` 取消註解啟用。

建議
 - 在 `setup()` 明確呼叫 `pinMode(enablePin, OUTPUT);`。
 - 若需穩健的 PID 行為，可調整 `pidController` 的 anti-windup 策略或限制積分範圍。

如需，我可再加入「上傳到 ESP32 的快速步驟」或更完整的測試腳本。
# esp32 motor controller with USB
esp32 motor controller connect with usb port by UART  
micro controller -> ESP32-S2  
motor controller -> FB6612FNG  
encoder -> esp32 PCNT 

## commend format

`"[commend]+\space+[arg1,arg2,...]+\n"`
## commend table
|commend|      function         |args   |
|----   |----                   |---    |
|`sta`    |   start controller    |NaN       |
|`end`    |   end controller      |NaN       |
|`res`    |   reset encoder       |NaN       |
|`sms`    |   set motor speed     |`speed1`, `speed2`|


## python libary

## Overview
此專案在 ESP32-S2 上實作簡單的馬達控制器，透過 USB(UART) 接收主機命令並回傳編碼器讀數。主機（PC）可用 Python 程式控制馬達速度並讀取回傳資料。

## Command format

`[command] [arg1,arg2,...]\n`

## Command table
| command | function | args |
|----|----|---|
| `sta` | start controller | NaN |
| `end` | end controller | NaN |
| `res` | reset encoder | NaN |
| `sms` | set motor speed | `speed1`, `speed2` |

說明：`sms` 指令的範例為 `sms 50,-30\n`，代表將馬達1 設為 50，馬達2 設為 -30（負值代表反轉）。

## Python library
專案內提供 `serialControl.py` 範例類別，可用來與 ESP 串接：

- 會在建立連線時發送 `sta\n` 並等待最多 2 秒回覆 `sta`，回覆成功後會啟動接收執行緒。
- 傳送馬達速度時會使用 `sms <int>,<int>\n` 格式（例如 `sms 50,-30\n`）。

`serialControl.py` 行為補充：
- 建構子會發送 `sta\n`，等待最多 2 秒收到 `sta` 回應；若成功則把 `is_enabled` 設為 `True` 並啟動背景接收執行緒，否則會印出警告但仍啟動接收執行緒。
- `send_motor_speeds(speed1, speed2)` 會以 `sms <int>,<int>\n` 發送（由程式轉為整數並檢查序列埠是否開啟）。
- `close()` 會將 `is_enabled=False`，嘗試 join 背景執行緒並關閉序列埠。

### 安裝相依
```bash
pip install pyserial
```

### 使用範例
```python
from serialControl import SerialControl

c = SerialControl('COM5')
try:
	# 設定馬達速度（整數）
	c.send_motor_speeds(50, -30)
finally:
	c.close()
```

### 注意事項
- 若裝置未在 2 秒內回應 `sta`，`SerialControl` 會印出警告但仍會啟動接收執行緒。
- `send_motor_speeds` 會將速度轉為整數並以 `sms ` 前綴發送，請勿直接發送僅 `50,30`（需加 `sms `）。
- 若需非阻塞或高效能資料接收，可改寫 `_get_serial_line` 為非阻塞讀取或使用 callback 機制。

如果需要，我可以把 README 裡的範例改為更完整的測試腳本或加入如何在 Windows/Unix 上尋找 COM port 的說明。

## Arduino (ESP32) 端說明
下面說明對應本專案 `esp_rpi_mc.ino` 的硬體連接、關鍵設定與序列通訊行為，方便主機端與韌體端配合。

### 連接對應（預設）
- 馬達驅動 PWM 與方向腳位：
	- Motor1: `pwmPin=4`, `pinA=17`, `pinB=16`
	- Motor2: `pwmPin=21`, `pinA=18`, `pinB=19`
- 編碼器（PCNT）腳位：
	- Encoder1: `GPIO_NUM_13`, `GPIO_NUM_14`
	- Encoder2: `GPIO_NUM_22`, `GPIO_NUM_23`
- 啟用腳位： `enablePin = 2`

（如需變更腳位，請編輯 `esp_rpi_mc.ino` 中的 `Motor motor1(...)` / `Motor motor2(...)` 與 `EncoderPCNT` 建構子參數。）

### 重要韌體參數
- Serial 波特率：115200（在 `setup()` 使用 `Serial.begin(115200);`）
- PID：兩個 `pidController` 實例初始化為 `(1.0, 0.5, 0.1)`，函式簽名為 `compute(setpoint, actual, max, min, dt)`。
	- `compute` 會回傳被限制在 `min..max` 的輸出（目前範例使用 `255`、`-255` 作上下界），並在輸出未飽和時更新積分項（anti-windup）。
- 控制迴圈（範例）: `motor_contorl_task` 使用 `dt = 0.1s`，讀取編碼器、計算 PID、設定馬達，並以 `vTaskDelay(pdMS_TO_TICKS(100))` 迴圈。

### 序列指令與回傳
- 可用指令（皆以 `\n` 結尾）：
	- `sta`：使能控制器，韌體會回傳 `sta` 並開始回傳編碼器資料（`encoder_read_task`）。
	- `end`：停用控制器與馬達。
	- `res`：重置編碼器計數。
	- `sms <speed1>,<speed2>`：立即設定馬達速度（不經 PID 路徑，直接呼叫 `Motor::setSpeed`），範例：`sms 50,-30`。

- 韌體的輸出樣式：
	- 編碼器讀數（由 `encoder_read_task` 每 10 ms 輸出）:
		- 格式：`<count1>,<count2>\n`（例如 `123, -45`）
	- 速度回饋（`feedback_motorSpeed` 範例）:
		- 格式：`Motor1 Speed: %d, Motor2 Speed: %d\n`

### setup() 中建立的任務
- 在 `setup()` 中目前會建立兩個任務：
  - `encoder_read_task`：每 10 ms 讀取編碼器並在 `is_enabled==true` 時透過 Serial 輸出 `<count1>,<count2>`。此任務在範例中已建立。
  - `set_motor_speed_task`：讀取 Serial 指令（使用 `Serial.readStringUntil('\n')`），解析並執行 `sta`/`end`/`res`/`sms` 指令，此任務在範例中已建立。
- 範例中 `motor_contorl_task`（PID 閉環控制）與 `feedback_motorSpeed` 被註解，若需啟用請在 `setup()` 取消註解並確認資源（stack size）與優先權。

### `Motor::setSpeed` 行為
- 接受的輸入範圍為 -100 .. 100（正負代表方向）。
- 內部會把速度線性映射到 PWM magnitude 0..255，並透過兩個方向腳位控制旋轉方向。

註：本程式已將 `Motor::setSpeed` 改為使用浮點映射並計算 PWM magnitude，例如 `pwm = int(fabs((speed/100.0) * 255.0) + 0.5)`，方向由 `pinA`/`pinB` 控制。

### 建議與注意事項
- 若要使用 PID 閉環控制，請啟用 `motor_contorl_task` 的建立呼叫（`xTaskCreatePinnedToCore`）並調整 `setpoint` 與 `dt`。目前範例中該任務被註解。\
- 若系統需要更精確的 PWM，可調整 MCU PWM 設定（頻率、解析度），本程式碼假設 `analogWrite` 以 0..255 表示 PWM 值。\
- 在現場測試前，請先確認驅動器接地共用、馬達與驅動器供電正確，並以小速度測試以免損壞硬體。

### 建議修正（程式碼補強）
- 建議在 `setup()` 裡明確呼叫 `pinMode(enablePin, OUTPUT);`，因為程式會對 `enablePin` 呼叫 `digitalWrite`。
- `Serial.readStringUntil('\n')` 會阻塞，若希望更即時或非阻塞的命令處理，可改為手動緩衝解析或縮短 timeout。
- `pidController::compute` 已包含 anti-windup 機制，但可改進為使用 epsilon 比較或 back-calculation 方法以提高穩健性。

如果要，我可以把 README 再加上一段「快速上手」步驟（例如如何用 Arduino IDE 或 PlatformIO 編譯並上傳），或把常用指令與範例輸出整理成測試腳本。 

