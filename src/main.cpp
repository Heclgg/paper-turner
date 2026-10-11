#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "esp32-hal-rgb-led.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif


#define PIN_LED             10
#define BRIGHTNESS_SCALE    168
#define BREATH_PERIOD_MS    2000   // 呼吸灯完整周期（明→暗→明）
#define BREATH_UPDATE_MS    20     // 呼吸灯刷新间隔
#define GREEN_BLINK_MS      200

// ==================== XT-EINK BLE 协议 ====================
#define SERVICE_UUID            "00001812-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID     "00002a4d-0000-1000-8000-00805f9b34fb"

// 自定义命令码
#define CMD_UP              0x01
#define CMD_DOWN            0x02
#define CMD_BACK            0x03
#define CMD_CONFIRM         0x04
#define CMD_SLEEP           0x05
#define CMD_SHUTDOWN        0x06
#define CMD_REMOTE_OFF      0x07

// ==================== 按键配置 ====================
#define PIN_BTN_NXT         11      // GPIO11: 下一页
#define PIN_BTN_PRE         12      // GPIO12: 上一页

// 消抖参数
#define DEBOUNCE_MS         30

// 单键长按确认参数
#define LONG_PRESS_CONFIRM_MS 1500

// 双键同时长按关机检测
#define BOTH_PRESS_SHUTDOWN_MS      2500
#define BOTH_PRESS_GRACE_MS         50   // 同时按下检测窗口：延迟单键命令以等待另一键

// 配对超时进入低功耗
#define PAIR_TIMEOUT_MS             10000


static void ws2812_show(uint8_t r, uint8_t g, uint8_t b)
{
    r = static_cast<uint16_t>(r) * BRIGHTNESS_SCALE / 255;
    g = static_cast<uint16_t>(g) * BRIGHTNESS_SCALE / 255;
    b = static_cast<uint16_t>(b) * BRIGHTNESS_SCALE / 255;
    rgbLedWrite(PIN_LED, r, g, b);
}

static uint32_t redTimer = 0;
static bool greenActive = false;
static uint8_t greenStep = 0; // 0-5
static uint32_t greenTimer = 0;

static BLEServer* pServer = nullptr;
static BLEService* pService = nullptr;
static BLECharacteristic* pCharacteristic = nullptr;
static BLEAdvertising* pAdvertising = nullptr;
static bool deviceConnected = false;

static bool wasConnected = false;
static uint32_t connectedAt = 0;
static uint32_t advStartedAt = 0;  // BLE 广播开始时间，用于配对超时检测

struct ButtonState
{
    uint8_t pin;
    uint8_t command;
    const char* label;
    bool lastStable; // 上次稳定电平
    bool lastRaw; // 上次原始读取值
    uint32_t lastChangeMs;
};

static ButtonState btnNxt;
static ButtonState btnPre;

static bool bothPressed = false;
static uint32_t bothPressedAt = 0;
static bool shutdownSent = false;

// 延迟边沿处理（用于同时按下检测，避免误触发翻页）
static bool nxtEdgePending = false;
static uint32_t nxtEdgePendingAt = 0;
static bool preEdgePending = false;
static uint32_t preEdgePendingAt = 0;

// preBtn 单键长按 1.5s → CMD_CONFIRM
static bool preBtnLongPressSent = false;
static uint32_t preBtnPressStartMs = 0;

// nxtBtn 单键长按 1.5s → CMD_BACK
static bool nxtBtnLongPressSent = false;
static uint32_t nxtBtnPressStartMs = 0;

namespace
{
    class MyServerCallbacks : public BLEServerCallbacks
    {
        void onConnect(BLEServer* pServer) override
        {
            deviceConnected = true;
        }

        void onDisconnect(BLEServer* pServer) override
        {
            deviceConnected = false;
            wasConnected = false;
            greenActive = false; // 取消绿灯闪烁
            advStartedAt = millis(); // 重置配对超时计时器
            pServer->getAdvertising()->start();
        }
    };
}

static void sendBLECommand(uint8_t command)
{
    if (deviceConnected && pCharacteristic)
    {
        pCharacteristic->setValue(&command, 1);
        pCharacteristic->notify();
        Serial.printf("[CMD] Sent 0x%02X\n", command);
    }
    else
    {
        Serial.println("[CMD] Not connected");
    }
}

// ==================== 按键消抖检测 ====================
// 上拉模式：未按下 = HIGH，按下 = LOW（active LOW）
// 每周期调用一次：更新消抖状态，返回当前稳定按下状态（true = 按下）
// 同时通过 pressedEdge 输出"本次检测到一次按下事件"（下降沿，仅在按下瞬间触发一次）
static bool updateButton(ButtonState& btn, bool& pressedEdge)
{
    const bool raw = digitalRead(btn.pin); // HIGH = 未按, LOW = 按下
    const uint32_t now = millis();
    pressedEdge = false;

    // 原始电平发生变化，记录时间戳，稳定电平保持不变
    if (raw != btn.lastRaw)
    {
        btn.lastRaw = raw;
        btn.lastChangeMs = now;
        return btn.lastStable == LOW;
    }

    // 电平稳定时间不足消抖窗口，忽略
    if (now - btn.lastChangeMs < DEBOUNCE_MS)
        return btn.lastStable == LOW;

    // 稳定电平与上次不同 → 更新并检测到边沿
    if (raw != btn.lastStable)
    {
        btn.lastStable = raw;

        // 下降沿（HIGH → LOW）= 按下瞬间
        if (raw == LOW)
        {
            Serial.println(btn.label);
            pressedEdge = true;
        }
    }

    return btn.lastStable == LOW;
}

// ==================== BLE 初始化（可重复调用） ====================
static void initBLE()
{
    BLEDevice::init("RC-DD-X4");

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );
    pCharacteristic->addDescriptor(new BLE2902());

    uint8_t initVal[1] = {0};
    pCharacteristic->setValue(initVal, 1);

    pService->start();

    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);

    BLEAdvertisementData scanResp;
    scanResp.setName("DDT");
    scanResp.setCompleteServices(BLEUUID(SERVICE_UUID));
    pAdvertising->setScanResponseData(scanResp);

    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    pAdvertising->start();

    Serial.println("BLE 广播已开始 (Name: RC-DD-X4)");
}

// ==================== 低功耗模式 ====================
static void enterLowPowerMode()
{
    Serial.println("\n[LOWPOWER] 10s 未配对，进入低功耗模式...");

    // 1. 关闭 WS2812
    ws2812_show(0, 0, 0);

    // 2. 关闭 BLE（停止广播 + 释放控制器）
    BLEDevice::deinit();
    pServer = nullptr;
    pService = nullptr;
    pCharacteristic = nullptr;
    pAdvertising = nullptr;

    // 3. 配置 GPIO 唤醒源（按键 active LOW，低电平唤醒）
    gpio_wakeup_enable((gpio_num_t)PIN_BTN_NXT, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)PIN_BTN_PRE, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    // 4. 进入 Light Sleep
    Serial.println("[LOWPOWER] 进入 light sleep，等待按键唤醒...");
    Serial.flush();
    delay(50);

    esp_light_sleep_start();

    // 5. 唤醒后清理 GPIO 唤醒配置
    gpio_wakeup_disable((gpio_num_t)PIN_BTN_NXT);
    gpio_wakeup_disable((gpio_num_t)PIN_BTN_PRE);

    Serial.println("[LOWPOWER] 已唤醒，重新初始化...");

    // 6. 等待按键释放（避免唤醒按键误触发翻页）
    while (digitalRead(PIN_BTN_NXT) == LOW || digitalRead(PIN_BTN_PRE) == LOW)
    {
        delay(10);
    }

    // 7. 重新初始化 BLE
    initBLE();

    // 8. 重置状态
    advStartedAt = millis();
    redTimer = millis();
    deviceConnected = false;
    wasConnected = false;
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    // ---- 初始化按键 ----
    // 上拉模式：引脚内部上拉电阻使能，外部开关接地
    btnNxt.pin = PIN_BTN_NXT;
    btnNxt.command = CMD_UP;
    btnNxt.label = "[NEXT] CMD_UP (0x01)";
    btnNxt.lastStable = HIGH;
    btnNxt.lastRaw = HIGH;
    btnNxt.lastChangeMs = 0;

    btnPre.pin = PIN_BTN_PRE;
    btnPre.command = CMD_DOWN;
    btnPre.label = "[PREV] CMD_DOWN (0x02)";
    btnPre.lastStable = HIGH;
    btnPre.lastRaw = HIGH;
    btnPre.lastChangeMs = 0;

    pinMode(PIN_BTN_NXT, INPUT_PULLUP);
    pinMode(PIN_BTN_PRE, INPUT_PULLUP);

    // ---- 初始化 WS2812 (RMT 外设) ----
    ws2812_show(0, 0, 0); // 初始熄灭（同时把 GPIO10 拉低，避免悬空）

    Serial.println("\n=============================");
    Serial.println("esp32h2 paper turner");
    Serial.println("=============================");
    Serial.println("Waiting for connection...");

    initBLE();
    advStartedAt = millis();
}

void loop()
{
    // ==================== 未连接：红灯闪烁 ====================
    if (!deviceConnected)
    {
        // ==================== 配对超时检测 ====================
        if (millis() - advStartedAt >= PAIR_TIMEOUT_MS)
        {
            enterLowPowerMode();
            return;
        }

        const uint32_t now = millis();
        if (now - redTimer >= BREATH_UPDATE_MS)
        {
            redTimer = now;
            // 以 advStartedAt 为起点，用正弦波驱动呼吸亮度 0→255→0
            const uint32_t phaseMs = (now - advStartedAt) % BREATH_PERIOD_MS;
            const float phase = (float)phaseMs / BREATH_PERIOD_MS * (2.0f * PI);
            const auto brightness = static_cast<uint8_t>((sinf(phase) * 0.5f + 0.5f) * 255.0f);
            ws2812_show(brightness, 0, 0);
        }

        static uint32_t lastPrint = 0;
        if (now - lastPrint > 1000)
        {
            Serial.print(".");
            lastPrint = now;
        }
        return;
    }

    // ==================== 连接后：绿灯闪3次 ====================
    if (greenActive)
    {
        if (const uint32_t now = millis(); now - greenTimer >= GREEN_BLINK_MS)
        {
            greenTimer = now;
            if (greenStep % 2 == 0)
                ws2812_show(0, 255, 0); // 亮
            else
                ws2812_show(0, 0, 0); // 灭

            greenStep++;
            if (greenStep >= 6)
            {
                greenActive = false;
                ws2812_show(0, 0, 0);
            }
        }
    }

    if (!wasConnected)
    {
        wasConnected = true;
        connectedAt = millis();
        // 触发绿灯3闪
        greenActive = true;
        greenStep = 0;
        greenTimer = millis();
        Serial.println("\n[BT] Connected! Waiting 3s...");
    }

    if (millis() - connectedAt < 3000)
    {
        return;
    }

    // ---- 读取双键消抖状态（每周期各更新一次） ----
    bool nxtEdge = false, preEdge = false;
    const bool nxtHeld = updateButton(btnNxt, nxtEdge);
    const bool preHeld = updateButton(btnPre, preEdge);

    // 记录边沿，延迟处理（等待 grace period 以排除同时按下）
    if (nxtEdge)
    {
        nxtEdgePending = true;
        nxtBtnLongPressSent = false;

        const uint32_t currentMs = millis();
        nxtEdgePendingAt = currentMs;
        nxtBtnPressStartMs = currentMs;
    }

    if (preEdge)
    {
        preEdgePending = true;
        preBtnLongPressSent = false;

        const uint32_t currentMs = millis();
        preEdgePendingAt = currentMs;
        preBtnPressStartMs = currentMs;
    }

    // ---- 双键同时按下：长按 2.5 秒 → 发送 CMD_SHUTDOWN（期间不翻页） ----
    if (nxtHeld && preHeld)
    {
        // 双键已同时按下，取消所有待处理的单键命令
        nxtEdgePending = false;
        preEdgePending = false;

        // 取消单键长按检测
        preBtnPressStartMs = 0;
        nxtBtnPressStartMs = 0;

        if (!bothPressed)
        {
            bothPressed = true;
            bothPressedAt = millis();
            Serial.println("[BTN] 双键同时按下，长按 2.5 秒确认...");
        }
        else if (!shutdownSent && (millis() - bothPressedAt >= BOTH_PRESS_SHUTDOWN_MS))
        {
            sendBLECommand(CMD_SHUTDOWN);
            shutdownSent = true;
        }
    }
    else
    {
        // 双键释放 → 重置状态，允许下次再次触发 CMD_SHUTDOWN
        if (bothPressed)
        {
            bothPressed = false;
            shutdownSent = false;
        }

        // 处理延迟的单键边沿：经过 grace period 且另一键未按下时才执行翻页
        if (nxtEdgePending && (millis() - nxtEdgePendingAt >= BOTH_PRESS_GRACE_MS))
        {
            // 不在此处发送 CMD_UP，改为释放时发送以区分短按/长按
            nxtEdgePending = false;
        }

        if (preEdgePending && (millis() - preEdgePendingAt >= BOTH_PRESS_GRACE_MS))
        {
            // 不在此处发送 CMD_DOWN，改为释放时发送以区分短按/长按
            preEdgePending = false;
        }

        // ---- DOWN 短按释放 → CMD_DOWN；长按 1.5s → CMD_CONFIRM（不翻页）----
        if (!preHeld)
        {
            if (preBtnPressStartMs > 0 && !preBtnLongPressSent)
                sendBLECommand(btnPre.command);
            preBtnPressStartMs = 0;
            preBtnLongPressSent = false;
        }
        else if (!nxtHeld && !preBtnLongPressSent && preBtnPressStartMs > 0 &&
            (millis() - preBtnPressStartMs >= LONG_PRESS_CONFIRM_MS))
        {
            sendBLECommand(CMD_CONFIRM);
            preBtnLongPressSent = true;
        }

        // ---- UP 短按释放 → CMD_UP；长按 1.5s → CMD_BACK（不翻页）----
        if (!nxtHeld)
        {
            if (nxtBtnPressStartMs > 0 && !nxtBtnLongPressSent)
                sendBLECommand(btnNxt.command);
            nxtBtnPressStartMs = 0;
            nxtBtnLongPressSent = false;
        }
        else if (!preHeld && !nxtBtnLongPressSent && nxtBtnPressStartMs > 0 &&
            (millis() - nxtBtnPressStartMs >= LONG_PRESS_CONFIRM_MS))
        {
            sendBLECommand(CMD_BACK);
            nxtBtnLongPressSent = true;
        }
    }

    delay(5);
}
