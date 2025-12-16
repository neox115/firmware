#include "MiniSolarIna228Pt100Sensor.h"

#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include(<Adafruit_MAX31865.h>)

#include "configuration.h"

MiniSolarIna228Pt100Sensor::MiniSolarIna228Pt100Sensor()
    : TelemetrySensor(meshtastic_TelemetrySensorType_CUSTOM_SENSOR, "MiniSolar INA228+PT100"),
      rtd(MINISOLAR_PT100_CS_PIN)
{
}

bool MiniSolarIna228Pt100Sensor::initDevice(TwoWire *bus, ScanI2C::FoundDevice * /*dev*/)
{
    wire = bus;

    if (!wire)
    {
        LOG_ERROR("MiniSolar: wire pointer null");
        return false;
    }

    // INA228 Lowレンジ設定
    configureIna228LowRange();

    // SPIはボード初期化側で設定済み想定だが、念のため開始
    SPI.begin();

    if (!rtd.begin(MAX31865_3WIRE))
    {
        LOG_ERROR("MiniSolar: MAX31865 init failed");
        // 温度なし運用でもよいなら true にしてもよい
        // とりあえずセンサー自体は有効にしておく
    }
    else
    {
        LOG_INFO("MiniSolar: MAX31865 init OK");
    }

    status = 1;
    initialized = true;
    LOG_INFO("MiniSolarIna228Pt100Sensor initialized");
    return true;
}

int32_t MiniSolarIna228Pt100Sensor::runOnce()
{
    // 特に周期処理は不要。環境テレメトリ側が getMetrics() を呼ぶ。
    return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
}

// CONFIGレジスタ bit4 = 1 で Low レンジに設定
void MiniSolarIna228Pt100Sensor::configureIna228LowRange()
{
    const uint8_t REG_CONFIG = 0x00;
    uint16_t config = 0;

    wire->beginTransmission(MINISOLAR_INA228_ADDR);
    wire->write(REG_CONFIG);
    if (wire->endTransmission(false) != 0)
    {
        LOG_WARN("MiniSolar: INA228 CONFIG read failed (tx)");
        return;
    }

    if (wire->requestFrom(MINISOLAR_INA228_ADDR, (uint8_t)2) != 2)
    {
        LOG_WARN("MiniSolar: INA228 CONFIG read failed (rx)");
        return;
    }

    uint8_t hi = wire->read();
    uint8_t lo = wire->read();
    config = (uint16_t(hi) << 8) | lo;

    // bit4 を 1 にして Low レンジへ
    config |= (1U << 4);

    wire->beginTransmission(MINISOLAR_INA228_ADDR);
    wire->write(REG_CONFIG);
    wire->write((uint8_t)(config >> 8));
    wire->write((uint8_t)(config & 0xFF));
    wire->endTransmission();

    LOG_INFO("MiniSolar: INA228 CONFIG set to 0x%04X", config);
}

// CURRENTレジスタ(0x07)から20bit符号付き値を読む
int32_t MiniSolarIna228Pt100Sensor::readIna228Raw20()
{
    const uint8_t REG_CURRENT = 0x07;
    uint8_t b1, b2, b3;

    wire->beginTransmission(MINISOLAR_INA228_ADDR);
    wire->write(REG_CURRENT);
    if (wire->endTransmission(false) != 0)
    {
        LOG_WARN("MiniSolar: INA228 CURRENT read failed (tx)");
        return 0;
    }

    if (wire->requestFrom(MINISOLAR_INA228_ADDR, (uint8_t)3) != 3)
    {
        LOG_WARN("MiniSolar: INA228 CURRENT read failed (rx)");
        return 0;
    }

    b1 = wire->read();
    b2 = wire->read();
    b3 = wire->read();

    // [b1 b2 b3] の上位20bitが有効
    uint32_t raw20 = (uint32_t(b1) << 12) | (uint32_t(b2) << 4) | (uint32_t(b3) >> 4);

    // 20bit 2の補数 → 32bit符号拡張
    if (raw20 & 0x80000)
    {
        raw20 |= 0xFFF00000;
    }

    return (int32_t)raw20;
}

bool MiniSolarIna228Pt100Sensor::getMetrics(meshtastic_Telemetry *measurement)
{
    if (!wire)
    {
        LOG_WARN("MiniSolar: wire not set");
        return false;
    }

    // 1) INA228: 20bit生値 → Lowレンジ → A
    int32_t raw20 = readIna228Raw20();
    float current_A = (float)raw20 / 25600.0f; // 読み値 / 25600 = A直読
    float current_mA = current_A * 1000.0f;

    // 2) MAX31865: PT100裏面温度
    float tempC = rtd.temperature(MINISOLAR_PT100_RNOMINAL, MINISOLAR_PT100_RREF);

    auto &env = measurement->variant.environment_metrics;

    env.has_current = true;
    env.current = current_A; // Influx側で "isc" として扱う想定

    env.has_temperature = true;
    env.temperature = tempC;

    LOG_DEBUG("MiniSolar: raw20=%ld Isc[mA]=%0.3f Temp[C]=%0.2f", (long)raw20, current_mA, tempC);

    return true;
}

#endif // HAS_TELEMETRY ...
