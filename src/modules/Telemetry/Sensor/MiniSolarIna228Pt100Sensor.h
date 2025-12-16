#pragma once
#include "configuration.h"

#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include(<Adafruit_MAX31865.h>)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"
#include "detect/ScanI2C.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MAX31865.h>

// 設定しやすいようにマクロでピン等を外出し
#ifndef MINISOLAR_INA228_ADDR
#define MINISOLAR_INA228_ADDR 0x40
#endif

#ifndef MINISOLAR_PT100_CS_PIN
#define MINISOLAR_PT100_CS_PIN 4
#endif

#ifndef MINISOLAR_PT100_RREF
#define MINISOLAR_PT100_RREF 430.0f
#endif

#ifndef MINISOLAR_PT100_RNOMINAL
#define MINISOLAR_PT100_RNOMINAL 100.0f
#endif

class MiniSolarIna228Pt100Sensor : public TelemetrySensor
{
  public:
    MiniSolarIna228Pt100Sensor();

    // TelemetrySensor API
    virtual int32_t runOnce() override;
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
    virtual bool initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev) override;

#if WIRE_INTERFACES_COUNT > 1
    // Wire1専用にしたい場合はここをtrueに
    virtual bool onlyWire1() override { return false; }
#endif

  private:
    TwoWire *wire = nullptr;
    Adafruit_MAX31865 rtd;

    void configureIna228LowRange();
    int32_t readIna228Raw20();
};

#endif // HAS_TELEMETRY ...
