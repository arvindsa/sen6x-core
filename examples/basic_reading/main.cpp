// basic_reading — SEN6x bring-up and continuous measurement example
// Demonstrates: init, serial/product/version read, device status, SEN68 measurement.
// Board: XIAO ESP32-S3

#include <Arduino.h>
#include <Wire.h>
#include "sen6x_core.h"

// Change to readMeasurementSEN65() / readMeasurement() for SEN65 / SEN66.
static constexpr uint8_t ADDR = sen6x::DEFAULT_ADDR;

sen6x::Core aq(Wire, ADDR);

static void waitForKey()
{
    Serial.println("Type 'f' to continue...");
    while (true) {
        if (Serial.available() && (Serial.read() | 0x20) == 'f') break;
        delay(10);
    }
}

static const char *statusName(sen6x::Status s)
{
    switch (s) {
    case sen6x::Status::OK:              return "OK";
    case sen6x::Status::I2C_ERROR:       return "I2C_ERROR";
    case sen6x::Status::CRC_ERROR:       return "CRC_ERROR";
    case sen6x::Status::TIMEOUT:         return "TIMEOUT";
    case sen6x::Status::INVALID_ARG:     return "INVALID_ARG";
    case sen6x::Status::NOT_INITIALIZED: return "NOT_INITIALIZED";
    default:                             return "UNKNOWN";
    }
}

static void scanI2C()
{
    Serial.println("\n[I2C] Scanning...");
    uint8_t found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission(true) == 0) {
            Serial.printf("  - Device at 0x%02X\n", addr);
            found++;
        }
        delay(2);
    }
    if (!found) Serial.println("  (none)");
}

static void printDeviceStatus(const sen6x::DeviceStatus &d)
{
    Serial.printf("DSR = 0x%08lX%s\n",
                  (unsigned long)d.raw, d.raw ? "" : "  (all OK)");
    if (d.fan_speed_warning) Serial.println("  ! Fan speed warning");
    if (d.fan_error)         Serial.println("  ! Fan error");
    if (d.rht_error)         Serial.println("  ! RH&T error");
    if (d.gas_error)         Serial.println("  ! Gas sensor error");
    if (d.co2_2_error)       Serial.println("  ! CO2-2 error");
    if (d.hcho_error)        Serial.println("  ! HCHO error");
    if (d.pm_error)          Serial.println("  ! PM error");
    if (d.co2_1_error)       Serial.println("  ! CO2-1 error");
}

void setup()
{
    Serial.begin(115200);
    delay(200);
    waitForKey();

    Wire.begin();
    Wire.setClock(100000);
    scanI2C();

    aq.setDebug(&Serial);
    sen6x::Status st = aq.begin();
    Serial.printf("begin()         -> %s\n", statusName(st));
    if (st != sen6x::Status::OK) return;

    String serial, product;
    if (aq.readSerialNumber(serial) == sen6x::Status::OK)
        Serial.printf("Serial Number   : %s\n", serial.c_str());
    if (aq.readProductName(product) == sen6x::Status::OK)
        Serial.printf("Product Name    : %s\n", product.c_str());

    sen6x::FirmwareVersion ver;
    if (aq.getVersion(ver) == sen6x::Status::OK)
        Serial.printf("Firmware        : %u.%u\n", ver.major, ver.minor);

    sen6x::DeviceStatus dsr;
    if (aq.getDeviceStatus(dsr) == sen6x::Status::OK)
        printDeviceStatus(dsr);

    st = aq.startMeasurement();
    Serial.printf("startMeasurement() -> %s\n", statusName(st));
}

void loop()
{
    static uint32_t last = 0;

    if (millis() - last < 10000) return;
    last = millis();

    uint8_t ready = 0;
    sen6x::Status s = aq.getDataReady(ready);
    if (s != sen6x::Status::OK || !ready) return;

    // ── SEN68 reading (HCHO, no CO₂) ──────────────────────────────────────
    sen6x::SEN68Measured r;
    s = aq.readMeasurementSEN68(r);
    if (s == sen6x::Status::OK) {
        Serial.printf("[SEN68] PM1=%.1f PM2.5=%.1f PM4=%.1f PM10=%.1f µg/m³ | "
                      "RH=%.1f%% T=%.1f°C | VOC=%.0f NOx=%.0f | HCHO=%.1f ppb\n",
                      r.pm1_0, r.pm2_5, r.pm4_0, r.pm10_0,
                      r.humidity, r.temperature,
                      r.voc_index, r.nox_index, r.hcho);
    } else {
        Serial.printf("[SEN68] readMeasurementSEN68() -> %s\n", statusName(s));
    }

    // ── Number concentrations ─────────────────────────────────────────────
    sen6x::NumberConcentration nc;
    if (aq.readNumberConcentration(nc) == sen6x::Status::OK) {
        Serial.printf("        #conc: PM0.5=%.1f PM1=%.1f PM2.5=%.1f "
                      "PM4=%.1f PM10=%.1f #/cm³\n",
                      nc.pm0_5, nc.pm1_0, nc.pm2_5, nc.pm4_0, nc.pm10_0);
    }
}
