// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include "sen6x_core.h"

void waitForF();

// ===== Board wiring (XIAO ESP32-S3) =====
// Grove I2C header is commonly: SDA=GPIO6, SCL=GPIO7.
// Change if your wiring differs.

// ===== Sensor address =====
#define SEN6X_I2C_ADDR 0x6B

// Create the core driver (uses default Wire instance)
sen6x::Core aq(Wire, SEN6X_I2C_ADDR);

// Simple I2C scanner (handy for first bring-up)
static void scanI2C()
{
  Serial.println("\n[I2C] Scanning...");
  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++)
  {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission(true);
    if (err == 0)
    {
      Serial.printf("  - Device at 0x%02X\n", addr);
      found++;
    }
    else if (err == 4)
    {
      Serial.printf("  - Unknown error at 0x%02X\n", addr);
    }
    delay(2);
  }
  if (!found)
    Serial.println("  (none)");
}

static const char *statusName(sen6x::Status s)
{
  switch (s)
  {
  case sen6x::Status::OK:
    return "OK";
  case sen6x::Status::I2C_ERROR:
    return "I2C_ERROR";
  case sen6x::Status::CRC_ERROR:
    return "CRC_ERROR";
  case sen6x::Status::TIMEOUT:
    return "TIMEOUT";
  case sen6x::Status::INVALID_ARG:
    return "INVALID_ARG";
  case sen6x::Status::NOT_INITIALIZED:
    return "NOT_INITIALIZED";
  default:
    return "UNKNOWN";
  }
}

static void printDeviceStatus(const sen6x::DeviceStatus &d);

void setup()
{
  Serial.begin(115200);
  delay(200);
  waitForF();

  // Bring up I2C explicitly on XIAO pins
  bool ok = Wire.begin();
  Wire.setClock(100000); // 100 kHz
  // bool ok = Wire.begin(/*SDA=*/6, /*SCL=*/7, /*frequency=*/100000);
  Serial.printf("Wire tatus %s\n", ok ? "OK" : "FAIL");
  delay(20);

  scanI2C();

  // Initialize SEN6x core
  aq.setDebug(&Serial); // optional: enable debug prints from core
  sen6x::Status st = aq.begin();
  Serial.printf("sen6x.begin() -> %s\n", statusName(st));
  if (st != sen6x::Status::OK)
  {
    Serial.println("Sensor probe failed. Check wiring, address, and pull-ups.");
    // keep running so you can rescan/retry
  }

  // ───── Read and print serial number ─────
  String serial;
  st = aq.readSerialNumber(serial);
  if (st == sen6x::Status::OK)
  {
    Serial.print("Serial Number: ");
    Serial.println(serial);
  }
  else
  {
    Serial.printf("readSerialNumber() -> %s\n", statusName(st));
  }

  // ───── Read and print product name ───── String product;
  String product;
  st = aq.readProductName(product);
  if (st == sen6x::Status::OK)
  {
    Serial.print("Product Name: ");
    Serial.println(product);
  }
  else
  {
    Serial.printf("readProductName() -> %s\n", statusName(st));
  }

  // ───── Read and print device status ─────
  uint32_t dsr = 0;
  st = aq.readDeviceStatus(dsr);
  sen6x::DeviceStatus status;
  if (aq.getDeviceStatus(status) == sen6x::Status::OK)
  {
    printDeviceStatus(status);
  }
  if (st == sen6x::Status::OK)
  {
    Serial.printf("DSR = 0x%08lX\n", (unsigned long)dsr);

    if (dsr & sen6x_bits::WARN_SPEED)
      Serial.println(" - Warning: fan speed");
    if (dsr & sen6x_bits::ERR_FAN)
      Serial.println(" - Error: FAN");
    if (dsr & sen6x_bits::ERR_RHT)
      Serial.println(" - Error: RH&T");
    if (dsr & sen6x_bits::ERR_GAS)
      Serial.println(" - Error: GAS");
    if (dsr & sen6x_bits::ERR_CO2_2)
      Serial.println(" - Error: CO₂-2");
    if (dsr & sen6x_bits::ERR_HCHO)
      Serial.println(" - Error: HCHO");
    if (dsr & sen6x_bits::ERR_PM)
      Serial.println(" - Error: PM");
    if (dsr & sen6x_bits::ERR_CO2_1)
      Serial.println(" - Error: CO₂-1");
  }
  else
  {
    Serial.printf("readDeviceStatus() -> %d\n", (int)st);
  }

  // Start periodic measurement (replace with real command in core when you fill TODOs)
  st = aq.startMeasurement();
  Serial.printf("sen6x.startMeasurement() -> %s\n", statusName(st));
}

void loop()
{
  static uint32_t last = 0;
  static uint8_t sampleCount = 0;
  const uint32_t now = millis();

  // Stop after 3 samples
  if (sampleCount >= 3)
  {
    static bool stopped = false;
    if (!stopped)
    {
      sen6x::Status stopStatus = aq.stopMeasurement();
      Serial.printf("sen6x.stopMeasurement() -> %s\n", statusName(stopStatus));
      stopped = true;
    }
    return;
  }

  if (now - last >= 10000)
  {
    last = now;

    uint8_t ready = 0;
    sen6x::Status s = aq.getDataReady(ready);
    if (s == sen6x::Status::OK && ready)
    {
      sen6x::SEN66Measured r;
      s = aq.readMeasurement(r);

      if (s == sen6x::Status::OK)
      {
        sampleCount++;
        Serial.printf("[SEN66] #%u | PM1=%.1f PM2.5=%.1f PM4=%.1f PM10=%.1f | RH=%.2f%% T=%.2f°C | VOC=%.1f NOx=%.1f CO2=%.0f ppm\n",
                      sampleCount,
                      r.pm1_0, r.pm2_5, r.pm4_0, r.pm10_0,
                      r.humidity, r.temperature,
                      r.voc_index, r.nox_index, r.co2);
      }
      else
      {
        Serial.printf("[SEN60] readMeasurement() -> %s\n", statusName(s));
      }
    }
    else if (s == sen6x::Status::OK)
    {
      Serial.println("[SEN60] Data not ready yet...");
    }
    else
    {
      Serial.printf("[SEN60] getDataReady() -> %s\n", statusName(s));
    }
  }

  delay(5);
}

void waitForF()
{
  Serial.println("Type 'f' to continue...");
  while (true)
  {
    if (Serial.available())
    {
      char c = Serial.read();
      if (c == 'f' || c == 'F')
        break;
    }
    delay(10);
  }
}

static void printDeviceStatus(const sen6x::DeviceStatus &d)
{
  Serial.printf("DSR = 0x%08lX\n", (unsigned long)d.raw);

  if (d.fan_speed_warning)
    Serial.println(" - Warning: Fan speed");
  if (d.fan_error)
    Serial.println(" - Error: FAN");
  if (d.rht_error)
    Serial.println(" - Error: RH&T");
  if (d.gas_error)
    Serial.println(" - Error: GAS");
  if (d.co2_2_error)
    Serial.println(" - Error: CO₂-2");
  if (d.hcho_error)
    Serial.println(" - Error: HCHO");
  if (d.pm_error)
    Serial.println(" - Error: PM");
  if (d.co2_1_error)
    Serial.println(" - Error: CO₂-1");

  if (!d.raw)
    Serial.println(" - All OK ✅");
}