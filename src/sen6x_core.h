/**
 * @file sen6x_core.h
 * @brief Core definitions and API for Sensirion SEN6x environmental sensor family.
 *
 * Provides type definitions, constants, and class declarations for communicating
 * with SEN6x devices (SEN60 / SEN63C / SEN65 / SEN66 / SEN68) over I²C.
 *
 * Based on: Sensirion SEN6x Datasheet – Version 0.92 (December 2025)
 *           and official Sensirion embedded drivers for SEN65, SEN66, SEN68.
 *
 * Variant measurement commands:
 *   SEN66  → 0x0300  9 words  PM + RH + T + VOC + NOx + CO₂
 *   SEN65  → 0x0446  8 words  PM + RH + T + VOC + NOx  (no CO₂, no HCHO)
 *   SEN68  → 0x0467  9 words  PM + RH + T + VOC + NOx + HCHO  (no CO₂)
 *   SEN63C → 0x0300  9 words  PM + RH + T + CO₂  (VOC/NOx fields return NaN)
 *   SEN60  → addr 0x6C, PM only
 *
 * @author  Arvind S.A.
 * @date    2025-10-05 (updated 2026-04-20)
 */

#pragma once
#include <Arduino.h>
#include <Wire.h>

namespace sen6x
{

constexpr uint8_t DEFAULT_ADDR = 0x6B; ///< SEN63C, SEN65, SEN66, SEN68
constexpr uint8_t SEN60_ADDR   = 0x6C; ///< SEN60 only

enum class Status : uint8_t
{
    OK = 0,
    I2C_ERROR,
    CRC_ERROR,
    TIMEOUT,         ///< startMeasurement() called before stop-cooldown elapsed
    INVALID_ARG,
    NOT_INITIALIZED
};

struct DeviceStatus
{
    bool fan_speed_warning = false;
    bool fan_error         = false;
    bool rht_error         = false;
    bool gas_error         = false;
    bool co2_2_error       = false;
    bool hcho_error        = false;
    bool pm_error          = false;
    bool co2_1_error       = false;
    uint32_t raw           = 0;
};

// ── Measurement output structs ─────────────────────────────────────────────

/// SEN66 / SEN63C: PM + RH + T + VOC + NOx + CO₂  (cmd 0x0300, 9 words)
struct SEN66Measured
{
    float pm1_0;       ///< µg/m³
    float pm2_5;       ///< µg/m³
    float pm4_0;       ///< µg/m³
    float pm10_0;      ///< µg/m³
    float humidity;    ///< %RH
    float temperature; ///< °C
    float voc_index;   ///< unitless (NaN on SEN63C)
    float nox_index;   ///< unitless (NaN on SEN63C)
    float co2;         ///< ppm
};

/// SEN65: PM + RH + T + VOC + NOx  (cmd 0x0446, 8 words; no CO₂/HCHO field)
struct SEN65Measured
{
    float pm1_0;
    float pm2_5;
    float pm4_0;
    float pm10_0;
    float humidity;
    float temperature;
    float voc_index;
    float nox_index;
};

/// SEN68: PM + RH + T + VOC + NOx + HCHO  (cmd 0x0467, 9 words; no CO₂)
struct SEN68Measured
{
    float pm1_0;
    float pm2_5;
    float pm4_0;
    float pm10_0;
    float humidity;
    float temperature;
    float voc_index;
    float nox_index;
    float hcho;        ///< ppb
};

/// Particle number concentrations (cmd 0x0316, 5 words; all PM variants)
struct NumberConcentration
{
    float pm0_5;  ///< #/cm³
    float pm1_0;  ///< #/cm³
    float pm2_5;  ///< #/cm³
    float pm4_0;  ///< #/cm³
    float pm10_0; ///< #/cm³
};

/// Raw sensor ticks before algorithm processing.
/// cmd 0x0455 for SEN65/SEN68; cmd 0x0405 for SEN66/SEN63C.
/// co2_or_hcho: raw CO₂ ppm (SEN66/SEN63C) or raw HCHO ppb (SEN68); absent for SEN65.
struct RawValues
{
    int16_t  humidity;      ///< raw RH ticks
    int16_t  temperature;   ///< raw T ticks
    uint16_t voc;           ///< SGVOC raw ticks
    uint16_t nox;           ///< SGNOx raw ticks
    uint16_t co2_or_hcho;   ///< raw CO₂ ppm or HCHO ppb
};

/// RH/T readings from the integrated heater (cmd 0x6790)
struct ShtHeaterData
{
    float humidity;    ///< %RH
    float temperature; ///< °C
};

/// Firmware version (cmd 0xD100)
struct FirmwareVersion
{
    uint8_t major;
    uint8_t minor;
};

/// VOC algorithm tuning parameters (cmd 0x60D0). Same layout for NOx (cmd 0x60E1).
struct VocTuning
{
    int16_t index_offset;            ///< VOC/NOx index offset, default 100
    int16_t learning_time_offset_h;  ///< Offset learning time-constant (hours), default 12
    int16_t learning_time_gain_h;    ///< Gain learning time-constant (hours), default 12
    int16_t gating_max_duration_min; ///< Gating max duration (minutes); 0 = no gating
    int16_t std_initial;             ///< Initial std-dev estimate, default 50
    int16_t gain_factor;             ///< Gain factor, default 230
};

using NoxTuning = VocTuning; ///< NOx tuning has the same parameter layout as VOC

// ── Core driver class ──────────────────────────────────────────────────────

class Core
{
public:
    explicit Core(TwoWire &w = Wire, uint8_t addr = DEFAULT_ADDR)
        : wire_(w), address_(addr)
    {
    }

    // ── Initialization ─────────────────────────────────────────────────────

    /**
     * Probe the I²C address and mark the driver as initialised.
     * Must be called before any other method.
     */
    Status begin();

    // ── Measurement control ────────────────────────────────────────────────

    /**
     * Start continuous measurement (cmd 0x0021).
     * Returns Status::TIMEOUT if the mandatory 1 s stop-cooldown has not elapsed yet.
     * First valid data is available ~1.1 s after this call.
     */
    Status startMeasurement();

    /**
     * Stop continuous measurement (cmd 0x0104). Non-blocking: sets a 1 s idle
     * guard so startMeasurement() will reject calls made too soon.
     * Use isIdleReady() to poll or rely on isDataReadyByTime() for timing.
     */
    Status stopMeasurement();

    /// True once the 1 s stop-cooldown after stopMeasurement() has elapsed.
    bool isIdleReady() const;

    /// Poll the sensor for data-ready flag (cmd 0x0202). out_ready: 0x01 = ready.
    Status getDataReady(uint8_t &out_ready);

    /// True if enough time has passed since startMeasurement() (~1.1 s).
    bool isDataReadyByTime() const;

    // ── Measurement reads ──────────────────────────────────────────────────

    /// SEN66 / SEN63C: PM + RH + T + VOC + NOx + CO₂  (cmd 0x0300, 9 words)
    Status readMeasurement(SEN66Measured &out);

    /// SEN65: PM + RH + T + VOC + NOx  (cmd 0x0446, 8 words)
    Status readMeasurementSEN65(SEN65Measured &out);

    /// SEN68: PM + RH + T + VOC + NOx + HCHO  (cmd 0x0467, 9 words)
    Status readMeasurementSEN68(SEN68Measured &out);

    /// Particle number concentrations (cmd 0x0316, 5 words; all PM-capable variants)
    Status readNumberConcentration(NumberConcentration &out);

    /**
     * Raw sensor ticks before algorithm processing (5 words).
     * @param sen66_variant  true → uses cmd 0x0405 (SEN66/SEN63C);
     *                       false → uses cmd 0x0455 (SEN65/SEN68)
     */
    Status readRawValues(RawValues &out, bool sen66_variant = false);

    // ── Device information ─────────────────────────────────────────────────

    Status readSerialNumber(String &out);
    Status readProductName(String &out);
    Status getVersion(FirmwareVersion &out);
    Status readDeviceStatus(uint32_t &out_status);
    Status getDeviceStatus(DeviceStatus &out);
    Status readAndClearDeviceStatus(uint32_t &out_status);
    Status getAndClearDeviceStatus(DeviceStatus &out);

    /// Full device reset — equivalent to power cycle. Re-call begin() afterwards.
    Status softReset();

    // ── Fan cleaning ───────────────────────────────────────────────────────

    /**
     * Trigger fan-cleaning cycle (cmd 0x5607). Only call in Idle mode.
     * Cleaning runs ~10 s in the background; wait ≥10 s before restarting
     * a measurement. Must stop measurement before calling.
     */
    Status startFanCleaning();

    // ── Temperature compensation (cmd 0x60B2) ─────────────────────────────

    /**
     * Set one of 5 additive temperature-offset slots.
     * @param offset_m200   Offset in 1/200 °C units  (e.g. +1 °C → pass 200)
     * @param slope_10k     Slope factor × 10 000      (e.g. 0.001 → pass 10)
     * @param time_const_s  Smoothing time constant (seconds)
     * @param slot          Slot index 0–4
     */
    Status setTemperatureOffset(int16_t offset_m200, int16_t slope_10k,
                                uint16_t time_const_s, uint8_t slot);

    /// Read back the temperature offset parameters currently stored in the sensor.
    Status getTemperatureOffset(int16_t &offset_m200, int16_t &slope_10k,
                                uint16_t &time_const_s, uint16_t &slot_out);

    // ── Temperature acceleration (cmd 0x6100) ─────────────────────────────

    /**
     * Set RH/T acceleration parameters (k, p, t1, t2 — see datasheet §4.3.5).
     * All four values are dimensionless uint16 as per the datasheet encoding.
     */
    Status setTemperatureAcceleration(uint16_t k, uint16_t p,
                                      uint16_t t1, uint16_t t2);

    // ── VOC algorithm tuning (cmd 0x60D0) ─────────────────────────────────

    Status setVocTuning(const VocTuning &params);
    Status getVocTuning(VocTuning &out);

    // ── NOx algorithm tuning (cmd 0x60E1) ─────────────────────────────────

    Status setNoxTuning(const NoxTuning &params);
    Status getNoxTuning(NoxTuning &out);

    // ── VOC algorithm state (cmd 0x6181) ──────────────────────────────────

    /// Save 8-byte VOC algorithm state (to restore across power cycles).
    Status setVocState(const uint8_t state[8]);

    /// Retrieve current 8-byte VOC algorithm state.
    Status getVocState(uint8_t state_out[8]);

    // ── CO₂ calibration  (SEN63C / SEN66 only — SEN68 has no CO₂) ─────────

    /**
     * Forced CO₂ recalibration (FRC). cmd 0x6707.
     * Call while measurement is running. Blocks 500 ms for sensor processing.
     * @param target_ppm   Known reference CO₂ concentration in ppm
     * @param correction   Signed correction applied by the sensor (ppm).
     *                     Returns Status::INVALID_ARG if FRC failed (no reference signal).
     */
    Status forcedCO2Recalibration(uint16_t target_ppm, int16_t &correction);

    /// Enable/disable CO₂ automatic self-calibration (ASC). cmd 0x6711.
    Status setCO2AutoSelfCalibration(bool enable);
    Status getCO2AutoSelfCalibration(bool &enabled);

    // ── Ambient pressure / altitude (cmd 0x6720 / 0x6736) ─────────────────

    /// Compensate CO₂ readings for ambient pressure (700–1200 hPa). cmd 0x6720.
    Status setAmbientPressure(uint16_t pressure_hPa);
    Status getAmbientPressure(uint16_t &out_hPa);

    /// Set installation altitude in metres for automatic pressure compensation. cmd 0x6736.
    Status setSensorAltitude(uint16_t altitude_m);
    Status getSensorAltitude(uint16_t &out_m);

    // ── SHT heater (cmd 0x6765 / 0x6790) ──────────────────────────────────

    /**
     * Activate the integrated RH/T heater for one shot (~200 mW, ~1 s).
     * cmd 0x6765. Only call in Idle mode. Removes condensation from the RH/T sensor.
     * Call getShtHeaterMeasurements() immediately after to read heater results.
     */
    Status activateShtHeater();

    /// Read RH/T values taken during the heater activation. cmd 0x6790.
    Status getShtHeaterMeasurements(ShtHeaterData &out);

    // ── Low-level access ───────────────────────────────────────────────────

    /// Write a 2-byte command + optional data words (each CRC-protected) over I²C.
    Status sendCommand(uint16_t cmd, const uint16_t *words = nullptr,
                       size_t word_count = 0);

    /// Read N 16-bit words (each CRC-checked) from the sensor.
    Status readWords(uint16_t *out_words, size_t word_count, uint32_t wait_ms = 0);

    uint8_t address() const { return address_; }
    void    setAddress(uint8_t addr) { address_ = addr; }
    void    setDebug(Print *p) { dbg_ = p; }

private:
    TwoWire      &wire_;
    uint8_t       address_;
    bool          initialized_ = false;
    Print        *dbg_         = nullptr;
    unsigned long dataReadyBy_ = 0; ///< millis() timestamp: when first data is expected
    unsigned long idleUntil_   = 0; ///< millis() timestamp: when idle cooldown expires

    static uint8_t crc8(const uint8_t *data, size_t len);
    Status tx_(const uint8_t *bytes, size_t len, bool sendStop = true);
    Status rx_(uint8_t *bytes, size_t len, uint32_t wait_ms = 0);

    /// Read 16 words of ASCII from cmd and assemble into a String (for serial/product name).
    Status readAsciiWords_(uint16_t cmd, String &out);

    // Scaling helpers — all return NAN for sentinel "unknown" values
    static float scaleU10_(uint16_t v)  { return (v == 0xFFFF) ? NAN : v / 10.0f; }
    static float scaleI100_(int16_t v)  { return (v == (int16_t)0x7FFF) ? NAN : v / 100.0f; }
    static float scaleI200_(int16_t v)  { return (v == (int16_t)0x7FFF) ? NAN : v / 200.0f; }
    static float scaleI10_(int16_t v)   { return (v == (int16_t)0x7FFF) ? NAN : v / 10.0f; }
    static float scaleCO2_(uint16_t v)  { return (v == 0xFFFF) ? NAN : (float)v; }
};

} // namespace sen6x

// ── Device Status Register bit constants ──────────────────────────────────
namespace sen6x_bits
{
    constexpr uint32_t WARN_SPEED = (1u << 21); ///< Fan speed warning
    constexpr uint32_t ERR_FAN    = (1u << 4);  ///< Fan error
    constexpr uint32_t ERR_RHT    = (1u << 6);  ///< RH&T sensor error
    constexpr uint32_t ERR_GAS    = (1u << 7);  ///< Gas sensor error
    constexpr uint32_t ERR_CO2_2  = (1u << 9);  ///< CO₂-2 error
    constexpr uint32_t ERR_HCHO   = (1u << 10); ///< HCHO error
    constexpr uint32_t ERR_PM     = (1u << 11); ///< PM sensor error
    constexpr uint32_t ERR_CO2_1  = (1u << 12); ///< CO₂-1 error
}
