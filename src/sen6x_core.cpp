/**
 * @file sen6x_core.cpp
 * @brief Core I²C driver implementation for Sensirion SEN6x environmental sensor family.
 *
 * Based on: Sensirion SEN6x Datasheet – Version 0.92 (December 2025)
 *           and official Sensirion embedded drivers for SEN65, SEN66, SEN68.
 *
 * @author  Arvind S.A.
 * @date    2025-10-05 (updated 2026-04-20)
 */

#include "sen6x_core.h"

namespace sen6x
{

static constexpr uint8_t CRC8_POLY = 0x31;
static constexpr uint8_t CRC8_INIT = 0xFF;

// ── CRC & I²C primitives ──────────────────────────────────────────────────

uint8_t Core::crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = CRC8_INIT;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? ((crc << 1) ^ CRC8_POLY) : (crc << 1);
    }
    return crc;
}

Status Core::tx_(const uint8_t *bytes, size_t len, bool sendStop)
{
    wire_.beginTransmission(address_);
    size_t w = wire_.write(bytes, len);
    uint8_t err = wire_.endTransmission(sendStop);
    if (w != len || err != 0)
        return Status::I2C_ERROR;
    return Status::OK;
}

Status Core::rx_(uint8_t *bytes, size_t len, uint32_t wait_ms)
{
    if (wait_ms) delay(wait_ms);
    int r = wire_.requestFrom((int)address_, (int)len);
    if (r != (int)len) return Status::I2C_ERROR;
    for (size_t i = 0; i < len; ++i)
        bytes[i] = wire_.read();
    return Status::OK;
}

// ── Public low-level ──────────────────────────────────────────────────────

Status Core::sendCommand(uint16_t cmd, const uint16_t *words, size_t word_count)
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    uint8_t buf[2 + word_count * 3];
    buf[0] = static_cast<uint8_t>(cmd >> 8);
    buf[1] = static_cast<uint8_t>(cmd & 0xFF);

    for (size_t i = 0; i < word_count; ++i) {
        uint8_t hi = static_cast<uint8_t>(words[i] >> 8);
        uint8_t lo = static_cast<uint8_t>(words[i] & 0xFF);
        uint8_t pair[2] = {hi, lo};
        size_t base = 2 + i * 3;
        buf[base + 0] = hi;
        buf[base + 1] = lo;
        buf[base + 2] = crc8(pair, 2);
    }
    return tx_(buf, sizeof(buf), true);
}

Status Core::readWords(uint16_t *out_words, size_t word_count, uint32_t wait_ms)
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    if (wait_ms) delay(wait_ms);

    const size_t bytes_expected = word_count * 3;
    int r = wire_.requestFrom((int)address_, (int)bytes_expected);
    if (r != (int)bytes_expected) {
        if (dbg_)
            dbg_->printf("[sen6x] requestFrom %u -> %d\n",
                         (unsigned)bytes_expected, r);
        return Status::I2C_ERROR;
    }

    for (size_t i = 0; i < word_count; ++i) {
        uint8_t hi  = wire_.read();
        uint8_t lo  = wire_.read();
        uint8_t crc = wire_.read();
        uint8_t pair[2] = {hi, lo};
        if (crc8(pair, 2) != crc) {
            if (dbg_)
                dbg_->printf("[sen6x] CRC mismatch word %u\n", (unsigned)i);
            return Status::CRC_ERROR;
        }
        out_words[i] = (static_cast<uint16_t>(hi) << 8) | lo;
    }
    return Status::OK;
}

// ── Private helper ────────────────────────────────────────────────────────

Status Core::readAsciiWords_(uint16_t cmd, String &out)
{
    Status s = sendCommand(cmd);
    if (s != Status::OK) return s;
    delay(20);

    constexpr size_t WORDS = 16; // 32 ASCII bytes
    uint16_t w[WORDS] = {};
    s = readWords(w, WORDS);
    if (s != Status::OK) return s;

    char buf[33];
    size_t idx = 0;
    for (size_t i = 0; i < WORDS && idx < 32; ++i) {
        uint8_t hi = static_cast<uint8_t>(w[i] >> 8);
        uint8_t lo = static_cast<uint8_t>(w[i] & 0xFF);
        if (hi == '\0') break;
        buf[idx++] = static_cast<char>(hi);
        if (idx >= 32 || lo == '\0') break;
        buf[idx++] = static_cast<char>(lo);
    }
    buf[idx] = '\0';
    out = String(buf);
    return Status::OK;
}

static DeviceStatus decodeDSR(uint32_t raw)
{
    DeviceStatus d;
    d.raw              = raw;
    d.fan_speed_warning = raw & (1UL << 21);
    d.fan_error         = raw & (1UL << 4);
    d.rht_error         = raw & (1UL << 6);
    d.gas_error         = raw & (1UL << 7);
    d.co2_2_error       = raw & (1UL << 9);
    d.hcho_error        = raw & (1UL << 10);
    d.pm_error          = raw & (1UL << 11);
    d.co2_1_error       = raw & (1UL << 12);
    return d;
}

// ── Initialization ────────────────────────────────────────────────────────

Status Core::begin()
{
    wire_.beginTransmission(address_);
    uint8_t err = wire_.endTransmission(true);
    if (err != 0) {
        if (dbg_)
            dbg_->printf("[sen6x] Probe 0x%02X failed, err=%u\n", address_, err);
        return Status::I2C_ERROR;
    }
    initialized_ = true;
    return Status::OK;
}

// ── Measurement control ───────────────────────────────────────────────────

Status Core::startMeasurement()
{
    if (!initialized_) return Status::NOT_INITIALIZED;
    if (millis() < idleUntil_) return Status::TIMEOUT;

    constexpr uint16_t CMD = 0x0021;
    Status s = sendCommand(CMD);
    if (s != Status::OK) return s;
    delay(50);
    dataReadyBy_ = millis() + 1100;
    if (dbg_)
        dbg_->printf("[sen6x] Measurement started, data ready by %lu ms\n", dataReadyBy_);
    return Status::OK;
}

Status Core::stopMeasurement()
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    constexpr uint16_t CMD = 0x0104;
    Status s = sendCommand(CMD);
    if (s != Status::OK) return s;

    idleUntil_   = millis() + 1000; // guard: startMeasurement checks this
    dataReadyBy_ = 0;

    if (dbg_) dbg_->println("[sen6x] Measurement stopped");
    return Status::OK;
}

bool Core::isIdleReady() const
{
    return millis() >= idleUntil_;
}

Status Core::getDataReady(uint8_t &out_ready)
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    if (millis() < dataReadyBy_) {
        out_ready = 0x00;
        return Status::OK;
    }

    constexpr uint16_t CMD = 0x0202;
    Status s = sendCommand(CMD);
    if (s != Status::OK) return s;
    delay(20);

    uint8_t bytes[2];
    s = rx_(bytes, 2);
    if (s != Status::OK) return s;

    uint8_t crc;
    s = rx_(&crc, 1);
    if (s != Status::OK) return s;

    if (crc8(bytes, 2) != crc) return Status::CRC_ERROR;
    out_ready = bytes[1];
    return Status::OK;
}

bool Core::isDataReadyByTime() const
{
    return millis() >= dataReadyBy_;
}

// ── Measurement reads ─────────────────────────────────────────────────────

Status Core::readMeasurement(SEN66Measured &out)
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    constexpr uint16_t CMD = 0x0300;
    Status s = sendCommand(CMD);
    if (s != Status::OK) return s;
    delay(20);

    uint16_t w[9] = {};
    s = readWords(w, 9);
    if (s != Status::OK) return s;

    out.pm1_0       = scaleU10_(w[0]);
    out.pm2_5       = scaleU10_(w[1]);
    out.pm4_0       = scaleU10_(w[2]);
    out.pm10_0      = scaleU10_(w[3]);
    out.humidity    = scaleI100_((int16_t)w[4]);
    out.temperature = scaleI200_((int16_t)w[5]);
    out.voc_index   = scaleI10_((int16_t)w[6]);
    out.nox_index   = scaleI10_((int16_t)w[7]);
    out.co2         = scaleCO2_(w[8]);
    return Status::OK;
}

Status Core::readMeasurementSEN65(SEN65Measured &out)
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    constexpr uint16_t CMD = 0x0446;
    Status s = sendCommand(CMD);
    if (s != Status::OK) return s;
    delay(20);

    uint16_t w[8] = {};
    s = readWords(w, 8);
    if (s != Status::OK) return s;

    out.pm1_0       = scaleU10_(w[0]);
    out.pm2_5       = scaleU10_(w[1]);
    out.pm4_0       = scaleU10_(w[2]);
    out.pm10_0      = scaleU10_(w[3]);
    out.humidity    = scaleI100_((int16_t)w[4]);
    out.temperature = scaleI200_((int16_t)w[5]);
    out.voc_index   = scaleI10_((int16_t)w[6]);
    out.nox_index   = scaleI10_((int16_t)w[7]);
    return Status::OK;
}

Status Core::readMeasurementSEN68(SEN68Measured &out)
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    constexpr uint16_t CMD = 0x0467;
    Status s = sendCommand(CMD);
    if (s != Status::OK) return s;
    delay(20);

    uint16_t w[9] = {};
    s = readWords(w, 9);
    if (s != Status::OK) return s;

    out.pm1_0       = scaleU10_(w[0]);
    out.pm2_5       = scaleU10_(w[1]);
    out.pm4_0       = scaleU10_(w[2]);
    out.pm10_0      = scaleU10_(w[3]);
    out.humidity    = scaleI100_((int16_t)w[4]);
    out.temperature = scaleI200_((int16_t)w[5]);
    out.voc_index   = scaleI10_((int16_t)w[6]);
    out.nox_index   = scaleI10_((int16_t)w[7]);
    out.hcho        = scaleU10_(w[8]);
    return Status::OK;
}

Status Core::readNumberConcentration(NumberConcentration &out)
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    constexpr uint16_t CMD = 0x0316;
    Status s = sendCommand(CMD);
    if (s != Status::OK) return s;
    delay(20);

    uint16_t w[5] = {};
    s = readWords(w, 5);
    if (s != Status::OK) return s;

    out.pm0_5  = scaleU10_(w[0]);
    out.pm1_0  = scaleU10_(w[1]);
    out.pm2_5  = scaleU10_(w[2]);
    out.pm4_0  = scaleU10_(w[3]);
    out.pm10_0 = scaleU10_(w[4]);
    return Status::OK;
}

Status Core::readRawValues(RawValues &out, bool sen66_variant)
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    const uint16_t cmd = sen66_variant ? 0x0405 : 0x0455;
    Status s = sendCommand(cmd);
    if (s != Status::OK) return s;
    delay(20);

    uint16_t w[5] = {};
    s = readWords(w, 5);
    if (s != Status::OK) return s;

    out.humidity     = (int16_t)w[0];
    out.temperature  = (int16_t)w[1];
    out.voc          = w[2];
    out.nox          = w[3];
    out.co2_or_hcho  = w[4];
    return Status::OK;
}

// ── Device information ────────────────────────────────────────────────────

Status Core::readSerialNumber(String &out)
{
    return readAsciiWords_(0xD033, out);
}

Status Core::readProductName(String &out)
{
    return readAsciiWords_(0xD014, out);
}

Status Core::getVersion(FirmwareVersion &out)
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    constexpr uint16_t CMD = 0xD100;
    Status s = sendCommand(CMD);
    if (s != Status::OK) return s;
    delay(20);

    uint16_t w;
    s = readWords(&w, 1);
    if (s != Status::OK) return s;

    out.major = static_cast<uint8_t>(w >> 8);
    out.minor = static_cast<uint8_t>(w & 0xFF);
    return Status::OK;
}

Status Core::readDeviceStatus(uint32_t &out_status)
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    constexpr uint16_t CMD = 0xD206;
    Status s = sendCommand(CMD);
    if (s != Status::OK) return s;
    delay(20);

    uint16_t w[2];
    s = readWords(w, 2);
    if (s != Status::OK) return s;

    out_status = (static_cast<uint32_t>(w[0]) << 16) | w[1];
    if (dbg_)
        dbg_->printf("[sen6x] DSR=0x%08lX\n", (unsigned long)out_status);
    return Status::OK;
}

Status Core::getDeviceStatus(DeviceStatus &out)
{
    uint32_t raw = 0;
    Status s = readDeviceStatus(raw);
    if (s != Status::OK) return s;
    out = decodeDSR(raw);
    return Status::OK;
}

Status Core::readAndClearDeviceStatus(uint32_t &out_status)
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    constexpr uint16_t CMD = 0xD210;
    Status s = sendCommand(CMD);
    if (s != Status::OK) return s;
    delay(20);

    uint16_t w[2] = {};
    s = readWords(w, 2);
    if (s != Status::OK) return s;

    out_status = (static_cast<uint32_t>(w[0]) << 16) | w[1];
    return Status::OK;
}

Status Core::getAndClearDeviceStatus(DeviceStatus &out)
{
    uint32_t raw = 0;
    Status s = readAndClearDeviceStatus(raw);
    if (s != Status::OK) return s;
    out = decodeDSR(raw);
    return Status::OK;
}

Status Core::softReset()
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    constexpr uint16_t CMD = 0xD304;
    Status s = sendCommand(CMD);
    if (s != Status::OK) return s;

    delay(1200); // datasheet: device ready after 1.2 s
    initialized_ = false;
    idleUntil_   = 0;
    dataReadyBy_ = 0;
    if (dbg_) dbg_->println("[sen6x] Device reset");
    return Status::OK;
}

// ── Fan cleaning ──────────────────────────────────────────────────────────

Status Core::startFanCleaning()
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    constexpr uint16_t CMD = 0x5607;
    Status s = sendCommand(CMD);
    if (s != Status::OK) return s;
    delay(20);
    // Cleaning runs ~10 s in the background; caller must wait ≥10 s
    // before issuing startMeasurement().
    if (dbg_) dbg_->println("[sen6x] Fan cleaning started (~10 s)");
    return Status::OK;
}

// ── Temperature compensation ──────────────────────────────────────────────

Status Core::setTemperatureOffset(int16_t offset_m200, int16_t slope_10k,
                                   uint16_t time_const_s, uint8_t slot)
{
    if (!initialized_) return Status::NOT_INITIALIZED;
    if (slot > 4) return Status::INVALID_ARG;

    uint16_t w[4] = {
        (uint16_t)offset_m200,
        (uint16_t)slope_10k,
        time_const_s,
        (uint16_t)slot
    };
    Status s = sendCommand(0x60B2, w, 4);
    if (s != Status::OK) return s;
    delay(20);
    return Status::OK;
}

Status Core::getTemperatureOffset(int16_t &offset_m200, int16_t &slope_10k,
                                   uint16_t &time_const_s, uint16_t &slot_out)
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    Status s = sendCommand(0x60B2);
    if (s != Status::OK) return s;
    delay(20);

    uint16_t w[4] = {};
    s = readWords(w, 4);
    if (s != Status::OK) return s;

    offset_m200  = (int16_t)w[0];
    slope_10k    = (int16_t)w[1];
    time_const_s = w[2];
    slot_out     = w[3];
    return Status::OK;
}

Status Core::setTemperatureAcceleration(uint16_t k, uint16_t p,
                                         uint16_t t1, uint16_t t2)
{
    if (!initialized_) return Status::NOT_INITIALIZED;
    uint16_t w[4] = {k, p, t1, t2};
    Status s = sendCommand(0x6100, w, 4);
    if (s != Status::OK) return s;
    delay(20);
    return Status::OK;
}

// ── VOC / NOx tuning ──────────────────────────────────────────────────────

static Status setTuning_(Core &core, uint16_t cmd, const VocTuning &p)
{
    uint16_t w[6] = {
        (uint16_t)p.index_offset,
        (uint16_t)p.learning_time_offset_h,
        (uint16_t)p.learning_time_gain_h,
        (uint16_t)p.gating_max_duration_min,
        (uint16_t)p.std_initial,
        (uint16_t)p.gain_factor
    };
    Status s = core.sendCommand(cmd, w, 6);
    if (s != Status::OK) return s;
    delay(20);
    return Status::OK;
}

static Status getTuning_(Core &core, uint16_t cmd, VocTuning &out)
{
    Status s = core.sendCommand(cmd);
    if (s != Status::OK) return s;
    delay(20);
    uint16_t w[6] = {};
    s = core.readWords(w, 6);
    if (s != Status::OK) return s;
    out.index_offset            = (int16_t)w[0];
    out.learning_time_offset_h  = (int16_t)w[1];
    out.learning_time_gain_h    = (int16_t)w[2];
    out.gating_max_duration_min = (int16_t)w[3];
    out.std_initial             = (int16_t)w[4];
    out.gain_factor             = (int16_t)w[5];
    return Status::OK;
}

Status Core::setVocTuning(const VocTuning &p) { return setTuning_(*this, 0x60D0, p); }
Status Core::getVocTuning(VocTuning &out)      { return getTuning_(*this, 0x60D0, out); }
Status Core::setNoxTuning(const NoxTuning &p)  { return setTuning_(*this, 0x60E1, p); }
Status Core::getNoxTuning(NoxTuning &out)       { return getTuning_(*this, 0x60E1, out); }

// ── VOC algorithm state ───────────────────────────────────────────────────

Status Core::setVocState(const uint8_t state[8])
{
    if (!initialized_) return Status::NOT_INITIALIZED;
    uint16_t w[4];
    for (size_t i = 0; i < 4; ++i)
        w[i] = (static_cast<uint16_t>(state[i * 2]) << 8) | state[i * 2 + 1];
    Status s = sendCommand(0x6181, w, 4);
    if (s != Status::OK) return s;
    delay(20);
    return Status::OK;
}

Status Core::getVocState(uint8_t state_out[8])
{
    if (!initialized_) return Status::NOT_INITIALIZED;
    Status s = sendCommand(0x6181);
    if (s != Status::OK) return s;
    delay(20);
    uint16_t w[4] = {};
    s = readWords(w, 4);
    if (s != Status::OK) return s;
    for (size_t i = 0; i < 4; ++i) {
        state_out[i * 2]     = static_cast<uint8_t>(w[i] >> 8);
        state_out[i * 2 + 1] = static_cast<uint8_t>(w[i] & 0xFF);
    }
    return Status::OK;
}

// ── CO₂ calibration ───────────────────────────────────────────────────────

Status Core::forcedCO2Recalibration(uint16_t target_ppm, int16_t &correction)
{
    if (!initialized_) return Status::NOT_INITIALIZED;

    Status s = sendCommand(0x6707, &target_ppm, 1);
    if (s != Status::OK) return s;
    delay(500); // sensor processing time per datasheet

    uint16_t raw;
    s = readWords(&raw, 1);
    if (s != Status::OK) return s;

    if (raw == 0x7FFF) return Status::INVALID_ARG; // FRC failed: no valid reference
    correction = static_cast<int16_t>(raw - 0x8000);
    return Status::OK;
}

Status Core::setCO2AutoSelfCalibration(bool enable)
{
    if (!initialized_) return Status::NOT_INITIALIZED;
    uint16_t param = enable ? 1u : 0u;
    Status s = sendCommand(0x6711, &param, 1);
    if (s != Status::OK) return s;
    delay(20);
    return Status::OK;
}

Status Core::getCO2AutoSelfCalibration(bool &enabled)
{
    if (!initialized_) return Status::NOT_INITIALIZED;
    Status s = sendCommand(0x6711);
    if (s != Status::OK) return s;
    delay(20);
    uint16_t raw;
    s = readWords(&raw, 1);
    if (s != Status::OK) return s;
    enabled = (raw != 0);
    return Status::OK;
}

// ── Ambient pressure / altitude ───────────────────────────────────────────

Status Core::setAmbientPressure(uint16_t pressure_hPa)
{
    if (!initialized_) return Status::NOT_INITIALIZED;
    Status s = sendCommand(0x6720, &pressure_hPa, 1);
    if (s != Status::OK) return s;
    delay(20);
    return Status::OK;
}

Status Core::getAmbientPressure(uint16_t &out_hPa)
{
    if (!initialized_) return Status::NOT_INITIALIZED;
    Status s = sendCommand(0x6720);
    if (s != Status::OK) return s;
    delay(20);
    return readWords(&out_hPa, 1);
}

Status Core::setSensorAltitude(uint16_t altitude_m)
{
    if (!initialized_) return Status::NOT_INITIALIZED;
    Status s = sendCommand(0x6736, &altitude_m, 1);
    if (s != Status::OK) return s;
    delay(20);
    return Status::OK;
}

Status Core::getSensorAltitude(uint16_t &out_m)
{
    if (!initialized_) return Status::NOT_INITIALIZED;
    Status s = sendCommand(0x6736);
    if (s != Status::OK) return s;
    delay(20);
    return readWords(&out_m, 1);
}

// ── SHT heater ───────────────────────────────────────────────────────────

Status Core::activateShtHeater()
{
    if (!initialized_) return Status::NOT_INITIALIZED;
    Status s = sendCommand(0x6765);
    if (s != Status::OK) return s;
    delay(20);
    return Status::OK;
}

Status Core::getShtHeaterMeasurements(ShtHeaterData &out)
{
    if (!initialized_) return Status::NOT_INITIALIZED;
    Status s = sendCommand(0x6790);
    if (s != Status::OK) return s;
    delay(20);
    uint16_t w[2] = {};
    s = readWords(w, 2);
    if (s != Status::OK) return s;
    out.humidity    = scaleI100_((int16_t)w[0]);
    out.temperature = scaleI200_((int16_t)w[1]);
    return Status::OK;
}

} // namespace sen6x
