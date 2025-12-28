#include <Arduino.h>
#include <Wire.h>
#include <cmath>
#include "shared.h"

// ======================================
// I2C pins
// ======================================
static constexpr uint8_t I2C_SDA = 21;
static constexpr uint8_t I2C_SCL = 22;

// ======================================
// Two TCA9548A addresses
// ======================================
static constexpr uint8_t TCA0 = 0x70; // A2 A1 A0 = 000
static constexpr uint8_t TCA1 = 0x71; // A2 A1 A0 = 001

// ======================================
// MCP45HV51 addresses (A1/A0 = 00..11)
// ======================================
static constexpr uint8_t MCP_BASE = 0x3C;        // mcp 1 -> 0x3C ... mcp 4 -> 0x3F
static constexpr uint8_t MCP_WIPER0_ADDR = 0x00; // write register address
static constexpr uint8_t MCP_READ_WIPER0_CMD = 0x0C; // read wiper0 command for AD=0x00

// ======================================
// I2C helpers
// ======================================
static bool i2cAck(uint8_t addr7) {
  Wire.beginTransmission(addr7);
  return Wire.endTransmission() == 0;
}

static bool tcaSelect(uint8_t tcaAddr, uint8_t ch0to7) {
  Wire.beginTransmission(tcaAddr);
  Wire.write(uint8_t(1u << ch0to7));
  bool ok = Wire.endTransmission() == 0;
  if (ok) {
    delayMicroseconds(100);  // Let bus settle after TCA switch
  }
  return ok;
}

static void tcaDisableAll(uint8_t tcaAddr) {
  Wire.beginTransmission(tcaAddr);
  Wire.write(uint8_t(0x00));
  Wire.endTransmission();
  delayMicroseconds(50);
}

static bool writeMcpWiper0(uint8_t mcpAddr, uint8_t wiper) {
  Wire.beginTransmission(mcpAddr);
  Wire.write(MCP_WIPER0_ADDR);
  Wire.write(wiper);
  return Wire.endTransmission() == 0;
}

// Write with retry
static bool writeMcpWiper0Retry(uint8_t mcpAddr, uint8_t wiper, int retries = 3) {
  for (int i = 0; i < retries; i++) {
    if (writeMcpWiper0(mcpAddr, wiper)) return true;
    delayMicroseconds(200);
  }
  return false;
}

static bool readMcpWiper0(uint8_t mcpAddr, uint8_t &outWiper) {
  // Send read command, then read two bytes back
  Wire.beginTransmission(mcpAddr);
  Wire.write(MCP_READ_WIPER0_CMD);
  if (Wire.endTransmission(false) != 0) { // repeated start
    return false;
  }

  int n = Wire.requestFrom((int)mcpAddr, 2);
  if (n != 2) {
    return false;
  }
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  (void)msb;           // for 8-bit devices, wiper is effectively in lsb
  outWiper = lsb;
  return true;
}

// ======================================
// Channel mapping helpers
// ======================================
// Map channel 1..16 -> (tcaAddr, tcaChannel0..7)
static bool mapChannel16(int ch1to16, uint8_t &tcaAddrOut, uint8_t &tcaChOut) {
  if (ch1to16 < 1 || ch1to16 > 16) return false;
  if (ch1to16 <= 8) {
    tcaAddrOut = TCA0;
    tcaChOut   = (uint8_t)(ch1to16 - 1); // 1->0 ... 8->7
  } else {
    tcaAddrOut = TCA1;
    tcaChOut   = (uint8_t)(ch1to16 - 9); // 9->0 ... 16->7
  }
  return true;
}

// Map mcp 1..4 -> I2C address 0x3C..0x3F
static bool mapMcp4(int dev1to4, uint8_t &mcpAddrOut) {
  if (dev1to4 < 1 || dev1to4 > 4) return false;
  mcpAddrOut = MCP_BASE + (uint8_t)(dev1to4 - 1);
  return true;
}

// ======================================
// MCP write with channel selection
// ======================================
// ch1to16: TCA channel (1-16), dev1to4: MCP device on that channel (1-4), val: wiper value (0-255)
static bool writeMcpValue(int ch1to16, int dev1to4, uint8_t val) {
  uint8_t tcaAddr, tcaCh, mcpAddr;
  if (!mapChannel16(ch1to16, tcaAddr, tcaCh)) {
    Serial.printf("[CTRL] ERR channel %d out of range 1..16\n", ch1to16);
    return false;
  }
  if (!mapMcp4(dev1to4, mcpAddr)) {
    Serial.printf("[CTRL] ERR mcp %d out of range 1..4\n", dev1to4);
    return false;
  }

  if (!i2cAck(tcaAddr)) {
    Serial.printf("[CTRL] FAIL: TCA 0x%02X NOACK\n", tcaAddr);
    return false;
  }
  if (!tcaSelect(tcaAddr, tcaCh)) {
    Serial.printf("[CTRL] FAIL: TCA 0x%02X select ch%u\n", tcaAddr, tcaCh);
    tcaDisableAll(tcaAddr);
    return false;
  }
  if (!i2cAck(mcpAddr)) {
    Serial.printf("[CTRL] FAIL: ch=%d mcp=%d addr=0x%02X NOACK\n", ch1to16, dev1to4, mcpAddr);
    tcaDisableAll(tcaAddr);
    return false;
  }

  bool ok = writeMcpWiper0(mcpAddr, val);
  tcaDisableAll(tcaAddr);

  if (ok) {
    Serial.printf("[CTRL] OK WRITE ch=%d mcp=%d val=%d\n", ch1to16, dev1to4, (int)val);
  } else {
    Serial.printf("[CTRL] FAIL WRITE ch=%d mcp=%d val=%d\n", ch1to16, dev1to4, (int)val);
  }
  return ok;
}

// Read MCP wiper value
static bool readMcpValue(int ch1to16, int dev1to4, uint8_t &outVal) {
  uint8_t tcaAddr, tcaCh, mcpAddr;
  if (!mapChannel16(ch1to16, tcaAddr, tcaCh)) return false;
  if (!mapMcp4(dev1to4, mcpAddr)) return false;

  if (!i2cAck(tcaAddr)) return false;
  if (!tcaSelect(tcaAddr, tcaCh)) {
    tcaDisableAll(tcaAddr);
    return false;
  }
  if (!i2cAck(mcpAddr)) {
    tcaDisableAll(tcaAddr);
    return false;
  }

  bool ok = readMcpWiper0(mcpAddr, outVal);
  tcaDisableAll(tcaAddr);
  return ok;
}

// ======================================
// Local state cache for change detection
// ======================================
static EqSharedSnapshot lastSnapshot;
static bool snapshotInitialized = false;

// ======================================
// MCP45HV51 I2C addresses (base 0x3C)
// ======================================
// A1=0, A0=0 → 0x3C
// A1=0, A0=1 → 0x3D
// A1=1, A0=0 → 0x3E
// A1=1, A0=1 → 0x3F
static constexpr uint8_t MCP_A00 = 0x3C;  // A1=0, A0=0
static constexpr uint8_t MCP_A01 = 0x3D;  // A1=0, A0=1
static constexpr uint8_t MCP_A10 = 0x3E;  // A1=1, A0=0
static constexpr uint8_t MCP_A11 = 0x3F;  // A1=1, A0=1

// ======================================
// Hardware mapping structures
// ======================================

// Filter parameter MCPs on a specific TCA channel
struct FilterMcpMapping {
  uint8_t tcaChannel;       // TCA channel for freq/Q MCPs
  uint8_t freqMcp1;         // First frequency MCP address
  uint8_t freqMcp2;         // Second frequency MCP address
  uint8_t qMcp1;            // First Q MCP address
  uint8_t qMcp2;            // Second Q MCP address
  uint8_t gainTcaChannel;   // TCA channel for gain MCP
  uint8_t gainMcp;          // Gain MCP address
};

// Right channel filter mappings (TCA0 = 0x70)
// CH0-CH5 are on TCA0
// Band 0: Peak 1 - CH0, gain on CH4
// Band 1: Peak 2 - CH1, gain on CH4
// Band 2: Peak 3 - CH2, gain on CH4
// Band 3: Low Shelf - CH3, gain on CH4
// Freq MCPs: A01 (0x3D) + A10 (0x3E)
// Q MCPs: A11 (0x3F) + A00 (0x3C)
static const FilterMcpMapping rightFilters[4] = {
  // Peak 1: CH0, freq=0x3D+0x3E, Q=0x3F+0x3C, gain CH4 0x3C
  { 0, MCP_A01, MCP_A10, MCP_A11, MCP_A00, 4, MCP_A00 },
  // Peak 2: CH1, freq=0x3D+0x3E, Q=0x3F+0x3C, gain CH4 0x3E
  { 1, MCP_A01, MCP_A10, MCP_A11, MCP_A00, 4, MCP_A10 },
  // Peak 3: CH2, freq=0x3D+0x3E, Q=0x3F+0x3C, gain CH4 0x3D
  { 2, MCP_A01, MCP_A10, MCP_A11, MCP_A00, 4, MCP_A01 },
  // Low Shelf: CH3, freq=0x3D+0x3E, Q=0x3F+0x3C, gain CH4 0x3F
  { 3, MCP_A01, MCP_A10, MCP_A11, MCP_A00, 4, MCP_A11 },
};

// Left channel filter mappings
// CH6 = TCA0 ch6, CH7 = TCA0 ch7 (Peak 1 & 2 on TCA0!)
// CH8 = TCA1 ch0, CH9 = TCA1 ch1 (Peak 3 & Low Shelf on TCA1)
// CH10 = TCA1 ch2 (gain MCPs for all left filters)
// Freq MCPs: A01 (0x3D) + A10 (0x3E)
// Q MCPs: A11 (0x3F) + A00 (0x3C)

// We need separate mappings since left filters span two TCAs
// Peak 1 & 2 use TCA0, Peak 3 & Low Shelf use TCA1
// But all gains are on TCA1 ch2

// Left Peak 1: TCA0 ch6, gain on TCA1 ch2
// Left Peak 2: TCA0 ch7, gain on TCA1 ch2  
// Left Peak 3: TCA1 ch0, gain on TCA1 ch2
// Left Low Shelf: TCA1 ch1, gain on TCA1 ch2

// For left channel, we need to track which TCA to use for freq/Q vs gain
struct LeftFilterMapping {
  uint8_t freqQTca;         // TCA for freq/Q (TCA0 or TCA1)
  uint8_t tcaChannel;       // TCA channel for freq/Q MCPs
  uint8_t freqMcp1;         // First frequency MCP address
  uint8_t freqMcp2;         // Second frequency MCP address
  uint8_t qMcp1;            // First Q MCP address
  uint8_t qMcp2;            // Second Q MCP address
  uint8_t gainTca;          // TCA for gain (always TCA1)
  uint8_t gainTcaChannel;   // TCA channel for gain MCP (always ch2)
  uint8_t gainMcp;          // Gain MCP address
};

static const LeftFilterMapping leftFiltersNew[4] = {
  // Peak 1: TCA0 ch6, freq=0x3D+0x3E, Q=0x3F+0x3C, gain TCA1 ch2 0x3C
  { TCA0, 6, MCP_A01, MCP_A10, MCP_A11, MCP_A00, TCA1, 2, MCP_A00 },
  // Peak 2: TCA0 ch7, freq=0x3D+0x3E, Q=0x3F+0x3C, gain TCA1 ch2 0x3E
  { TCA0, 7, MCP_A01, MCP_A10, MCP_A11, MCP_A00, TCA1, 2, MCP_A10 },
  // Peak 3: TCA1 ch0, freq=0x3D+0x3E, Q=0x3F+0x3C, gain TCA1 ch2 0x3D
  { TCA1, 0, MCP_A01, MCP_A10, MCP_A11, MCP_A00, TCA1, 2, MCP_A01 },
  // Low Shelf: TCA1 ch1, freq=0x3D+0x3E, Q=0x3F+0x3C, gain TCA1 ch2 0x3F
  { TCA1, 1, MCP_A01, MCP_A10, MCP_A11, MCP_A00, TCA1, 2, MCP_A11 },
};

// Overall channel gain - BOTH on TCA0 CH5, different MCPs!
// Right: TCA0 CH5, MCP A00 (0x3C)
// Left:  TCA0 CH5, MCP A01 (0x3D)
static constexpr uint8_t RIGHT_OVERALL_GAIN_CH = 5;   // TCA0 ch5
static constexpr uint8_t RIGHT_OVERALL_GAIN_MCP = MCP_A00;  // 0x3C
static constexpr uint8_t LEFT_OVERALL_GAIN_CH = 5;    // TCA0 ch5 (same channel!)
static constexpr uint8_t LEFT_OVERALL_GAIN_MCP = MCP_A01;   // 0x3D

// ======================================
// Value conversion functions
// ======================================

// Hardware filter frequency ranges (measured with wiper 0-255)
// Each filter has different actual frequency range due to component variations
// Format: {minHz (wiper=255), maxHz (wiper=0)} - inverted mapping
struct FilterFreqRange {
  float minHz;  // frequency at wiper 255 (max resistance)
  float maxHz;  // frequency at wiper 0 (min resistance)
};

// Measured frequency ranges for each hardware filter - RIGHT channel
// Hardware filter indices: 0=Peak1, 1=Peak2, 2=Peak3, 3=LowShelf
static const FilterFreqRange hwFilterFreqRangesRight[4] = {
  { 14.7f, 254.0f },     // Peak 1 - measured: wiper 255 = 14.7 Hz, wiper 0 = 254 Hz
  { 249.0f, 4650.0f },   // Peak 2 - measured: wiper 255 = 249 Hz, wiper 0 = 4650 Hz
  { 958.0f, 16500.0f },  // Peak 3 - measured: wiper 255 = 958 Hz, wiper 0 = 16500 Hz
  { 249.0f, 4650.0f },   // Low Shelf - TODO: measure (using Peak 2 values as placeholder)
};

// Measured frequency ranges for each hardware filter - LEFT channel
static const FilterFreqRange hwFilterFreqRangesLeft[4] = {
  { 20.5f, 352.0f },     // Peak 1 - measured: wiper 255 = 20.5 Hz, wiper 0 = 352 Hz
  { 257.0f, 4650.0f },   // Peak 2 - measured: wiper 255 = 257 Hz, wiper 0 = 4650 Hz
  { 954.0f, 16500.0f },  // Peak 3 - measured: wiper 255 = 954 Hz, wiper 0 = 16500 Hz
  { 257.0f, 4650.0f },   // Low Shelf - TODO: measure (using Peak 2 values as placeholder)
};

// Convert frequency to wiper value with hardware compensation
// Uses inverse (1/f) relationship based on f = 1/(2*pi*R*C)
// Since wiper controls R and f ∝ 1/R, we need inverse mapping
// INVERTED: low freq -> high wiper, high freq -> low wiper
// isLeftChannel: true for left, false for right
static uint8_t freqToWiperCompensated(uint16_t freqHz, uint8_t hwFilterIndex, bool isLeftChannel) {
  if (hwFilterIndex >= 4) hwFilterIndex = 0;
  
  const FilterFreqRange &range = isLeftChannel ? 
    hwFilterFreqRangesLeft[hwFilterIndex] : hwFilterFreqRangesRight[hwFilterIndex];
  
  // Clamp input frequency to hardware's actual range
  float freq = (float)freqHz;
  if (freq < range.minHz) freq = range.minHz;
  if (freq > range.maxHz) freq = range.maxHz;
  
  // RC filter formula: f = 1/(2*pi*R*C)
  // Wiper position is proportional to R, so wiper ∝ 1/f
  // 
  // At wiper=255 (max R): f_min → 1/f_min (largest)
  // At wiper=0 (min R):   f_max → 1/f_max (smallest)
  //
  // Linear interpolation in 1/f space:
  // wiper = 255 * (1/f - 1/f_max) / (1/f_min - 1/f_max)
  
  float invF = 1.0f / freq;
  float invFmin = 1.0f / range.minHz;  // largest, at wiper 255
  float invFmax = 1.0f / range.maxHz;  // smallest, at wiper 0
  
  float normalized = (invF - invFmax) / (invFmin - invFmax);
  int wiper = (int)(normalized * 255.0f);
  
  if (wiper < 0) wiper = 0;
  if (wiper > 255) wiper = 255;
  return (uint8_t)wiper;
}

// Original frequency conversion (without compensation) - kept for reference/master gain
// Convert frequency (0-30000 Hz) to wiper value (0-255)
// Linear mapping: 15000 Hz = 128 (midpoint)
// INVERTED: 0 Hz -> 255, 30000 Hz -> 0
static uint8_t freqToWiper(uint16_t freqHz) {
  if (freqHz > 30000) freqHz = 30000;
  uint32_t wiper = ((uint32_t)freqHz * 255) / 30000;
  return (uint8_t)(255 - wiper);  // Invert!
}

// Convert Q (0.1 - 10.0, stored as q10: 1-100) to wiper value (0-255)
// Logarithmic mapping: Q=0.1 -> 0, Q=1.0 -> 127, Q=10.0 -> 255
static uint8_t qToWiper(uint16_t q10) {
  // q10 is Q * 10, so Q=0.1 -> q10=1, Q=1.0 -> q10=10, Q=10.0 -> q10=100
  if (q10 < 1) q10 = 1;
  if (q10 > 100) q10 = 100;
  
  // Logarithmic mapping: log10(0.1)=-1, log10(1)=0, log10(10)=1
  // Map Q range [0.1, 10] to wiper [0, 255] with Q=1.0 at midpoint (127)
  // log10(Q) ranges from -1 to +1, map to 0..255
  float Q = (float)q10 / 10.0f;
  float logQ = log10f(Q);  // -1 to +1
  float normalized = (logQ + 1.0f) / 2.0f;  // 0 to 1
  int wiper = (int)(normalized * 255.0f);
  if (wiper < 0) wiper = 0;
  if (wiper > 255) wiper = 255;
  return (uint8_t)wiper;
}

// Hardware filter gain ranges (measured at wiper 0 and 255)
// Format: {minGainDb (wiper=255), maxGainDb (wiper=0)} for bell filters (inverted)
struct FilterGainRange {
  float minGainDb;  // gain at wiper 255 (most negative / cut)
  float maxGainDb;  // gain at wiper 0 (most positive / boost)
};

// Measured gain ranges for each hardware filter - RIGHT channel
// Hardware filter indices: 0=Peak1, 1=Peak2, 2=Peak3, 3=LowShelf
static const FilterGainRange hwFilterGainRangesRight[4] = {
  { -10.0f, 8.5f },   // Peak 1 - measured
  { -11.0f, 11.5f },  // Peak 2 - measured
  { -11.5f, 12.0f },  // Peak 3 - measured
  { -11.0f, 11.5f },  // Low Shelf - TODO: measure (using Peak 2 values)
};

// Measured gain ranges for each hardware filter - LEFT channel
static const FilterGainRange hwFilterGainRangesLeft[4] = {
  { -7.7f, 7.0f },    // Peak 1 - measured
  { -11.0f, 11.7f },  // Peak 2 - measured
  { -10.0f, 12.0f },  // Peak 3 - measured
  { -11.0f, 11.7f },  // Low Shelf - TODO: measure (using Peak 2 values)
};

// Convert gain to wiper value with hardware compensation
// Maps website gain (-15 to +15 dB) to actual hardware gain range
// For BELL filters: INVERTED (negative gain = high wiper)
// isLeftChannel: true for left, false for right
static uint8_t gainToWiperBellCompensated(int16_t gain10, uint8_t hwFilterIndex, bool isLeftChannel) {
  if (hwFilterIndex >= 4) hwFilterIndex = 0;
  
  const FilterGainRange &range = isLeftChannel ? 
    hwFilterGainRangesLeft[hwFilterIndex] : hwFilterGainRangesRight[hwFilterIndex];
  
  // Convert gain10 to dB
  float gainDb = (float)gain10 / 10.0f;
  
  // Clamp to hardware's actual range
  if (gainDb < range.minGainDb) gainDb = range.minGainDb;
  if (gainDb > range.maxGainDb) gainDb = range.maxGainDb;
  
  // Linear mapping within hardware's range
  // gainDb maps from [minGainDb, maxGainDb] to wiper [255, 0] (inverted for bell)
  // wiper = 255 - ((gainDb - minGainDb) * 255) / (maxGainDb - minGainDb)
  float gainSpan = range.maxGainDb - range.minGainDb;
  if (gainSpan < 0.1f) return 128;  // Safety: avoid div by zero
  
  float normalized = (gainDb - range.minGainDb) / gainSpan;
  int wiper = (int)((1.0f - normalized) * 255.0f);  // Inverted for bell
  
  if (wiper < 0) wiper = 0;
  if (wiper > 255) wiper = 255;
  return (uint8_t)wiper;
}

// Convert gain for LOW SHELF filter (not inverted) with compensation
static uint8_t gainToWiperShelfCompensated(int16_t gain10, uint8_t hwFilterIndex, bool isLeftChannel) {
  if (hwFilterIndex >= 4) hwFilterIndex = 0;
  
  const FilterGainRange &range = isLeftChannel ? 
    hwFilterGainRangesLeft[hwFilterIndex] : hwFilterGainRangesRight[hwFilterIndex];
  
  // Convert gain10 to dB
  float gainDb = (float)gain10 / 10.0f;
  
  // Clamp to hardware's actual range
  if (gainDb < range.minGainDb) gainDb = range.minGainDb;
  if (gainDb > range.maxGainDb) gainDb = range.maxGainDb;
  
  // Linear mapping within hardware's range
  // gainDb maps from [minGainDb, maxGainDb] to wiper [0, 255]
  float gainSpan = range.maxGainDb - range.minGainDb;
  if (gainSpan < 0.1f) return 128;  // Safety: avoid div by zero
  
  float normalized = (gainDb - range.minGainDb) / gainSpan;
  int wiper = (int)(normalized * 255.0f);
  
  if (wiper < 0) wiper = 0;
  if (wiper > 255) wiper = 255;
  return (uint8_t)wiper;
}

// Legacy gain functions (kept for master gain which doesn't need per-filter compensation)
// Convert gain (-15.0 to +15.0 dB, stored as gain10: -150 to +150) to wiper value (0-255)
// For BELL filters: INVERTED (negative gain = high wiper)
static uint8_t gainToWiperBell(int16_t gain10) {
  // Map -150..+150 to 255..0 (inverted: -15dB -> 255, +15dB -> 0)
  int32_t mapped = ((int32_t)gain10 + 150) * 255 / 300;
  if (mapped < 0) mapped = 0;
  if (mapped > 255) mapped = 255;
  return (uint8_t)(255 - mapped);  // Invert for bell filters!
}

// Convert gain for LOW SHELF filter (not inverted)
static uint8_t gainToWiperShelf(int16_t gain10) {
  // Map -150..+150 to 0..255 (center at 128)
  int32_t mapped = ((int32_t)gain10 + 150) * 255 / 300;
  if (mapped < 0) mapped = 0;
  if (mapped > 255) mapped = 255;
  return (uint8_t)mapped;
}

// ======================================
// Low-level write helpers with status tracking
// ======================================
static uint8_t gWriteSuccessCount = 0;
static uint8_t gWriteFailCount = 0;
static uint8_t gReadSuccessCount = 0;
static uint8_t gReadFailCount = 0;

// Record last error for Firebase debugging
static void recordError(uint8_t failType, uint8_t tcaAddr, uint8_t tcaCh, uint8_t mcpAddr, const char* msg) {
  gMcpFeedback.lastFailType = failType;
  gMcpFeedback.lastFailTca = tcaAddr;
  gMcpFeedback.lastFailChannel = tcaCh;
  gMcpFeedback.lastFailMcp = mcpAddr;
  snprintf(gMcpFeedback.lastError, sizeof(gMcpFeedback.lastError), "%s", msg);
}

// Write to MCP on specific TCA and channel (basic, no per-pot tracking)
// Simplified to match working BLE code pattern
static bool writeMcpOnChannelBasic(uint8_t tcaAddr, uint8_t tcaCh, uint8_t mcpAddr, uint8_t val) {
  // Select TCA channel (includes settling delay)
  if (!tcaSelect(tcaAddr, tcaCh)) {
    char buf[64];
    snprintf(buf, sizeof(buf), "TCA 0x%02X sel ch%u fail", tcaAddr, tcaCh);
    recordError(1, tcaAddr, tcaCh, mcpAddr, buf);
    Serial.printf("[CTRL] FAIL: %s\n", buf);
    gWriteFailCount++;
    return false;
  }
  
  // Write directly to MCP (with retry)
  bool ok = writeMcpWiper0Retry(mcpAddr, val, 3);
  tcaDisableAll(tcaAddr);
  
  if (!ok) {
    char buf[64];
    snprintf(buf, sizeof(buf), "MCP 0x%02X write fail TCA%02X ch%u", mcpAddr, tcaAddr, tcaCh);
    recordError(3, tcaAddr, tcaCh, mcpAddr, buf);
    Serial.printf("[CTRL] FAIL: %s\n", buf);
    gWriteFailCount++;
  } else {
    gWriteSuccessCount++;
  }
  return ok;
}

// Record last write debug info
static void recordLastWrite(const char* name, uint8_t tcaAddr, uint8_t tcaCh, uint8_t mcpAddr,
                            uint8_t sentVal, uint8_t readVal, bool writeOk, bool verified) {
  snprintf(gMcpFeedback.lastWriteName, sizeof(gMcpFeedback.lastWriteName), "%s", name);
  gMcpFeedback.lastWriteTca = tcaAddr;
  gMcpFeedback.lastWriteChannel = tcaCh;
  gMcpFeedback.lastWriteMcp = mcpAddr;
  gMcpFeedback.lastWriteSentValue = sentVal;
  gMcpFeedback.lastWriteReadValue = readVal;
  gMcpFeedback.lastWriteSuccess = writeOk;
  gMcpFeedback.lastWriteVerified = verified;
}

// Write to MCP with per-potentiometer tracking and debug recording
static bool writeMcpOnChannelWithDebug(uint8_t tcaAddr, uint8_t tcaCh, uint8_t mcpAddr, uint8_t val,
                                        const char* name, uint8_t *okCounter, uint8_t *failCounter) {
  bool ok = writeMcpOnChannelBasic(tcaAddr, tcaCh, mcpAddr, val);
  
  // Read back for verification
  uint8_t readVal = 0;
  bool readOk = false;
  if (ok) {
    // Re-select channel and read back
    if (tcaSelect(tcaAddr, tcaCh)) {
      readOk = readMcpWiper0(mcpAddr, readVal);
      tcaDisableAll(tcaAddr);
    }
  }
  
  bool verified = readOk && (readVal == val);
  recordLastWrite(name, tcaAddr, tcaCh, mcpAddr, val, readVal, ok, verified);
  
  if (okCounter && failCounter) {
    if (ok) {
      if (*okCounter < 255) (*okCounter)++;
    } else {
      if (*failCounter < 255) (*failCounter)++;
    }
  }
  return ok;
}

// Write to MCP with per-potentiometer tracking (no debug recording - for bulk writes)
static bool writeMcpOnChannel(uint8_t tcaAddr, uint8_t tcaCh, uint8_t mcpAddr, uint8_t val,
                               uint8_t *okCounter, uint8_t *failCounter) {
  bool ok = writeMcpOnChannelBasic(tcaAddr, tcaCh, mcpAddr, val);
  if (okCounter && failCounter) {
    if (ok) {
      if (*okCounter < 255) (*okCounter)++;
    } else {
      if (*failCounter < 255) (*failCounter)++;
    }
  }
  return ok;
}

// ======================================
// Apply filter settings to hardware (with per-pot tracking)
// ======================================

// Apply a single filter with per-potentiometer tracking and debug recording
// isBell: true for bell/peak filters (inverted gain), false for shelf filters
// hwFilterIndex: 0=Peak1, 1=Peak2, 2=Peak3, 3=LowShelf (for frequency compensation)
static void applyFilterWithTracking(uint8_t tcaAddr, const FilterMcpMapping &map, 
                                     uint16_t freq, uint16_t q10, int16_t gain10, bool enabled,
                                     bool isBell, uint8_t hwFilterIndex,
                                     const char* channelName, const char* filterName,
                                     uint8_t *freq1Ok, uint8_t *freq1Fail,
                                     uint8_t *freq2Ok, uint8_t *freq2Fail,
                                     uint8_t *q1Ok, uint8_t *q1Fail,
                                     uint8_t *q2Ok, uint8_t *q2Fail,
                                     uint8_t *gainOk, uint8_t *gainFail) {
  char nameBuf[32];
  
  if (!enabled) {
    // When disabled, set gain to minimum (mute)
    snprintf(nameBuf, sizeof(nameBuf), "%s/%s/gain", channelName, filterName);
    writeMcpOnChannelWithDebug(tcaAddr, map.gainTcaChannel, map.gainMcp, 0, nameBuf, gainOk, gainFail);
    return;
  }
  
  // Right channel uses isLeftChannel=false
  uint8_t freqWiper = freqToWiperCompensated(freq, hwFilterIndex, false);
  uint8_t qWiper = qToWiper(q10);
  uint8_t gainWiper = isBell ? gainToWiperBellCompensated(gain10, hwFilterIndex, false) 
                            : gainToWiperShelfCompensated(gain10, hwFilterIndex, false);
  
  // Write frequency (two MCPs)
  snprintf(nameBuf, sizeof(nameBuf), "%s/%s/freq1", channelName, filterName);
  writeMcpOnChannelWithDebug(tcaAddr, map.tcaChannel, map.freqMcp1, freqWiper, nameBuf, freq1Ok, freq1Fail);
  delay(1);
  snprintf(nameBuf, sizeof(nameBuf), "%s/%s/freq2", channelName, filterName);
  writeMcpOnChannelWithDebug(tcaAddr, map.tcaChannel, map.freqMcp2, freqWiper, nameBuf, freq2Ok, freq2Fail);
  delay(1);
  
  // Write Q (two MCPs)
  snprintf(nameBuf, sizeof(nameBuf), "%s/%s/q1", channelName, filterName);
  writeMcpOnChannelWithDebug(tcaAddr, map.tcaChannel, map.qMcp1, qWiper, nameBuf, q1Ok, q1Fail);
  delay(1);
  snprintf(nameBuf, sizeof(nameBuf), "%s/%s/q2", channelName, filterName);
  writeMcpOnChannelWithDebug(tcaAddr, map.tcaChannel, map.qMcp2, qWiper, nameBuf, q2Ok, q2Fail);
  delay(1);
  
  // Write gain (on separate TCA channel)
  snprintf(nameBuf, sizeof(nameBuf), "%s/%s/gain", channelName, filterName);
  writeMcpOnChannelWithDebug(tcaAddr, map.gainTcaChannel, map.gainMcp, gainWiper, nameBuf, gainOk, gainFail);
  delay(1);
  
  Serial.printf("[CTRL] Filter TCA0x%02X ch%u: freq=%u->%u, Q=%.1f->%u, gain=%.1f->%u\n",
                tcaAddr, map.tcaChannel, freq, freqWiper, q10/10.0f, qWiper, gain10/10.0f, gainWiper);
}

// Apply a LEFT channel filter (uses LeftFilterMapping which has separate TCA for freq/Q vs gain)
// isBell: true for bell/peak filters (inverted gain), false for shelf filters
// hwFilterIndex: 0=Peak1, 1=Peak2, 2=Peak3, 3=LowShelf (for frequency compensation)
static void applyLeftFilterWithTracking(const LeftFilterMapping &map, 
                                         uint16_t freq, uint16_t q10, int16_t gain10, bool enabled,
                                         bool isBell, uint8_t hwFilterIndex,
                                         const char* filterName,
                                         uint8_t *freq1Ok, uint8_t *freq1Fail,
                                         uint8_t *freq2Ok, uint8_t *freq2Fail,
                                         uint8_t *q1Ok, uint8_t *q1Fail,
                                         uint8_t *q2Ok, uint8_t *q2Fail,
                                         uint8_t *gainOk, uint8_t *gainFail) {
  char nameBuf[32];
  
  if (!enabled) {
    // When disabled, set gain to minimum (mute) - gain is on gainTca
    snprintf(nameBuf, sizeof(nameBuf), "left/%s/gain", filterName);
    writeMcpOnChannelWithDebug(map.gainTca, map.gainTcaChannel, map.gainMcp, 0, nameBuf, gainOk, gainFail);
    return;
  }
  
  // Left channel uses isLeftChannel=true
  uint8_t freqWiper = freqToWiperCompensated(freq, hwFilterIndex, true);
  uint8_t qWiper = qToWiper(q10);
  uint8_t gainWiper = isBell ? gainToWiperBellCompensated(gain10, hwFilterIndex, true) 
                            : gainToWiperShelfCompensated(gain10, hwFilterIndex, true);
  
  // Write frequency (two MCPs) - on freqQTca
  snprintf(nameBuf, sizeof(nameBuf), "left/%s/freq1", filterName);
  writeMcpOnChannelWithDebug(map.freqQTca, map.tcaChannel, map.freqMcp1, freqWiper, nameBuf, freq1Ok, freq1Fail);
  delay(1);
  snprintf(nameBuf, sizeof(nameBuf), "left/%s/freq2", filterName);
  writeMcpOnChannelWithDebug(map.freqQTca, map.tcaChannel, map.freqMcp2, freqWiper, nameBuf, freq2Ok, freq2Fail);
  delay(1);
  
  // Write Q (two MCPs) - on freqQTca
  snprintf(nameBuf, sizeof(nameBuf), "left/%s/q1", filterName);
  writeMcpOnChannelWithDebug(map.freqQTca, map.tcaChannel, map.qMcp1, qWiper, nameBuf, q1Ok, q1Fail);
  delay(1);
  snprintf(nameBuf, sizeof(nameBuf), "left/%s/q2", filterName);
  writeMcpOnChannelWithDebug(map.freqQTca, map.tcaChannel, map.qMcp2, qWiper, nameBuf, q2Ok, q2Fail);
  delay(1);
  
  // Write gain - on gainTca (TCA1 ch2)
  snprintf(nameBuf, sizeof(nameBuf), "left/%s/gain", filterName);
  writeMcpOnChannelWithDebug(map.gainTca, map.gainTcaChannel, map.gainMcp, gainWiper, nameBuf, gainOk, gainFail);
  delay(1);
  
  Serial.printf("[CTRL] Left %s: freqQ on TCA0x%02X ch%u, gain on TCA0x%02X ch%u: freq=%u->%u, Q=%.1f->%u, gain=%.1f->%u\n",
                filterName, map.freqQTca, map.tcaChannel, map.gainTca, map.gainTcaChannel,
                freq, freqWiper, q10/10.0f, qWiper, gain10/10.0f, gainWiper);
}

// ======================================
// Read back MCP values from hardware
// ======================================
static uint8_t readMcpOnChannel(uint8_t tcaAddr, uint8_t tcaCh, uint8_t mcpAddr, bool &success) {
  uint8_t val = 0;
  if (!tcaSelect(tcaAddr, tcaCh)) {
    success = false;
    gReadFailCount++;
    return 0;
  }
  bool ok = readMcpWiper0(mcpAddr, val);
  tcaDisableAll(tcaAddr);
  success = ok;
  if (ok) {
    gReadSuccessCount++;
  } else {
    gReadFailCount++;
  }
  return val;
}

static void readAllMcpFeedback(bool isInitialRead) {
  gMcpFeedback.seq++;  // Begin write (odd)
  
  // Reset counters
  gReadSuccessCount = 0;
  gReadFailCount = 0;
  
  // Check TCA presence
  gMcpFeedback.tca0Ok = i2cAck(TCA0);
  gMcpFeedback.tca1Ok = i2cAck(TCA1);
  
  bool ok;
  
  // Read right channel (TCA0)
  for (uint8_t i = 0; i < 4; ++i) {
    const FilterMcpMapping &map = rightFilters[i];
    gMcpFeedback.rightFreq1[i] = readMcpOnChannel(TCA0, map.tcaChannel, map.freqMcp1, ok);
    gMcpFeedback.rightFreq2[i] = readMcpOnChannel(TCA0, map.tcaChannel, map.freqMcp2, ok);
    gMcpFeedback.rightQ1[i] = readMcpOnChannel(TCA0, map.tcaChannel, map.qMcp1, ok);
    gMcpFeedback.rightQ2[i] = readMcpOnChannel(TCA0, map.tcaChannel, map.qMcp2, ok);
    gMcpFeedback.rightGain[i] = readMcpOnChannel(TCA0, map.gainTcaChannel, map.gainMcp, ok);
  }
  gMcpFeedback.rightMaster = readMcpOnChannel(TCA0, RIGHT_OVERALL_GAIN_CH, RIGHT_OVERALL_GAIN_MCP, ok);
  
  // Read left channel filters (using LeftFilterMapping - freq/Q may be on TCA0 or TCA1, gain on TCA1)
  for (uint8_t i = 0; i < 4; ++i) {
    const LeftFilterMapping &map = leftFiltersNew[i];
    gMcpFeedback.leftFreq1[i] = readMcpOnChannel(map.freqQTca, map.tcaChannel, map.freqMcp1, ok);
    gMcpFeedback.leftFreq2[i] = readMcpOnChannel(map.freqQTca, map.tcaChannel, map.freqMcp2, ok);
    gMcpFeedback.leftQ1[i] = readMcpOnChannel(map.freqQTca, map.tcaChannel, map.qMcp1, ok);
    gMcpFeedback.leftQ2[i] = readMcpOnChannel(map.freqQTca, map.tcaChannel, map.qMcp2, ok);
    gMcpFeedback.leftGain[i] = readMcpOnChannel(map.gainTca, map.gainTcaChannel, map.gainMcp, ok);
  }
  // Left master is on TCA0 CH5
  gMcpFeedback.leftMaster = readMcpOnChannel(TCA0, LEFT_OVERALL_GAIN_CH, LEFT_OVERALL_GAIN_MCP, ok);
  
  gMcpFeedback.readSuccessCount = gReadSuccessCount;
  gMcpFeedback.readFailCount = gReadFailCount;
  gMcpFeedback.lastUpdateMs = millis();
  gMcpFeedback.valid = true;
  
  if (isInitialRead) {
    gMcpFeedback.initialReadDone = true;
  }
  
  gMcpFeedback.seq++;  // End write (even)
  
  Serial.printf("[CTRL] 📖 MCP feedback: read %u OK, %u FAIL, TCA0=%s TCA1=%s\n",
                gReadSuccessCount, gReadFailCount,
                gMcpFeedback.tca0Ok ? "OK" : "FAIL",
                gMcpFeedback.tca1Ok ? "OK" : "FAIL");
}

// ======================================
// Apply EQ settings to hardware
// ======================================
static void applyEqToHardware(const EqSharedSnapshot &snap) {
  // Reset write counters
  gWriteSuccessCount = 0;
  gWriteFailCount = 0;
  
  Serial.printf("[CTRL] Applying EQ: master=%s gain=%.1f dB\n",
                snap.masterEnabled ? "ON" : "OFF",
                snap.masterGain10 / 10.0f);

  if (!snap.masterEnabled) {
    // Master disabled - mute both channels (both masters on TCA0 CH5)
    writeMcpOnChannel(TCA0, RIGHT_OVERALL_GAIN_CH, RIGHT_OVERALL_GAIN_MCP, 0,
                      &gMcpFeedback.rightMasterWriteOk, &gMcpFeedback.rightMasterWriteFail);
    writeMcpOnChannel(TCA0, LEFT_OVERALL_GAIN_CH, LEFT_OVERALL_GAIN_MCP, 0,
                      &gMcpFeedback.leftMasterWriteOk, &gMcpFeedback.leftMasterWriteFail);
    Serial.println("[CTRL] Master disabled - both channels muted");
    
    // Store write stats
    gMcpFeedback.writeSuccessCount = gWriteSuccessCount;
    gMcpFeedback.writeFailCount = gWriteFailCount;
    
    readAllMcpFeedback(false);  // Read back after write
    return;
  }

  // Website band to hardware filter mapping:
  // Website Band 0 = lowcut (not used)
  // Website Band 1 = lowshelf → Hardware filter 3 (Low Shelf)
  // Website Band 2 = bell 1   → Hardware filter 0 (Peak 1)
  // Website Band 3 = bell 2   → Hardware filter 1 (Peak 2)
  // Website Band 4 = bell 3   → Hardware filter 2 (Peak 3)
  // Website Band 5 = highshelf (not used)
  // Website Band 6 = highcut (not used)
  
  // Map: hardware filter index -> website band index
  static const uint8_t hwToWebBand[4] = { 2, 3, 4, 1 };  // peak1->band2, peak2->band3, peak3->band4, lowshelf->band1
  static const bool hwIsBell[4] = { true, true, true, false };  // peak1,2,3 = bell, lowShelf = shelf
  const char* filterNames[] = {"peak1", "peak2", "peak3", "lowShelf"};

  // Apply right channel filters
  for (uint8_t hw = 0; hw < 4; ++hw) {
    uint8_t webBand = hwToWebBand[hw];
    if (webBand >= SHARED_MAX_BANDS) continue;
    const BandShared &b = snap.bands[webBand];
    applyFilterWithTracking(TCA0, rightFilters[hw], b.freq, b.q10, b.gain10, b.enabled,
                            hwIsBell[hw], hw, "right", filterNames[hw],
                            &gMcpFeedback.rightFreq1WriteOk[hw], &gMcpFeedback.rightFreq1WriteFail[hw],
                            &gMcpFeedback.rightFreq2WriteOk[hw], &gMcpFeedback.rightFreq2WriteFail[hw],
                            &gMcpFeedback.rightQ1WriteOk[hw], &gMcpFeedback.rightQ1WriteFail[hw],
                            &gMcpFeedback.rightQ2WriteOk[hw], &gMcpFeedback.rightQ2WriteFail[hw],
                            &gMcpFeedback.rightGainWriteOk[hw], &gMcpFeedback.rightGainWriteFail[hw]);
  }
  
  // Apply left channel filters (using new mapping - peak1/2 on TCA0, peak3/lowShelf on TCA1)
  for (uint8_t hw = 0; hw < 4; ++hw) {
    uint8_t webBand = hwToWebBand[hw];
    if (webBand >= SHARED_MAX_BANDS) continue;
    const BandShared &b = snap.bands[webBand];
    applyLeftFilterWithTracking(leftFiltersNew[hw], b.freq, b.q10, b.gain10, b.enabled,
                                hwIsBell[hw], hw, filterNames[hw],
                                &gMcpFeedback.leftFreq1WriteOk[hw], &gMcpFeedback.leftFreq1WriteFail[hw],
                                &gMcpFeedback.leftFreq2WriteOk[hw], &gMcpFeedback.leftFreq2WriteFail[hw],
                                &gMcpFeedback.leftQ1WriteOk[hw], &gMcpFeedback.leftQ1WriteFail[hw],
                                &gMcpFeedback.leftQ2WriteOk[hw], &gMcpFeedback.leftQ2WriteFail[hw],
                                &gMcpFeedback.leftGainWriteOk[hw], &gMcpFeedback.leftGainWriteFail[hw]);
  }

  // Apply overall channel gains with debug recording (both on TCA0 CH5)
  // Master gain uses shelf mapping (not inverted)
  uint8_t masterWiper = gainToWiperShelf(snap.masterGain10);
  writeMcpOnChannelWithDebug(TCA0, RIGHT_OVERALL_GAIN_CH, RIGHT_OVERALL_GAIN_MCP, masterWiper,
                    "right/master", &gMcpFeedback.rightMasterWriteOk, &gMcpFeedback.rightMasterWriteFail);
  writeMcpOnChannelWithDebug(TCA0, LEFT_OVERALL_GAIN_CH, LEFT_OVERALL_GAIN_MCP, masterWiper,
                    "left/master", &gMcpFeedback.leftMasterWriteOk, &gMcpFeedback.leftMasterWriteFail);
  
  // Store write stats
  gMcpFeedback.writeSuccessCount = gWriteSuccessCount;
  gMcpFeedback.writeFailCount = gWriteFailCount;
  
  Serial.printf("[CTRL] Write complete: %u OK, %u FAIL. Master gain: %.1f dB -> wiper %u\n", 
                gWriteSuccessCount, gWriteFailCount,
                snap.masterGain10 / 10.0f, masterWiper);
  
  // Read back all MCP values after applying
  readAllMcpFeedback(false);
}

// Log what changed between two snapshots
static void logChanges(const EqSharedSnapshot &oldSnap, const EqSharedSnapshot &newSnap, bool isFirst) {
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  if (isFirst) {
    Serial.println("[CTRL] 🆕 Initial state received:");
  } else {
    Serial.println("[CTRL] 🔄 Changes detected:");
  }
  
  // Master changes
  if (isFirst || oldSnap.masterEnabled != newSnap.masterEnabled) {
    Serial.printf("  Master Power: %s\n", newSnap.masterEnabled ? "ON ✅" : "OFF ❌");
  }
  if (isFirst || oldSnap.masterGain10 != newSnap.masterGain10) {
    Serial.printf("  Master Gain: %.1f dB → wiper %u\n", 
                  newSnap.masterGain10 / 10.0f, gainToWiperShelf(newSnap.masterGain10));
  }
  
  // Band changes
  for (uint8_t i = 0; i < 4; ++i) {  // Only log bands 0-3 (the ones we use)
    const BandShared &oldB = oldSnap.bands[i];
    const BandShared &newB = newSnap.bands[i];
    
    bool bandChanged = isFirst ||
                       oldB.freq != newB.freq ||
                       oldB.gain10 != newB.gain10 ||
                       oldB.q10 != newB.q10 ||
                       oldB.enabled != newB.enabled;
    
    if (bandChanged) {
      bool isBell = (i < 3);  // Bands 0,1,2 are Peak (bell), band 3 is LowShelf
      const char* filterType = isBell ? "Peak" : "LowShelf";
      Serial.printf("  Band %d (%s):\n", i, filterType);
      
      if (isFirst || oldB.enabled != newB.enabled) {
        Serial.printf("    Enabled: %s\n", newB.enabled ? "YES" : "NO");
      }
      if (isFirst || oldB.freq != newB.freq) {
        // Use hardware filter index for frequency compensation (show left channel wiper as example)
        Serial.printf("    Freq: %u Hz → L:%u R:%u (L:%.0f-%.0f R:%.0f-%.0f Hz)\n", 
                      newB.freq, 
                      freqToWiperCompensated(newB.freq, i, true),
                      freqToWiperCompensated(newB.freq, i, false),
                      hwFilterFreqRangesLeft[i].minHz, hwFilterFreqRangesLeft[i].maxHz,
                      hwFilterFreqRangesRight[i].minHz, hwFilterFreqRangesRight[i].maxHz);
      }
      if (isFirst || oldB.q10 != newB.q10) {
        Serial.printf("    Q: %.1f → wiper %u\n", newB.q10 / 10.0f, qToWiper(newB.q10));
      }
      if (isFirst || oldB.gain10 != newB.gain10) {
        uint8_t gainWiperL = isBell ? gainToWiperBellCompensated(newB.gain10, i, true) 
                                    : gainToWiperShelfCompensated(newB.gain10, i, true);
        uint8_t gainWiperR = isBell ? gainToWiperBellCompensated(newB.gain10, i, false) 
                                    : gainToWiperShelfCompensated(newB.gain10, i, false);
        Serial.printf("    Gain: %.1f dB → L:%u R:%u\n", newB.gain10 / 10.0f, gainWiperL, gainWiperR);
      }
    }
  }
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

// Check if snapshot changed
static bool snapshotChanged(const EqSharedSnapshot &a, const EqSharedSnapshot &b) {
  if (a.masterGain10 != b.masterGain10) return true;
  if (a.masterEnabled != b.masterEnabled) return true;
  for (uint8_t i = 0; i < SHARED_MAX_BANDS; ++i) {
    if (a.bands[i].freq != b.bands[i].freq) return true;
    if (a.bands[i].gain10 != b.bands[i].gain10) return true;
    if (a.bands[i].q10 != b.bands[i].q10) return true;
    if (a.bands[i].enabled != b.bands[i].enabled) return true;
  }
  return false;
}

// ======================================
// Control task (runs on Core 1 - same as Arduino setup/loop)
// ======================================
static void controlTaskFunc(void *param) {
  (void)param;

  Serial.println("[CTRL] Control task started");
  
  // Initial read of all MCP values on startup
  Serial.println("[CTRL] 🔍 Performing initial MCP read...");
  readAllMcpFeedback(true);  // true = initial read

  for (;;) {
    EqSharedSnapshot snap;
    if (eqSharedReadSnapshot(gEqShared, snap)) {
      // Check if settings changed
      if (!snapshotInitialized || snapshotChanged(lastSnapshot, snap)) {
        logChanges(lastSnapshot, snap, !snapshotInitialized);
        applyEqToHardware(snap);
        lastSnapshot = snap;
        snapshotInitialized = true;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));  // Poll at ~20 Hz
  }
}

// ======================================
// Start control task (called from main.cpp)
// ======================================
extern "C" void startControlTask() {
  // Initialize I2C HERE (in main.cpp context, on Core 1)
  // This matches how the working BLE code does it in setup()
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // 100 kHz
  Wire.setTimeOut(50);   // Match working BLE code timeout
  
  Serial.println("[CTRL] I2C initialized");
  delay(100);  // Let I2C settle
  
  xTaskCreatePinnedToCore(
    controlTaskFunc,
    "ControlTask",
    4096,
    nullptr,
    1,
    nullptr,
    1  // Run on Core 1 (same as Arduino loop) - try this instead of Core 0
  );
}