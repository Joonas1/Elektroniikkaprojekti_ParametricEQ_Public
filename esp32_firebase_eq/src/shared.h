#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <math.h>   // lroundf

constexpr uint8_t SHARED_MAX_BANDS = 7;

/*
 Fixed-point layout:
  - freq:  20..20000   (uint16_t, Hz)
  - gain10:  -150..+150  (int16_t, tenths of dB)
  - q10:     0..1000     (uint16_t, tenths of Q)
  - enabled: bool
  - masterGain10: -150..+150 (tenths of dB)
  - masterEnabled: bool
*/

struct BandShared {
  uint16_t freq;
  int16_t  gain10;
  uint16_t q10;
  bool     enabled;
};

struct EqShared {
  volatile uint32_t seq;                 // only this needs volatile
  BandShared        bands[SHARED_MAX_BANDS];
  int16_t           masterGain10;
  bool              masterEnabled;
};

// SINGLE global instance is DEFINED in main.cpp (note: NOT volatile)
extern EqShared gEqShared;

/*** Writer-side helpers (used by main.cpp only) ***/
static inline void eqSharedBeginWrite(EqShared& s) { s.seq++; } // odd
static inline void eqSharedEndWrite  (EqShared& s) { s.seq++; } // even

// Clamp helpers
static inline uint16_t clampFreq(uint32_t hz) { return (uint16_t)constrain(hz, 20u, 20000u); }
static inline int16_t  clampGain10(int32_t g) { return (int16_t)constrain(g, -150, 150); }
static inline uint16_t clampQ10(uint32_t q)   { return (uint16_t)constrain(q, 0u, 1000u); }

/*** Reader-side snapshot for control.cpp ***/
struct EqSharedSnapshot {
  BandShared bands[SHARED_MAX_BANDS];
  int16_t    masterGain10;
  bool       masterEnabled;
};

static inline bool eqSharedReadSnapshot(const EqShared& src, EqSharedSnapshot& out) {
  uint32_t a, b;
  do {
    a = src.seq;
    if (a & 1u) continue; // writer in progress
    // copy payload
    for (uint8_t i = 0; i < SHARED_MAX_BANDS; ++i) out.bands[i] = src.bands[i];
    out.masterGain10  = src.masterGain10;
    out.masterEnabled = src.masterEnabled;
    b = src.seq;
  } while (a != b || (b & 1u));
  return true;
}

/*** Optional packer from floats ***/
static inline BandShared packBand(float freq, float gainDb, float q, bool enabled) {
  BandShared b;
  b.freq  = clampFreq((uint32_t)lroundf(freq));
  b.gain10  = clampGain10((int32_t)lroundf(gainDb * 10.0f));
  b.q10     = clampQ10((uint32_t)lroundf(q * 10.0f));
  b.enabled = enabled;
  return b;
}

// ======================================
// MCP Feedback structure (control.cpp writes, main.cpp reads)
// ======================================
struct McpFeedback {
  volatile uint32_t seq;
  // Per band: freq1, freq2, q1, q2, gain (5 MCPs per filter × 4 filters = 20)
  // Plus 2 master gain MCPs
  uint8_t rightFreq1[4];   // Right channel freq MCP1 for bands 0-3
  uint8_t rightFreq2[4];   // Right channel freq MCP2 for bands 0-3
  uint8_t rightQ1[4];      // Right channel Q MCP1 for bands 0-3
  uint8_t rightQ2[4];      // Right channel Q MCP2 for bands 0-3
  uint8_t rightGain[4];    // Right channel gain for bands 0-3
  uint8_t leftFreq1[4];    // Left channel freq MCP1 for bands 0-3
  uint8_t leftFreq2[4];    // Left channel freq MCP2 for bands 0-3
  uint8_t leftQ1[4];       // Left channel Q MCP1 for bands 0-3
  uint8_t leftQ2[4];       // Left channel Q MCP2 for bands 0-3
  uint8_t leftGain[4];     // Left channel gain for bands 0-3
  uint8_t rightMaster;     // Right overall gain
  uint8_t leftMaster;      // Left overall gain
  bool    valid;           // Set to true after first successful read
  
  // I2C status tracking
  bool    tca0Ok;          // TCA0 (0x70) responding
  bool    tca1Ok;          // TCA1 (0x71) responding
  uint8_t writeSuccessCount;   // Number of successful MCP writes in last operation
  uint8_t writeFailCount;      // Number of failed MCP writes in last operation
  uint8_t readSuccessCount;    // Number of successful MCP reads in last operation
  uint8_t readFailCount;       // Number of failed MCP reads in last operation
  uint32_t lastUpdateMs;       // millis() when last updated
  bool    initialReadDone;     // True after first read on startup
  
  // Last error tracking for debugging
  char    lastError[64];       // Last error message for Firebase debug
  uint8_t lastFailTca;         // TCA address of last failure (0x70 or 0x71)
  uint8_t lastFailChannel;     // TCA channel of last failure
  uint8_t lastFailMcp;         // MCP address of last failure
  uint8_t lastFailType;        // 0=TCA_ACK, 1=TCA_SELECT, 2=MCP_ACK, 3=MCP_WRITE, 4=MCP_READ
  
  // Last write debug - shows last modified potentiometer with sent vs read values
  char    lastWriteName[32];   // e.g. "right/peak1/freq1" or "left/master"
  uint8_t lastWriteTca;        // TCA address used
  uint8_t lastWriteChannel;    // TCA channel used
  uint8_t lastWriteMcp;        // MCP address used
  uint8_t lastWriteSentValue;  // Value we tried to send
  uint8_t lastWriteReadValue;  // Value read back after write
  bool    lastWriteSuccess;    // Did the write succeed?
  bool    lastWriteVerified;   // Did read-back match sent value?
  
  // Per-potentiometer write success/fail tracking
  // Right channel: 5 MCPs per filter × 4 filters = 20, plus 1 master = 21
  // Left channel: same = 21, total = 42 individual MCPs
  uint8_t rightFreq1WriteOk[4];   // Write success count for right freq MCP1
  uint8_t rightFreq1WriteFail[4]; // Write fail count for right freq MCP1
  uint8_t rightFreq2WriteOk[4];   // Write success count for right freq MCP2
  uint8_t rightFreq2WriteFail[4]; // Write fail count for right freq MCP2
  uint8_t rightQ1WriteOk[4];      // Write success count for right Q MCP1
  uint8_t rightQ1WriteFail[4];    // Write fail count for right Q MCP1
  uint8_t rightQ2WriteOk[4];      // Write success count for right Q MCP2
  uint8_t rightQ2WriteFail[4];    // Write fail count for right Q MCP2
  uint8_t rightGainWriteOk[4];    // Write success count for right gain
  uint8_t rightGainWriteFail[4];  // Write fail count for right gain
  uint8_t leftFreq1WriteOk[4];    // Write success count for left freq MCP1
  uint8_t leftFreq1WriteFail[4];  // Write fail count for left freq MCP1
  uint8_t leftFreq2WriteOk[4];    // Write success count for left freq MCP2
  uint8_t leftFreq2WriteFail[4];  // Write fail count for left freq MCP2
  uint8_t leftQ1WriteOk[4];       // Write success count for left Q MCP1
  uint8_t leftQ1WriteFail[4];     // Write fail count for left Q MCP1
  uint8_t leftQ2WriteOk[4];       // Write success count for left Q MCP2
  uint8_t leftQ2WriteFail[4];     // Write fail count for left Q MCP2
  uint8_t leftGainWriteOk[4];     // Write success count for left gain
  uint8_t leftGainWriteFail[4];   // Write fail count for left gain
  uint8_t rightMasterWriteOk;     // Write success count for right master
  uint8_t rightMasterWriteFail;   // Write fail count for right master
  uint8_t leftMasterWriteOk;      // Write success count for left master
  uint8_t leftMasterWriteFail;    // Write fail count for left master
};

extern McpFeedback gMcpFeedback;
