/* Arduino code for my (many) button boxes.
 * Copyright 2024 Harris Enniss. Licensed under the GNU GPL Version 3.
 *
 * This encorporates bits of Ben Buxton's rotary handler, available at https://github.com/buxtronix/arduino/tree/master/libraries/Rotary
 *
 * 
 */

#include <stdint.h>
#include <stdarg.h>

#include <Joystick.h>
#include <Wire.h>

// If >0, output debug info to Serial, and do not act as a joystick (trying to do so seems to interfere
// with the serial monitor)
#define DEBUG 0
#define DEBUG_WINDOWS (DEBUG & 0x2)
#define DEBUG_MATRIX (DEBUG & 0x4)
#define DEBUG_TIMING (DEBUG & 0x8)

// ADS1115 specific codes.
// Assuming ADDR wired to GND... adjust as needed.
#define ADDR 0b1001000
#define REG_CONV 0b00
#define REG_CONFIG 0b01

// This determines how many bits to discard from analog axes. This should be at least 1
// (ADS1115 supports only 15 bits in absolute mode), but in practice, I find throwing
// 3 bits out (13 bit resolution) is necessary to de-noise the axes on my device.
#define WINDOW_SIZE 3
#define WINDOW_H (WINDOW_SIZE ? (0b1 << (WINDOW_SIZE - 1)) : 0)
#define WINDOW_L (WINDOW_SIZE ? (-WINDOW_H + 1) : 0)

// Matrix specific defines.
#define MAT_W 0x3
#define MAT_H 0x3

// in hz
#define SAMPLE_FREQ 50
#define SAMPLE_PERIOD_MS (1000/SAMPLE_FREQ)
// Normally we don't send joystick updates unless something changes, but if >0, send additional 
// updates ever COLD_UPDATE_MS miliseconds.
#define COLD_UPDATE_MS 1000
#define COLD_UPDATE_N (COLD_UPDATE_MS / SAMPLE_PERIOD_MS)
#define COLD_UPDATE (COLD_UPDATE_PERIOD_MS > 0)

// Forward Declarations.
// Wouldn't normally be necessary, but arduino's IDE seems to sometimes insert these in the wrong place.
typedef struct {
  // addr: 7 bit i2c address
  // mask: 4 bit mask determining which axes (A0-A3) should be read.
  uint8_t addr, mask;
  uint16_t config;
  // Raw state of analog reads.
  int16_t raw[4];
  // Smoothed state of analog reads.
  int16_t cur[4];
} ads1115_state;

typedef struct {
  uint8_t rpins[MAT_W], wpins[MAT_H];
  uint8_t codes[MAT_H][MAT_W];
} matrix_pins_t;
typedef struct {
  uint8_t s[MAT_H * MAT_W];
} matrix_state_t;

void dprintf(const char *format, ...);

uint8_t initAdc(ads1115_state *adc);
uint8_t readConfig(ads1115_state *adc);
uint8_t writeConfig(ads1115_state *adc, uint16_t config);
// Split the config word on sections into a null-terminated string.
const char* bitConfigz(const ads1115_state *adc);
// Parse the config word into a null-terminated string.
const char* humanConfigz(const ads1115_state *adc);
uint8_t checkConversionReady(ads1115_state *adc);
uint8_t readAdcI(ads1115_state *adc, uint8_t idx);
uint8_t readAdc(ads1115_state *adc);
// Compute the new axis value from the previous value, and the raw read.
uint16_t moveWindow(uint16_t prev, uint16_t next);

uint8_t initMatrix(const matrix_pins_t * pins, matrix_state_t * state);
uint8_t scanMatrix(const matrix_pins_t * pins, matrix_state_t * state, void (*cb)(uint8_t code, uint8_t change), uint8_t * changed);
void buttonChange(uint8_t code, uint8_t change);

void initJoystick();
Joystick_ joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, 9, 0, true, true, true, true, false, false, false, false, false, false, false);

matrix_pins_t pins = {
  // We'll write LOW to wpins. Fill these such that current can flow from rpins to wpins.
  .rpins = {7, 8, 9},
  .wpins = {4, 5, 6},

  // If we fill .codes in the naive way:
  //  .codes = {
  //    {0, 1, 2},
  //    {3, 4, 5},
  //    {6, 7, 8},
  //  },
  // we wind up with a confusing mapping of physical buttons to codes.
  // To make sure they're in a sensible order, start with .codes as defined above.
  // Flip through your buttons in an order that makes sense (e.g., top to bottom)
  // and fill the p(x) line of the following table, in order:
  // id(x): 012345678
  // p(x):  581067432
  // Write down the permutation p this defines:
  // (0 5 7 4 6 3)(1 8 2)
  // and apply the inverse of this permutation to the values in .codes
  // to get a button code order that will be sequential on your hardware.
  .codes = {
    {3, 2, 8},
    {6, 7, 0},
    {4, 5, 1},
  },
};

// Begin Definitions.

// Serial debugging IO
#define SERIAL_BUF_LEN 128
void dprintf(const char * format, ...) {
#if DEBUG
  // Making this static so memory-exhaustion happens at compile time.
  static char serial_buf[SERIAL_BUF_LEN];
  va_list args;
  va_start (args, format);
  vsnprintf(serial_buf, SERIAL_BUF_LEN, format, args);
  Serial.println(serial_buf);
  va_end(args);
#else
#endif
}

// ADC section.

// half-nibble -> char
#define hntoc(n) \
  ( n & 0b100 ? (n & 0b010 ? (n & 0b001 ? '7' : '6') : (n & 0b001 ? '5' : '4') ) : (n & 0b010 ? (n & 0b001 ? '3' : '2') : (n & 0b001 ? '1' : '0') ))

// Loads the configuration presently on the device, setting adc->config
uint8_t readConfig(ads1115_state *adc) {
  uint8_t err;
  if (adc == NULL) {
    dprintf("readConfig(): NULL pointer");
    return 1;
  }

  // Transmit i2c address, register;
  Wire.beginTransmission(adc->addr);
  Wire.write(REG_CONFIG);
  err = Wire.endTransmission(adc->addr);
  if (err != 0) {
    dprintf("Wire.endTransmission(): %d", err);
    return err;
  }
  // Request 2 bytes, then read them or fail.
  Wire.requestFrom(adc->addr, (uint8_t)2);
  if (2 <= Wire.available()) {
    adc->config = Wire.read();
    adc->config <<= 8;
    adc->config |= Wire.read();
  }
  else {
    dprintf("readConfig(): wire not available.");
    return 2;
  }
  return 0;
}

uint8_t writeConfig(ads1115_state *adc, uint16_t config) {
  uint8_t err;
  if (adc == NULL) {
    dprintf("readConfig(): NULL pointer");
    return 1;
  }

  Wire.beginTransmission(adc->addr);
  Wire.write(REG_CONFIG);
  Wire.write(0xff & (config >> 8));
  Wire.write(0xff & (config >> 0));
  err = Wire.endTransmission(adc->addr);
  if (err != 0) {
    dprintf("Wire.endTransmission(): couldn't write config: %d", err);
    return err;
  }

#if DEBUG
  // This check is expensive; only do it in debug mode.
  err = readConfig(adc);
  if (err != 0) {
    dprintf("writeConfig(): Unexpected failure checking result.");
    return err;
  }
  // skip bit 15; it has unique R/W semantics.
  if ((config & 0x7fff) != (adc->config & 0x7fff)) {
    dprintf("writeConfig(): Mismatched result.");
    dprintf("expected: %s", bitConfigz(config));
    dprintf("got:      %s", bitConfigz(adc->config));
  }
#endif
  adc->config = (config & 0x7fff);
}


// DEBUG only.
// Split the config word on sections into a null-terminated string.
const char* bitConfigz(uint16_t config) {
#if DEBUG
  static char buf[26];
  uint8_t wh = 0;
  for (int8_t i = 15; i >= 0; --i) {
    // post-increment, i.e. write to the vacated index value.
    buf[wh++] = ((config >> i) & 0b1) ? '1' : '0';
    // Don't increment.
    buf[wh] = '\0';
    switch (i) {
      // Terminal bits of each section, from p18 of the ADS1115 datasheet.
      case 15:
      case 12:
      case 9:
      case 8:
      case 5:
      case 4:
      case 3:
      case 2:
        buf[wh++] = ' ';
        break;
      case 0:
        // Technically supurflous, but for clarity.
        goto ret;
      default:
        // Technically supurflous, but for clarity.
        continue;
    }
  }
ret:
  return buf;
#else
  return "";
#endif
}

// DEBUG only.
// Parse the config word into a null-terminated string.
const char* humanConfigz(uint16_t config) {
#if DEBUG
  static char buf[26];
  char scratch[16];
  uint8_t wh = 0, val = 0, b = 0;
  for (int8_t i = 15; i >= 0; --i) {
    val <<= 1;
    val |= ((config >> i) & 0b1);
    switch (i) {
      // Terminal bits of each section, from p18 of the ADS1115 datasheet.
      case 15:
        buf[wh++] = val ? 'I' : 'C';
        goto done;
      case 12:
        if (val & 0b100) {
          // Absolute
          buf[wh++] = 'A';
          buf[wh++] = hntoc(val & 0b011);
          buf[wh++] = 'G';
        }
        else if (val) {
          buf[wh++] = 'D';
          buf[wh++] = hntoc((val & 0b011) - 1);
          buf[wh++] = '3';
        }
        else {
          buf[wh++] = 'D';
          buf[wh++] = '0';
          buf[wh++] = '1';
        }
        goto done;
      case 9:
        switch (val) {
          case 0:
            strncpy(buf + wh, "6.1", 3);
            break;
          case 1:
            strncpy(buf + wh, "4.0", 3);
            break;
          case 2:
            strncpy(buf + wh, "2.0", 3);
            break;
          case 3:
            strncpy(buf + wh, "1.0", 3);
            break;
          case 4:
            strncpy(buf + wh, "0.5", 3);
            break;
          default:
            strncpy(buf + wh, "0.2", 3);
            break;
        }
        wh += 3;
        goto done;
      case 8:
        buf[wh++] = val ? 'D' : 'C';
        goto done;
      case 5:
        switch (val) {
          case 7:
            strncpy(buf + wh, "860", 3);
            break;
          case 6:
            strncpy(buf + wh, "475", 3);
            break;
          default:
            itoa(8 << val, scratch, 10);
            strncpy(buf + wh, scratch, 3);
            break;
        }
        wh += 3;
        goto done;
      case 4:
        buf[wh++] = val ? 'W' : 'T';
        goto done;
      case 3:
        buf[wh++] = val ? 'H' : 'L';
        goto done;
      case 2:
        buf[wh++] = val ? 'L' : 'N';
        goto done;
      case 0:
        switch (val) {
          case 3:
            // Blank out comparator elements.
            for (int8_t j = -6; j < 0; ++j) {
              buf[wh + j] = ' ';
            }
            strncpy(buf + wh, "DD", 2);
            break;
          case 2:
            strncpy(buf + wh, "Q4", 2);
            break;
          case 1:
            strncpy(buf + wh, "Q2", 2);
            break;
          default:
            strncpy(buf + wh, "Q1", 2);
            break;
        }
        wh += 2;
        goto ret;
      default:
        continue;
    }
done:
    buf[wh++] = ' ';
    val = 0;
  }
ret:
  buf[wh] = '\0';
  return buf;
#else
  return "";
#endif
}

uint8_t initAdc(ads1115_state *adc) {
  uint16_t config;
  uint8_t err;
  if (adc == NULL) {
    dprintf("initAdc(): NULL pointer");
    return 1;
  }
  // Don't bother configuring input mutex: that's set per-read.
  //         4V gain       single-shot      475SPS     disable comparator
  config = ((0b001 << 9) | (0b1 << 8) | (0b110 << 5) |  (0b11 << 0));
  writeConfig(adc, config);
}

uint8_t checkConversionReady(ads1115_state *adc, uint8_t *ready) {
  uint8_t err;
  err = readConfig(adc);
  if (err != 0) {
    return err;
  }
  // Necessary to shift this right so it fits in uint8_t
  *ready = (adc->config >> 15) & 0b1;
  return 0;
}


uint8_t readAdcI(ads1115_state *adc, uint8_t idx) {
  uint16_t config;
  uint8_t err;
#if DEBUG_TIMING
  static uint8_t sample;
  uint32_t start, end;
  ++sample;
#endif
  if (adc == NULL) {
    dprintf("readAdcI(): NULL pointer");
    return 1;
  }
  // Clamp to 0-3
  idx &= 0b11;

  if (!((0b1 << idx) & adc->mask)) {
    // Index is masked out.
    return 0;
  }

  config = adc->config;
  // Zero out mux config.
  config &= ~(0b111 << 12);
  // Absolute mode only.
  config |= (0b1 << 14);
  config |= (idx << 12);
  // Request conversion
  config |= (0b1 << 15);
  err = writeConfig(adc, config);
  if (err != 0) {
    return err;
  }
#if DEBUG_TIMING
  if (sample % 200 == 1) {
    start = millis();
    dprintf("Started conversion");
  }
#endif

  uint8_t ready;
  for (uint16_t i = 0;; ++i) {
    // I'm not sure if this is actually a good approach...
    // It's simpler and marginally slower to just use a fixed delay (e.g. 5ms)
    // although this does at least have the virtue of getting us timing data 
    // to use to tune that delay. Consider ripping this out in the future. 
    err = checkConversionReady(adc, &ready);
    if (err != 0) {
      return err;
    }
    if (ready) {
#if DEBUG_TIMING
      if (sample % 200 == 1) {
        end = millis();
        dprintf("Conversion ready after %d millis.", end - start);
      }
#endif
      break;
    }
#if DEBUG_TIMING
    if ((sample % 200 == 1) &&  (i % 4 == 2)) {
      end = millis();
      dprintf("Conversion not ready after %d millis.", end - start);
    }
#endif
    delay(1);
  }

  Wire.beginTransmission(adc->addr);
  Wire.write(REG_CONV);
  err = Wire.endTransmission(adc->addr);
  if (err != 0) {
    dprintf("Wire.endTransmission(): couldn't read conversion: %d", err);
    return err;
  }
  Wire.requestFrom(adc->addr, byte(2));
  if (2 <= Wire.available()) {
    adc->raw[idx] = Wire.read();
    adc->raw[idx] <<= 8;
    adc->raw[idx] |= Wire.read();
  }
  else {
    dprintf("readAdcI(): wire not available.");
    return 2;
  }

  return 0;
}

uint8_t readAdc(ads1115_state *adc) {
  uint8_t err;
  if (adc == NULL) {
    dprintf("readAdc(): NULL pointer");
    return 1;
  }

  for (uint8_t i = 0; i < 4; ++i) {
    err = readAdcI(adc, i);
    if (err != 0) {
      return err;
    }
    adc->cur[i] = moveWindow(adc->cur[i], adc->raw[i]);
  }
  return 0;
}

// The algorithm used here is designed to produce output that is a little "sticky", that is, it doesn't move
// on its own once the axis is parked. This is a good fit for set-and-forget inputs like trim wheels, as
// it helps avoid ghostly inputs. It's not appropriate for hands-on inputs like a joystick; a moving average
// would be more appropriate there.

// Essentially, think of an interval: [X-WINDOW_L, X+WINDOW_H].
// When we get a new read Y from the ADC, make the smallest possible update to X so that
// this interval overlaps Y. Then use X as the value the device reports.
uint16_t moveWindow(uint16_t prev, uint16_t next) {
  uint16_t ret;
#if DEBUG_WINDOWS
  static uint8_t ranFlag;
  if (ranFlag == 0) {
    ranFlag = 1;
    dprintf("WINDOW_SIZE: %d, WINDOW_L: %d, WINDOW_H %d", WINDOW_SIZE, WINDOW_L, WINDOW_H);
  }
#endif
  //  0 == UINT16_MIN
  //   if prev isn't sitting against the low stop
  //                            if the new value is definitely smaller.
  if ((0 - WINDOW_L < prev) && (next < (prev + WINDOW_L))) {
    ret = next - WINDOW_L;
#if DEBUG_WINDOWS
    dprintf("Lower to %d", ret);
#endif
    return ret;

  }
  //        if prev isn't sitting against the high stop
  //                                  if the new value is difinitively larger
  else if ((UINT16_MAX - WINDOW_H > prev) && (next > (prev + WINDOW_H))) {
    ret = next - WINDOW_H;
#if DEBUG_WINDOWS
    dprintf("Raise to %d", ret);
#endif
    return ret;
  }
  else {
    // prev falls into the pre-existing interval, so don't move it at all.
    return prev;
  }
}


// Axes

void initJoystick() {
#if DEBUG
#else
  //             autoSendState
  joystick.begin(false);
  joystick.setXAxisRange(INT16_MIN, INT16_MAX);
  joystick.setYAxisRange(INT16_MIN, INT16_MAX);
  joystick.setZAxisRange(INT16_MIN, INT16_MAX);
  joystick.setRxAxisRange(INT16_MIN, INT16_MAX);
#endif
}

// Encoders

// TODO

// Button Matrix

matrix_state_t matrix_state;

uint8_t initMatrix(const matrix_pins_t * pins, matrix_state_t * state) {
  if (pins == NULL) {
    dprintf("initMatrix(): Null pointer.");
    return 1;
  }
  for (uint8_t i = 0; i < sizeof(pins->wpins); ++i) {
    pinMode(pins->wpins[i], OUTPUT);
  }
  for (uint8_t j = 0; j < sizeof(pins->rpins); ++j) {
    pinMode(pins->rpins[j], INPUT_PULLUP);
  }
  for (uint8_t k = 0; k < sizeof(state->s); ++k) {
    state->s[k] = HIGH;
  }
  dprintf("matrix ready");
  return 0;
}

uint8_t scanMatrix(const matrix_pins_t * pins, matrix_state_t * state, void (*cb)(uint8_t code, uint8_t change), uint8_t *changed) {
  uint8_t code, value;
  if (pins == NULL) {
    dprintf("initMatrix(): Null pointer.");
    return 1;
  }

  for (uint8_t i = 0; i < sizeof(pins->wpins); ++i) {
    digitalWrite(pins->wpins[i], LOW);
    for (uint8_t j = 0; j < sizeof(pins->rpins); ++j) {
      value = digitalRead(pins->rpins[j]);
      code = pins->codes[i][j];
#if DEBUG_MATRIX
      if (value == LOW) {
        dprintf("%d -> %d LOW (button %d)", pins->wpins[i], pins->rpins[j], code);
      }
#endif
      if ( value != state->s[code]) {
        (*cb)(code, value);
        *changed = true;
        state->s[code] = (uint8_t)value;
      }
    }
    digitalWrite(pins->wpins[i], HIGH);
  }
}

void buttonChange(uint8_t code, uint8_t change) {
  dprintf("Button %d set to %s.", code, change == LOW ? "LOW" : "HIGH");
#if !DEBUG
  joystick.setButton(code, change == LOW ? 1 : 0);
#endif
}

// Global state.
ads1115_state adc {.addr = ADDR, .mask = 0b1111};

// Entrypoints.

void setup() {
#if DEBUG
  // Wait a little for the serial console to become available.
  delay(4000);
#endif
  uint8_t err;

  dprintf("Preparing config...");
  err = initAdc(&adc);
  if (err != 0) {
    return;
  }
  dprintf(bitConfigz(adc.config));
  dprintf(humanConfigz(adc.config));

  initJoystick();
  initMatrix(&pins, &matrix_state);

  dprintf("done");
}



void loop() {
  static uint8_t changed;
  static uint8_t counter;
  uint8_t err;
#if DEBUG
  static uint16_t last[4];
#endif

  delay(SAMPLE_PERIOD_MS);
  err = readAdc(&adc);
  if (err != 0) {
    err = readConfig(&adc);
    if (err != 0) {
      return;
    }
    // We'd like to see what config is set to, in this case...
    dprintf(bitConfigz(adc.config));
    dprintf(humanConfigz(adc.config));
  }

  for (uint8_t i = 0; i < 4; ++i) {
#if DEBUG
    if (adc.cur[i] != last[i]) {
      changed = true;
      dprintf("A%d:%d", i, adc.cur[i]);
      last[i] = adc.cur[i];
    }
#else
    switch (i) {
      case 0:
        joystick.setXAxis((int32_t)adc.cur[i]);
        break;
      case 1:
        joystick.setYAxis((int32_t)adc.cur[i]);
        break;
      case 2:
        joystick.setZAxis((int32_t)adc.cur[i]);
        break;
      case 3:
        joystick.setRxAxis((int32_t)adc.cur[i]);
        break;
      default:
        dprintf("Impossible joystick index %d.", i);
    }
#endif
  }

  err = scanMatrix(&pins, &matrix_state, &buttonChange, &changed);
  if (err != 0) {
#if DEBUG_MATRIX
    dprintf("scanMatrix(): error: %d", err);
#endif
  }

  if (changed || !counter) {
    dprintf("sendState()");
    joystick.sendState();
    changed = false;
  }
  counter = (counter + COLD_UPDATE_N - 1) % COLD_UPDATE_N;
}
