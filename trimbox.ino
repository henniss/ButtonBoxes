#include <stdint.h>
#include <stdarg.h>

#include <Joystick.h>
#include <Wire.h>

// If >0, output debug info to Serial, and do not act as a joystick.
#define DEBUG 0
#define DEBUG_WINDOWS (DEBUG & 0x2)

// ADS1115 specific codes.
// Assuming ADDR wired to GND... adjust as needed.
#define ADDR 0b1001000
#define REG_CONV 0b00
#define REG_CONFIG 0b01

// This determines how many bits to discard from analog axes. This should be at least 1
// (ADS1115 supports only 15 bits), but in practice, I find throwing 3 bits out (13 bit resolution)
// is necessary to de-noise the axes.
#define WINDOW_SIZE 3
#define WINDOW_H (WINDOW_SIZE ? (0b1 << (WINDOW_SIZE - 1)) : 0)
#define WINDOW_L (WINDOW_SIZE ? (-WINDOW_H + 1) : 0)

// Matrix specific defines.
#define MAT_W 0x3
#define MAT_H 0x3

// Forward Declarations.
// Wouldn't normally be necessary, but arduino's IDE seems to sometimes insert these in the wrong place.
typedef uint8_t error_t;
typedef struct {
  uint8_t addr, mask;
  uint16_t config;
  // Raw state of analog reads.
  int16_t a[4];
  // Smoothed state of analog reads.
  int16_t wa[4];
} ads1115_state;

typedef struct {
  uint8_t rpins[MAT_W], wpins[MAT_H];
  uint8_t codes[MAT_H][MAT_W];
} matrix_pins_t;
typedef struct {
  uint8_t s[MAT_H * MAT_W];
} matrix_state_t;

void dprintf(const char *format, ...);


error_t prepareAdc(ads1115_state *adc);
error_t readConfig(ads1115_state *adc);
error_t writeConfig(ads1115_state *adc, uint16_t config);
// Split the config word on sections into a null-terminated string.
const char* bitConfigz(const ads1115_state *adc);
// Parse the config word into a null-terminated string.
const char* humanConfigz(const ads1115_state *adc);
error_t readAdc(ads1115_state *adc, uint8_t idx);
error_t readAll(ads1115_state *adc);
error_t updateWindows(const ads1115_state *adc, int16_t *wa);
error_t initJoystick();

Joystick_ joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, 0, 0, true, true, true, true, false, false, false, false, false, false, false);



error_t prepMatrix(const matrix_pins_t * pins, matrix_state_t * state);
error_t scanMatrix(const matrix_pins_t * pins, matrix_state_t * state, void (*cb)(uint8_t code, uint8_t change));
void buttonChange(uint8_t code, uint8_t change);


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
error_t readConfig(ads1115_state *adc) {
  error_t err;
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

error_t writeConfig(ads1115_state *adc, uint16_t config) {
  error_t err;
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

error_t prepareAdc(ads1115_state *adc) {
  uint16_t config;
  error_t err;
  if (adc == NULL) {
    dprintf("prepareAdc(): NULL pointer");
    return 1;
  }
  // Don't bother configuring input mutex: that's set per-read.
  //         4V gain      single-shot      128SPS     disable comparator
  config = ((0b001 << 9) | (0b1 << 8) | (0b100 << 5) |  (0b11 << 0));
  writeConfig(adc, config);
}


error_t readAdc(ads1115_state *adc, uint8_t idx) {
  uint16_t config;
  error_t err;
  if (adc == NULL) {
    dprintf("readAdc(): NULL pointer");
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

  // Todo(tune this)
  delay(10);

  Wire.beginTransmission(adc->addr);
  Wire.write(REG_CONV);
  err = Wire.endTransmission(adc->addr);
  if (err != 0) {
    dprintf("Wire.endTransmission(): couldn't read conversion: %d", err);
    return err;
  }
  Wire.requestFrom(adc->addr, byte(2));
  if (2 <= Wire.available()) {
    adc->a[idx] = Wire.read();
    adc->a[idx] <<= 8;
    adc->a[idx] |= Wire.read();
  }
  else {
    dprintf("readAdc(): wire not available.");
    return 2;
  }

  return 0;
}

error_t readAll(ads1115_state *adc) {
  error_t err;
  if (adc == NULL) {
    dprintf("readAll(): NULL pointer");
    return 1;
  }

  for (uint8_t idx = 0; idx < 4; ++idx) {
    err = readAdc(adc, idx);
    if (err != 0) {
      return err;
    }
  }
  updateWindows(adc);
  return 0;
}

// Update the values of the analog axes, as filtered by a moving window.
// The algorithm used here is designed to produce output that is a little "sticky", that is, it doesn't move
// on its own once the axis is parked. This is a good fit for set-and-forget inputs like trim wheels, as
// it helps avoid ghostly inputs. It's not appropriate for hands-on inputs like a joystick; a moving average
// would be more appropriate there.
void updateWindows(ads1115_state * adc) {
  uint16_t r, o;

#if DEBUG_WINDOWS
  static uint8_t ranFlag;
  if (ranFlag == 0) {
    ranFlag = 1;
    dprintf("WINDOW_SIZE: %d, WINDOW_L: %d, WINDOW_H %d", WINDOW_SIZE, WINDOW_L, WINDOW_H);
  }
#endif
  for (uint8_t i = 0; i < 4; ++i) {
    r = adc->a[i];
    o = adc->wa[i];
#if DEBUG_WINDOWS
    dprintf("Read i=%d: r=%d, o=%d", i, r, o);
#endif

    // if this would overflow/underflow, then no need to move window in respective direction.
    // UINT16_MIN
    if ((0 - WINDOW_L < o) && (r < (o + WINDOW_L))) {
      adc->wa[i] = r - WINDOW_L;
#if DEBUG_WINDOWS
      dprintf("Lower to %d", adc->wa[i]);
#endif
    }
    else if ((UINT16_MAX - WINDOW_H > o) && (r > (o + WINDOW_H))) {
      adc->wa[i] = r - WINDOW_H;
#if DEBUG_WINDOWS
      dprintf("Raise to %d", adc->wa[i]);
#endif
    }
  }
}

// Axes

void joystickInit() {
#if DEBUG
#else
  //             autoSendState
  joystick.begin(true);
  joystick.setXAxisRange(INT16_MIN, INT16_MAX);
  joystick.setYAxisRange(INT16_MIN, INT16_MAX);
  joystick.setZAxisRange(INT16_MIN, INT16_MAX);
  joystick.setRxAxisRange(INT16_MIN, INT16_MAX);
#endif
}

// Encoders
// Button Matrix


// Matrix code

matrix_pins_t pins = {
  .rpins = {7, 8, 9},
  .wpins = {4, 5, 6},

  .codes = {
    {0, 1, 2},
    {3, 4, 5},
    {6, 7, 8},
  },
};
matrix_state_t matrix_state;

error_t prepMatrix(const matrix_pins_t * pins, matrix_state_t * state) {
  if (pins == NULL) {
    dprintf("prepMatrix(): Null pointer.");
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

uint8_t scan_count = 0;

error_t scanMatrix(const matrix_pins_t * pins, matrix_state_t * state, void (*cb)(uint8_t code, uint8_t change)) {
  uint8_t code, value;
  if (pins == NULL) {
    dprintf("prepMatrix(): Null pointer.");
    return 1;
  }
  
  for (uint8_t i = 0; i < sizeof(pins->wpins); ++i) {
    digitalWrite(pins->wpins[i], LOW);
    for (uint8_t j = 0; j < sizeof(pins->rpins); ++j) {
      value = digitalRead(pins->rpins[j]);
      code = pins->codes[i][j];
      if ( value != state->s[code]) {
        (*cb)(code, value);
        state->s[code] = (uint8_t)value;
      }
    }
    digitalWrite(pins->wpins[i], HIGH);
  }
}


void buttonChange(uint8_t code, uint8_t change) {
  dprintf("Button %d set to %s.", code, change == LOW ? "LOW" : "HIGH");
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
  err = prepareAdc(&adc);
  if (err != 0) {
    return;
  }
  dprintf(bitConfigz(adc.config));
  dprintf(humanConfigz(adc.config));

  joystickInit();
  
  prepMatrix(&pins, &matrix_state);

  dprintf("done");
}


uint8_t err_counter;

void loop() {
  uint8_t err;
  static uint16_t last[4];

  delay(20);
  err = readAll(&adc);
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
    if (adc.wa[i] != last[i]) {
      dprintf("A%d:%d", i, adc.wa[i]);
      last[i] = adc.wa[i];
    }
#else
    switch (i) {
      case 0:
        joystick.setXAxis((int32_t)adc.wa[i]);
        break;
      case 1:
        joystick.setYAxis((int32_t)adc.wa[i]);
        break;
      case 2:
        joystick.setZAxis((int32_t)adc.wa[i]);
        break;
      case 3:
        joystick.setRxAxis((int32_t)adc.wa[i]);
        break;
      default:
        dprintf("Impossible joystick index %d.", i);
    }
#endif
  }
  
  err = scanMatrix(&pins, &matrix_state, &buttonChange);
  if (err != 0){
   --err_counter; 
  if (err_counter = 0) {
    dprintf("scanMatrix(): error: %d", err);
    err_counter = 100;
  }
  }
  
  joystick.sendState();
}
