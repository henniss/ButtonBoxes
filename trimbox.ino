#include <stdint.h>
#include <stdarg.h>

#include <Key.h>
#include <Keypad.h>
#include <Joystick.h>
#include <Wire.h>

// If >0, output debug info to Serial, and do not act as a joystick.
#define DEBUG 1

// ADS1115 specific codes.
// Assuming ADDR wired to GND... adjust as needed.
#define ADDR 0b1001000
#define REG_CONV 0b00
#define REG_CONFIG 0b01

// Forward Declarations.
// Wouldn't normally be necessary, but arduino's IDE seems to sometimes insert these in the wrong place.
typedef uint8_t error_t;
typedef struct {
  uint8_t addr, mask;
  uint16_t config;
  int16_t a[4];
} ads1115_state;

void dprintf(const char *format, ...);

error_t read_config(ads1115_state *adc);
// Split the config word on sections into a null-terminated string.
const char* bitConfigz(const ads1115_state *adc);
// Parse the config word into a null-terminated string.
const char* humanConfigz(const ads1115_state *adc);
error_t readAdc(ads1115_state *adc, uint8_t idx);
error_t readAll(ads1115_state *adc);

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
// global state.
ads1115_state adc {.addr = ADDR, .mask = 0b0011};

// nibble -> char
#define ntoc(n) \
  ( n & 0b100 ? (n & 0b010 ? (n & 0b001 ? '7' : '6') : (n & 0b001 ? '5' : '4') ) : (n & 0b010 ? (n & 0b001 ? '3' : '2') : (n & 0b001 ? '1' : '0') ))

// Loads the configuration presently on the device, setting adc->config
error_t read_config(ads1115_state *adc) {
  error_t err;
  if (adc == NULL) {
    dprintf("read_config(): NULL pointer");
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
    dprintf("read_config(): wire not available.");
    return 2;
  }
  return 0;
}

// DEBUG only.
// Split the config word on sections into a null-terminated string.
const char* bitConfigz(const ads1115_state *adc) {
#if DEBUG
  static char buf[26];
  uint8_t wh = 0;
  for (int8_t i = 15; i >= 0; --i) {
    // post-increment, i.e. write to the vacated index value.
    buf[wh++] = ((adc->config >> i) & 0b1) ? '1' : '0';
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
const char* humanConfigz(const ads1115_state *adc) {
#if DEBUG
  static char buf[26];
  char scratch[16];
  uint8_t wh = 0, val = 0, b = 0;
  for (int8_t i = 15; i >= 0; --i) {
    val <<= 1;
    val |= ((adc->config >> i) & 0b1);
    switch (i) {
      // Terminal bits of each section, from p18 of the ADS1115 datasheet.
      case 15:
        buf[wh++] = val ? 'I' : 'C';
        goto done;
      case 12:
        if (val & 0b100) {
          // Absolute
          buf[wh++] = 'A';
          buf[wh++] = ntoc(val & 0b011);
          buf[wh++] = 'G';
        }
        else if (val) {
          buf[wh++] = 'D';
          buf[wh++] = ntoc((val & 0b011) - 1);
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


error_t readAdc(ads1115_state *adc, uint8_t idx) {
  uint16_t config;
  error_t err;
  if (adc == NULL) {
    dprintf("read_adc(): NULL pointer");
    return 1;
  }
  // Clamp to 0-3
  idx &= 0b11;

  if (!((0b1 << idx) & adc->mask)) {
    // Index is masked out.
    return 0;
  }

  // Don't want to commit this until write succeedes.
  config = adc->config;

  // Zero out mux config.
  config &= ~(0b111 << 12);
  // Absolute mode only.
  config |= (0b1 << 14);
  config |= (idx << 12);


  // Zero out PGA config.
  config &= ~(0b111 << 9);
  // Drop gain.
  config |= (0b001 << 9);

  Wire.beginTransmission(adc->addr);
  Wire.write(byte(REG_CONFIG));
  // Write with conversion bit set.
  Wire.write(byte(0x80 | (config >> 8)));
  Wire.write(byte(config & 0xff));
  err = Wire.endTransmission(adc->addr);
  if (err != 0) {
    dprintf("Wire.endTransmission(): couldn't write config: %d", err);
    return err;
  }
  adc->config = config;

  // Todo(tune this)
  delay(70);

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
  return 0;
}

void setup() {
  delay(4000);
  uint8_t err;
  uint16_t config, dat;

  dprintf("Reading config...");
  err = read_config(&adc);
  if (err != 0) {
    return;
  }
  dprintf(bitConfigz(&adc));
  dprintf(humanConfigz(&adc));

  err = readAll(&adc);
  if (err != 0) {
    err = read_config(&adc);
    if (err != 0) {
      return;
    }
    // We'd like to see what config is set to, in this case...
  }

  for (uint8_t i = 0; i < 4; ++i) {
    dprintf("A%d:%d", i, adc.a[i]);
  }

  dprintf(bitConfigz(&adc));
  dprintf(humanConfigz(&adc));

  dprintf("done");
}

void loop() {}
