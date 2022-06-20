#include <stdint.h>
#include <Wire.h>

// This provides a minimal i2c debugging environment.
#define ADDR 0b1001000

#define REG_CONV 0b00
#define REG_CONFIG 0b01

// uint8_t -> char
#define cton(n) \
  ( n & 0b100 ? (n & 0b010 ? (n & 0b001 ? '7' : '6') : (n & 0b001 ? '5' : '4') ) : (n & 0b010 ? (n & 0b001 ? '3' : '2') : (n & 0b001 ? '1' : '0') ))


typedef uint8_t error_t;

typedef struct {
  uint8_t addr, mask;
  uint16_t config;
  int16_t a[4];
} ads1115_state;

ads1115_state adc;

// Loads the configuration presently on the device, setting adc->config
error_t read_config(ads1115_state *adc) {
  error_t err;
  if (adc == NULL) {
    Serial.println("read_config(): NULL pointer");
    return 1;
  }

  // Transmit i2c address; register;
  Wire.beginTransmission(adc->addr);
  Wire.write(REG_CONFIG);
  err = Wire.endTransmission(adc->addr);
  if (err != 0) {
    label("Wire.endTransmission()", err);
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
    Serial.println("read_config(): wire not available.");
    return 2;
  }
  return 0;
}

// Split the config word on sections into a null-terminated string.
char* bitConfigz(const ads1115_state *adc) {
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
}

// Parse the config word into a null-terminated string.
char* humanConfigz(const ads1115_state *adc) {
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
          buf[wh++] = cton(val & 0b011);
          buf[wh++] = 'G';
        }
        else if (val) {
          buf[wh++] = 'D';
          buf[wh++] = cton((val & 0b011) - 1);
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
}


void label(const char *prefix, unsigned int i) {
  Serial.print(prefix);
  Serial.print(": ");
  Serial.println(i);
}

error_t readAdc(ads1115_state *adc, uint8_t idx) {
  uint16_t config;
  error_t err;
  if (adc == NULL) {
    Serial.println("read_adc(): NULL pointer");
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
    label("Wire.endTransmission(): couldn't write config.", err);
    return err;
  }
  adc->config = config;

  // Todo(tune this)
  delay(70);

  Wire.beginTransmission(adc->addr);
  Wire.write(REG_CONV);
  err = Wire.endTransmission(adc->addr);
  if (err != 0) {
    label("Wire.endTransmission(): couldn't read conversion.", err);
    return err;
  }
  Wire.requestFrom(adc->addr, byte(2));
  if (2 <= Wire.available()) {
    adc->a[idx] = Wire.read();
    adc->a[idx] <<= 8;
    adc->a[idx] |= Wire.read();
  }
  else {
    Serial.println("readAdc(): wire not available.");
    return 2;
  }

  return 0;
}

error_t readAll(ads1115_state *adc){
  error_t err;
  if (adc == NULL){
    Serial.println("readAll(): NULL pointer");
    return 1;
  }

  for (uint8_t idx = 0; idx < 4; ++idx){
    err = readAdc(adc, idx);
    if (err != 0){
      return err;
    }
  }
  return 0;
}

void setup() {
  delay(4000);
  uint8_t err;
  uint16_t config, dat;
  adc.addr = ADDR;
  adc.mask = 0b0011;
  Serial.println("Reading config...");
  err = read_config(&adc);
  if (err != 0) {
    return;
  }
  Serial.println(bitConfigz(&adc));
  Serial.println(humanConfigz(&adc));

  err = readAll(&adc);
  if (err != 0){
    err = read_config(&adc);
    if (err != 0){
      return;
    }
    // We'd like to see what config is set to, in this case...
  }

  for (uint8_t i = 0; i < 4; ++i){
    Serial.print("A");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(adc.a[i]);
  }

  Serial.println(bitConfigz(&adc));
  Serial.println(humanConfigz(&adc));
  
  Serial.println("done");
}

void loop() {}
