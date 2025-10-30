#include <Wire.h>
#include "flexio_i2c_sniff.h"

/* For Teensy 4.1: Connect the three I2C SCL pins together (pins 16, 19 and 24)
 * and the three I2C SDA pins together (pins 17, 18 and 25).
 * The I2C0/Wire port acts as a slave, I2C2/Wire2 acts and master and I2C1/Wire1 is
 * routed to FlexIO3 to act as a sniffer.
 */

static uint8_t eeprom_addr;
static uint8_t eeprom_data[4] = {0xAA,0xBB,0xCC,0xDD};

static void onReceive(int numBytes) {
  if (numBytes==0) return;
  eeprom_addr = Wire.read();
  while (--numBytes) {
    eeprom_data[eeprom_addr++ & 3] = Wire.read();
  }
}

static void onRequest(void) {
  /* This writes all four bytes because we have no way to know how many will be accepted.
   * This means eeprom_addr won't be incremented like a proper EEPROM...
   * If the master tries to receive more the slave just sends zeroes.
   */
  for (int i=0; i < 4; i++)
    Wire.write(eeprom_data[eeprom_addr++ & 3]);
}

void setup() {
  Serial.begin(0);
  while (!Serial);

  if (CrashReport) Serial.print(CrashReport);

  Wire2.begin();

  Wire.begin(0x50);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  I2C_Sniff::begin();
  Serial.println("FlexIO I2C scanner started.");
}

// Stream::parseInt() is dumb and returns 0 (a valid integer) for invalid integers.
int parseInt(Stream& s) {
  int i = -1;
  while (s.available() > 0) {
    int c = s.read();
    if (c >= '0' && c <= '9') {
      if (i < 0) i = c - '0';
      else i = i*10 + c-'0';
    }
    else if (i >= 0) break;
  }
  return i;
}

void loop() {
  I2C_Sniff ispy;

  int r;
  switch (Serial.read()) {
    case 'w':
      // use: "w {X, Y, Z...}" to write consecutive values
      Wire2.beginTransmission(0x50);
      while ((r = parseInt(Serial)) >= 0) {
        Wire2.write(r);
      }
      r = Wire2.endTransmission();
      Serial.print("endTransmission returned ");
      Serial.println(r);
      break;
    case 'r':
      // use: "r" to read 1 byte from current address
      // use: "r X" to read X bytes from current address
      // use: "r X Y" to read X bytes from address Y
      r = parseInt(Serial);
      Serial.print("requestFrom(0x50, ");
      if (r < 1) {
        r = Wire2.requestFrom(0x50, 1);
        Serial.print("1) returned ");
      }
      else {
        int len = r;
        Serial.print(len);
        r = parseInt(Serial);
        if (r < 0) {
          Serial.print(") returned ");
          r = Wire2.requestFrom(0x50, len);
        } else {
          Serial.print(", ");
          Serial.print(r);
          Serial.print(", 1, 1) returned ");
          r = Wire2.requestFrom(0x50, len, r, 1, 1);
        }
      }
      Serial.println(r);
      break;
  }

  if (ispy) ispy.process();
}
