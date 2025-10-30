#include <Arduino.h>

class I2C_Sniff {
  static uint32_t data[(257*9 + 1 + 31) / 32]; // 256 bytes + stop bit max
  static size_t pos;
  static bool ready;

public:
  static void process(void);
  static void add_data(uint32_t src, uint32_t mask, bool finished);

  operator bool() const { return ready; }

  static void begin(void);
  static void flexISR(void);
};
