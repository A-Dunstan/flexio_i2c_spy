#include <Arduino.h>

class I2C_Sniff {
  static uint32_t data[128]; // 256 bytes + stop bit max
  static uint32_t *rec;
  static uint32_t pos;
  static bool ready;

  static void decode(const uint32_t *s);
public:
  static void process(void);
  static void add_data(uint32_t src, uint32_t mask, bool finished);

  operator bool() const { return ready; }

  static void begin(void);
  static void flexISR(void);
};
