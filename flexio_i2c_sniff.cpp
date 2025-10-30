#include "flexio_i2c_sniff.h"

void I2C_Sniff::process(void) {
  if (!ready) return;

  const uint32_t *s = data;
  uint32_t src = *s++;
  size_t i = 0;
  if (pos > 9) {
    Serial.printf("ADDR %02X+%s|%s", src>>25, src&0x01000000 ? "read":"write", src&0x00800000 ? "NACK":"ACK");
    src <<= 9;
    i += 9;
    pos -= 9;
  }
  // intermediate bytes
  while (pos > 9) {
    uint8_t d;
    if (i <= 24) {
      d = src >> 24;
      src <<= 8;
      i += 8;
    } else {
      d = src >> i;
      src = *s++;
      i -= 24;
      d = (d << i) | (src >> (32-i));
      src <<= i;
    }
    Serial.printf("\t%02X|", d);
    if (i >= 32) src = *s++;
    Serial.printf("%s", src&0x80000000 ? "NACK":"ACK");
    src <<= 1;
    ++i;
    pos -= 9;
  }

  if (pos) {
    if (i >= 32) src = *s++;
    Serial.printf("\t%s\n", src & 0x80000000 ? "RESTART":"STOP");
    pos = 0;
    data[0] = 0;
  }

  ready = false;
  NVIC_ENABLE_IRQ(IRQ_FLEXIO3);
}

void I2C_Sniff::add_data(uint32_t src, uint32_t mask, bool finished) {
  if (mask) {
    size_t lead;
    size_t i = (pos & 31);
    asm volatile("clz %0, %1" : "=r"(lead) : "r"(mask));
    size_t length = 32 - lead;
    src <<= lead;
    
    uint32_t *dst = &data[pos / 32];
    *dst |= src >> i;
    if (length > i) {
      *(++dst) = src << (32 - i);
    }
    pos += length;
  }

  if (finished) {
    NVIC_DISABLE_IRQ(IRQ_FLEXIO3);
    ready = true;
  }
}

FLASHMEM void I2C_Sniff::begin(void) {
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_07 = 0x19; // pin 16 = FlexIO3 pin 7 (SCL)
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06 = 0x19; // pin 17 = FlexIO3 pin 6 (SDA)
  // configure pins for I2C
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_07 = IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_06 = \
    (IOMUXC_PAD_ODE | IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(4) | IOMUXC_PAD_SPEED(1) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS);

  CCM_CCGR7 |= CCM_CCGR7_FLEXIO3(CCM_CCGR_ON);

  FLEXIO3_CTRL = FLEXIO_CTRL_SWRST;
  asm volatile("dsb");
  FLEXIO3_CTRL = FLEXIO_CTRL_FLEXEN;
  while (FLEXIO3_CTRL & FLEXIO_CTRL_SWRST);

  // state 4: Idle. SCL HIGH + SDA LOW -> state 5. Pin 0 = low.
  FLEXIO3_SHIFTCTL4 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(3) | FLEXIO_SHIFTCTL_PINSEL(6) | FLEXIO_SHIFTCTL_SMOD(6);
  FLEXIO3_SHIFTCFG4 = FLEXIO_SHIFTCFG_PWIDTH(15) | FLEXIO_SHIFTCFG_SSTOP(3) | FLEXIO_SHIFTCFG_SSTART(2);
  FLEXIO3_SHIFTBUF4 = 037645444544;
  // state 5: SCL HIGH with SDA LOW. SCL LOW -> state 7, SDA HIGH -> state 4, Pin 0 = low.
  FLEXIO3_SHIFTCTL5 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(3) | FLEXIO_SHIFTCTL_PINSEL(6) | FLEXIO_SHIFTCTL_SMOD(6);
  FLEXIO3_SHIFTCFG5 = FLEXIO_SHIFTCFG_PWIDTH(15) | FLEXIO_SHIFTCFG_SSTOP(3) | FLEXIO_SHIFTCFG_SSTART(2);
  FLEXIO3_SHIFTBUF5 = 037645774577;
  // state 6: SCL HIGH with SDA HIGH. SCL LOW -> state 7, SDA LOW -> state 4. Pin 0 = low.
  FLEXIO3_SHIFTCTL6 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(3) | FLEXIO_SHIFTCTL_PINSEL(6) | FLEXIO_SHIFTCTL_SMOD(6);
  FLEXIO3_SHIFTCFG6 = FLEXIO_SHIFTCFG_PWIDTH(15) | FLEXIO_SHIFTCFG_SSTOP(3) | FLEXIO_SHIFTCFG_SSTART(2);
  FLEXIO3_SHIFTBUF6 = 037664776477;
  // state 7: Active, SCL is LOW. SCL + SDA HIGH -> state 6, SCL HIGH + SDA LOW -> state 5. Pin 0 = high. If pin 8 goes high -> state 3 (shifter read pending)
  FLEXIO3_SHIFTCTL7 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(3) | FLEXIO_SHIFTCTL_PINSEL(6) | FLEXIO_SHIFTCTL_SMOD(6);
  FLEXIO3_SHIFTCFG7 = FLEXIO_SHIFTCFG_PWIDTH(15) | FLEXIO_SHIFTCFG_SSTOP(3) | FLEXIO_SHIFTCFG_SSTART(2);
  FLEXIO3_SHIFTBUF7 = 037733336577;
  // state 3: Active, pin 8 is high so receiver is full, hold SCL LOW until it is read (clock stretch)
  FLEXIO3_SHIFTCTL3 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(1) | FLEXIO_SHIFTCTL_PINSEL(6) | FLEXIO_SHIFTCTL_SMOD(6);
  FLEXIO3_SHIFTCFG3 = FLEXIO_SHIFTCFG_PWIDTH(7) | FLEXIO_SHIFTCFG_SSTOP(3) | FLEXIO_SHIFTCFG_SSTART(2);
  FLEXIO3_SHIFTBUF3 = 020133337777;

  // timer 1: disable when trigger falls, enable when pin rises
  FLEXIO3_TIMCFG1 = FLEXIO_TIMCFG_TIMDIS(6) | FLEXIO_TIMCFG_TIMENA(4);
  FLEXIO3_TIMCMP1 = 0xFFFF;
  // trigger = inverse shifter 4, pin = pin 0
  FLEXIO3_TIMCTL1 = FLEXIO_TIMCTL_TRGSEL(4*4+1) | FLEXIO_TIMCTL_TRGPOL | FLEXIO_TIMCTL_TRGSRC | FLEXIO_TIMCTL_PINSEL(0) | FLEXIO_TIMCTL_TIMOD(3);

  // timer 2: enable/disable with previous timer, decrement on SCL
  FLEXIO3_TIMCFG2 = FLEXIO_TIMCFG_TIMDEC(2) | FLEXIO_TIMCFG_TIMDIS(1) | FLEXIO_TIMCFG_TIMENA(1);
  // receive 32-bits max before storing
  FLEXIO3_TIMCMP2 = (32*2)-1;
  // pin = SCL, 16-bit counter
  FLEXIO3_TIMCTL2 = FLEXIO_TIMCTL_PINSEL(7) | FLEXIO_TIMCTL_TIMOD(3);

  // shifter 0: shift 1 bit
  FLEXIO3_SHIFTCFG0 = 0;
  // use timer 2, pin 6 (SDA), receive mode
  FLEXIO3_SHIFTCTL0 = FLEXIO_SHIFTCTL_TIMSEL(2) | FLEXIO_SHIFTCTL_PINSEL(6) | FLEXIO_SHIFTCTL_SMOD(1);
  // shifter 1: shift 1 bit
  FLEXIO3_SHIFTCFG1 = 0;
  // use timer 2, pin 7 (SCLK), receive mode
  FLEXIO3_SHIFTCTL1 = FLEXIO_SHIFTCTL_TIMSEL(2) | FLEXIO_SHIFTCTL_PINSEL(7) | FLEXIO_SHIFTCTL_SMOD(1);

  // use timer3 to set pin 8 high when receiver shifters are full
  FLEXIO3_TIMCFG3 = FLEXIO_TIMCFG_TIMOUT(0) | FLEXIO_TIMCFG_TIMDEC(1) | FLEXIO_TIMCFG_TIMDIS(6) | FLEXIO_TIMCFG_TIMENA(6);
  FLEXIO3_TIMCMP3 = 0xFFFF;
  FLEXIO3_TIMCTL3 = FLEXIO_TIMCTL_TRGSEL(4*0+1) | FLEXIO_TIMCTL_TRGSRC | FLEXIO_TIMCTL_PINCFG(3) | FLEXIO_TIMCTL_PINSEL(8) | FLEXIO_TIMCTL_TIMOD(3);

  FLEXIO3_SHIFTSTATE = 4; // initial state = idle

  attachInterruptVector(IRQ_FLEXIO3, flexISR);
  NVIC_SET_PRIORITY(IRQ_FLEXIO3, 32);
  NVIC_ENABLE_IRQ(IRQ_FLEXIO3);

  // clear shifter 0 state
  FLEXIO3_SHIFTSTAT = 1;
  // clear timer 2 state
  FLEXIO3_TIMSTAT = 1<<2;
  // enable interrupt for shifter 0
  FLEXIO3_SHIFTSIEN = 1;

  // timer 0: freerunning FlexIO clock / 4
  FLEXIO3_TIMCFG0 = 0;
  FLEXIO3_TIMCMP0 = 4-1;
  FLEXIO3_TIMCTL0 = FLEXIO_TIMCTL_TIMOD(3);
}

void I2C_Sniff::flexISR(void) {
  uint32_t shiftstat = FLEXIO3_SHIFTSTAT;
  uint32_t timstat = FLEXIO3_TIMSTAT;
  uint32_t shifterr = FLEXIO3_SHIFTERR;

  if (shiftstat & 1) {
    uint32_t mask = FLEXIO3_SHIFTBUFBIS1;
    bool cont = timstat & (1<<2);
    FLEXIO3_TIMSTAT |= 1<<2;
    add_data(FLEXIO3_SHIFTBUFBIS0, mask, !cont);
    timstat &= ~(1<<2);
    shiftstat &= ~1;
  }

  if ((shifterr &= FLEXIO3_SHIFTEIEN))
    FLEXIO3_SHIFTERR = shifterr;
  if ((timstat &= FLEXIO3_TIMIEN))
    FLEXIO3_TIMSTAT = timstat;
  if ((shiftstat &= FLEXIO3_SHIFTSIEN))
    FLEXIO3_SHIFTSTAT = shiftstat;
  asm volatile("dsb");
}

uint32_t I2C_Sniff::data[(257*9 + 1 + 31) / 32] = {0};
size_t I2C_Sniff::pos = 0;
bool I2C_Sniff::ready = false;
