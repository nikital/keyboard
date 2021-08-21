#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHz clock speed
#endif

#include <avr/io.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define TWSR_TWS (TWSR & ~(_BV(TWPS0) | _BV(TWPS1)))

#define IODIRA (0x00)
#define IODIRB (0x01)
#define IPOLA (0x02)
#define IPOLB (0x03)
#define GPPUA (0x0c)
#define GPPUB (0x0d)
#define GPIOA (0x12)
#define GPIOB (0x13)

static void led_init()
{
  PORTB &= ~_BV(PIN0);
  DDRB |= _BV(PIN0);
}

static void led_blink(int times)
{
  static const int DELAY = 200;
  while (times-- > 0)
  {
    PORTB |= _BV(PIN0);
    _delay_ms(DELAY);
    PORTB &= ~_BV(PIN0);
    _delay_ms(DELAY);
  }
}

static void led_error()
{
  static const int DELAY = 50;
  while (1)
  {
    PORTB |= _BV(PIN0);
    _delay_ms(DELAY);
    PORTB &= ~_BV(PIN0);
    _delay_ms(DELAY);
  }
}

static void twi_init()
{
  // No internal pullup for TWI
  PORTD &= ~_BV(PIN0);
  PORTD &= ~_BV(PIN1);
  // No prescaler
  TWSR &= ~(_BV(TWPS0) | _BV(TWPS1));
  // OK bitrate
  TWBR = 0x20;
  // Enable
  TWCR = _BV(TWEN);
}

static void twi_write(const uint8_t * data, int size)
{
  // Wait for a previous STOP condition to clear
  loop_until_bit_is_clear(TWCR, TWSTO);

  // START condition
  TWCR = _BV(TWSTA) | _BV(TWINT) | _BV(TWEN);
  loop_until_bit_is_set(TWCR, TWINT);
  if (TWSR_TWS != 0x08 && TWSR_TWS != 0x10) led_error();

  // SLA+W
  TWDR = (0x20 << 1) | 0;
  TWCR = _BV(TWINT) | _BV(TWEN);
  loop_until_bit_is_set(TWCR, TWINT);
  if (TWSR_TWS != 0x18) led_error();

  // Data
  for (int i = 0; i < size; i++)
  {
    TWDR = data[i];
    TWCR = _BV(TWINT) | _BV(TWEN);
    loop_until_bit_is_set(TWCR, TWINT);
    if (TWSR_TWS != 0x28) led_error();
  }

  // STOP condition
  TWCR = _BV(TWSTO) | _BV(TWINT) | _BV(TWEN);
}

static void twi_read(uint8_t * data, int size)
{
  // Wait for a previous STOP condition to clear
  loop_until_bit_is_clear(TWCR, TWSTO);

  // START condition
  TWCR = _BV(TWSTA) | _BV(TWINT) | _BV(TWEN);
  loop_until_bit_is_set(TWCR, TWINT);
  if (TWSR_TWS != 0x08 && TWSR_TWS != 0x10) led_error();

  // SLA+R
  TWDR = (0x20 << 1) | 1;
  TWCR = _BV(TWINT) | _BV(TWEN);
  loop_until_bit_is_set(TWCR, TWINT);
  if (TWSR_TWS != 0x40) led_error();

  // Data
  for (int i = 0; i < size; i++)
  {
    int last = i == size - 1;
    TWCR = (last ? 0 : _BV(TWEA)) | _BV(TWINT) | _BV(TWEN);
    loop_until_bit_is_set(TWCR, TWINT);
    if (last ? (TWSR_TWS != 0x58) : (TWSR_TWS != 0x50)) led_error();
    data[i] = TWDR;
  }

  // STOP condition
  TWCR = _BV(TWSTO) | _BV(TWINT) | _BV(TWEN);
}

static void mcp_init()
{
  // Assume device is in IOCON.BANK=0

  // Make sure writes increment pointer
  uint8_t enable_seqop[2] = {0xA, 0};
  twi_write(enable_seqop, sizeof(enable_seqop));

  // First thing, flip direction to input
  uint8_t init1[1 + 0xa] = {0, 0xff, 0xff};
  twi_write(init1, sizeof(init1));

  // Skip IOCON

  // Zero the rest
  uint8_t init2[1 + 0xa] = {0xC};
  twi_write(init2, sizeof(init2));
}

static void mcp_write(uint8_t reg, uint8_t value)
{
  uint8_t data[2] = {reg, value};
  twi_write(data, sizeof(data));
}

static uint8_t mcp_read(uint8_t reg)
{
  uint8_t addr[1] = {reg};
  twi_write(addr, sizeof(addr));
  uint8_t value[1];
  twi_read(value, sizeof(value));
  return value[0];
}

int main(void)
{
  // DFU enables WDT
  MCUSR &= ~_BV(WDRF);
  wdt_disable();

  clock_prescale_set(clock_div_1);

  DDRB = DDRC = DDRD = DDRE = DDRF = 0;
  PORTB = PORTC = PORTD = PORTE = PORTF = ~0;

  led_init();
  twi_init();
  mcp_init();

  // COL0/COL1 are inputs with pullup
  DDRD &= ~(_BV(PIN2) | _BV(PIN3));
  PORTD |= _BV(PIN2) | _BV(PIN3);

  // ROW0/ROW1 are outputs pulled low
  DDRD |= _BV(PIN4) | _BV(PIN5);
  PORTD |= _BV(PIN4) | _BV(PIN5);

  mcp_write(GPPUB, 0b11000001); // Pullup on columns and LED
  mcp_write(IPOLB, 0b11000000); // 1 means ground
  mcp_write(GPIOA, 0b11); // Disconnect rows from ground
  mcp_write(IODIRA, ~0b11); // Output on rows
  mcp_write(IODIRB, ~0b1); // Output on LED

  // Light show
  for (int i = 0; i < 5; i++)
  {
    PORTB |= _BV(PIN0);
    mcp_write(GPIOB, 0b0);
    _delay_ms(150);

    PORTB &= ~_BV(PIN0);
    mcp_write(GPIOB, 0b1);
    _delay_ms(150);
  }
  mcp_write(GPIOB, 0b0);

  while (1)
  {
    static const int SCAN_DELAY = 1;
    uint8_t result;

    // Scan ROW 0 and ROW 2
    PORTD &= ~_BV(PIN5);
    mcp_write(GPIOA, 0b01);
    _delay_ms(SCAN_DELAY);

    if ((PIND & _BV(PIN2)) == 0) led_blink(2);
    if ((PIND & _BV(PIN3)) == 0) led_blink(4);
    result = mcp_read(GPIOB);
    if (result & 0b01000000) led_blink(6);
    if (result & 0b10000000) led_blink(8);
    PORTD |= _BV(PIN5);

    // Scan ROW 1 and ROW 3
    PORTD &= ~_BV(PIN4);
    mcp_write(GPIOA, 0b10);
    _delay_ms(SCAN_DELAY);

    if ((PIND & _BV(PIN2)) == 0) led_blink(3);
    if ((PIND & _BV(PIN3)) == 0) led_blink(5);
    result = mcp_read(GPIOB);
    if (result & 0b01000000) led_blink(7);
    if (result & 0b10000000) led_blink(9);
    PORTD |= _BV(PIN4);
  }
}
