
#define DS18B20_PIN BIT0 // P7.0
#define DS18B20_DIR P7DIR
#define DS18B20_OUT P7OUT
#define DS18B20_IN P7IN


uint8_t onewire_reset(void);

void onewire_write_bit(uint8_t v);
uint8_t onewire_read_bit(void);

void onewire_write(uint8_t v, uint8_t power /* = 0 */);
uint8_t onewire_read(void);

void onewire_select(uint8_t rom[8]);

float ds_getTemp(uint8_t rom[8]);
