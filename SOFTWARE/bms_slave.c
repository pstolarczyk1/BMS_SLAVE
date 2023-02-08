#include <avr/io.h>

#include <util/delay.h>

#include <util/twi.h>

#include <avr/interrupt.h>

#define NODE_ADDRESS 0x48

#define LED_PING_PORT PB3
#define LED_CHARGE_PORT PB2

#define TEMPERATURE_SENSOR_1_PORT PA4
#define TEMPERATURE_SENSOR_2_PORT PA5
#define TEMPERATURE_SENSOR_3_PORT PA6
#define TEMPERATURE_SENSOR_4_PORT PA7

#define TEMPERATURE_SENSOR_PCB_1 PB5
#define TEMPERATURE_SENSOR_PCB_1 PB4

#define VOLTAGE_SENSOR_1 PA2
#define VOLTAGE_SENSOR_2 PA1

#define PWM_B1_PORT PC1
#define PWM_B2_PORT PC0

#define I2C_SDA_PORT PB1
#define I2C_SCL_PORT PB0

#define RESISTANCE_PULL 100

void PWM_init(void) {
  TCCR0A |= (1 << WGM00) | (1 << WGM01); // Set fast PWM mode
  TCCR0A |= (1 << COM0A1); // Set non-inverting mode for OC0A
  TCCR0B |= (1 << CS00); // No prescaler
  DDRB |= (1 << PWM_B1_PORT) | (1 << PWM_B2_PORT); // Set outputs
}

void ADC_init(void) {
  ADMUX |= (1 << REFS0); // Select Vcc as reference
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128
  ADCSRA |= (1 << ADEN); // Enable ADC
}

void i2c_init(void) {
  TWSR = 0; // set prescaler to 1
  TWBR = 72; // set SCL frequency to 100kHz
  TWCR = (1 << TWEN); // enable TWI
}

uint16_t readADC(uint8_t channel) {
  channel &= 0b00000111; // Channel must be 0-7
  ADMUX = (ADMUX & 0xF8) | channel; // Clear last 3 bits of ADMUX, OR with channel
  ADCSRA |= (1 << ADSC); // Start conversion
  while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
  return (ADC);
}

uint8_t i2c_start(void) {
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // send start condition
  while (!(TWCR & (1 << TWINT))); // wait for TWINT flag
  return (TWSR & 0xF8); // return status register
}

uint8_t i2c_write(uint8_t data) {
  TWDR = data; // load data into TWDR
  TWCR = (1 << TWINT) | (1 << TWEN); // clear TWINT flag to start transmission
  while (!(TWCR & (1 << TWINT))); // wait for TWINT flag
  return (TWSR & 0xF8); // return status register
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // send stop condition
}

int main(void) {
  // Initialization part for LEDs

  // Initialization part for PWM
  PWM_init();

  // Initialization part for I2C
  i2c_init();
  i2c_start();
  i2c_write(NODE_ADDRESS << 1); // send slave address with write bit

  // Initialization part for ADC
  ADC_init();

  uint16_t adc_value_b1 = readADC(VOLTAGE_SENSOR_1);
  uint16_t adc_value_b2 = readADC(VOLTAGE_SENSOR_2);

  uint16_t adc_value_temperature_1 = readADC(TEMPERATURE_SENSOR_1_PORT);
  uint16_t adc_value_temperature_2 = readADC(TEMPERATURE_SENSOR_2_PORT);
  uint16_t adc_value_temperature_3 = readADC(TEMPERATURE_SENSOR_3_PORT);
  uint16_t adc_value_temperature_4 = readADC(TEMPERATURE_SENSOR_4_PORT);

  // PWM TEST
  while (1) {

    for (uint8_t i = 0; i < 255; i++) {
      OCR0A = i; // Set PWM duty cycle
      _delay_ms(10); // Delay 10ms
    }

    for (uint8_t i = 255; i > 0; i--) {
      OCR0A = i; // Set PWM duty cycle
      _delay_ms(10); // Delay 10ms
    }

    // Send voltage data
    I2C_write(adc_value_b1 >> 8); 
    I2C_write(adc_value_b1 & 0xFF); 
    I2C_write(adc_value_b2 >> 8); 
    I2C_write(adc_value_b1 & 0xFF); 

    // Send temperature data
    I2C_write(adc_value_temperature_1 >> 8); 
    I2C_write(adc_value_temperature_1 & 0xFF); 
    I2C_write(adc_value_temperature_2 >> 8); 
    I2C_write(adc_value_temperature_2 & 0xFF); 
    I2C_write(adc_value_temperature_3 >> 8); 
    I2C_write(adc_value_temperature_3 & 0xFF); 
    I2C_write(adc_value_temperature_4 >> 8); 
    I2C_write(adc_value_temperature_4 & 0xFF); 
  }

 // uint8_t data = 0x00; // Need to encapsulate readings from temperature sensors and voltage

  i2c_write(data); // send data
  i2c_stop();
  return 0;
}
