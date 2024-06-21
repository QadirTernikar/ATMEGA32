#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

// Define CPU frequency (e.g., 16MHz)
#define F_CPU 16000000UL

// Function Prototypes
void lcd_init(void);
void lcd_command(unsigned char);
void lcd_data(unsigned char);
void lcd_string(const char*);
void adc_init(void);
uint16_t adc_read(uint8_t);
void motor_control(uint8_t);
void buzzer_control(uint8_t);
void led_control(uint8_t);

int main(void) {
	// Initialize LCD, ADC, and configure pins
	lcd_init();
	adc_init();
	
	// Motor control pins
	DDRD |= (1 << PD2) | (1 << PD3);
	// Buzzer control pin
	DDRB |= (1 << PB2);
	// LED control pins
	DDRB |= (1 << PB3) | (1 << PB4);
	
	lcd_string("Gas Sensor Value:");
	while (1) {
		// Read the gas sensor value
		uint16_t gas_value = adc_read(0);
		
		// Display gas value on LCD
		lcd_command(0xC0); // Move cursor to second line
		char buffer[16];
		sprintf(buffer, "%u", gas_value);
		lcd_string(buffer);
		
		// Control motor and buzzer based on gas value
		if (gas_value > 50) {
			motor_control(1);  // Turn on motor
			buzzer_control(1); // Turn on buzzer
			led_control(0);    // Turn on red LED, turn off green LED
			} else {
			motor_control(0);  // Turn off motor
			buzzer_control(0); // Turn off buzzer
			led_control(1);    // Turn on green LED, turn off red LED
		}
		
		_delay_ms(1000);
	}
}

// Initialize LCD
void lcd_init(void) {
	DDRC = 0xFF; // Set PORTC as output for LCD
	_delay_ms(20);
	lcd_command(0x02); // Initialize LCD in 4-bit mode
	lcd_command(0x28); // 2 lines, 5x7 matrix
	lcd_command(0x0C); // Display on, cursor off
	lcd_command(0x06); // Increment cursor
	lcd_command(0x01); // Clear display
	_delay_ms(2);
}

// Send command to LCD
void lcd_command(unsigned char cmd) {
	PORTC = (PORTC & 0x0F) | (cmd & 0xF0);
	PORTC &= ~(1 << PC0); // RS = 0 for command
	PORTC |= (1 << PC2);  // Enable pulse
	_delay_us(1);
	PORTC &= ~(1 << PC2);
	
	_delay_us(200);
	
	PORTC = (PORTC & 0x0F) | (cmd << 4);
	PORTC |= (1 << PC2);
	_delay_us(1);
	PORTC &= ~(1 << PC2);
	
	_delay_ms(2);
}

// Send data to LCD
void lcd_data(unsigned char data) {
	PORTC = (PORTC & 0x0F) | (data & 0xF0);
	PORTC |= (1 << PC0); // RS = 1 for data
	PORTC |= (1 << PC2); // Enable pulse
	_delay_us(1);
	PORTC &= ~(1 << PC2);
	
	_delay_us(200);
	
	PORTC = (PORTC & 0x0F) | (data << 4);
	PORTC |= (1 << PC2);
	_delay_us(1);
	PORTC &= ~(1 << PC2);
	
	_delay_ms(2);
}

// Display string on LCD
void lcd_string(const char* str) {
	while (*str) {
		lcd_data(*str++);
	}
}

// Initialize ADC
void adc_init(void) {
	ADMUX = (1 << REFS0); // AVcc with external capacitor at AREF pin
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC and set prescaler to 64
}

// Read ADC value
uint16_t adc_read(uint8_t ch) {
	ch &= 0b00000111; // Select ADC channel (0-7)
	ADMUX = (ADMUX & 0xF8) | ch;
	ADCSRA |= (1 << ADSC); // Start conversion
	while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
	return ADC;
}

// Control DC Motor
void motor_control(uint8_t state) {
	if (state) {
		PORTD |= (1 << PD2); // Motor on
		PORTD &= ~(1 << PD3);
		} else {
		PORTD &= ~(1 << PD2); // Motor off
		PORTD &= ~(1 << PD3);
	}
}

// Control Buzzer
void buzzer_control(uint8_t state) {
	if (!state) {
		PORTB |= (1 << PB2); // Buzzer on
		} else {
		PORTB &= ~(1 << PB2); // Buzzer off
	}
}

// Control LEDs
void led_control(uint8_t state) {
	if (state) {
		PORTB |= (1 << PB3); // Green LED on
		PORTB &= ~(1 << PB4); // Red LED off
		} else {
		PORTB |= (1 << PB4); // Red LED on
		PORTB &= ~(1 << PB3); // Green LED off
	}
}
