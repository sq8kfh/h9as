/*
 * h9-eu1.c
 *
 * Created: 2017-09-07 19:07:15
 * Author : SQ8KFH
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h> 
#include <util/delay.h>

#include "can/can.h"

volatile uint8_t output1 = 0;
volatile uint8_t output2 = 0;
	
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_PIN PB1

void
set_output(uint8_t o1, uint8_t o2)
{
	if (o1) {
		PORTB |= _BV(PB5);
		output1 = 1;
	}
	else {
		PORTB &= ~_BV(PB5);
		output1 = 0;
	}
	if (o2) {
		PORTB |= _BV(PB6);
		output2 = 1;
	}
	else {
		PORTB &= ~_BV(PB6);
		output2 = 0;
	}
		
	/*switch (o) {
		case 1:
			PORTC |= _BV(PC7);
			PORTB &= ~_BV(PB5);
			PORTB &= ~_BV(PB6);
			PORTB &= ~_BV(PB7);
			PORTD &= ~_BV(PD0);
			PORTC &= ~_BV(PC0);
			PORTD &= ~_BV(PD1);
			PORTC &= ~_BV(PC1);
			break;
		case 2:
			PORTB |= _BV(PB5);
			PORTC &= ~_BV(PC7);
			PORTB &= ~_BV(PB6);
			PORTB &= ~_BV(PB7);
			PORTD &= ~_BV(PD0);
			PORTC &= ~_BV(PC0);
			PORTD &= ~_BV(PD1);
			PORTC &= ~_BV(PC1);
			break;
		case 3:
			PORTB |= _BV(PB6);
			PORTC &= ~_BV(PC7);
			PORTB &= ~_BV(PB5);
			PORTB &= ~_BV(PB7);
			PORTD &= ~_BV(PD0);
			PORTC &= ~_BV(PC0);
			PORTD &= ~_BV(PD1);
			PORTC &= ~_BV(PC1);
			break;
		case 4:
			PORTB |= _BV(PB7);
			PORTC &= ~_BV(PC7);
			PORTB &= ~_BV(PB5);
			PORTB &= ~_BV(PB6);
			PORTD &= ~_BV(PD0);
			PORTC &= ~_BV(PC0);
			PORTD &= ~_BV(PD1);
			PORTC &= ~_BV(PC1);
			break;
		case 5:
			PORTD |= _BV(PD0);
			PORTC &= ~_BV(PC7);
			PORTB &= ~_BV(PB5);
			PORTB &= ~_BV(PB6);
			PORTB &= ~_BV(PB7);
			PORTC &= ~_BV(PC0);
			PORTD &= ~_BV(PD1);
			PORTC &= ~_BV(PC1);
			break;
		case 6:
			PORTC |= _BV(PC0);
			PORTC &= ~_BV(PC7);
			PORTB &= ~_BV(PB5);
			PORTB &= ~_BV(PB6);
			PORTB &= ~_BV(PB7);
			PORTD &= ~_BV(PD0);
			PORTD &= ~_BV(PD1);
			PORTC &= ~_BV(PC1);
			break;
		case 7:
			PORTD |= _BV(PD1);
			PORTC &= ~_BV(PC7);
			PORTB &= ~_BV(PB5);
			PORTB &= ~_BV(PB6);
			PORTB &= ~_BV(PB7);
			PORTD &= ~_BV(PD0);
			PORTC &= ~_BV(PC0);
			PORTC &= ~_BV(PC1);
			break;
		case 8:
			PORTC |= _BV(PC1);
			PORTC &= ~_BV(PC7);
			PORTB &= ~_BV(PB5);
			PORTB &= ~_BV(PB6);
			PORTB &= ~_BV(PB7);
			PORTD &= ~_BV(PD0);
			PORTC &= ~_BV(PC0);
			PORTD &= ~_BV(PD1);
			break;
		default:
			PORTC &= ~_BV(PC7);
			PORTB &= ~_BV(PB5);
			PORTB &= ~_BV(PB6);
			PORTB &= ~_BV(PB7);
			PORTD &= ~_BV(PD0);
			PORTC &= ~_BV(PC0);
			PORTD &= ~_BV(PD1);
			PORTC &= ~_BV(PC1);
			return 0;
	}
	return o;*/
}

int main(void)
{
	DDRB = 0xff;
	DDRC = 0xff ^ (1 << PC6); //ADC
	DDRD = 0xff;
	DDRE = 0xff;
	
	ADMUX = (1 << MUX3) | (1 << MUX1);
	ADCSRA = (1 << ADEN) | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2);
	ADCSRB = (1 << AREFEN);
	
	CAN_init();
	sei();
	
	_delay_ms(100);
	CAN_send_turned_on_broadcast();

	uint32_t led_counter = 0x1000;
	
	while (1) {
		if (led_counter == 0) {
			LED_PORT ^= (1<<LED_PIN);
			if (LED_PORT & (1<<LED_PIN)) {
				led_counter = 0x1000;
			}
			else {
				led_counter = 0x80000;
			}
		}
		--led_counter;
		
		set_output(output1, output2);
		
		h9msg_t cm;
		if (CAN_get_msg(&cm)) {
			LED_PORT |= (1<<LED_PIN);
			led_counter = 0x10000;
			if (cm.type == H9_TYPE_GET_REG) {
				h9msg_t cm_res;
				CAN_init_response_msg(&cm, &cm_res);
				cm_res.dlc = 2;
				cm_res.data[0] = cm.data[0];
				switch (cm.data[0]) {
					case 10:
						cm_res.data[1] = output1;
						CAN_put_msg(&cm_res);
						break;
					case 11:
						cm_res.data[1] = output2;
						CAN_put_msg(&cm_res);
						break;
					case 12:
						ADCSRA |= (1<<ADSC); //ADSC: uruchomienie pojedynczej konwersji
						cm_res.dlc = 3;
						while(ADCSRA & (1<<ADSC)); //czeka na zako�czenie konwersji
						uint32_t tmp = ADCL;
						uint16_t adc = ADCH;
						adc <<= 8;
						adc |= tmp;
						tmp = (uint32_t)625*5*adc/256;
						cm_res.data[1] = (uint8_t)((tmp >> 8) & 0xff);
						cm_res.data[2] = (uint8_t)(tmp & 0xff);
						CAN_put_msg(&cm_res);
						break;
				}
			}
			else if (cm.type == H9_TYPE_SET_REG) {
				h9msg_t cm_res;
				CAN_init_response_msg(&cm, &cm_res);
				cm_res.dlc = 0;
				cm_res.data[0] = cm.data[0];
				switch (cm.data[0]) {
					case 10:
						set_output(cm.data[1], output2);
						_NOP();
						_NOP();
						_NOP();
						_NOP();
						cm_res.data[1] = output1;
						cm_res.dlc = 2;
						CAN_put_msg(&cm_res);
						break;
					case 11:
						set_output(output1, cm.data[1]);
						_NOP();
						_NOP();
						_NOP();
						_NOP();
						cm_res.data[1] = output2;
						cm_res.dlc = 2;
						CAN_put_msg(&cm_res);
						break;
				}
			}
		}
	}
	return 0;
}

