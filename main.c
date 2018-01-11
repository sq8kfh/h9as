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

volatile uint8_t output = 0;
	
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_PIN PB1

void
set_output(uint8_t o)
{
	switch (o) {
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
            output = 0;
			return;
	}
    output = o;
}

int main(void)
{
	DDRB = 0xff;
	DDRC = 0xff ^ ((1 << PC6) | (1 << PC5) | (1 << PC4)); //ADC
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
		
		set_output(output);
		
		h9msg_t cm;
		if (CAN_get_msg(&cm)) {
			LED_PORT |= (1<<LED_PIN);
			led_counter = 0x10000;
			if (cm.type == H9MSG_TYPE_GET_REG) {
				h9msg_t cm_res;
				CAN_init_response_msg(&cm, &cm_res);
				cm_res.dlc = 2;
				cm_res.data[0] = cm.data[0];

                uint32_t tmp;
                uint16_t adc;
				switch (cm.data[0]) {
					case 10:
						cm_res.data[1] = output;
						CAN_put_msg(&cm_res);
						break;
					case 11:
                        ADMUX = (1 << MUX3) | (1 << MUX1);
						ADCSRA |= (1<<ADSC); //ADSC: uruchomienie pojedynczej konwersji
						cm_res.dlc = 3;
						while(ADCSRA & (1<<ADSC)); //czeka na zako�czenie konwersji
                        tmp = ADCL;
                        adc = ADCH;
                        //cm_res.data[1] = adc;
                        //cm_res.data[2] = tmp;
                        adc <<= 8;
						adc |= tmp;
						tmp = (uint32_t)3125*5*adc/2304; //for Rl 18kOhm in 1/10 mA
 						cm_res.data[1] = (uint8_t)((tmp >> 8) & 0xff);
						cm_res.data[2] = (uint8_t)(tmp & 0xff);
						CAN_put_msg(&cm_res);
						break;
				}
			}
			else if (cm.type == H9MSG_TYPE_SET_REG) {
				h9msg_t cm_res;
				CAN_init_response_msg(&cm, &cm_res);
				cm_res.dlc = 0;
				cm_res.data[0] = cm.data[0];
				switch (cm.data[0]) {
					case 10:
						set_output(cm.data[1]);
						_NOP();
						_NOP();
						_NOP();
						_NOP();
						cm_res.data[1] = output;
						cm_res.dlc = 2;
						CAN_put_msg(&cm_res);
						break;
				}
			}
		}
	}
}

