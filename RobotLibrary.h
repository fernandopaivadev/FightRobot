/*
* AUTHOR: FERNANDO PAIVA
* BRAZIL, BELÉM, 16/08/2016
*/

#ifndef Robot_Library_h
#define Robot_Library_h
//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <stdlib.h>

#define F_CPU 16000000

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif

#define abs(x) ((x)>0 ? (x) : -(x))

// WRITES 1 IN THE bit OF THE INFORMED variable
#define bit_set(variable, bit) (variable) |= (1 << (bit))

// WRITES 0 IN THE bit OF THE INFORMED variable
#define bit_clear(variable, bit) (variable) &= ~(1 << (bit))

// WRITES 1 OR 0 (DEFINED IN value) IN THE bit OF THE INFORMED variable
#define bit_write(variable, bit, value) (value) ? bit_set((variable), (bit)) : bit_clear((variable), (bit))

// READ THE bit OF THE INFORMED variable
#define bit_read(variable, bit) ((variable) >> (bit)) & 1

//MACRO TO JOIN TWO 8 bit VALUES IN ONE 16 bit VALUE
#define join_two_bytes(msb, lsb) (uint16_t) (((msb) << 8) | (lsb))

// MACROS TO OBTAIN A RANDOM VALUE
#define random_value(howsmall, howbig) (((howsmall) >= (howbig)) ? (howsmall) : random((howbig) - (howsmall)) + (howsmall))
#define random_seed(seed) (((seed) !=0) ? srandom((seed)) : (void)0)

// MACROS TO TURN ON OR TURN OFF GLOBAL INTERRUPTS
#define interrupts_on() SREG |= 0x80
#define interrupts_off() SREG &= (~0x80)

// ADC CAN BE TURNED OFF TO SAVE POWER
#define adc_on() ADCSRA |= 0x80
#define adc_off() ADCSRA &= (~0x80)

// 1 -> SET AS OUTPUT
// 0 -> SET AS INPUT
#define pin_mode(pin, value) interrupts_off(); (pin <= 7) ? bit_write(DDRD, pin, (value & 1)) : (pin <= 13) ? bit_write(DDRB, (pin - 8), (value & 1)) : adc_off(); bit_write(DDRC, (pin - 14), (value & 1)); interrupts_on()
#define digital_write(pin, value) (pin <= 7) ? bit_write(PORTD, pin, (value & 1)) : (pin <= 13) ? bit_write(PORTB, (pin - 8), (value & 1)) : adc_off(); bit_write(PORTC, (pin - 14), (value & 1))

// FUNCTION TO SET OR CLEAR THE I/O DIGITAL PINS
uint8_t digital_read(uint8_t pin)
{
	uint8_t r;
	(pin <= 7) ? r = bit_read(PIND, pin) : (pin <= 13) ? r = bit_read(PINB, (pin - 8)) : adc_off(); r = bit_read(PINC, (pin - 14));
	return r;
}

int16_t analog_read(uint8_t pin)
{
	pin_mode(pin, INPUT); // set as INPUT
	PRR &= (~0x01); // turn off the adc power save
	uint8_t low, high;
	pin = pin - 14;
	ADMUX = 0b01000000 | (pin & 0b00000111); // set the right pin to be read

	// starts the conversion
  ADCSRA = 0b11010111;

	while (ADCSRA & 0b01000000); // wait the until conversion finish

	// read the result registers
	low  = ADCL;
	high = ADCH;

	return join_two_bytes(high, low); // combine the two bytes;
}

// MACROS TO ROBOT MOVEMENT
#define move_stop() digital_write(IN1, LOW); digital_write(IN2, LOW); digital_write(IN3, LOW); digital_write(IN4, LOW)
#define move_foward() move_stop(); digital_write(IN1, HIGH); digital_write(IN2, LOW); digital_write(IN3, LOW); digital_write(IN4, HIGH)
#define move_back() move_stop(); digital_write(IN1, LOW); digital_write(IN2, HIGH); digital_write(IN3, HIGH); digital_write(IN4, LOW)
#define turn_right() move_stop(); digital_write(IN1, HIGH); digital_write(IN2, LOW); digital_write(IN3, HIGH); digital_write(IN4, LOW)
#define turn_left() move_stop(); digital_write(IN1, LOW); digital_write(IN2, HIGH); digital_write(IN3, LOW); digital_write(IN4, HIGH)

//==============================================================================
// LSR - LINE SENSOR RIGHT
// LSL - LINE SENSOR LEFT
// LSB - LINE SENSOR BACK
// FOS - FRONT OBSTACLE SENSOR
// ROS - RIGHT OBSTACLE SENSOR
// LOS - LEFT OBSTACLE SENSOR
// TFOS - TOGGLE FRONT OBSTACLE SENSOR
// TROS - TOGGLE RIGHT OBSTACLE SENSOR
// TLOS - TOGGLE LEFT OBSTACLE SENSOR

// SENSOR PINS
#define LSR 14
#define LSL 15
#define LSB 16
#define FOS 17
#define ROS 18
#define LOS 19
#define TFOS 5
#define TROS 7
#define TLOS 6
// H BRIDGE CONTROL PINS
#define IN1 8
#define IN2 9
#define IN3 11
#define IN4 10
//==============================================================================

#define LINE_DETECT (RIGHT_LINE_DETECT || LEFT_LINE_DETECT || BACK_LINE_DETECT)
#define FRONT_LINE_DETECT (RIGHT_LINE_DETECT && LEFT_LINE_DETECT)
#define ALL_LINE_DETECT (FRONT_LINE_DETECT && BACK_LINE_DETECT)
#define ENEMY_DETECT (FRONT_ENEMY_DETECT || RIGHT_ENEMY_DETECT || LEFT_ENEMY_DETECT)

#define ESCAPE 0
#define ATTACK 1
#define SEARCH 2

#define OBS_DIF 10 // define a diferença a ser considerada entre a referência e a leitura dos sensores IR de obstáculo
#define LINE_DIF 150 // define a diferença a ser considerada entre a referência e a leitura dos sensores IR de linha

int16_t OBS_REF; // variável que guardará a referência para leitura IR dos sensores de obstáculo
int16_t LINE_REF; // variável que guardará a referência para leitura IR dos sensores de linha

uint8_t FRONT_ENEMY_DETECT = 0;
uint8_t RIGHT_ENEMY_DETECT = 0;
uint8_t LEFT_ENEMY_DETECT = 0;
uint8_t RIGHT_LINE_DETECT = 0;
uint8_t LEFT_LINE_DETECT = 0;
uint8_t BACK_LINE_DETECT = 0;

void ir_reference()
{  
  // pins set up
	pin_mode(TFOS, OUTPUT);
	pin_mode(TROS, OUTPUT);
	pin_mode(TLOS, OUTPUT);

	digital_write(TFOS, LOW);
  digital_write(TROS, LOW);
  digital_write(TLOS, LOW);

  int16_t fos_read_off = analog_read(FOS);
  int16_t ros_read_off = analog_read(ROS);
  int16_t los_read_off = analog_read(LOS);

  digital_write(TFOS, HIGH);
  digital_write(TROS, HIGH);
  digital_write(TLOS, HIGH);

	int16_t fos_read_on = analog_read(FOS);
  int16_t ros_read_on = analog_read(ROS);
  int16_t los_read_on = analog_read(LOS);

	int16_t fos_read = fos_read_on - fos_read_off;
  int16_t ros_read = ros_read_on - ros_read_off;
  int16_t los_read = los_read_on - los_read_off;

  int16_t lsr_read = analog_read(LSR);
  int16_t lsl_read = analog_read(LSL);
  int16_t lsb_read = analog_read(LSB);  
  
	OBS_REF = (fos_read + ros_read + los_read) / 3; // referência para detecção dos sensores de obstáculo
  LINE_REF = (lsr_read + lsl_read + lsb_read) / 3; // referência para detecção dos sensores de linha
}

void sense()
{
  digital_write(TFOS, LOW);
  digital_write(TROS, LOW);
  digital_write(TLOS, LOW);

  int16_t fos_read_off = analog_read(FOS);
  int16_t ros_read_off = analog_read(ROS);
  int16_t los_read_off = analog_read(LOS);

  digital_write(TFOS, HIGH);
  digital_write(TROS, HIGH);
  digital_write(TLOS, HIGH);

  int16_t fos_read_on = analog_read(FOS);
  int16_t ros_read_on = analog_read(ROS);
  int16_t los_read_on = analog_read(LOS);

  int16_t fos_read = fos_read_on - fos_read_off;
  int16_t ros_read = ros_read_on - ros_read_off;
  int16_t los_read = los_read_on - los_read_off;

  int16_t lsr_read = analog_read(LSR);
  int16_t lsl_read = analog_read(LSL);
  int16_t lsb_read = analog_read(LSB);  

  // SENSORES DE OBSTÁCULO
  //-----------------------------------
  // TESTE DO SENSOR DE OBSTÁCULO FRONTAL
	if (abs(fos_read - OBS_REF) > OBS_DIF)
	{
		FRONT_ENEMY_DETECT = 1;
	}
	else
	{
		FRONT_ENEMY_DETECT = 0;
	}
  // TESTE DO SENSOR DE OBSTÁCULO DIREITO
	if (abs(ros_read - OBS_REF) > OBS_DIF)
	{
		RIGHT_ENEMY_DETECT = 1;
	}
	else
	{
		RIGHT_ENEMY_DETECT = 0;
	}
  // TESTE DO SENSOR DE OBSTÁCULO ESQUERDO
	if (abs(los_read - OBS_REF) > OBS_DIF)
	{
		LEFT_ENEMY_DETECT = 1;
	}
	else
	{
		LEFT_ENEMY_DETECT = 0;
	}

  // SENSORES DE LINHA
  //-----------------------------------
  // TESTE DO SENSOR DE LINHA DIREITO
  if (abs(LINE_REF - lsr_read) > LINE_DIF)
  {
    RIGHT_LINE_DETECT = 1;
  }
  else
  {
    RIGHT_LINE_DETECT = 0;    
  }
  // TESTE DO SENSOR DE LINHA ESQUERDO
  if (abs(LINE_REF - lsl_read) > LINE_DIF)
  {
    LEFT_LINE_DETECT = 1;
  }
  else
  {
    LEFT_LINE_DETECT = 0;    
  }
  // TESTE DO SENSOR DE LINHA TRASEIRO
  if (abs(LINE_REF - lsb_read) > LINE_DIF)
  {
    BACK_LINE_DETECT = 1;
  }
  else
  {
    BACK_LINE_DETECT = 0;    
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif
