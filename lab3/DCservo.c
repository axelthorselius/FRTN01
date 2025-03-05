/** 
 * AVR program for control of the DC-servo process, 2024 edition.
 *  
 * User communication via the serial line. Commands:
 *   s: start controller
 *   t: stop controller
 *   r: change sign of reference (+/- 5.0 volt)
 * 
 * To compile for the ATmega8 AVR:
 *   avr-gcc -mmcu=atmega8 -O -g -Wall -o DCservo.elf DCservo.c   
 * 
 * To upload to the ATmega8 AVR:
 *   avr-objcopy -Osrec DCservo.elf DCservo.sr
 *   avrdude -e -p atmega8 -P /dev/ttyACM0 -c avrisp2 -U flash:w:DCservo.sr:a
 * 
 * To compile for the ATmega16 AVR:
 *   avr-gcc -mmcu=atmega16 -O -g -Wall -o DCservo.elf DCservo.c   
 * 
 * To upload to the ATmega16 AVR:
 *   avr-objcopy -Osrec DCservo.elf DCservo.sr
 *   avrdude -e -p atmega16 -P usb -c avrisp2 -U flash:w:DCservo.sr:a
 * 
 * To view the assembler code:
 *   avr-objdump -S DCservo.elf
 * 
 * To open a serial terminal on the PC:
 *   simcom -38400 /dev/ttyS0 
 */

#include <avr/io.h>
#include <avr/interrupt.h>

/* Controller parameters and variables (add your own code here) */

#define n 13 // ??? KB = 1.3.... K = 2.6.... kh/Ti = 0.289....

#define K 21412  //0101001100100100  // 2.613

#define KBETA   10704  // 1.3065 * 8192
#define KHTI    2366   // 0.2890 * 8192
#define KR      14606  // 1.7831 * 8192

#define LE_0    16677  // 2.0361 * 8192
#define LE_1    10186  // 1.2440 * 8192
#define LE_2    26287  // 3.2096 * 8192

#define KV_0    26946  // 3.2898 * 8192
#define KV_1    14606  // 1.7831 * 8192

#define GAMMA_0 919    // 0.1122 * 8192
#define GAMMA_1 115    // 0.0140 * 8192

#define PHI_00  8145   // 0.9940 * 8192
#define PHI_01  0      // 0 * 8192
#define PHI_10  2041   // 0.2493 * 8192
#define PHI_11  8192   // 1.0000 * 8192


int8_t on = 0;                     /* 0=off, 1=on */
int16_t r = 255;                   /* Reference, corresponds to +5.0 V */



static inline int16_t mul(int16_t X, int16_t Y)
{
  int32_t tmp;
  tmp = (int32_t) X * Y;
  tmp += (1 << n-1);
  tmp = tmp >> n;
  if (tmp > __INT16_MAX__)
  {
    tmp = __INT16_MAX__;
  }
  else if (tmp < -__INT16_MAX__ - 1)
  {
    tmp = -__INT16_MAX__ - 1;
  }
  return tmp;
}

static inline int16_t div(int16_t X, int16_t Y)
{

}

static inline int16_t add(int16_t X, int16_t Y)
{
  int32_t tmp;
  tmp = (int16_t)X + Y;
  return tmp;
}

static inline int16_t sub(int16_t X, int16_t Y)
{
  int32_t tmp;
  tmp = (int16_t) X - Y;
  return tmp;
}
/** 
 * Write a character on the serial connection
 */
static inline void put_char(char ch){
  while ((UCSRA & 0x20) == 0) {}; /* Wait until USART Data Register is empty */
  UDR = ch; /* Write to USART Data Register, sends data via serial cable */
}

/**
 * Write 10-bit output using the PWM generator
 */
static inline void writeOutput(int16_t val) {
  val += 512;
  OCR1AH = (uint8_t) (val>>8);
  OCR1AL = (uint8_t) val;
}

/**
 * Read 10-bit input using the AD converter from channel (0 or 1)
 */
static inline int16_t readInput(char chan) {
  uint8_t low, high;
  ADMUX = 0xc0 + chan;             /* Specify reference voltage and channel */
  ADCSRA |= 0x40;                  /* Start the conversion */
  while (ADCSRA & 0x40);           /* Wait for conversion to finish */
  low = ADCL;                      /* Read input, low byte first! */
  high = ADCH;                     /* Read input, high byte */
  return ((high<<8) | low) - 512;  /* 10 bit ADC value [-512..511] */ 
}  

/**
 * Interrupt handler for receiving characters over serial connection
 * Interrupt occurs when data has been received
 */
ISR(USART_RXC_vect){ 
  switch (UDR) {                   /* USART I/O Data Register */
  case 's':                        /* Start the controller */
    put_char('s');
    on = 1;
    break;
  case 't':                        /* Stop the controller */
    put_char('t');
    on = 0;
    break;
  case 'r':                        /* Change sign of reference */
    put_char('r');
    r = -r;
    break;
  }
}

/**
 * Interrupt handler for the periodic timer. Interrupts are generated
 * every 10 ms. The control algorithm is executed every 50 ms.
 */
ISR(TIMER2_COMP_vect){
  static int8_t ctr = 0;
  if (++ctr < 5) return;
  ctr = 0;
  if (on) {
    /* Insert your controller code here */

  } else {                     
    writeOutput(0);     /* Off */
  }
}

/**
 * Main program
 */
int main(){

  /* Set port data directions and configure ADC */
  DDRB = 0x02;    /* Enable PWM output for ATmega8 */
  DDRD = 0x20;    /* Enable PWM output for ATmega16 */
  DDRC = 0x30;    /* Enable time measurement pins */
  ADCSRA = 0xc7;  /* ADC enable + start + prescaling */

  /* Timer/Counter configuration */
  TCCR1A = 0xf3;  /* Timer 1: OC1A & OC1B 10 bit fast PWM */
  TCCR1B = 0x09;  /* Clock / 1 (i.e. no prescaling) */

  TCNT2 = 0x00;   /* Timer 2: Reset counter (periodic timer) */
  TCCR2 = 0x0f;   /* Clock / 1024, clear after compare match (CTC) */
  OCR2 = 144;     /* Set the output compare register, corresponds to ~100 Hz */

  /* Configure serial communication */
  /* Set USART Control and Status Registers */
  UCSRA = 0x00;   /* USART: */
  UCSRB = 0x98;   /* USART: RXC enable, Receiver enable, Transmitter enable */
  UCSRC = 0x86;   /* USART: 8bit, no parity, asynchronous */
  /* 12bit USART baud rate register (high and low byte) */
  UBRRH = 0x00;   /* USART: 38400 @ 14.7456MHz */
  UBRRL = 23;     /* USART: 38400 @ 14.7456MHz */

  TIMSK = 1<<OCIE2; /* Start periodic timer */

  sei();          /* Enable interrupts */

  while (1);
}
