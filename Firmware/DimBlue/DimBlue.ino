#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define LEDON PORTC |= (1<<7);   
#define LEDOFF PORTC &= ~(1<<7);   

volatile uint8_t synccnt;
volatile uint16_t tick;
volatile uint16_t ticks;
volatile uint16_t ledValues[16];
volatile uint16_t led[16];

// HARDWARE INT6/PE6 LINE FREQUENCY SYNC
ISR(INT6_vect) {
  ticks=tick;
  tick=0;

  // Turn off all optocouplers
  PORTB=0;
  PORTF=0;
  PORTD&=(0b00001100);

  // Reload counters with the original values 
  // for a new count-down
  led[0]=ledValues[0];
  led[1]=ledValues[1];
  led[2]=ledValues[2];
  led[3]=ledValues[3];
  led[4]=ledValues[4];
  led[5]=ledValues[5];
  led[6]=ledValues[6];
  led[7]=ledValues[7];
  led[8]=ledValues[8];
  led[9]=ledValues[9];
  led[10]=ledValues[10];
  led[11]=ledValues[11];
  led[12]=ledValues[12];
  led[13]=ledValues[13];
  led[14]=ledValues[14];
  led[15]=ledValues[15];
  synccnt++;
}


// TIMER1 COMPARE INTERRUPT
ISR(TIMER1_COMPA_vect) {
  // Keep track of the number of timer interrupts for each
  // power period. (Not really used for anything)  
  tick++;
  
  // Count down each light-counter and turn on the triac
  // when counter reaches zero
  if ((led[0]--)==0)  PORTB|=(1<<7);   
  if ((led[1]--)==0)  PORTB|=(1<<6);   
  if ((led[2]--)==0)  PORTB|=(1<<5);   
  if ((led[3]--)==0)  PORTB|=(1<<4);   
  if ((led[4]--)==0)  PORTB|=(1<<3);   
  if ((led[5]--)==0)  PORTB|=(1<<2);   
  if ((led[6]--)==0)  PORTB|=(1<<1);   
  if ((led[7]--)==0)  PORTB|=(1<<0);   
  if ((led[8]--)==0)  PORTD|=(1<<7);   
  if ((led[9]--)==0)  PORTD|=(1<<6);   
  if ((led[10]--)==0) PORTD|=(1<<5);   
  if ((led[11]--)==0) PORTD|=(1<<4);   
  if ((led[12]--)==0) PORTF|=(1<<0);   
  if ((led[13]--)==0) PORTF|=(1<<1);   
  if ((led[14]--)==0) PORTD|=(1<<1);   
  if ((led[15]--)==0) PORTD|=(1<<0);   
}




void setup() {
  int i;
  
  Serial.begin(9600);
  while (!Serial) ;
  Serial.println("DimBlue 32U4");

//  pinMode(7, INPUT);
//  digitalWrite(7,LOW);      // Turn off pullup
  synccnt=0;

  DDRB|=0b11111111;    // Outputs for LIGHT1-LIGHT8
  DDRD|=0b11110011;    // Outputs for LIGHT9-12 & LIGHT15-16
  DDRF|=0b00000011;    // Outputs for LIGHT13-14
  DDRC|=0b10000000;    // Output for PC7 Status LED    
  DDRE&=~0b01000000;   // PE6 is input for line freq interrup
  PORTD&=~0b01000000;  // Turn off PE6 pullup
  
  for (i=0; i<16;i++) ledValues[i]=480;

  cli();                  // Disable global interrupts during setup

  // Setup hardware INT6 on Falling Edge pin PE6 for line frequency synchronization  
  EICRB = (EICRA & ~((1 << ISC60) | (1 << ISC61))) | (1 << ISC61);
  EIMSK |= (1<<INT6);

  // Setup Timer1 for interrupts every 20uS - each power half cycle is 500 periods
  TCCR1A = 0;             // set entire TCCR1A register to 0
  TCCR1B = 0;             // same for TCCR1B
  OCR1A = 319;            // Compare match register
  TCCR1B |= (1<<WGM12);   // turn on CTC mode
  TCCR1B |= (1<<CS10);    // Set CS10 for /1 prescaler

  TIMSK1 |= (1<<OCIE1A);  // enable timer compare interrupt:
  
  sei();                  // enable global interrupts

  DDRC |= (1<<7);   
}

void loop() {
  int c=0;
  int dir=1;
  uint16_t v;
  for (;;) {
    if (synccnt>=10) {
      synccnt=0;
      v=ticks;
      Serial.println(ledValues[0]);
      ledValues[0]+=dir;
      if (ledValues[0]==0) dir=-dir;
      if (ledValues[0]==499) dir=-dir;
    }
  }

  for (;;);
}
