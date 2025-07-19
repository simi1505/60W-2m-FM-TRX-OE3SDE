//Libraries
#include <avr/io.h>      // This defers to avr/io.h for GCC
#include <avr/interrupt.h>

uint8_t lastState = 0;
uint8_t currentState = 0;
uint16_t timer1Count = 0;

void INT0_Init(){
  //enable Interrupt
  EIMSK |= (1 << INT0); //Ext. Int0 ein
  
  //Set every Edge Interrupt)
  EICRA &= ~(1 << ISC01);
  EICRA |= (1 << ISC00);
}

void Timer1_COMPA_Init(){
  //Control Registers
  TCCR1A = 0; TCCR1B = 0; TCCR1C = 0; //Reset
  
  // set CTC mode
  //TCCR1A |= (1 << WGM10);
  //TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12);
  
  // Set Prescaler 1024 (TCCR1B = (1 << WGM12) | (0x5 << CS10);)
  TCCR1B |= (1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B |= (1 << CS10);
  
  // initialize compare value
  OCR1A = 16;
  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

void setup(){
  INT0_Init();
  Timer1_COMPA_Init();
  sei();
 
  //Set Input
  DDRC &= ~(1 << 0);  //PC0 - A0 - PCINT8
  DDRC &= ~(1 << 1);  //PC1 - A1 - PCINT9
  DDRD &= ~(1 << 2);          //set PD2 (SW1) as Input
  
  //Set Pull-UP-Resitor
  PORTC |= (1 << 0);  //PC0 - A0 - PCINT8
  PORTC |= (1 << 1);  //PC1 - A1 - PCINT9
  PORTD |= (1 << 2);          //activate Pull-Up-R at PD2 (SW1)   

  DDRC &= ~(1 << 3);      //set PC3 (ADC_VDD) as Input  
  PORTC &= ~(1 << 3);     //deactivate Pull-Up-R at PC3 (ADC_VDD)
  Serial.begin(9600);

  lastState = PINC & (1 << 0);
}

void loop(){
  //ToDo
}

//ISR
ISR(TIMER1_COMPA_vect){  // every 1ms
  timer1Count++;

  currentState = PINC & (1 << 0);
  
  if (currentState != lastState && currentState == 1){
    if(!(PINC & (1 << 1)) == currentState) {
      Serial.println("CCW!");
    } else {
      Serial.println("CW!");
    }
  }
  lastState = currentState;
}

ISR(INT0_vect){  //Rotary Encoder Switch
  if(PIND & (1 << 2)){    // HIGH?
    if(timer1Count >= 250){
      Serial.println("Long pressed!");
    } else {
      Serial.println("Short pressed!");
    }

    timer1Count = 0;
  }

  if(!(PIND & (1 << 2))){ // LOW?
    timer1Count = 0;
  }
}
