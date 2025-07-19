//Arduino Code for DRA818V FM TRX
//This code is written by OE3SDE, Simon Dorrer!

//Define CPU-Clock-Speed (16MHz internal clock)
//#define F_CPU 16000000UL

//Define Baudrate for UART
//#define BAUD 9600UL

//Include Libraries
#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>

//I2C
#include <Wire.h>

//Libs. for SD1306 OLED-Display
#include <Adafruit_GFX.h>               // Include core graphics library for the display
#include <Adafruit_SSD1306.h>           // Include Adafruit_SSD1306 library to drive the display
#include <Fonts/FreeMonoBold12pt7b.h>   // Add a custom font
#include <Fonts/FreeMono9pt7b.h>        // Add a custom font
/*List of fonts that support right alignment:
FreeMono9pt7b.h
FreeMono12pt7b.h
FreeMono18pt7b.h
FreeMono24pt7b.h
FreeMonoBold9pt7b.h
FreeMonoBold12pt7b.h
FreeMonoBold18pt7b.h
FreeMonoBold24pt7b.h
FreeMonoBoldOblique9pt7b.h
FreeMonoBoldOblique12pt7b.h
FreeMonoBoldOblique18pt7b.h
FreeMonoBoldOblique24pt7b.h
FreeMonoOblique9pt7b.h  
FreeMonoOblique12pt7b.h
FreeMonoOblique18pt7b.h
FreeMonoOblique24pt7b.h*/
//----------------------------------------------------------------------------

//Definitions
//I/O Declaration
//#define TX        0     //PD0 / 0
//#define RX        1     //PD1 / 1
#define SW1         2     //PD2 / 2 - INT0
#define PTT_IN      3     //PD3 / 3 - INT1
#define PTT_OUT     7     //PD7 / 7
#define PD          0     //PB0 / 8
#define H_L         1     //PB1 / 9
#define HC05_TX     2     //PB2 / 10
#define HC05_RX     3     //PB3 / 11
#define TX_LED      5     //PB5 / 13
#define E1          0     //PC0 / 14 (A0)
#define E2          1     //PC1 / 15 (A1)
//----------------------------------------------------------------------------

//Global variables

//OLED
Adafruit_SSD1306 display(128, 64);  // Create display
uint16_t timer1Count = 500;

//DRA818V Variables
uint8_t bw = 1;                //Bandwith in KHz (0= 12.5KHz or 1= 25KHz)
float tx_f = 145.5000;         //TX-Frequency in MHz (134.0000 - 174.0000)
float rx_f = 145.5000;         //RX-Frequency in MHz (134.0000 - 174.0000)
String tx_ctcss = "0000";      //CTCSS frequency (0000 - 0038); 0000 = "no CTCSS" 
String rx_ctcss = "0000";      //CTCSS frequency (0000 - 0038); 0000 = "no CTCSS" 
uint8_t squ = 4;               //Squelch level  (0 - 8); 0 = "open" 
uint8_t vol = 8;
uint8_t PRF = 0;
uint8_t HPF = 0;
uint8_t LPF = 0;
uint8_t tx = 0;                 // 0... RX; 1... TX

//Rotary Encoder Variables
uint8_t encoderPressedCounter = 1;
uint16_t timer2Count = 0;
int counter = 0; 
int aState;
int aLastState;  
//----------------------------------------------------------------------------

//Subprograms
//IO Subprograms
void IO_Init()                  //initialize IO
{
  //Define INPUTs
  DDRD &= ~(1 << SW1);          //set PD2 (SW1) as Input
  PORTD |= (1 << SW1);          //activate Pull-Up-R at PD2 (SW1)    
  
  DDRD &= ~(1 << PTT_IN);       //set PD3 (PTT-IN) as Input
  PORTD |= (1 << PTT_IN);       //activate Pull-Up-R at PD3 (PTT_IN)     
  
  DDRC &= ~(1 << E1);           //set PC0 (E1) as Input    
  DDRC &= ~(1 << E2);           //set PC1 (E2) as Input
  
  //Define OUTPUTs
  DDRD |= (1 << PTT_OUT);       //set PD7 (PTT_OUT) as Output
  DDRB |= (1 << PD);            //set PB0 (PD) as Output
  DDRB |= (1 << H_L);           //set PB1 (H_L) as Output
  
  DDRB |= (1 << TX_LED);        //set PB5 (TX_LED) as Output
  PORTB &= ~(1 << TX_LED);      //set PB5 (TX_LED) LOW at the beginning
}

void Timer1_COMPA_Init()
{
  //Control Registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
  
  // set CTC mode
  //TCCR1A |= (1 << WGM10);
  //TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12);
  
  // Set Prescaler 1024
  TCCR1B |= (1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B |= (1 << CS10);
  
  // initialize compare value
  OCR1A = 16;
  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

void Timer2_COMPA_Init()
{
  //Timer2 settings
  //Enable Timer2 CTC Mode
  TCCR2A = 0x00;
  TCCR2B = 0x00;
  
  //TCCR2A |= (1 << WGM20);
  TCCR2A |= (1 << WGM21);
  //TCCR2B |= (1 << WGM22);
  
  //Prescaler
  // 1024 prescaling for Timer2
  TCCR2B |= (1 << CS20);
  TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS22);

  //Initialize compare value
  OCR2A = 0;
  
  //initialize TIMER0-Counter
  //TCNT2 = 0; // set counter value FORMEL: x = maximaler Zählwert - ((CPUtakt/PRESCALER)/ gesuchte Frequenz)
  
  // disable Timer compare interrupt
  TIMSK2 &= ~(1 << OCIE2A);
}

void INT0_Init()
{
  //enable Interrupt
  (EIMSK |= (1 << INT0)); //Ext. Int0 ein
  
  //Set falling Edge Interrupt
  (EICRA &= ~(1 << ISC00));
  (EICRA |= (1 << ISC01));
}
void INT1_Init()
{
  //enable Interrupt
  (EIMSK |= (1 << INT1)); //Ext. Int1 ein
  
  //Set rising & falling Edge Interrupt
  (EICRA |= (1 << ISC10));
  (EICRA &= ~(1 << ISC11));
}

void DRA818V_setGroup()
{
  Serial.print("AT+DMOSETGROUP=");         // begin message
  Serial.print(bw);
  Serial.print(",");
  Serial.print(tx_f,4);
  Serial.print(",");
  Serial.print(rx_f,4);
  Serial.print(",");
  Serial.print(tx_ctcss);
  Serial.print(",");
  Serial.print(squ);
  Serial.print(",");
  Serial.println(rx_ctcss);
}
void DRA818V_setVolume()
{
  Serial.print("AT+DMOSETVOLUME=");
  Serial.println(vol);
}
void DRA818V_setFilter()
{
  Serial.print("AT+SETFILTER=");
  Serial.print(PRF);
  Serial.print(",");
  Serial.print(HPF);
  Serial.print(",");
  Serial.println(LPF);
}
void DRA818V_Init()             // initialize DRA818V
{
  PORTB &= ~(1 << PD);
  delay(2000);
  
  //I/O
  PORTD |= (1 << PTT_OUT);      // set PD7 (PTT_OUT) HIGH at the beginning (RX-Mode)
  PORTB |= (1 << PD);           // set PB0 (PD) HIGH at the beginning (Normal-Mode)
  PORTB &= ~(1 << H_L);         // set PB1 (H/L) LOW at the beginning (LOW-Power = 0.5W)

  //UART
  Serial.begin(9600);
  delay(10);
  DRA818V_setGroup();
  delay(500);
  DRA818V_setVolume();
  delay(500);
  DRA818V_setFilter();
  delay(500);
  //Serial.end();
}

void refreshDisplay()
{
  display.clearDisplay();
  
  // TX / RX
  if(tx == 1)
  {
    display.setCursor(10, 15);                  // (x,y)
    display.println("TX");
  }
  else if(tx == 0)
  {
    display.setCursor(10, 15);                  // (x,y)
    display.println("RX");
  }
  
  // Power
  display.setCursor(50, 15);                    // (x,y)
  display.println("PWR:.5W");

  // Frequency
  display.setCursor(30, 45);                    // (x,y)
  display.println(tx_f);

  display.display();
}
//----------------------------------------------------------------------------

//Setup (Initialize hardware)
void setup()
{
  // HW Init
  IO_Init();
  INT0_Init();
  INT1_Init();
  Timer1_COMPA_Init();
  Timer2_COMPA_Init();
  sei();                                      //enable global interrupts 

  // OLED Init
  delay(100);                                 // This delay is needed to let the display to initialize
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize display with the I2C address of 0x78
  display.clearDisplay();                     // Clear the buffer
  display.setTextColor(WHITE);                // Set color of the text
  display.setRotation(2);                     // Set orientation. Goes from 0, 1, 2 or 3
  display.setTextWrap(false);                 // By default, long lines of text are set to automatically “wrap” back to the leftmost column. 
                                              // To override this behavior (so text will run off the right side of the display - useful for scrolling marquee effects), use setTextWrap(false).
  display.dim(0);                             //Set brightness (0 is maximun and 1 is a little dim)
  display.setFont(&FreeMono9pt7b);            // Set a custom font
  display.setTextSize(0);                     // Set text size. We are using a custom font so you should always use the text size of 0
  
  display.setCursor(10, 15);                  // (x,y)
  display.println("1W FM TRX");                  // Text or value to print
  display.setCursor(10, 35);                  // (x,y)
  display.println("by OE3SDE");               // Text or value to print
  display.setCursor(10, 55);                  // (x,y)
  display.println("S. Dorrer");               // Text or value to print
  display.display();  // Print everything we set previously

  // DRA818V Init
  DRA818V_Init();
  
  // Reads the initial state of the outputA (E2)
  aLastState = digitalRead(E2); 

  display.clearDisplay();
  display.display();
}
//----------------------------------------------------------------------------

//Loop
void loop()
{
  if(timer1Count >= 500)
  {
    refreshDisplay();
    timer1Count = 0;
  }
  
  aState = digitalRead(E2); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState != aLastState)
   {     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(E1) != aState) 
     { 
       counter ++;
     } 
     else 
     {
       counter --;
     }
     display.setCursor(15, 55);                  // (x,y)
     display.println(counter);                  // Text or value to print
   } 
   aLastState = aState; // Updates the previous state of the outputA with the current state

}
//----------------------------------------------------------------------------

//ISP (Interrupt Service Routine) 
ISR(TIMER1_COMPA_vect)  // every 1ms
{
  timer1Count++;
}

ISR(TIMER2_COMPA_vect) // execute by TIMER2 CTC
{
  timer2Count++; //inkrementieren
  
  if(timer2Count == 4000) //256ms sperren
  {
    //enable INT0
    EIMSK |= (1 << INT0); //Ext. Int0 on
    
    //clear INTF0
    EIFR |= (1 << INTF0);
    
    //disable TIMER2 compare interrupt
    TIMSK2 &= ~(1 << OCIE2A);
    
    //reset of counter value
    timer2Count = 0;
  }
}

ISR(INT0_vect)  //Rotary Encoder Switch
{
  encoderPressedCounter++;
  
  //disable INT0
  EIMSK &= ~(1 << INT0); //Ext. Int0 off
  
  //enable TIMER2 compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  
  switch (encoderPressedCounter) 
  {
  case 1:
    tx_f = 145.5000;         //TX-Frequency in MHz
    rx_f = 145.5000;         //RX-Frequency in MHz
    DRA818V_setGroup();
    break;
  case 2:
    tx_f = 144.2000;         //TX-Frequency in MHz
    rx_f = 144.2000;         //RX-Frequency in MHz
    DRA818V_setGroup();
    break;
  case 3:
    tx_f = 144.5000;         //TX-Frequency in MHz
    rx_f = 144.5000;         //RX-Frequency in MHz
    DRA818V_setGroup();
    break;
  case 4:
    tx_f = 145.2000;         //TX-Frequency in MHz
    rx_f = 145.2000;         //RX-Frequency in MHz
    DRA818V_setGroup();
    break;
  case 5:
    tx_f = 145.0000;         //TX-Frequency in MHz
    rx_f = 145.0000;         //RX-Frequency in MHz
    DRA818V_setGroup();
    encoderPressedCounter = 0;
    break;
  default:
    encoderPressedCounter = 0;
    break;
  }
}

ISR(INT1_vect) //PTT-IN (active LOW)
{
  if(!(PIND & (1 << PTT_IN)))         //PTT_IN changed to LOW
  {
    PORTD &= ~(1 << PTT_OUT);
    PORTB |= (1 << TX_LED);           //set PB5 (TX_LED) HIGH
    tx = 1;
  }
  else if (PIND & (1 << PTT_IN))      //PTT_IN changed to HIGH
  {
    PORTD |= (1 << PTT_OUT);
    PORTB &= ~(1 << TX_LED);      //set PB5 (TX_LED) LOW
    tx = 0;
  }
}
//----------------------------------------------------------------------------
//End of Code!
