//Arduino Code for DRA818V FM TRX + 60W RA60H1317M1A
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

//MCP4728
#include <Adafruit_MCP4728.h>

//DS18B20
#include <OneWire.h> 
#include <DallasTemperature.h>

//Libs. for SD1306 OLED-Display
#include <U8x8lib.h>
//----------------------------------------------------------------------------

//Definitions
//I/O Declaration
//1W FM TRX Board (DRA818V)
//#define TX        0     //PD0 / D0
//#define RX        1     //PD1 / D1
#define SW1         2     //PD2 / D2 - INT0
#define PTT_IN      3     //PD3 / D3 - INT1
#define PTT_OUT     7     //PD7 / D7
#define PD          0     //PB0 / D8
#define H_L         1     //PB1 / D9
#define HC05_TX     2     //PB2 / D10
#define HC05_RX     3     //PB3 / D11
#define E1          0     //PC0 / A0
#define E2          1     //PC1 / A1

//60W FM TRX Board (RA60H1317M1A)
#define PGA_SHDN    5     //PB5 / D13 (LOW --> PGA OFF, HIGH --> PGA ON) + RX Switch
#define TX_SW       4     //PB4 / D12 (HIGH: TX Switch ON, RX Switch OFF)
#define DS18B20_OW  6     //PD6 / D6  (One wire input of DS18B20 Temp. Sensor)
#define SWR_REV     7     //PC7 / A7  (ADC samples reverse voltage)
#define SWR_FWD     6     //PC6 / A6  (ADC samples forward voltage)
#define ADC_VDD     3     //PC3 / A3  (ADC samples VDD voltage) 

//I2C: OLED Display, MCP4728 (MCP4728_CHANNEL_C is unused!)

//Makros
//RX or TX ATTENUATOR
#define RX_ATTENUATOR_CHANNEL  MCP4728_CHANNEL_A
#define TX_ATTENUATOR_CHANNEL  MCP4728_CHANNEL_B

//PA max. ratings
#define VDD_LOWER_LIMIT 10.7  //in V
#define VDD_UPPER_LIMIT 14.2  //in V
#define SWR_LOWER_LIMIT 0.5   
#define SWR_UPPER_LIMIT 2
#define HEAT_UPPER_LIMIT 45   //in degrees
//----------------------------------------------------------------------------

//Global variables / Objects
uint8_t TX = 0;  // 0... RX; 1... TX
uint8_t switched = 0;

//OLED
U8X8_SH1106_128X64_NONAME_HW_I2C SH1106(/* reset=*/ U8X8_PIN_NONE); //0x3C I²C Address
uint8_t page = 1;
uint8_t clearOLED = 0;
uint8_t save = 1;

volatile uint16_t updateCount = 0;
volatile uint16_t pressedCount = 0;

//DRA818 Variables
typedef struct{
  float txFreq;         //TX-Frequency in MHz (134.0000 - 174.0000)
  float rxFreq;         //RX-Frequency in MHz (134.0000 - 174.0000)
  String txCTCSS;       //CTCSS frequency (0000 - 0038); 0000 = "no CTCSS" 
  String rxCTCSS;       //CTCSS frequency (0000 - 0038); 0000 = "no CTCSS" 
  uint8_t bw;           //Bandwith in kHz (0 = 12.5KHz or 1 = 25KHz)
  uint8_t squ;          //Squelch level  (0 - 8); 0 = "open" 
  uint8_t vol;
  uint8_t prf;
  uint8_t hpf;
  uint8_t lpf;
  uint8_t steps;        //frequency steps (1 = 12.5kHz, 2 = 25kHz, 4 = 50kHz, 8 = 100kHz)
}DRA818;

//CTCSS:
//https://en.wikipedia.org/wiki/Continuous_Tone-Coded_Squelch_System
//https://www.oevsv.at/oevsv/aktuelles/UKW-Bandplaene-Relais-etc.-CTCSS-in-OE-mit-System/
/*Wien (OE1)              EIA 01    67,0 Hz
Salzburg (OE2)            EIA 08    88,5 Hz
Niederösterreich (OE3)    EIA 26    162,2 Hz
Burgenland (OE4)          EIA 11    97,4 Hz
Oberösterreich (OE5)      EIA 18    123,0 Hz
Steiermark (OE6)          EIA 13    103,5 Hz
Tirol (OE7)               EIA 04    77,0 Hz
Kärnten (OE8)             EIA 08    88,5 Hz
Vorarlberg (OE9)          EIA 07    85,4 Hz*/
String ctcss_arr[] = {"0000", "0001", "0008", "0026", "0011", "0018", "0013", "0004", "0008", "0007"};
uint8_t idxTXCTCSS = 0;
uint8_t idxRXCTCSS = 0;

DRA818 dra818 = {145.5000, 145.5000, ctcss_arr[0], ctcss_arr[0], 0, 4, 8, 0, 0, 0, 2};

//DS18B20
OneWire oneWire(DS18B20_OW); 
DallasTemperature ds18b20(&oneWire);

//MCP4728
Adafruit_MCP4728 mcp4728;

//Rotary Encoder Variables
uint8_t lastState = 0;
uint8_t currentState = 0;

//RA60H1317M1A Variables in dB
uint8_t idxPWR = 0; //TX: 0: 5W = 37dBm, 1: 10W = 40dBm, 2: 20W = 43dBm, 3: 32W = 45dBm, 4: 50W = 47dBm, 5: no PA, only DRA818
String pwr_arr[] = {"5W", "10W", "20W", "32W", "50W", "0.5W"};
float txGain = 1;   //0... 0V, 1... 4.7V, 2... 5V @ RA60H Gate; RX --> 0, TX = 5W to 32W --> 1, TX = 50W --> 2
float txAtt = 12;   //5W = 17, 10W = 12, 20W = 3, 32W = 0, 40W = 0 @ 12.5VDC --> Higher PWR with higher VDD (max. 14.2V)
float rxAtt = 0;
uint8_t monitorPA = 0;  //monitorPA = 1... PA is OK!
//----------------------------------------------------------------------------

//Subprograms
//Hardware Init Methods
void IO_Init(){                 //initialize IO
  //Define INPUTs
  DDRD &= ~(1 << SW1);          //set PD2 (SW1) as Input
  PORTD |= (1 << SW1);          //activate Pull-Up-R at PD2 (SW1)    
  
  DDRD &= ~(1 << PTT_IN);       //set PD3 (PTT-IN) as Input
  PORTD |= (1 << PTT_IN);       //activate Pull-Up-R at PD3 (PTT_IN)     
  
  DDRC &= ~(1 << E1);           //set PC0 (E1) as Input    
  DDRC &= ~(1 << E2);           //set PC1 (E2) as Input
  PORTC |= (1 << E1);           //activate Pull-Up-R at PC0 (E1)
  PORTC |= (1 << E2);           //activate Pull-Up-R at PC1 (E2)
  
  DDRC &= ~(1 << ADC_VDD);      //set PC3 (ADC_VDD) as Input  
  PORTC |= (1 << ADC_VDD);      //activate Pull-Up-R at PC3 (ADC_VDD)

  DDRC &= ~(1 << SWR_FWD);      //set PC6 (SWR_FWD) as Input  
  PORTC |= (1 << SWR_FWD);      //activate Pull-Up-R at PC6 (SWR_FWD)

  DDRC &= ~(1 << SWR_REV);      //set PC7 (SWR_REV) as Input  
  PORTC |= (1 << SWR_REV);      //activate Pull-Up-R at PC7 (SWR_REV)
  
  //Define OUTPUTs
  DDRD |= (1 << PTT_OUT);       //set PD7 (PTT_OUT) as Output
  DDRB |= (1 << PD);            //set PB0 (PD) as Output
  DDRB |= (1 << H_L);           //set PB1 (H_L) as Output
  
  DDRB |= (1 << TX_SW);         //set PB4 (TX_Switch) as Output
  DDRB |= (1 << PGA_SHDN);      //set PB5 (PGA_SHDN) as Output
}

void Timer1_COMPA_Init(){
  //Control Registers
  TCCR1A = 0; 
  TCCR1B = 0; 
  TCCR1C = 0; //Reset
  
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

void INT0_Init(){
  //enable Interrupt
  EIMSK |= (1 << INT0); //Ext. Int0 ein
  
  //Set every Edge Interrupt
  EICRA |= (1 << ISC00);
  EICRA &= ~(1 << ISC01);
}

void INT1_Init(){
  //enable Interrupt
  EIMSK |= (1 << INT1); //Ext. Int1 ein
  
  //Set rising & falling Edge Interrupt (EICRA |= (0x2 << ISC10);)
  EICRA |= (1 << ISC10);
  EICRA &= ~(1 << ISC11);
}

void ADC_Init(){
  //ADC Setup
  //Set Reference Voltage (VCC = VREF)
  ADMUX &= ~(1 << REFS1);
  ADMUX |= (1 << REFS0);

  //Prescaler --> 128 (50kHz - 200kHz)
  ADCSRA |= (1 << ADPS0);
  ADCSRA |= (1 << ADPS1);
  ADCSRA |= (1 << ADPS2);
  
  ADCSRA |= (1 << ADEN); //Enable ADC

  //Dummy Readout to warm up the ADC
  ADCSRA |= (1<<ADSC);          //Start ADC conclusion
  while (ADCSRA & (1<<ADSC)){}  //wait until ADC is ready
  (void) ADC;
}

uint16_t ADC_ReadValue(uint8_t channel){
  ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F); //which ADCx? Bit Mask to secure ADMUX settings in Init() function
  
  ADCSRA |= (1<<ADSC);          //Start ADC conclusion
  while (ADCSRA & (1<<ADSC)){}  //wait until ADC is ready
  
  return ADC;                   //return the ADC value
}
//--------------------------------------------------------------------------

//DRA818V Methods
void DRA818V_setGroup(){
  Serial.print("AT+DMOSETGROUP=");
  Serial.print(dra818.bw);
  Serial.print(",");
  Serial.print(dra818.txFreq, 4);
  Serial.print(",");
  Serial.print(dra818.rxFreq, 4);
  Serial.print(",");
  Serial.print(dra818.txCTCSS);
  Serial.print(",");
  Serial.print(dra818.squ);
  Serial.print(",");
  Serial.println(dra818.rxCTCSS);
}

void DRA818V_setVolume(){
  Serial.print("AT+DMOSETVOLUME=");
  Serial.println(dra818.vol);
}

void DRA818V_setFilter(){
  Serial.print("AT+SETFILTER=");
  Serial.print(dra818.prf);
  Serial.print(",");
  Serial.print(dra818.hpf);
  Serial.print(",");
  Serial.println(dra818.lpf);
}

void DRA818V_Init(){             // initialize DRA818V
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
}
//--------------------------------------------------------------------------

//RA60H1317M1A Methods
float getVDD(){      //Get VDD Voltage (10.8VDC to 13.6VDC)
  float vdd = ADC_ReadValue(ADC_VDD) * 5.0 / 1024.0;
  if(ADC_ReadValue(ADC_VDD) < 1020){
    vdd = vdd * ((4700 + 18000) / 4700) + 0.8; //magic numbers equals the voltage divider resistor values
  }  
  return vdd;  //returns supply voltage without voltage division
}

float getSWR(){      //Get SWR of output load (should be between 0.5 and 2)
  float rev = 0;
  float fwd = 0;
  
  for(int i = 0; i < 10; i++){   //10 iterations for a more precise result
    rev = rev + ADC_ReadValue(SWR_REV);
    fwd = fwd + ADC_ReadValue(SWR_FWD);
  }
  rev = rev / 10 / 1024 * 5;
  fwd = fwd / 10 / 1024 * 5;

  //SWR Calculation
  return((fwd + rev) / (fwd - rev));
}

float getPWR(){
  float fwd = 0;
  
  for(int i = 0; i < 10; i++){   //10 iterations for a more precise result
    fwd = fwd + ADC_ReadValue(SWR_FWD);
  }
  fwd = fwd / 10 / 1024 * 5;

  //PWR Calculation
  return((fwd * fwd) / 50);
}

float getHeat(){     //Get heat of RA60H1317M1A measured by DS18B20, should be smaller than 50
  ds18b20.requestTemperatures();
  float temp = ds18b20.getTempCByIndex(0);
  if(temp <= -30){temp = -30;}
  if(temp >= 60){temp = 60;}
  return temp;
}

void setTXGain(float gain){  //Set Gain of RA60H1317M1A via MCP4728 (VOUTD)
  if(gain == 0){
    mcp4728.setChannelValue(MCP4728_CHANNEL_D, 0);
  }else if(gain == 1){
    mcp4728.setChannelValue(MCP4728_CHANNEL_D, 3700);
  }else if(gain == 2){
    mcp4728.setChannelValue(MCP4728_CHANNEL_D, 4000);
  }
}

void setAttenuation(MCP4728_channel_t channel, float att){ //Attenuation of U11 (RX) or U12 (TX) (F2255NLGK)
  //Note that the PGA-103 has a constant gain of 15dB for RX path! RX_ATTENUATOR_CHANNEL MCP4728_CHANNEL_A
  //Note that an 3dB Attenuator is assembled on board for TX path! TX_ATTENUATOR_CHANNEL MCP4728_CHANNEL_B
  
  if(att > 35){
    mcp4728.setChannelValue(channel, 2048);
  }else if(att < 2.5){
    mcp4728.setChannelValue(channel, 0);
  }else{
    att = att * (-1);
    mcp4728.setChannelValue(channel, (uint16_t)(4096 * (-0.03692 * att + 0.60769) / 5));  //0.7V (-2.5dB) to 1.9V (-35dB), magic numbers are calculated according to the attenuation curve of the datasheet!
  }
}

void setTXPower(uint8_t pwr){
  switch(pwr) {
    case 0:   //5W
        txGain = 1;
        txAtt = 17;
        break;
    case 1:   //10W
        txGain = 1;
        txAtt = 12;
        break;
    case 2:   //20W
        txGain = 1;
        txAtt = 3;
        break;
    case 3:   //32W
        txGain = 1;
        txAtt = 0;
        break;
    case 4:   //50W
        txGain = 2;
        txAtt = 0;
        break;
    case 5:   //0.5W
        txGain = 0;
        txAtt = 36;
        break;
    default:
        break;
  }
}

void switchPAtoTX(uint8_t TX_Att, uint8_t TX_Gain){
  if(monitorPA == 1){
    //turn OFF RX:
    PORTB &= ~(1 << PGA_SHDN);                        //turn OFF PGA RX-Amp. and RX-IN Switch (D13 to LOW)
    setAttenuation(RX_ATTENUATOR_CHANNEL, 36);        //set full attenuation of U11 (F2255NLGK)
    
    delay(100);     //100ms settling time
    
    //turn ON TX:
    PORTB |= (1 << TX_SW);                            //turn ON TX-OUT Switch, turn OFF RX-OUT Switch (D12 to HIGH)
    PORTD &= ~(1 << PTT_OUT);                         //turn TX Path in TRX-IN Switch ON (D7 LOW)
    setAttenuation(TX_ATTENUATOR_CHANNEL, TX_Att);    //set attenuation of U12 (F2255NLGK)
    setTXGain(TX_Gain);                               //Set TX gain (VGG (Gate) voltage) of RA60H1317M1A
    delay(100);                                       //100ms settling time
  }else{
    switchPAtoRX(rxAtt);
  }
}

void switchPAtoRX(float RX_Att){
  //turn OFF TX:
  setTXGain(0);                                       //Set gain of RA60H1317M1A to 0
  setAttenuation(TX_ATTENUATOR_CHANNEL, 36);          //set full attenuation of U12 (F2255NLGK)
  PORTD |= (1 << PTT_OUT);                            //turn TX Path in TRX-IN Switch OFF (D7 HIGH)
  PORTB &= ~(1 << TX_SW);                             //turn ON RX-OUT Switch, turn OFF TX-OUT Switch (D12 to LOW)
  delay(100);                                         //100ms settling time
  
  //turn ON RX
  PORTB |= (1 << PGA_SHDN);                           //turn ON PGA RX-Amp. and RX-IN Switch (D13 to HIGH)
  setAttenuation(RX_ATTENUATOR_CHANNEL, RX_Att);      //set attenuation of U11 (F2255NLGK)
  delay(100);                                         //100ms settling time
}

void verifyPA(){   //Shutdown RA60H1317M1A according to SWR, VDD and Heat
  float vdd = getVDD();
  float swr = getSWR();
  float heat = getHeat();
  
  //if((vdd >= VDD_LOWER_LIMIT && vdd <= VDD_UPPER_LIMIT) && (swr >= SWR_LOWER_LIMIT && swr <= SWR_UPPER_LIMIT) && heat <= HEAT_UPPER_LIMIT)
  if((vdd >= VDD_LOWER_LIMIT && vdd <= VDD_UPPER_LIMIT) && heat <= HEAT_UPPER_LIMIT){
    monitorPA = 1;  //PA is OK!
  }else{
    monitorPA = 0;
    if(TX == 1){
      switchPAtoRX(rxAtt);
    }
  }
}

//Display Methods
void displayInit(){
  SH1106.begin();
  SH1106.clear();
  //FONTS: https://github.com/olikraus/u8g2/wiki/fntlist8x8
  SH1106.setFont(u8x8_font_8x13_1x2_r);
}

void displayStartScreen(){
  //Set start Text 
  SH1106.setFont(u8x8_font_8x13B_1x2_r);
  SH1106.drawString(3, 0, "60W FM TRX");
  SH1106.drawString(3, 3, "by OE3SDE");
  SH1106.drawString(3, 6, "S. Dorrer");
  SH1106.setFont(u8x8_font_8x13_1x2_r);
}

void refreshDisplay(){
  if(clearOLED == 1){
    SH1106.clear();
    clearOLED = 0;
  }

  if(save == 1){
    DRA818V_setGroup();
    DRA818V_setVolume();        
    setTXPower(idxPWR);  //set output power of PA
    setAttenuation(RX_ATTENUATOR_CHANNEL, rxAtt);  //set RX attenuation
    SH1106.setFont(u8x8_font_courB18_2x3_r);
    SH1106.drawString(2, 3, "Saved!");
    SH1106.setFont(u8x8_font_8x13_1x2_r);
    clearOLED = 1;
    save = 0;
  }else{
    if(page == 1){
      if(TX == 1){ // TX / RX
        if(getHeat() > HEAT_UPPER_LIMIT){
          SH1106.drawString(0, 0, "HOT!");
        } else {
          SH1106.drawString(0, 0, "TX");
        }
        // Frequency
        SH1106.setCursor(1, 2);
        SH1106.setFont(u8x8_font_courB18_2x3_r);
        SH1106.print(dra818.txFreq, 3);
      }else if(TX == 0){
        SH1106.drawString(0, 0, "RX");
        // Frequency
        SH1106.setCursor(1, 2);
        SH1106.setFont(u8x8_font_courB18_2x3_r);
        SH1106.print(dra818.rxFreq, 3);
      }
      SH1106.setFont(u8x8_font_8x13_1x2_r);
      
      //Power
      SH1106.setCursor(11, 0);
      SH1106.print(pwr_arr[idxPWR]);
      
      //Values
      SH1106.setCursor(0, 5);
      SH1106.print(getVDD());
      SH1106.print("V");
      
      SH1106.setCursor(10, 5);
      SH1106.print(getHeat(), 0);
      SH1106.print("'C");
      
    }else if (page == 2){
      SH1106.drawString(0, 0, "Frequency Step: ");
      SH1106.drawString(0, 2, "----------------");
      SH1106.setFont(u8x8_font_8x13B_1x2_r);
      SH1106.setCursor(3, 5);
      if(dra818.steps == 1){
        SH1106.print("> 12.5kHz <");
      } else if(dra818.steps == 2){
        SH1106.print("> 25kHz <");
      } else if(dra818.steps == 4){
        SH1106.print("> 50kHz <");
      } else if(dra818.steps == 8){
        SH1106.print("> 100kHz <");
      }
      SH1106.setFont(u8x8_font_8x13_1x2_r);
      SH1106.setCursor(14, 6); 
      SH1106.print(page);
    
    }else if (page == 3){
      SH1106.drawString(0, 0, "TX-Frequency: ");
      SH1106.drawString(0, 2, "----------------");
      SH1106.setFont(u8x8_font_8x13B_1x2_r);
      SH1106.setCursor(2, 5);
      SH1106.print("> ");
      SH1106.print(dra818.txFreq, 4);
      SH1106.print(" <");
      SH1106.setFont(u8x8_font_8x13_1x2_r);
      SH1106.setCursor(14, 6); 
      SH1106.print(page);
      
    }else if (page == 4){
      SH1106.drawString(0, 0, "RX-Frequency: ");
      SH1106.drawString(0, 2, "----------------");
      SH1106.setFont(u8x8_font_8x13B_1x2_r);
      SH1106.setCursor(2, 5);
      SH1106.print("> ");
      SH1106.print(dra818.rxFreq, 4);
      SH1106.print(" <");
      SH1106.setFont(u8x8_font_8x13_1x2_r);
      SH1106.setCursor(14, 6); 
      SH1106.print(page);
      
    }else if (page == 5){
      SH1106.drawString(0, 0, "Bandwidth: ");
      SH1106.drawString(0, 2, "----------------");
      SH1106.setFont(u8x8_font_8x13B_1x2_r);
      SH1106.setCursor(2, 5);
      if(dra818.bw == 0){
        SH1106.print("> 12.5kHz <");
      }
      if(dra818.bw == 1){
        SH1106.print("> 25kHz <");
      }
      SH1106.setFont(u8x8_font_8x13_1x2_r);
      SH1106.setCursor(14, 6); 
      SH1106.print(page);
      
    }else if (page == 6){
      SH1106.drawString(0, 0, "TX-Power: ");
      SH1106.drawString(0, 2, "----------------");
      SH1106.setFont(u8x8_font_8x13B_1x2_r);
      SH1106.setCursor(4, 5);
      SH1106.print("> ");
      SH1106.print(pwr_arr[idxPWR]);
      SH1106.print(" <");
      
      SH1106.setFont(u8x8_font_8x13_1x2_r);
      SH1106.setCursor(14, 6); 
      SH1106.print(page);
      
    }else if (page == 7){
      SH1106.drawString(0, 0, "RX-Attenuation: ");
      SH1106.drawString(0, 2, "----------------");
      SH1106.setFont(u8x8_font_8x13B_1x2_r);
      SH1106.setCursor(4, 5);
      SH1106.print("> ");
      SH1106.print(rxAtt + 10, 0);
      SH1106.print("dB <");
      SH1106.setFont(u8x8_font_8x13_1x2_r);
      SH1106.setCursor(14, 6); 
      SH1106.print(page);
      
    }else if (page == 8){
      SH1106.drawString(0, 0, "Volume: ");          //1 - 8
      SH1106.drawString(0, 2, "----------------");
      SH1106.setFont(u8x8_font_8x13B_1x2_r);
      SH1106.setCursor(5, 5);
      SH1106.print("> ");
      SH1106.print(dra818.vol);
      SH1106.print(" <");
      SH1106.setFont(u8x8_font_8x13_1x2_r);
      SH1106.setCursor(14, 6); 
      SH1106.print(page);
      
    }else if (page == 9){
      SH1106.drawString(0, 0, "Squelch: ");         //0 - 8: 0 = "open" 
      SH1106.drawString(0, 2, "----------------");
      SH1106.setFont(u8x8_font_8x13B_1x2_r);
      SH1106.setCursor(5, 5);
      SH1106.print("> ");
      SH1106.print(dra818.squ);
      SH1106.print(" <");
      SH1106.setFont(u8x8_font_8x13_1x2_r);
      SH1106.setCursor(14, 6); 
      SH1106.print(page);
      
    }else if (page == 10){
      SH1106.drawString(0, 0, "TX-CTCSS: ");
      SH1106.drawString(0, 2, "----------------");
      SH1106.setFont(u8x8_font_8x13B_1x2_r);
      SH1106.setCursor(4, 5);
      SH1106.print("> ");
      SH1106.print(dra818.txCTCSS);
      SH1106.print(" <");
      SH1106.setFont(u8x8_font_8x13_1x2_r);
      SH1106.setCursor(13, 6); 
      SH1106.print(page);
      if(idxTXCTCSS != 0){
        SH1106.drawString(12, 0, "OE");
        SH1106.setCursor(14, 0);
        SH1106.print(idxTXCTCSS);
      }
      
    }else if (page == 11){
      SH1106.drawString(0, 0, "RX-CTCSS: ");
      SH1106.drawString(0, 2, "----------------");
      SH1106.setFont(u8x8_font_8x13B_1x2_r);
      SH1106.setCursor(4, 5);
      SH1106.print("> ");
      SH1106.print(dra818.rxCTCSS);
      SH1106.print(" <");
      SH1106.setFont(u8x8_font_8x13_1x2_r);
      SH1106.setCursor(13, 6); 
      SH1106.print(page);
      if(idxRXCTCSS != 0){
        SH1106.drawString(12, 0, "OE");
        SH1106.setCursor(14, 0);
        SH1106.print(idxRXCTCSS);
      }
      
    }else if (page == 12){
      SH1106.drawString(0, 0, "Instructions: ");
      SH1106.drawString(0, 2, "1 Tap: Next");
      SH1106.drawString(0, 4, "Rotate: Change");
      SH1106.drawString(0, 6, "Long Tap: Save");
    }
  }
}
//----------------------------------------------------------------------------

//Setup (Initialize hardware)
void setup(){
  //HW Init
  IO_Init();
  INT0_Init();
  INT1_Init();
  Timer1_COMPA_Init();
  ADC_Init();
  
  // OLED Init
  displayInit();
  displayStartScreen();

  //DRA818V Init
  DRA818V_Init();
  
  //PA HW Init
  if(getVDD() > VDD_LOWER_LIMIT){  //Just init. MCP4728 if voltage is bigger than VDD_LOWER_LIMIT
    ds18b20.begin();    //DS18B20 Init 
    
    //MCP4728 Init (MCP4728_CHANNEL_C is unused!)
    if(!mcp4728.begin()){
      Serial.println("Failed to find MCP4728 chip!");
      while (1){
        delay(10);
      }
    }
    
    //RA60H1317M1A
    verifyPA();
    switchPAtoRX(rxAtt); //standard attenuation of 10dB
  }

  sei(); //enable global interrupts

  SH1106.clear();

  lastState = PINC & (1 << E1);

  page = 1;
  dra818.steps = 2;
}
//----------------------------------------------------------------------------

//Loop
void loop(){
  if(updateCount >= 200){
    updateCount = 0;
    verifyPA();
    refreshDisplay();
  }

  if(TX == 1 && switched == 1){
    switchPAtoTX(txAtt, txGain);    //PA PCB is transmitting now!
    switched = 0;
  }else if (TX == 0 && switched == 1){
    switchPAtoRX(rxAtt);            //PA PCB is receiving now!
    switched = 0;
  }
}
//----------------------------------------------------------------------------

//ISP (Interrupt Service Routine) 
ISR(TIMER1_COMPA_vect){  // every 1ms
  updateCount++;
  pressedCount++;

  //Rotary Switch
  currentState = PINC & (1 << E1);
  
  if (currentState != lastState && currentState == 1){
    if(!(PINC & (1 << E2)) == currentState) {     //UP
      if(page == 2){                              //Page 2: Increasing steps
        if(dra818.steps == 8){dra818.steps = 1;}
        else{dra818.steps = dra818.steps * 2;}
        
      }else if(page == 3){                        //Page 3: Increasing TX-Frequency by step
        if(dra818.txFreq >= 146.000){dra818.txFreq = 144.000;}
        else{dra818.txFreq = dra818.txFreq + 0.0125 * dra818.steps;}
        
      }else if(page == 4){                        //Page 4: Increasing RX-Frequency by step
        if(dra818.rxFreq >= 146.000){dra818.rxFreq = 144.000;}
        else{dra818.rxFreq = dra818.rxFreq + 0.0125 * dra818.steps;}
        
      }else if(page == 5){                        //Page 5: Increasing Bandwidth
        if(dra818.bw == 0){dra818.bw = 1;}
        else {dra818.bw = 0;}
        
      }else if(page == 6){                        //Page 6: Increasing TX-Power
        if(idxPWR == 5){idxPWR = 0;}
        else{idxPWR++;}
        
      }else if(page == 7){                        //Page 7: Increasing RX-Attenuation
        if(rxAtt >= 35){rxAtt = 0;}
        else{rxAtt = rxAtt + 5;}
        
      }else if(page == 8){                        //Page 8: Increasing Volume
        if(dra818.vol == 8){dra818.vol = 1;}
        else{dra818.vol++;}
        
      }else if(page == 9){                        //Page 9: Increasing Squelch
        if(dra818.squ == 8){dra818.squ = 0;}
        else{dra818.squ++;}
        
      }else if(page == 10){                       //Page 10: Increasing TX-CTCSS
        if(idxTXCTCSS == 9){idxTXCTCSS = 0;}
        else{idxTXCTCSS++;}
        dra818.txCTCSS = ctcss_arr[idxTXCTCSS];
        
      }else if(page == 11){                       //Page 11: Increasing RX-CTCSS
        if(idxRXCTCSS == 9){idxRXCTCSS = 0;}
        else{idxRXCTCSS++;}
        dra818.rxCTCSS = ctcss_arr[idxRXCTCSS];
      }
    } else {                                      //DOWN
      if(page == 2){                              //Page 2: Decreasing steps
        if(dra818.steps == 1){dra818.steps = 8;}
        else{dra818.steps = dra818.steps / 2;}
        
      }else if(page == 3){                        //Page 3: Decreasing TX-Frequency by step
        if(dra818.txFreq <= 144.000){dra818.txFreq = 146.000;}
        else{dra818.txFreq = dra818.txFreq - 0.0125 * dra818.steps;}
        
      }else if(page == 4){                        //Page 4: Decreasing RX-Frequency by step
        if(dra818.rxFreq <= 144.000){dra818.rxFreq = 146.000;}
        else{dra818.rxFreq = dra818.rxFreq - 0.0125 * dra818.steps;}
        
      }else if(page == 5){                        //Page 5: Decreasing Bandwidth
        if(dra818.bw == 1){dra818.bw = 0;}
        else {dra818.bw = 1;}
        
      }else if(page == 6){                        //Page 6: Decreasing TX-Power
        if(idxPWR == 0){idxPWR = 5;}
        else{idxPWR--;}
       
      }else if(page == 7){                        //Page 7: Decreasing RX-Attenuation
        if(rxAtt <= 0){rxAtt = 35;}
        else{rxAtt = rxAtt - 5;}
        
      }else if(page == 8){                        //Page 8: Decreasing Volume
        if(dra818.vol == 1){dra818.vol = 8;}
        else{dra818.vol--;}
        
      }else if(page == 9){                        //Page 9: Decreasing Squelch
        if(dra818.squ == 0){dra818.squ = 8;}
        else{dra818.squ--;}
        
      }else if(page == 10){                       //Page 10: Decreasing TX-CTCSS
        if(idxTXCTCSS == 0){idxTXCTCSS = 9;}
        else{idxTXCTCSS--;}
        dra818.txCTCSS = ctcss_arr[idxTXCTCSS];
        
      }else if(page == 11){                       //Page 11: Decreasing RX-CTCSS
        if(idxRXCTCSS == 0){idxRXCTCSS = 9;}
        else{idxRXCTCSS--;}
        dra818.rxCTCSS = ctcss_arr[idxRXCTCSS];
      }
    }
    clearOLED = 1;
  }
  lastState = currentState;
}

ISR(INT0_vect){  //Rotary Encoder Switch
  if(PIND & (1 << 2)){    // HIGH?
    if(pressedCount >= 250){
      save = 1;
      page = 1;
    } else {
      if(page == 12){
        page = 0;
      }
      page++;
    }
    clearOLED = 1;
    pressedCount = 0;
  }

  if(!(PIND & (1 << 2))){ // LOW?
    pressedCount = 0;
  }
}

ISR(INT1_vect){ //PTT-IN (active LOW)
  switched = 1;
  page = 1;
  clearOLED = 1;
  
  if(!(PIND & (1 << PTT_IN))){         //PTT_IN changed to LOW
    TX = 1;
  }else if (PIND & (1 << PTT_IN)){      //PTT_IN changed to HIGH
    TX = 0;
  }
}
//----------------------------------------------------------------------------
//End of Code!
