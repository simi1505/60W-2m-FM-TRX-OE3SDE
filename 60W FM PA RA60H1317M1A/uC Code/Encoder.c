// Based on: Reading the encoder switch - http://web.engr.oregonstate.edu/%7Etraylor/ece473/student_projects/ReadingEncoderSwitches.pdf
#define F_CPU	8000000UL

#define CLOCKWISE 1
#define COUNTERCLOCKWISE 2
const int8_t encoderStates[] = { 0, 1, 2, 0, 2, 0, 0, 1, 1, 0, 0, 2, 0, 2, 1, 0 };
volatile uint8_t cmdReadEncoder = 0;


volatile uint8_t fooValue = 0;


#define TIMER0_PRELOAD		0xB2


ISR(INT2_vect) // Any edge of encoder A input
{
	cmdReadEncoder = 1;
}


ISR(TIMER0_OVF_vect) // 10ms Timer
{
	TCNT0 = TIMER0_PRELOAD;
	uint8_t tempSREG = SREG;
	static uint8_t encoderButtonHistory = 0;	
	
	// Button debouncing - based on [4]
	encoderButtonHistory <<= 1;
	encoderButtonHistory |= ENCODER_SW_PIN_PRESSED;
	
	if ((encoderButtonHistory & 0b11000111) == 0b00000111)
	{
		// Whatever needs to be done on Encoder button press
		fooValue = 0;
		encoderButtonHistory = 0b11111111;
	}
	
	SREG = tempSREG;
}


void setupMCU(void)
{
	// Remark: I2C / TWI initialization entirely done by twi library
	
	// Configure button pins as input with pullup enabled
	DDRB = (SDN_PIN | SCK_PIN | MOSI_PIN | SS_PIN);
	PORTB = (OLED_BUTTON_A_PIN);
	
	DDRC = (LED_PIN);
	PORTC = (OLED_BUTTON_C_PIN);
	
	// NSS output shall be immediately high
	DDRD = (NSS_PIN);
	PORTD = (OLED_BUTTON_B_PIN | NSS_PIN | ENCODER_SW_PIN | ENCODER_A_PIN);
	
	// PORTE pins unused by application
	
	DDRF = 0;
	PORTF = (BUTTON_1_PIN | BUTTON_2_PIN | BUTTON_3_PIN | BUTTON_4_PIN | ENCODER_B_PIN);
	
	// Start Bootloader if µC is started via reset button
	if (OLED_BUTTON_A_PRESSED || OLED_BUTTON_B_PRESSED || OLED_BUTTON_C_PRESSED)
	{
		LED(ON);
		// Bootloader section starts at address 0x3800 (as configured in fuses)
		// BOOTRST fuse is unprogrammed to directly start into application
		asm volatile ("jmp 0x3800");
	}
	
	// Interrupt on any edge for Encoder A input
	EICRA |= (1 << ISC20);
	EIFR |= (1 << INT2);
	EIMSK |= (1 << INT2);
	
	// Timer0 at fosc:1024
	TCNT0 = TIMER0_PRELOAD;
	TCCR0B = (5 << CS00);
	TIMSK0 = (1 << TOIE0);
	
	CONFIGURE_SPI_MASTER();
	
	// Interrupts must be enabled before starting display communication (I2C library needs interrupts)
	sei();
}

int main(void)
{
	uint8_t oldFooValue = 1;
	
	uint8_t encoderValue = 0;
	uint8_t encoderCount = 0;
	uint8_t encoderDirection = 0;
	
	setupMCU();

	while (1)
	{
		if (cmdReadEncoder)
		{
			// Get 2 new encoder bits
			encoderValue |= (ENCODER_B_VALUE | ENCODER_A_VALUE);
			encoderDirection = encoderStates[(encoderValue) & 0xF];
			
			if (encoderDirection == CLOCKWISE)
			{
				encoderCount++;
			}
			else if (encoderDirection == COUNTERCLOCKWISE)
			{
				encoderCount--;
			}
			
			if ((encoderValue & 3) == 3)
			{
				// Stretch valid limits for debouncing reasons
				if ((encoderCount >= 0x01) && (encoderCount < 0x70))
				{
					// Whatever needs to be done on clockwise turn
					fooValue++;
				}
				else if ((encoderCount <= 0xFF) && (encoderCount > 0x90))
				{
					fooValue--;
				}
				encoderCount = 0;
				cmdReadEncoder = 0;
			}
			
			// Make place for the next 2 encoder bits
			encoderValue = (encoderValue << 2) & 0xFC;
		}
		
		if (fooValue != oldFooValue)
		{
			// #TODO: Display fooValue
			oldFooValue = fooValue;
		}
	}

	return 0;
}
