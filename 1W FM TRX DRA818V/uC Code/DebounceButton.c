ISR(TIMER0_OVF_vect)	// 10ms Timer
{
	TCNT0 = TIMER0_PRELOAD;
	uint8_t tempSREG = SREG;
	static uint8_t encoderButtonHistory = 0;	
	
	// Button debouncing - based on http://hackaday.com/2015/12/10/embed-with-elliot-debounce-your-noisy-buttons-part-ii/
	encoderButtonHistory <<= 1;
	encoderButtonHistory |= ENCODER_SW_PIN_PRESSED;
	
	if ((encoderButtonHistory & 0b11000111) == 0b00000111)
	{
		fooValue = 0;	// Do your stuff
		encoderButtonHistory = 0b11111111;
	}
	
	SREG = tempSREG;
}