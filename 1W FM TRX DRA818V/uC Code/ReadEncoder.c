// Based on: http://web.engr.oregonstate.edu/%7Etraylor/ece473/student_projects/ReadingEncoderSwitches.pdf
const int8_t encoderStates[] = { 0, 1, 2, 0, 2, 0, 0, 1, 1, 0, 0, 2, 0, 2, 1, 0 };
#define CW 1
#define CCW 2


#define ENCODER_A_PIN				(1 << PD2)
#define ENCODER_B_PIN				(1 << PF0)

#define ENCODER_A_VALUE				((PIND & ENCODER_A_PIN) ? 0 : 0x01)
#define ENCODER_B_VALUE				((PINF & ENCODER_B_PIN) ? 0 : 0x02)


ISR(INT2_vect)			// Any edge of encoder A input
{
	cmdReadEncoder = 1;
}


int main (void)
{
	uint8_t fooValue = 0;
	uint8_t encoderValue = 0;
	uint8_t encoderCount = 0;
	uint8_t encoderDirection = 0;
	
	while (1)
	{
		if (cmdReadEncoder)
		{			
			encoderValue |= (ENCODER_B_VALUE | ENCODER_A_VALUE);
			encoderDirection = encoderStates[(encoderValue) & 0xF];
			
			if (encoderDirection == CW)
			{
				encoderCount++;
			}
			else if (encoderDirection == CCW)
			{
				encoderCount--;
			}
			
			if ((encoderValue & 3) == 3)
			{
				if ((encoderCount >= 0x01) && (encoderCount < 0x70))
				{
					fooValue++;
				}
				else if ((encoderCount <= 0xFF) && (encoderCount > 0x90))
				{
					fooValue--;
				}
				encoderCount = 0;
				cmdReadEncoder = 0;
			}
			
			encoderValue = (encoderValue << 2) & 0xFC;		
		}
	}
}
