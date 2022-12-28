/*
Template used: Atmega32 with Boot (V2.0)

NOTTE: Do not write to eeprom address 0 as it is used by boot loader to determine
program status


*/

#define F_CPU 11059200UL
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h> 
#include "unimech_lcd.h"
#include "delay_x.h"
#include <avr/eeprom.h>

/////// for CD4046 (begin) ///////////////
#define ACCN		1
#define STEADY		2
#define DECN		3

#define STOP_ON_COUNT				0
#define STOP_ON_PRESS_SENSOR		1
#define STOP_ON_INDEXER_SENSOR		2

#define CONVEYOR_POSITION_SENSOR    0 //bit_is_clear(PIND,0) //I1.......

#define START_PB				bit_is_clear(PIND,2)
#define STOP_PB					bit_is_clear(PIND,5)
#define PUNCH_HOME_SENSOR		bit_is_clear(PIND,6)
#define PAPER_SENSOR			bit_is_clear(PINC,5)
#define HL_SENSOR				bit_is_clear(PINC,6)
#define LL_SENSOR				bit_is_clear(PINC,7)

// PULSING OUTPUT ON PC2

#define PUNCH_MOTOR_ON			PORTC|=  (1<<PC3)
#define PUNCH_MOTOR_OFF			PORTC&= ~(1<<PC3)

// PWM OUTOUT ON PD4

char cd4046_motor_state=0;
char cd4046_run_mode=0;
volatile char cd4046_motorrun=0;
volatile unsigned int cd4046_ransteps;
volatile unsigned int cd4046_acc_steps;
volatile unsigned int cd4046_decn_steps;
unsigned int cd4046_max_speed_OCR=255;//100
unsigned int cd4046_min_speed_OCR=50;//30
unsigned int cd4046_accn_value=30;
volatile unsigned int cd4046_const_steps;
volatile char cd4046_stop_on_signal=0;
char cd4046_run;
volatile unsigned int cd4046_decn_point=0;
volatile unsigned int cd4046_decn_point2;
volatile unsigned int cd4046_setsteps;
volatile unsigned int cd4046_offset_value;
volatile char cd4046_sensed=0;
unsigned int cd4046_indexer_offset_value=10;
unsigned int cd4046_timer;
char cd4046_ra,cd4046_a=0;
int cd4046_za=0;
unsigned int total;
unsigned int cd4046_acc_percentage;
unsigned int cd4046_dec_percentage;
/////// for CD4046 (end) /////////////////

////////// for cut to length /////////////
unsigned int length=2000;
unsigned int temp_length;
float diameter_of_roller=65.04;
float mm_per_step;
unsigned int steps;
float circumference;
unsigned int stepper_PPR=1600;
///////////////////////////////////////////



////////// for menu //////
char settings=0;
char valchange, refreshval;
char menu=0;
char check0, check1;
int keypresscounter=0;    
int dbval=2000;                                                                                      
///////// for DC motor /////////////////
char run_pmdc_motor=0;
unsigned int pmdc_motor_acc_decn_counter=0;

/////////


/////////////
char a,ra,b,rb,c,rc,d,rd;
unsigned int za,zb,zc,zd;
unsigned int db1, db2, db3, db4, db5, db6, db7;
char run=0;
char changed=0;

/////////for Analog Keypad/////////////////
int key1press=0;
int key2press=0;
int key3press=0;
int key4press=0;
int key5press=0;
int key6press=0;

#define	STOP_KEY		key1press
#define	START_KEY		key2press
#define	SETTINGS_KEY	key3press
#define	EXIT_KEY		key4press
#define	DOWN_KEY		key5press
#define	UP_KEY			key6press

// for bootloader
uint8_t prog[270];
uint8_t j=0;

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

uint16_t EEMEM eeprombyte0;		
uint16_t EEMEM eeprombyte1; 		
uint16_t EEMEM eeprombyte2;	
uint16_t EEMEM eeprombyte3;	

void saveeepromdata(void)
{
//	eeprom_write_word(&eeprombyte0,3);
	eeprom_write_word(&eeprombyte1,3);
	eeprom_write_word(&eeprombyte2,11);
	eeprom_write_word(&eeprombyte3,length);
}

void geteepromdata(void)
{
//	check0 				= eeprom_read_word(&eeprombyte0);
	check0 				= eeprom_read_word(&eeprombyte1);
	check1				= eeprom_read_word(&eeprombyte2);
	length				= eeprom_read_word(&eeprombyte3);
}

SIGNAL(INT1_vect)   //PD3
{
	switch(cd4046_run_mode)
	{
		case 0: //calibrate
		if(cd4046_motorrun==1)
		{
			cd4046_ransteps++;
			switch(cd4046_motor_state)
			{
				case 1: 
				if(OCR2==cd4046_max_speed_OCR)
				{
					cd4046_acc_steps=cd4046_ransteps;
					cd4046_motor_state=STEADY;//2
				}
				break;
	
				case 2:
				if(OCR2==cd4046_max_speed_OCR&&cd4046_run==1)
				{
					cd4046_const_steps++;
				}
				else
				if(cd4046_run==0)
				cd4046_motor_state=DECN;//3
				break;
	
				case 3: 
				if(cd4046_run==0)
				{
					cd4046_decn_steps++;
				}
		
				if(OCR2==cd4046_min_speed_OCR)
				{
					cd4046_motor_state=4;
					cd4046_motorrun=0;
				}
				break; 
			}

		} //if(cd4046_motorrun==1)
		break;	
		
		case 1://accn + const + decn
		if(cd4046_motorrun==1)
		{
			cd4046_ransteps++;
			//if (cd4046_runmotor==INDEXER)
			PORTC = PORTC ^ 0x04;//	PORTA ^= (1<<2);	// stepper pulse @ PC2
// 			else
// 			if (cd4046_runmotor==PRESS)
		//	PORTC ^= (1<<5);//PORTC = PORTC ^ 0x20;		// stepper pulse @ P			
			switch(cd4046_motor_state)
			{
				case 1:  //accn
				if(OCR2==cd4046_max_speed_OCR)
				{
					cd4046_decn_point=cd4046_setsteps-cd4046_decn_steps;
					cd4046_motor_state=STEADY;
				}
				break;
				
				case 2: //steady
				if(cd4046_ransteps>=cd4046_decn_point)
				{
					cd4046_run=0;
					cd4046_motor_state=DECN;
				}
				break;

				case 3:
					switch(cd4046_stop_on_signal)
					{
						case STOP_ON_INDEXER_SENSOR:
							if(CONVEYOR_POSITION_SENSOR==1||cd4046_sensed==1)
							{
								cd4046_sensed=1;
								cd4046_offset_value--;
								if (cd4046_offset_value<=0)
								{
									cd4046_motorrun=0;
									cd4046_stop_on_signal=STOP_ON_COUNT;
									cd4046_offset_value=cd4046_indexer_offset_value;
									cd4046_sensed=0;
								}
							}						
						break;
						
						case STOP_ON_COUNT:
							if(cd4046_ransteps>=cd4046_setsteps)
							{
								cd4046_motorrun=0;
							}
						break;						
					}
				break;
			}
		}	//if(cd4046_motorrun==1)
		break;
		
		case 2://accn && decn
		if(cd4046_motorrun==1)
		{
			cd4046_ransteps++;
		//	if (cd4046_runmotor==INDEXER)
			PORTC = PORTC ^ 0x04;//	PORTA ^= (1<<2);	// stepper pulse @ PC2
// 			else
// 			if (cd4046_runmotor==PRESS)
// 			PORTC ^= (1<<5);//PORTC = PORTC ^ 0x20;		// stepper pulse @ PC5

			switch(cd4046_motor_state)
			{
				case 1: //accn
				if(cd4046_ransteps>=cd4046_decn_point2)
				{
					cd4046_run=0;
					cd4046_motor_state=STEADY;
				}
				break;

				case 2:
					switch(cd4046_stop_on_signal)
					{
/*
						case STOP_ON_PRESS_SENSOR:
						if(bit_is_clear(EXTIN,PRESS_HOMING)==1)
						{
							cd4046_motorrun=0;
							cd4046_stop_on_signal=STOP_ON_COUNT;
						}
						break;*/
						
						case STOP_ON_INDEXER_SENSOR:
						
						if(CONVEYOR_POSITION_SENSOR==1||cd4046_sensed==1)
						{
							cd4046_sensed=1;
							cd4046_offset_value--;
							if (cd4046_offset_value<=0)
							{
								cd4046_motorrun=0;
								cd4046_stop_on_signal=STOP_ON_COUNT;
								cd4046_offset_value=cd4046_indexer_offset_value;
								cd4046_sensed=0;
							}
						}
							
						break;
						
						case STOP_ON_COUNT:
						if(cd4046_ransteps>=cd4046_setsteps)
						{
							cd4046_motorrun=0;
						}
						break;
					}
				break;
			}
		}	//if(cd4046_motorrun==1)
		break;		

		case 3://freerun on input (jog)
		if(cd4046_motorrun==1)
		{
//			cd4046_ransteps++;
		//	if (cd4046_runmotor==INDEXER)
			PORTC = PORTC ^ 0x04;	//PORTA ^= (1<<2);	// stepper pulse @ PC2
// 			else
// 			if (cd4046_runmotor==PRESS)
// 			PORTC ^= (1<<5);//PORTC = PORTC ^ 0x20;		// stepper pulse @ PC5

			switch(cd4046_motor_state)
			{
				case 1:
				if(cd4046_run==0)
				{
					cd4046_motor_state=STEADY;
				}
				break;
				
				case 2:
				if(OCR2==cd4046_min_speed_OCR)
				{
					cd4046_motor_state=DECN;  //3
					cd4046_motorrun=0;
				}
				break;			
				
			}
			
		}	//if(cd4046_motorrun==1)
		break;
		
		
	}	//switch(cd4046_run_mode)
}

SIGNAL (TIMER0_COMP_vect)    // handler for Output Compare 0 overflow interrupt
{
	if (cd4046_timer<65000)
	cd4046_timer++;

	if (cd4046_run==1&&cd4046_timer>=cd4046_accn_value) //
	{
		cd4046_timer=0;
		if (OCR2<cd4046_max_speed_OCR)
		OCR2++;
	}

	if (cd4046_run==0&&cd4046_timer>=cd4046_accn_value)//=
	{
		cd4046_timer=0;
		if(OCR2>cd4046_min_speed_OCR)
		OCR2--;
	}

	if(run_pmdc_motor==1)
	{
		if (pmdc_motor_acc_decn_counter<100)
		pmdc_motor_acc_decn_counter++;
		
		if (OCR1B<255&&pmdc_motor_acc_decn_counter>99)
		{
			OCR1B++;
			pmdc_motor_acc_decn_counter=0;
		}
	}
	else
	{
		if (OCR1B>0)
		OCR1B--;		
	}
	
	if (za>0)
	{
		za--;
	}

	if (zb>0)
	{
		zb--;
	}

	if (zc>0)
	{
		zc--;
	}
}

void calibrating_cycle(void)
{
	if (cd4046_a==1&&cd4046_ra==0)
	{
		cd4046_run=1;
		cd4046_motor_state=ACCN;
		cd4046_motorrun=1;
		cd4046_a=2;
		cd4046_ra=1;
	}

	if (cd4046_a==2&&cd4046_ra==0)
	{
		if(cd4046_const_steps>20000)
		{
			cd4046_a=3;
			cd4046_run=0;
		}
		cd4046_ra=1;
	}

	if (cd4046_a==3&&cd4046_ra==0)
	{
		if(cd4046_motor_state==4)
		{
			cd4046_a=4;
			//save to eeprom not needed
		}
		cd4046_ra=1;
	}

	if (cd4046_a==4&&cd4046_ra==0)
	{
		cd4046_a=5;
		cd4046_ra=1;
	}

	cd4046_ra=0;
}

void runsteps(unsigned int steps)
{
	cd4046_motor_state=ACCN;
	cd4046_ransteps=0;
	cd4046_setsteps=steps;
	if (cd4046_setsteps<=(cd4046_acc_steps+cd4046_decn_steps))
	{
		cd4046_run_mode=2;
		cd4046_decn_point2=(float)cd4046_setsteps*cd4046_dec_percentage/1000;
		cd4046_decn_point2=cd4046_setsteps-cd4046_decn_point2;
	}
	else
	cd4046_run_mode=1;
	
	cd4046_run=1;
	cd4046_motorrun=1;
}

SIGNAL (USART_RXC_vect) // USART RX interrupt
{
	unsigned char in;
	in = UDR;
	prog[j]=in;

	if((j>4)&&(prog[0]==9&&prog[1]==8&&prog[2]==7&&prog[3]==6&&prog[4]==5&&prog[5]==4))
	{
		for(char i=0;i<10;i++)
		prog[i]=0;
		j=0;
		WDTCR = (1<<WDE)|(1<<WDP0);
		while(1)
		{}
	}
	
	if (j<5)
	j++;
	else
	{
		prog[0]=prog[1];
		prog[1]=prog[2];
		prog[2]=prog[3];
		prog[3]=prog[4];
		prog[4]=prog[5];
		prog[5]=0;
	}
	
}

void init_USART(void)
{
	// set baud rate
	UBRRL = BAUD_PRESCALE;
	UBRRH = (BAUD_PRESCALE >> 8);

	// Enable receiver and transmitter; enable RX interrupt, enable TX interrupt
	UCSRB = (1 << RXEN)| (1 << TXEN)| (1 << RXCIE); // | (1 << TXCIE);

	//asynchronous 8N1
	UCSRC = (1 << URSEL) | (3 << UCSZ0);//(1 << USBS) |
}

void dely(void)
{
	_delay_ms(240);
}

void uppress(void)
{
	changed=1;
	if (keypresscounter<50)
	keypresscounter++;
	if (menu==1)
	{
		if (temp_length>999)
		{refreshval=1;}
		else
		{
			temp_length++;
			refreshval=1;
		}
	}
}

void downpress(void)
{
	changed=1;
	if (keypresscounter<50)
	keypresscounter++;
	if (menu==1)
	{
		if (temp_length<11)
		{refreshval=1;}
		else
		{
			temp_length--;
			refreshval=1;
		}
	}
}

int main(void)
{
	unsigned int buffer[20];
	DDRA  = 0b00000000;	  // PA0 is analog input for KEYPAD
	PORTA = 0b11111110;  //

	//PORTB connects to LCD
	PORTB|= (1<<PB2);  // For LCD Backlight

	DDRC  = 0b00001100;	      // PC2 - Stepper Pulse, PC3-Stamping Motor, PC4- not used, PC5, PC6 & PC7 inputs from sensors
	PORTC = 0b11110000;       //

	DDRD  = 0b10010000;	  // PD0&PD1: Rx and Tx for bootloading, PD2-Start, PD3-Int from CD4046, PD4-Output PWM for PMDC motor
	PORTD = 0b01101111;   // PD5-stop PB, PD6-Homing sensor, PD7-PWM to CD4046

	sei();
	init_USART();
	
	lcd_init();
	lcd_clrscr();
	lcd_gotoxy(0,0);
	lcd_puts("   Paper Stamping  ");
	lcd_gotoxy(0,1);
	lcd_puts("       Machine     ");
	lcd_gotoxy(0,3);
	lcd_puts("        v1.0       ");	
	
	_delay_ms(100);

	///// Initialize timer 0
	TCCR0 |= _BV(CS00) | _BV(CS01) | _BV(WGM01); //64
	OCR0 = 10; //0.0636 mS.
	TIMSK  |= _BV(OCIE0);   // start timer interrupt
	
	///////////////////////// for CD4046 (begin)//////////////////////////////////
	// PWM config at PD7 -Timer2 //PWM, PhaseCorrect
	TCCR2|= _BV(CS21) |_BV(CS20) | _BV(WGM20) | _BV(COM21);//compare match
	OCR2=cd4046_min_speed_OCR;		
	
	MCUCR = _BV(ISC11) ;  // int1 on falling edge
	GICR |=  _BV(INT1);
	sei();

	///////////////////////// for CD4046 (end)//////////////////////////////////


	///////////////////////// for PWM for PMDC motor////////////////////////////
	TCCR1A |= _BV(WGM10)| _BV(COM1B1);
	TCCR1B |= _BV(CS12);
	////////////////////////////////////////////////////////////////////////////


	cd4046_offset_value=cd4046_indexer_offset_value;
	cd4046_a=1;
	cd4046_za=1000;
	
	while(cd4046_a!=5)
	calibrating_cycle();
	//	PORTD |= (1<<PD4);	
	total=cd4046_acc_steps+cd4046_decn_steps;
	cd4046_acc_percentage=((float)cd4046_acc_steps/(float)total)*1000.0;
	cd4046_dec_percentage=((float)cd4046_decn_steps/(float)total)*1000.0;

	lcd_clrscr();
/*
	lcd_gotoxy(0,0);
	lcd_puts("      DONE      ");
	lcd_gotoxy(0,1);
	itoa(cd4046_acc_steps, buffer, 10);
	lcd_puts(buffer);
	lcd_puts(" ");*/

cd4046_stop_on_signal=STOP_ON_COUNT;
//runsteps(3600);
	//OCR1B=255;

//	lcd_clrscr();
	circumference=3.142*diameter_of_roller;
	mm_per_step=circumference/stepper_PPR;

	geteepromdata();

	if ((bit_is_clear(PINA,0)==1)||(check0!=3)||(check1!=11))  // SETTINGS key
	{
		length=100;
		saveeepromdata();
		lcd_gotoxy(0,0);
		lcd_puts("Values reset");
		dely();
		dely();
		while((bit_is_clear(PINA,0)==1))
		{}
		lcd_clrscr();
}


	 a=10;
	 c=10;
	ADCSRA = _BV(ADEN) | _BV(ADPS2);
	// Select pin ADC0 using MUX
	ADMUX = 0;
	//menu=1;
//	 	TIMSK  &= ~_BV(OCIE0);
      while (1)
		{ 
		   //Start conversion
		   ADCSRA |= _BV(ADSC);
		   
		   // wait until converstion completed
		   while (ADCSRA & _BV(ADSC) ) {}
		   
		   // get converted value
		   int z = ADCW;

		   if (z<15)
		   {
			   if (key1press<500)
			   key1press++;
		   }
		   
		   if (z>45&&z<77)
		   {
			   if (key2press<500)
			   key2press++;
		   }
		   
		   if (z>133&&z<253)
		   {
			   if (key3press<500)
			   key3press++;
		   }
		   
		   if (z>327&&z<414)
		   {
			   if (key4press<500)
			   key4press++;
		   }
		   
		   if (z>511&&z<703)
		   {
			   if (key5press<500)
			   key5press++;
		   }
		   
		   if (z>785&&z<891)
		   {
			   if (key6press<500)
			   key6press++;
		   }

		   if (z>911)
		   {
			   key1press=0;
			   key2press=0;
			   key3press=0;
			   key4press=0;
			   key5press=0;
			   key6press=0;
		   }

	if (settings==0)
	{
		
		lcd_gotoxy(0,0);
		if(run==1)
		{
			lcd_puts("      Running       ");
		}
		else 
		if (run==0&&a!=10)
		{
			lcd_puts("      Stopping      ");			
		}
		else
		if (run==0&&a==10)
		{
			lcd_puts("      Stopped       ");
		}

		lcd_gotoxy(0,2);
		lcd_puts("Set Length:");
		itoa(length, buffer, 10);
		lcd_puts(buffer);
		lcd_puts(" mm  ");
			
			
		if (SETTINGS_KEY>10)//if (bit_is_clear(KEYPAD,ENTER)==1)  // settings push button
		{
			if (db3<20)
			db3++;
		}
		
		if (SETTINGS_KEY==0)//if (bit_is_clear(KEYPAD,ENTER)==0)
		{
			if (db3>=10)
			{
				settings=1;
				valchange=1;
				refreshval=1;
				menu=1;
				
				temp_length=length;

				lcd_clrscr();
			}
			db3=1;
		}			
			
	}

	if (settings==1)
	{
		if (SETTINGS_KEY>10)  
		{
			if (db4<100)
			db4++;
		}
		if (SETTINGS_KEY==0) 
		{
			if (db4>=50)
			{
				if (menu<1)
				{
					menu++;
					valchange=1;
					db4=1;
				}
				else
				{
					menu=1;
					valchange=1;
					db4=1;
				}
			}
			db4=1;
		}		

		if (EXIT_KEY>10)  
		{
			if (db5<100)
			db5++;
		}
		if (EXIT_KEY==0)
		{
			if (db5>=50)
			{
				lcd_clrscr();
				if (changed==1)
				{
					length=temp_length;
					saveeepromdata();
					changed=0;
				}
				settings=0;
				db5=1;
			}
		}		
		
		if (UP_KEY>10) 
		{
			if (db6<5000)
			db6++;
			if (db6>=dbval)
			{
				uppress();
				db6=1;
			}
		}
		if (UP_KEY==0) //if (bit_is_clear(KEYPAD,UP)==0)
		{
			if (db6>100)
			{
				uppress();
				db6=1;
			}
		}

		if (DOWN_KEY>10)
		{
			if (db7<5000)
			db7++;
			if (db7>=dbval)
			{
				downpress();
				db7=1;
			}
		}
		if (DOWN_KEY==0) //if (bit_is_clear(KEYPAD,UP)==0)
		{
			if (db7>100)
			{
				downpress();
				db7=1;
			}
		}

		if (keypresscounter>10&&keypresscounter<20)//
		{
			dbval=100;
		}

		if (keypresscounter>21)
		{
			dbval=10;
		}

		if (DOWN_KEY==0&&UP_KEY==0)
		{
			keypresscounter=0;
			dbval=2000;
		}
		
		if (valchange==1)
		{ //ok

		    lcd_clrscr();

			if (menu==1)
			{
				lcd_puts("Length");
				lcd_gotoxy(0,1);
				lcd_puts("mm:");
			}

			if (menu==2)
			{
				lcd_puts("time");
				lcd_gotoxy(0,1);
				lcd_puts("mm:");
			}

			refreshval=1;
			valchange=0;

		}//valchange //ok

//*******************
		if (refreshval==1)
		{
		int val;
			if (menu==1)    
				{
				val=temp_length;
				}

				lcd_gotoxy(9,1);
				itoa(val, buffer, 10);
				lcd_puts(buffer);
				lcd_puts(" ");

			refreshval=0;

		} // refreshval		
		
	} //if (settings==1)


		if(START_PB==1||START_KEY>=10)
		{
			if (db1<1000)
			db1++;
		}

		if(START_PB==0&&START_KEY==0)
		{
			if(db1>10)
			{
				run=1;
			}
			db1=0;
		}		

		if(STOP_PB==1||STOP_KEY>=10)
		{
			if (db2<1000)
			db2++;
		}

		if(STOP_PB==0&&STOP_KEY==0)
		{
			if(db2>10)
			{
				run=0;
			}
			db2=0;
		}

///*******************  A  *************************
////////////        Main cycle      /////////

         if (a==10&&ra==0)
         {
	         //if (HL_SENSOR==1) check if paper is available
			 if (run==1)
	         a=20;
	         ra=1;
         }

         if (a==20&&ra==0)
         {
	         //if (PAPER_SENSOR==1) check if paper is available
	         a=30;
	         ra=1;
         }

         if (a==30&&ra==0)
         {
			 steps=2*(length/mm_per_step);
			 runsteps(steps);
		     a=40;
	         ra=1;
         }

         if (a==40&&ra==0)
         {
			if(cd4046_motorrun==0)
			{
				a=50;
			}
	         ra=1;
         }		 

         if (a==50&&ra==0)
         {
	         if(za>0)
	         {
		         ra=1;
	         }
         }

         if (a==50&&ra==0)
         {
	         b=10; //PUNCHING CYCLE
	         a=60;
	         ra=1;
         }
         
         if (a==60&&ra==0)
         {
	         if(b==0) // check punching cycle completion
	         {
		         a=70;
	         }
	         ra=1;
         } 
 
         if (a==70&&ra==0)
         {
		     a=10;
	         ra=1;
         }	
 
 
      ra=0;

///*******************  B  *************************
////////////        Punching cycle      /////////

         if (b==10&&rb==0)
         {
	         if(zb>0)
	         {
		         rb=1;
	         }
         }

         if (b==10&&rb==0)
         {
	         PUNCH_MOTOR_ON;
	         b=20;
	         rb=1;
         }

         if (b==20&&rb==0)
         {
	         if(PUNCH_HOME_SENSOR==0)
			 {
				 b=30;
			 }
	         rb=1;
         }

         if (b==30&&rb==0)
         {
	         if(PUNCH_HOME_SENSOR==1)
	         {
		         b=40;
				 PUNCH_MOTOR_OFF;
	         }
	         rb=1;
         }

         if (b==40&&rb==0)
         {
		     b=0;
	         rb=1;
         }

          rb=0;

///*******************  C  *************************
////////////        Dancing Roller cycle      /////////

         if (c==10&&rc==0)
         {
			 if (HL_SENSOR==0)
	         {
				run_pmdc_motor=1;
				c=20;
			 }
			 else
			 {
				c=20; 
			 }
	         rc=1;
         }

         if (c==20&&rc==0)
         {
	        if(HL_SENSOR==1)
		    {
			    run_pmdc_motor=0;
			    c=30;
		    }
	         rc=1;
         }

         if (c==30&&rc==0)
         {
	         if(LL_SENSOR==1)
	         {
		         run_pmdc_motor=1;
		         c=20;
	         }
	         rc=1;
         }

      rc=0;

		}//while

}//main