/*
 * unimech_lcd.c
 *
 * Created: 6/30/2021 1:03:37 PM
 *  Author: unimech-PC2
 */ 

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "unimech_lcd.h"
#include "delay_x.h"

#define lcd_e_high()    LCD_E_PORT  |=  _BV(LCD_E_PIN);
#define lcd_e_low()     LCD_E_PORT  &= ~_BV(LCD_E_PIN);

#define lcd_rw_high()   LCD_RW_PORT |=  _BV(LCD_RW_PIN)
#define lcd_rw_low()    LCD_RW_PORT &= ~_BV(LCD_RW_PIN)

#define lcd_rs_high()   LCD_RS_PORT |=  _BV(LCD_RS_PIN)
#define lcd_rs_low()    LCD_RS_PORT &= ~_BV(LCD_RS_PIN)

#define lcd_e_delay()   __asm__ __volatile__( "rjmp 1f\n 1:" );


#if LCD_IO_MODE
#if LCD_LINES==1
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_1LINE 
#else
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_2LINES 
#endif
#else
#if LCD_LINES==1
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_8BIT_1LINE
#else
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_8BIT_2LINES
#endif
#endif


#if LCD_CONTROLLER_KS0073
#if LCD_LINES==4

#define KS0073_EXTENDED_FUNCTION_REGISTER_ON  0x24   /* |0|010|0100 4-bit mode extension-bit RE = 1 */
#define KS0073_EXTENDED_FUNCTION_REGISTER_OFF 0x20   /* |0|000|1001 4 lines mode */
#define KS0073_4LINES_MODE                    0x09   /* |0|001|0000 4-bit mode, extension-bit RE = 0 */

#endif
#endif



/*************************************************************************
 delay loop for small accurate delays: 16-bit counter, 4 cycles/loop
*************************************************************************/
static inline void _delayFourCycles(unsigned int __count)
{
    if ( __count == 0 )    
        __asm__ __volatile__( "rjmp 1f\n 1:" );    // 2 cycles
    else
        __asm__ __volatile__ (
    	    "1: sbiw %0,1" "\n\t"                  
    	    "brne 1b"                              // 4 cycles/loop
    	    : "=w" (__count)
    	    : "0" (__count)
    	   );
}


/************************************************************************* 
delay for a minimum of <us> microseconds
the number of loops is calculated at compile-time from MCU clock frequency
*************************************************************************/
#define delay(us)  _delayFourCycles( ( ( 1*(XTAL/4000) )*us)/1000 )




void lcd_command( unsigned char cmnd )
{
	LCD_PORT = (LCD_PORT & 0x0F) | (cmnd & 0xF0); /* sending upper nibble on port b*/
	lcd_rs_low() ;								  /* RS=0, command reg. */
	lcd_e_high();	
	lcd_e_delay();							      /* Enable pulse */
	lcd_e_low();  								  /* disable pulse */
	 
	 
	
	delay(2);
	
	LCD_PORT = (LCD_PORT & 0x0F) | (cmnd << 4);  /* sending lower nibble */
	lcd_e_high();	
	lcd_e_delay();							  /* Enable pulse */
	lcd_e_low();  							 /* disable pulse */
	 
	 
	//_delay_ms(2);
	delay(2000); //ms
	
	/* all data pins high (inactive) */
	
	/*LCD_PORT=(LCD_PORT & 0x0F) | 0x0F */
	 
}


void lcd_putc( unsigned char data )
{
	
	LCD_PORT = (LCD_PORT & 0x0F) | (data & 0xF0); /* sending upper nibble on port b*/
	lcd_rs_high() ;								  /* RS=0, command reg. */
	lcd_e_high();		
	lcd_e_delay();						  /* Enable pulse */
	lcd_e_low();  								  /* disable pulse */
	 
	 
    delay(2); ///us
	
	
	LCD_PORT = (LCD_PORT & 0x0F) | (data << 4);  /* sending lower nibble */
	lcd_e_high();								  /* Enable pulse */
    lcd_e_delay();
	lcd_e_low();  								  /* disable pulse */
	 
	 
	_delay_us(2);
	
	
}


void lcd_init(void)
{
	LCD_Dir = 0xFD;								/* Make LCD port direction as o/p */
	delay(16000);								/* LCD Power ON delay always >15ms */
	
	lcd_command(0x02);							/* send for 4 bit initialization of LCD  */
	lcd_command(LCD_FUNCTION_4BIT_2LINES);		/* 2 line, 5*7 matrix in 4-bit mode */
	lcd_command(LCD_DISP_ON);					/* Display on cursor off*/
	lcd_command(LCD_ENTRY_INC_);				/* Increment cursor (shift cursor to right)*/
	lcd_command(0x01);							/* Clear display screen*/

	

#if KS0073_4LINES_MODE
    // Display with KS0073 controller requires special commands for enabling 4 line mode //
	lcd_command(KS0073_EXTENDED_FUNCTION_REGISTER_ON);
	lcd_command(KS0073_4LINES_MODE);
	lcd_command(KS0073_EXTENDED_FUNCTION_REGISTER_OFF);
#else
    lcd_command(LCD_FUNCTION_DEFAULT);      // function set: display lines  
#endif
    lcd_command(LCD_DISP_OFF);              // display off                  
    lcd_clrscr();                           // display clear                 
    lcd_command(LCD_MODE_DEFAULT);          // set entry mode               
    lcd_command(LCD_DISP_ON);      


	
}



void lcd_puts(char *str)		/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)		/* Send each char of string till the NULL */
	{
		lcd_putc(str[i]);
	}
}


/*************************************************************************
Set cursor to specified position
Input:    x  horizontal position  (0: left most position)
          y  vertical position    (0: first line)
Returns:  none
*************************************************************************/

void lcd_gotoxy(uint8_t x, uint8_t y)
{
#if LCD_LINES==1
    lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
#endif
#if LCD_LINES==2
    if ( y==0 ) 
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
    else
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
#endif
#if LCD_LINES==4
    if ( y==0 )
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
    else if ( y==1)
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
    else if ( y==2)
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE3+x);
    else /* y==3 */
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE4+x);
#endif

}/* lcd_gotoxy */


/*************************************************************************
Clear display and set cursor to home position
*************************************************************************/
void lcd_clrscr(void)
{
    lcd_command(1<<LCD_CLR);   //0
}


/*************************************************************************
Set cursor to home position
*************************************************************************/
void lcd_home(void)
{
    lcd_command(1<<LCD_HOME);   //1
	
}










