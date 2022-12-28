/*
 * unimech_lcd.h
 *
 * Created: 6/30/2021 1:04:00 PM
 *  Author: unimech-PC2
 */ 


#ifndef UNIMECH_LCD_H_
#define UNIMECH_LCD_H_


#include <inttypes.h>
#include <avr/pgmspace.h>

/** 
 *  @name  Definitions for MCU Clock Frequency
 *  Adapt the MCU clock frequency in Hz to your target. 
 */
 
#define F_CPU 11059200          /**< clock frequency in Hz, used to calculate delay timer */
#define XTAL 11059200              /**< clock frequency in Hz, used to calculate delay timer */

/**
 * @name  Definition for LCD controller type
 * Use 0 for HD44780 controller, change to 1 for displays with KS0073 controller.
 */
 
#define LCD_CONTROLLER_KS0073 0  /**< Use 0 for HD44780 controller, 1 for KS0073 controller */

/** 
 *  @name  Definitions for Display Size 
 *  Change these definitions to adapt setting to your display
 */

#define LCD_LINES        4 /**< number of visible lines of the display */
#define LCD_DISP_LENGTH 20  /**< visibles characters per line of the display */
#define LCD_LINE_LENGTH  0x40     /**< internal line length of the display    */
#define LCD_START_LINE1  0x00     /**< DDRAM address of first char of line 1 */
#define LCD_START_LINE2  0x40     /**< DDRAM address of first char of line 2 */
#define LCD_START_LINE3  0x14     /**< DDRAM address of first char of line 3 */
#define LCD_START_LINE4  0x54     /**< DDRAM address of first char of line 4 */
#define LCD_WRAP_LINES      0     /**< 0: no wrap, 1: wrap at end of visible line */


#define LCD_IO_MODE      1         /**< 0: memory mapped mode, 1: IO port mode */
//#if LCD_IO_MODE

int unimech=10;
/* instruction register bit positions, see HD44780U data sheet */

#define LCD_CLR               0      /* DB0: clear display                  */
#define LCD_HOME              1      /* DB1: return to home position        */
#define LCD_ENTRY_MODE        2      /* DB2: set entry mode                 */
#define LCD_ENTRY_INC         1      /*   DB1: 1=increment, 0=decrement     */
#define LCD_CGRAM             6      /* DB6: set CG RAM address             */
#define LCD_DDRAM             7      /* DB7: set DD RAM address             */

/* display on/off, cursor on/off, blinking char at cursor position */
#define LCD_DISP_OFF             0x08   /* display off                            */
#define LCD_DISP_ON              0x0C   /* display on, cursor off                 */
#define LCD_DISP_ON_BLINK        0x0D   /* display on, cursor off, blink char     */
#define LCD_DISP_ON_CURSOR       0x0E   /* display on, cursor on                  */
#define LCD_DISP_ON_CURSOR_BLINK 0x0F   /* display on, cursor on, blink char      */


/* set entry mode: display shift on/off, dec/inc cursor move direction */
#define LCD_ENTRY_DEC            0x04   /* display shift off, dec cursor move dir */
#define LCD_ENTRY_DEC_SHIFT      0x05   /* display shift on,  dec cursor move dir */
#define LCD_ENTRY_INC_           0x06   /* display shift off, inc cursor move dir */
#define LCD_ENTRY_INC_SHIFT      0x07   /* display shift on,  inc cursor move dir */


/* function set: set interface data length and number of display lines */
#define LCD_FUNCTION_4BIT_1LINE  0x20   /* 4-bit interface, single line, 5x7 dots */
#define LCD_FUNCTION_4BIT_2LINES 0x28   /* 4-bit interface, dual line,   5x7 dots */
#define LCD_FUNCTION_8BIT_1LINE  0x30   /* 8-bit interface, single line, 5x7 dots */
#define LCD_FUNCTION_8BIT_2LINES 0x38   /* 8-bit interface, dual line,   5x7 dots */


#define LCD_MODE_DEFAULT     ((1<<LCD_ENTRY_MODE) | (1<<LCD_ENTRY_INC) )



/**
 *  @name Definitions for 4-bit IO mode
 *  Change LCD_PORT if you want to use a different port for the LCD pins.
 *
 *  The four LCD data lines and the three control lines RS, RW, E can be on the 
 *  same port or on different ports. 
 *  Change LCD_RS_PORT, LCD_RW_PORT, LCD_E_PORT if you want the control lines on
 *  different ports. 
 *
 *  Normally the four data lines should be mapped to bit 0..3 on one port, but it
 *  is possible to connect these data lines in different order or even on different
 *  ports by adapting the LCD_DATAx_PORT and LCD_DATAx_PIN definitions.
 *  
 */

#define LCD_PORT         PORTB 
#define LCD_Dir          DDRB			/* Define LCD data port direction */

#define LCD_RS_PORT      LCD_PORT     /**< port for RS line         */
#define LCD_RS_PIN       0            /**< pin  for RS line         */
#define LCD_RW_PORT      LCD_PORT     /**< port for RW line         */
#define LCD_RW_PIN       1            /**< pin  for RW line         */
#define LCD_E_PORT       LCD_PORT     /**< port for Enable line     */
#define LCD_E_PIN        3            /**< pin  for Enable line     */



/** 
 *  @name Functions
 */


/**
 @brief    Initialize display and select type of cursor
					\b LCD_DISP_OFF display off\n
                    \b LCD_DISP_ON display on, cursor off\n
                    \b LCD_DISP_ON_CURSOR display on, cursor on\n
                    \b LCD_DISP_ON_CURSOR_BLINK display on, cursor on flashing             
 @return  none
*/
extern void lcd_init(void);


/**
 @brief    Clear display and set cursor to home position
 @param    void                                        
 @return   none
*/
extern void lcd_clrscr(void);


/**
 @brief    Set cursor to home position
 @param    void                                        
 @return   none
*/
extern void lcd_home(void);


/**
 @brief    Set cursor to specified position
 
 @param    x horizontal position\n (0: left most position)
 @param    y vertical position\n   (0: first line)
 @return   none
*/
extern void lcd_gotoxy(uint8_t x, uint8_t y);



/**
 @brief    Display string from program memory without auto linefeed
 @param    s string from program memory be be displayed                                        
 @return   none
 @see      lcd_puts_P
*/
extern void lcd_puts(char *str);


/**
 @brief    Send LCD controller instruction command
 @param    cmd instruction to send to LCD controller, see HD44780 data sheet
 @return   none
*/
extern void lcd_command(uint8_t cmnd);


/**
 @brief    Send data byte to LCD controller 
 
 Similar to lcd_putc(), but without interpreting LF
 @param    data byte to send to LCD controller, see HD44780 data sheet
 @return   none
*/
extern void lcd_putc(uint8_t data);




#endif /* UNIMECH_LCD_H_ */