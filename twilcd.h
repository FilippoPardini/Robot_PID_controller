/****************************************************************************
* ATxmega library for the LM629 - Version 1.0                               *
* © Copyright 2012,2013 Filippo Pardini - filippo@robotica.eng.br           *
*                                                                           *
* This program is free software: you can redistribute it and/or modify it   *
* under the terms of the GNU Lesser General Public License as published by  *
* the Free Software Foundation, either version 3 of the License, or any     *
* later version.                                                            *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              *
* GNU Lesser General Public License for more details.                       *
*                                                                           *
* You should have received a copy of the GNU Lesser General Public License  *
* along with this program. If not, see <http://www.gnu.org/licenses/>.      *
****************************************************************************/

/*
Remember that the TWI bus must have pull-up resistors on SDA and SCL.
We recommend 1.8K or 4.7K resistors.
The TWI must be initialized as in the example below. In the example
has been used TWID:

*****************************************************************************

#include <twilcd.h>
.  .  .

// TWI clock rate [bps]
#define TWI_CLK_RATE 100000

// structure that holds information used by the TWID master
// for performing a TWI bus transaction
TWI_MASTER_INFO_t twid_master;

interrupt [TWID_TWIM_vect] void twid_master_isr(void)
{
	twi_master_int_handler(&twid_master);
}

void main(void)
{
	unsigned char h,m,s;

	// general TWID initialization
	// no external driver interface
	// no SDA hold time
	twi_init(&TWID,false,false);

	// initialize the TWID master
	twi_master_init(&twid_master,&TWID,TWI_MASTER_INTLVL_LO_gc,TWI_BAUD_REG(_MCU_CLOCK_FREQUENCY_,TWI_CLK_RATE));

	// set the LCD03 functions to use TWID
	lcd03_twi_init(&twid_master);

	// enable LO interrupt level
	PMIC.CTRL |= PMIC_LOLVLEN_bm;

	// enable interrupts
	#asm("sei")

	// now the rest of the LCD03 functions can be used

	// ....
}

*****************************************************************************

*/

#ifndef _TWILCD_INCLUDED_
#define _TWILCD_INCLUDED_

#pragma used+

//Initializes LCD03
//ptwim - TWI pointer
//rel - pointer to the relogio variable (in milliseconds)
//tot - keyboard time (in milliseconds)
void lcd03_twi_init(TWI_MASTER_INFO_t *ptwim,unsigned char addr,unsigned long int *rel,unsigned char tot);

//writes 1 byte to LCD03 (adress ender) register 0
//data - Byte to write
void lcd_byte_write(unsigned char data);

//writes 2 bytes to LCD03 (adress ender) register 0
//data1 and data2 - bytes to write
void lcd_2byte_write(unsigned char data1, unsigned char data2);

//writes 3 bytes to LCD03 (adress ender) register 0
//data1, data2 e data3 - bytes to write
void lcd_3byte_write(unsigned char data1, unsigned char data2, unsigned char data3);

//Cleans the lin line and places the cursor at the beginning of the line
void lcd_clear_line(unsigned char lin);

//Sends the str string to the LCD03
void lcd_char_string_write(char *str);

//Reads the LCD03 reg register
unsigned char lcd_register_read(unsigned char reg);

//Reads the keypad character, sends it to the LCD03 and returns it
//mostra = 1 - send to the LCD03 the own characters
//mostra = 0 - send to the LCD03 the characters '_'
char read_display_keypad(unsigned char mostra);

//Receives the command code from keypad
char receive_keypad_cod(unsigned char t_o);

//Receives a command parameter and returns its size
//t_o - Maximum wait time in seconds
//mostra = 1 - displays the real characters
//mostra = 0 - displays '_' characters
//ptro - pointer to the command area
//nmax - maximum bytes number to receive
unsigned char receive_keypad_par(unsigned char t_o, unsigned char mostra, char *ptro, unsigned char nmax);

//Reads the keypad character
char read_keypad(void);

//Converts the keypad bits to the corresponding characters
char convert_keypad(unsigned char tipo, unsigned char car);

//Displays a message
//If aguarda>=1 => waits the '*' keying
//If aguarda=1 => do not waits
void lcd_talk(char *fr1,char *fr2,char *fr3,unsigned char aguarda);

//Prepares the LCD03 to receive the keyed command
unsigned char receive_command(char *cmdo);

//Calculates the command code value. If OK, returns the value. On the contrary returns 0
unsigned char calcula(char *cmp);

//Displays lines 1,2,3
//If seg > 0 => waits seg seconds and clears the display
void lcd_display123(char *fr1,char *fr2,char *fr3, unsigned char seg);

//Displays lines 1,2,3,4
//If seg > 0 => waits seg seconds and clears the display
void lcd_display1234(char *fr1,char *fr2,char *fr3,char *fr4, unsigned char seg);

//Clears the display buffer
void limpa_display_buffer(void);

//Displays the str string from flash in the linha line
//limpa=1 => clears the display before
//limpa=0 => do not clears the display before
//seg>=0 => displays the line for seg seconds
//seg<0 => do not returns
//center=0 => line aligned to the left
//center=1 => line aligned to the center
//center<0 => line aligned at the present cursor position
//If remainder(linha/4) = 0 => line 4
//If remainder(linha/4) > 0 => this is the line
void display_flash(unsigned char limpa, unsigned char linha, signed char center, flash unsigned char *str, signed char seg);

//Displays the str string in the linha line
//limpa=1 => clears the display before
//limpa=0 => do not clears the display before
//seg>=0 => displays the line for seg seconds
//seg<0 => do not returns
//center=0 => line aligned to the left
//center=1 => line aligned to the center
//center<0 => line aligned at the present cursor position
//If remainder(linha/4) = 0 => line 4
//If remainder(linha/4) > 0 => this is the line
void display(unsigned char limpa, unsigned char linha, signed char center, unsigned char *str, signed char seg);

#pragma used-

#pragma library twilcd.lib

#endif




