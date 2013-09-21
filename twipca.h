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

	// set the PCA9554 functions to use TWID
	pca_twi_init(&twid_master);

	// enable LO interrupt level
	PMIC.CTRL |= PMIC_LOLVLEN_bm;

	// enable interrupts
	#asm("sei")

	// now the rest of the LCD03 functions can be used

	// ....
}

*****************************************************************************

*/

#ifndef _PCA9554_INCLUDED_
#define _PCA9554_INCLUDED_

#pragma used+

//Initializes PCA9554
//ptwim - pointer to TWI
void pca_twi_init(TWI_MASTER_INFO_t *ptwim);

//Configures all the PCA9554 pins. 0 - input; 1 - output
bool Configure_Port(unsigned char addr,unsigned char reg);

//Configures a pin of the PCA9554 port
// 0 => input; 1 => output
bool Configure_Pin(unsigned char addr,unsigned char pin,unsigned char val);

//Defines the PCA9554 polarity. 0 - do not invert; 1 - invert
bool Port_Polarity(unsigned char addr,unsigned char reg);

//Defines the polarity of a PCA9554 port pin. 0 - do not invert; 1 - invert
bool Pin_Polarity(unsigned char addr,unsigned char pin,unsigned char val);

//Sets the PCA9554 port to the value port
bool Put_Port(unsigned char addr,unsigned char port);

//Sets the PCA9554 port pin to the val value
bool Put_Pin(unsigned char addr,unsigned char Pin,unsigned char val);

//Reads the PCA9554 port
unsigned char Get_Port(unsigned char addr);

//Reads the PCA9554 port pin value
bool Get_Pin(unsigned char addr,unsigned char pin);

#pragma used-

#pragma library twipca.lib

#endif

