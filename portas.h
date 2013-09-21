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

#ifndef _PORTAS_INCLUDED_
#define _PORTAS_INCLUDED_

#pragma used+

//Defines the pins direction of the prta port
//io => each bit defines the pin direction (0 => input; 1 => output)
void prta_DIR(unsigned char prta, unsigned char io);

//Places the byt byte on the prta port
void prta_OUT(unsigned char prta, unsigned char byt);

//Defines the port pin *door as input
void pin_INPUT(char *door);

//Defines the port pin *door output
void pin_OUTPUT(char *door);

//Sets the port pin *door as high (v=1) or low (v=0)
void putpin(unsigned char *door,unsigned char v);

//Receives a byte from port prta
unsigned char prta_IN(unsigned char prta);

//Reads the port pin (high = 1, low = 0)
unsigned char getpin(unsigned char *door);

//Toggles the port pin
void togglepin(unsigned char *door);

#pragma used-

#pragma library portas.lib

#endif
