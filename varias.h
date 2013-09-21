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

#ifndef _VARIAS_INCLUDED_
#define _VARIAS_INCLUDED_

#pragma used+

//Returns the unsigned long int formed by b1,b2,b3,b4 from left to right
unsigned long int put_char_long(unsigned char b1,unsigned char b2,unsigned char b3,unsigned char b4);

//Returns the unsigned int formed by b1,b2 from left to right
unsigned int put_char_int(unsigned char b1,unsigned char b2);

//Returns the unsigned long int formed by w1,w2 from left to right
unsigned long int put_int_long(unsigned int w1,unsigned int w2);

//Returns the unsigned long int byte i counting from left to right
unsigned char get_char_long(unsigned char i,unsigned long int val);

//Returns the unsigned int word i counting from left to right
unsigned int get_int_long(unsigned char i,unsigned long int val);

//Returns the unsigned int byte i counting from left to right
unsigned char get_char_int(unsigned char i,unsigned int val);

//Copies the str2 string to str1 string (or vector) generating a NULL terminated string
//If the str1 lenght is shorter than the str2 lenght there will be truncation
//If the str2 lenght is shorter than the str1 lenght there will be filling with blanks
void copy_string(unsigned char *str1,unsigned char *str2);

#pragma used-

#pragma library varias.lib

#endif




















