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

This version was developed for the ATxmega128A1 using the CodeVisionAVR compiler.
This librarie contain all the macro functions needed to interface two LM629N-6
with an ATxmega128A1 microcontroller for the purpose of controlling the movements
of a robot with differential steering. The solution uses a PID feedback loop to
achieve the control of the robot movements in the position or velocity mode using
the sign/magnitude and quadrature incremental encoder interfaces of the LM629.

	http://www.ti.com/lit/ds/symlink/lm629.pdf
    http://www.ti.com/lit/an/snva025c/snva025c.pdf
    http://www.ti.com/lit/an/snoa184c/snoa184c.pdf
    http://www.ti.com/lit/an/snoa170b/snoa170b.pdf
    http://www.mil.ufl.edu/projects/gnuman/spec_sheets/LM629_interface_Guide.pdf
    http://www.cs.columbia.edu/~allen/F11/NOTES/icckinematics.pdf
    http://www.atmel.com/Images/doc8067.pdf
    http://www.atmel.com/Images/doc8077.pdf
    http://en.wikipedia.org/wiki/Rotary_encoder
    http://en.wikipedia.org/wiki/PID_controller
________________________________________________________________________________________________________________________

In this version I chose to use the NXP PCA9554 (8 bit I²C bus I/O port)

    http://www.nxp.com/documents/data_sheet/PCA9554_9554A.pdf

for the communication port with the two LM629. This has three reasons:
    - uses only 2 pins (TWI - SDA,SLC) and not 8 pins
    - These same pins are used for other TWI functions (ex: LCD)
    - It automatically acts as a logical level converter (Xmega(3.3V) <=> LM629(5V))

For this reason there is the library twipca.h/twipca.lib .

In addition to the port above, we need mor 7 pins to control the two LM629:

    2 for selection (CS - one for each)
    2 for reset (RST - one for each)
    1 for read (RD - one for both)
    1 for write (WR - one for both)
    1 for type command or data (PS - one for both)

These pins also need logical level convertion that may be obtained via a TXB0108

    http://www.ti.com/lit/ds/sces643e/sces643e.pdf

Since these entries in the LM629 are "active-low", they need "pull-up" resistors.
Care must be taken in choosing the resistors value. Due to the restrictions of the
TXB0108 they must be at least 50K (100K works OK).

For the TWI pins (SDA,SLC) also we need logical level conversion, but because we
need 1.8K "pull-up" resistors on the 3.3 side, we can't use the TXB0108. For the
prototype I used a "BOB-08745" from Sparkfun:

    https://www.sparkfun.com/products/8745

For messages and debug I use the I2C(TWI) LCD03 from Devantech:

    http://www.robot-electronics.co.uk/htm/Lcd03tech.htm

For this reason there is also the library twilcd.h/twilcd.lib .

For electromechanical control I used two LMD18200, two old 61.46.052 Buehler motors
and two old H5S-250-I from USDigital.

    http://www.ti.com/lit/ds/snvs091e/snvs091e.pdf
    http://www.usdigital.com/assets/datasheets/H5_datasheet.pdf?k=634913185143995582

In this version I chose not to use the LM629 interrupt system. So, all the interrupts
are detected via polling of the "Status Byte".

For the mechanical system with the motors used, the tuning of the PID filter resulted in:

Ds = 2
Kp = 10
Ki = 0
Kd = 1000
Ii = 1000

These parameters are not generic, they depend on the mechanical system and motors.
Therefore, for each case, a tuning need to be done according to "Tuning the PID Filter" in:

    http://www.ti.com/lit/an/snva025c/snva025c.pdf

To help in tuning there is the function "Tuning_PID" and to help in debug there is the
functions "debug_parameter" and "debug_SB".
________________________________________________________________________________________________________________________

LM629 COMMANDS:

    Name    Code      Data     Meaning
                      Bytes
    RESET   0x00/00    0       Reset LM629
    DFH     0x02/02    0       Define Home
    SIP     0x03/03    0       Set Index Position (não usado)
    LPEI    0x1B/27    2       0x0000 a 0x7FFF Load Position Error for Interrupt
    LPES    0x1A/26    2       0x0000 a 0x7FFF Load Position Error for Stopping
    SBPA    0x20/32    4       0xC0000000 a 0x3FFFFFFF Set Breakpoint Absolute
    SBPR    0x21/33    4       See Manual SBPR
    MSKI    0x1C/28    2       See Manual Mask Interrupts
    RSTI    0x1D/29    2       See Manual Reset Interrupts
    LFIL    0x1E/30    2 a 10  See Manual Load Filter Parameters
    UDF     0x04/04    0       Update Filter
    LTRJ    0x1F/31    2 a 14  See Manual Load Trajectory Parameters
    STT     0x01/01    0       Start Motion Control
    RDSTAT             1       Read Status Byte
    RDSIGS  0x0C/12    2       Read Signals Register
    RDIP    0x09/09    4       0xC0000000 a 0x3FFFFFFF Read Index Position (not used)
    RDDP    0x08/08    4       0xC0000000 a 0x3FFFFFFF Read Desired Position (not used)
    RDRP    0x0A/10    4       0xC0000000 a 0x3FFFFFFF Read Real Position (not used)
    RDDV    0x07/07    4       0xC0000000 a 0x3FFFFFFF Read Desired Velocity (not used)
    RDRV    0x0B/11    2       0xC000 a 0x3FFF Read Real Velocity (not used)
    RDSUM   0x0D/13    2       See Manual Read Integration Term (not used)

************************************************************************************************************************
************** The first three commands must be - Initialize_SysLM629, Initialize_Drive and Wakeup_Robot ***************
************************************************************************************************************************
*/

#ifndef _LM629_INCLUDED_
#define _LM629_INCLUDED_

#pragma used+

//************************** PROTOTYPE FUNCTIONS ***********************************************************************

//Initializes the LM629 - PID parameters
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//pds - Derivative-term sampling interval
//pkkp - PID filter Kp coefficient
//pkki - PID filter Ki coefficient
//pkkd - PID filter Kd coefficient
//pkii - PID filter Ii coefficient
//Example:  Initialize_Drive(LMB,5.0,0,15,0,0,0);
bool Initialize_Drive(unsigned char Lm,float multiplicador,unsigned char pds,unsigned int pkkp,
                      unsigned int pkki,unsigned int pkkd,unsigned int pkii);

//Initializes the LM629 - basic parameters
//rel - pointer to tempo variable (interrupt TCD1)
//cts - sampling period Ts
//prtc - TWI-PCA9554 address  Ex: 0x40
//cslmd - pin for selecting LM629-1 (right wheel)
//cslme - pin for selecting LM629-2 (left wheel)
//rstlmd - pin for resetting LM629-1
//rstlme - pin for resetting LM629-2
//prd - pin for sending status and data to the port
//pwr - pin for sending from port to command and data
//pps - pin for selecting the port mode: commands (0) or data (1)
//pamax - maximum acceleration em Cm/seg**2
//pvmax - maximum speed Cm/seg
//pcroda - wheel circumference Cm
//pdeixo - wheel to wheel distance Cm
//pcr - encoder counts per revolution channels A and B
//Example:
//Initialize_SysLM629(&relogio,0.0004096,PTA,"B1","B2","B3","B4","B5","B6","B7",30.0,80.0,31.0,33.5,1000);
bool Initialize_SysLM629(unsigned long int *rel,float cts,unsigned char prt,unsigned char *cslmd,
                         unsigned char *cslme,unsigned char *rstlmd,unsigned char *rstlme,unsigned char *prd,
                         unsigned char *pwr,unsigned char *pps,float pamax,float pvmax,float pcroda,float pdeixo,
                         unsigned int pcr);

//Defines the robot acceleration for a movement. The robot must be stopped. Remember that is not possible to change
//acceleration during a movement.
//Ac => Cm/seg/seg
void Set_A(float Ac);

//Defines the robot velocity before a trajectory command
//Ve => Cm/seg
void Set_V(float Ve);

//Selects the LM629
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
void Select_LM629(unsigned char Lm);

//Returns the LM629 Status Byte
//Lm = LMD => right wheel;Lm = LME => left wheel
//Example:
//X = RDSTAT(LMD) ...;
unsigned char RDSTAT(unsigned char Lm);

//Acceleration in Counts/Sample**2
//Lm = LMD => right wheel;Lm = LME => left wheel
//Acel: Cm/seg**2
unsigned long int AccCounts(unsigned char Lm,float Acel);

//Velocity in Counts/Sample
//Lm = LMD => right wheel;Lm = LME => left wheel
//Velo: Cm/Seg
unsigned long int VelCounts(unsigned char Lm,float Velo);

//Distance in Counts
//Lm = LMD => right wheel;Lm = LME => left wheel
//Posi: Cm
signed long int PosCounts(unsigned char Lm,float Posi);

//Reads bit Byt of Status Byte
//Lm = LMD => right wheel;Lm = LME => left wheel
//byt: 0 a 7
bool SB_Bit(unsigned char Lm,unsigned char byt);

//Waits Status Byte bit 0 be 0
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
void Wait_BusyBit(unsigned char Lm);

//Detects position error in both LM629
bool Position_Error(void);

//Detects if robot is stopped
bool Verify_Robot_Stopped(void);

//Sends a command or data byte to the LM629 saying if it is to wait
//Status Byte bit 0 be 0 after execution
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//Valor - Byte to send
//Tipo - 0 = command, 1 = first data byte, 2 = second data byte
void Send_Byte(unsigned char Lm,unsigned char Valor,unsigned char Tipo);

//Receives a byte in the port
//Lm = LMD => right wheel;Lm = LME => left wheel
unsigned char Receive_Byte(unsigned char Lm);

//Receives 2 bytes (int) from LM629 => [MSB][LSB]
//Lm = LMD => right wheel;Lm = LME => left wheel
unsigned int Receive_Word(unsigned char Lm);

//Receives 4 bytes (long int) from LM629 => [MSBH][LSBH][MSBL][LSBL]
//Lm = LMD => right wheel;Lm = LME => left wheel
unsigned long int Receive_Doubleword(unsigned char Lm);

//Sends a command to the LM629 saying the parameters number (word)
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//Cmnd - command code
//Np - parameters number (words)
void Send_Command(unsigned char Lm,unsigned char Cmnd,unsigned char Np);

//Reads the Signal Register
//Lm = LMD => right wheel;Lm = LME => left wheel
unsigned int Read_Signals_Register(unsigned char Lm);

//Waits the Signals Register On Target Flag of both LM629
void Wait_Robot_OnTarget(void);

//Executes the robot trajectory waiting or not for the robot on target
//wait - True => wait; False => do not wait
void Start_Robot_Trajectory(bool wait);

//Executes the RSTI command
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//mask - resets the interrupts
//If Bit1 ... Bit6 = 0, that interrupt will be resetted
//Bit0 -
//Bit1 - Command-Error Interrupt
//Bit2 - Trajectory-Complete Interrupt
//Bit3 - Index-Pulse Interrupt
//Bit4 - Wrap-Around Interrupt
//Bit5 - Position-Error Interrupt
//Bit6 - Breakpoint Interrupt
//Bit7 -
void Reset_Interrupts(unsigned char Lm,unsigned char mask);

//Executes the MSKI command
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//mask - mask the interrupts
//If Bit1 ... Bit6: 0 disable and 1 enable that interrupt
//Bit0 -
//Bit1 - Command-Error Interrupt
//Bit2 - Trajectory-Complete Interrupt
//Bit3 - Index-Pulse Interrupt
//Bit4 - Wrap-Around Interrupt
//Bit5 - Position-Error Interrupt
//Bit6 - Breakpoint Interrupt
//Bit7 -
void Mask_Interrupts(unsigned char Lm,unsigned char mask);

//Executes LPEI command
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//TresholdPosErr: treshold for position error detection in counts
//The detection occurs when the absolute value of the position error exceeds TresholdPosErr
void Load_Position_Error_for_Interrupt(unsigned char Lm,unsigned int TresholdPosErr);

//Executes LPES command
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//TresholdPosErr: treshold for position error detection in counts
//The detection occurs when the absolute value of the position error exceeds TresholdPosErr
void Load_Position_Error_for_Stopping(unsigned char Lm,unsigned int TresholdPosErr);

//Executes SBPA command
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//Breakpoint: Breakpoint for absolute position in centimeters
//The Status Byte bit 6 is set when the position is reached
void Set_Breakpoint_Absolute(unsigned char Lm,float Breakpoint);

//Executes SBPR command
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//Breakpoint: Breakpoint for relative position in centimeters
//The Status Byte bit 6 is set when the position is reached
void Set_Breakpoint_Relative(unsigned char Lm,float Breakpoint);

//Waits the Status Byte breakpoint reached flag
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
void Wait_Breakpoint(unsigned char Lm);

//Verifies what is the mode: True - Velocity Mode; False - Position Mode
//Lm = LMD => right wheel;Lm = LME => left wheel
bool Is_Velocity_Mode(unsigned char Lm);

//Defines the present position as the absolute position 0
//Lm = LMD => right wheel;Lm = LME => left wheel
void Define_Home(unsigned char Lm);

//Executes the LFIL command. It is important to remember that the
//PID filter tuning must be done to achieve a critical system dumping
//looking for an optimized performance.
//See "Tuning the PID Filter" at http://www.ti.com/lit/an/snva025c/snva025c.pdf
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//coef FCW LSB
//bit 0 => loads KIi
//bit 1 => loads KIi
//bit 2 => loads KIi
//bit 3 => loads KIi
//bits 4 a 7 => 0
void Update_Filter (unsigned char Lm,unsigned char coef);

//Executes the LTRJ command that loads the trajectory parameters
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//FDir: direction (only for velocity mode)
//MVel: velocity mode
//CAcc: loads acceleration
//AccR: the acceleration is relative
//CVel: loads velocity
//VelR: the velocity is relative
//CPos: loads position
//PosR: the position is relative
//Acc: acceleration in counts/sample**2
//Vel: velocity in counts/sample
//Pos: position in counts
void Load_Trajectory_Parameters (unsigned char Lm,bool FDir,bool MVel,bool CAcc,bool AccR,
                                bool CVel,bool VelR,bool CPos,bool PosR,signed long int Acc,signed long int Vel,
                                signed long int Pos);

//Stops the movement
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//Tipo: 10 smoothly, 9 abruptly, 8 motor off
void Stop_Motion(unsigned char Lm,unsigned char Tipo);

//Executes the LM629 hardware initialization. Mandatory before LM629 use
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//tout - time out in milliseconds (10ms multiple)
bool Hardware_Reset(unsigned char Lm,unsigned long int tout);

//Executes the LM629 software initialization. Can be used after the hardware initialization
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
void Software_Reset(unsigned char Lm);

//Waits the Status Byte motor off flag of both LM629
void Wait_Robot_Stop(void);

//Prepares an absolute position movement
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//PAcc: cm/(seg)**2
//Pvel: cm/seg
//PPos: cm
void Absolute_Position_Move(unsigned char Lm,float PAcc,float PVel,float PPos);

//Prepares a relative position movement
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//PAcc: cm/(seg)**2
//Pvel: cm/seg
//PPos: cm
void Relative_Position_Move(unsigned char Lm,float PAcc,float PVel,float PPos);

//Prepares a velocity mode movement. The calling program must issue the necessary
//SBPA, SBPR, STT and StopMotion commands
//Lm = LMD => right wheel;Lm = LME => left wheel;Lm = LMB => both
//FDir: direction (only for velocity mode)
//CAcc: loads acceleration
//AccR: the acceleration is relative
//CVel: loads velocity
//VelR: the velocity is relative
//PAcc: acceleration in cm/seg**2
//PVel: velocity in cm/seg
void Velocity_Mode_Move (unsigned char Lm,bool FDir,bool CAcc,bool AccR,bool CVel,bool VelR,
                        float PAcc,float PVel);

//Stright forward robot move in velocity mode
//Ac => acceleration in Cm/seg/seg; Ve => velocity in Cm/seg
void Drive_Forward(float Ac,float Ve);

//Stright backward robot move in velocity mode
//Ac => acceleration in Cm/seg/seg; Ve => velocity in Cm/seg
void Drive_Backward(float Ac,float Ve);

//Changes the robot velocity during a velocity mode movement
//Velo - velocity in cm/seg
void Change_Current_Move_Velocity(float Velo);

//Changes the robot velocity to default during a velocity mode movement
void Default_Current_Move_Velocity(void);

//The robot turns Giro degrees in place counterclockwise
//Giro - angle in degrees
void Turn_Left(float Giro);

//The robot turns Giro degrees in place clockwise
//Giro - angle in degrees
void Turn_Right(float Giro);

//Stright forward robot move in position mode by a distance
//Dist - distance in cm
void Drive_Forward_Distance(float Dist);

//Stright backward robot move in position mode by a distance
//Dist - distance in cm
void Drive_Backward_Distance(float Dist);

//Initializes the robot
bool Wakeup_Robot(bool Cp);

//Turns off the robot. Before, the robot executes a complete turn in place
void Sleep_Robot(void);

//Changes the robot velocity to half of the current velocity (velocity mode)
void Slower(void);

//Changes the robot velocity to double of the current velocity (velocity mode)
void Faster(void);

//Calculates the new velocity of one wheel for the robot to drive in a curved
//trajectory to the right with Raio cm gyration radius
//Raio: trajectory gyration radius in cm
void Go_Right(float Raio);

//Calculates the new velocity of one wheel for the robot to drive in a curved
//trajectory to the left with Raio cm gyration radius
//Raio: trajectory gyration radius in cm
void Go_Left(float Raio);

//The robot drives stright
void Go_Stright(void);

#pragma used-

#pragma library lm629.lib

#endif






