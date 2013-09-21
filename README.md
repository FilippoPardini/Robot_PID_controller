Robot_PID_controller
====================

Ansi C library to control robot movements via LM629 and LMD18200.


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

