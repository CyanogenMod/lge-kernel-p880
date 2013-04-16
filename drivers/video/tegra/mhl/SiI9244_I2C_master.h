/*===========================================================================

                        SiI9024A I2C MASTER.C
              

DESCRIPTION
  This file explains the SiI9024A initialization and call the virtual main function.
  

 Copyright (c) 2002-2009, Silicon Image, Inc.  All rights reserved.             
  No part of this work may be reproduced, modified, distributed, transmitted,    
 transcribed, or translated into any language or computer format, in any form   
or by any means without written permission of: Silicon Image, Inc.,            
1060 East Arques Avenue, Sunnyvale, California 94085                           
===========================================================================*/

/*===========================================================================

                      EDIT HISTORY FOR FILE

when              who                         what, where, why
--------        ---                        ----------------------------------------------------------
2010/10/25    Daniel Lee(Philju)      Initial version of file, SIMG Korea 
===========================================================================*/
#include "Common_Def.h"

#include <linux/types.h>

/*===========================================================================

===========================================================================*/


void I2C_WriteByte(byte deviceID, byte offset, byte value);
byte I2C_ReadByte(byte deviceID, byte offset);

byte ReadByteTPI (byte Offset); 
void WriteByteTPI (byte Offset, byte Data);
void WriteIndexedRegister (byte PageNum, byte Offset, byte Data);
void ReadModifyWriteIndexedRegister (byte PageNum, byte Offset, byte Mask, byte Data);
void ReadModifyWriteIndexedRegister (byte PageNum, byte Offset, byte Mask, byte Data);
void ReadModifyWriteTPI(byte Offset, byte Mask, byte Data);
void WriteByteCBUS(byte Offset, byte Data);
void ReadModifyWriteCBUS(byte Offset, byte Mask, byte Value) ;
byte ReadIndexedRegister (byte PageNum, byte Offset) ;
byte ReadByteCBUS (byte Offset) ;




