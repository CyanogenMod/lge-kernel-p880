//***************************************************************************
//!file     si_regioCbus.c
//!brief    CBUS register I/O function wrappers.
//
// No part of this work may be reproduced, modified, distributed,
// transmitted, transcribed, or translated into any language or computer
// format, in any form or by any means without written permission of
// Silicon Image, Inc., 1060 East Arques Avenue, Sunnyvale, California 94085
//
// Copyright 2009, Silicon Image, Inc.  All rights reserved.
//***************************************************************************/

#include <linux/types.h>

#include "SiI9244_I2C_master.h"
#include "SiI9244_I2C_slave_add.h"


static byte l_cbusPortOffsets [ MHD_MAX_CHANNELS ] = { 0x00 };

//------------------------------------------------------------------------------
// Function:    SiIRegioCbusRead
// Description: Read a one byte CBUS register with port offset.
//              The register address parameter is translated into an I2C slave
//              address and offset. The I2C slave address and offset are used
//              to perform an I2C read operation.
//------------------------------------------------------------------------------


byte SiIRegioCbusRead ( word regAddr, byte channel )
{
    return(I2C_ReadByte(SA_TX_CBUS_Primary + l_cbusPortOffsets[channel], regAddr));
}

//------------------------------------------------------------------------------
// Function:    SiIRegioCbusWrite
// Description: Write a one byte CBUS register with port offset.
//              The register address parameter is translated into an I2C
//              slave address and offset. The I2C slave address and offset
//              are used to perform an I2C write operation.
//------------------------------------------------------------------------------

void SiIRegioCbusWrite ( word regAddr, byte channel, byte value )
{

    I2C_WriteByte(SA_TX_CBUS_Primary + l_cbusPortOffsets[channel], regAddr, value);
}

