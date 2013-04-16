//***************************************************************************
//!file     si_RegioCbus.h
//!brief    CBUS register I/O function wrappers.
//
// No part of this work may be reproduced, modified, distributed,
// transmitted, transcribed, or translated into any language or computer
// format, in any form or by any means without written permission of
// Silicon Image, Inc., 1060 East Arques Avenue, Sunnyvale, California 94085
//
// Copyright 2009, Silicon Image, Inc.  All rights reserved.
//***************************************************************************/

#ifndef __SI_REGIO_H__
#define __SI_REGIO_H__

#include <linux/types.h>

#include "Common_Def.h"



byte SiIRegioCbusRead ( word regAddr, byte channel );
void SiIRegioCbusWrite ( word regAddr, byte channel, byte value );

#endif // __SI_REGIO_H__

