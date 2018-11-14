/**************************************************************************//**
 * @file em_device.h
 * @brief CMSIS Cortex-M Peripheral Access Layer for Silicon Laboratories
 *        microcontroller devices
 *
 * This is a convenience header file for defining the part number on the
 * build command line, instead of specifying the part specific header file.
 *
 * @verbatim
 * Example: Add "-DEFM32G890F128" to your build options, to define part
 *          Add "#include "em_device.h" to your source files

 *
 * @endverbatim
 * @version 5.4.0
 ******************************************************************************
 * # License
 * <b>Copyright 2017 Silicon Laboratories, Inc. www.silabs.com</b>
 ******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.@n
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.@n
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Laboratories, Inc.
 * has no obligation to support this Software. Silicon Laboratories, Inc. is
 * providing the Software "AS IS", with no express or implied warranties of any
 * kind, including, but not limited to, any implied warranties of
 * merchantability or fitness for any particular purpose or warranties against
 * infringement of any proprietary rights of a third party.
 *
 * Silicon Laboratories, Inc. will not be liable for any consequential,
 * incidental, or special damages, or any other relief, or for any claim by
 * any third party, arising from your use of this Software.
 *
 *****************************************************************************/

#ifndef EM_DEVICE_H
#define EM_DEVICE_H

#if defined(EFM32TG11B120F128GM32)
#include "efm32tg11b120f128gm32.h"

#elif defined(EFM32TG11B120F128GM64)
#include "efm32tg11b120f128gm64.h"

#elif defined(EFM32TG11B120F128GQ48)
#include "efm32tg11b120f128gq48.h"

#elif defined(EFM32TG11B120F128GQ64)
#include "efm32tg11b120f128gq64.h"

#elif defined(EFM32TG11B120F128IM32)
#include "efm32tg11b120f128im32.h"

#elif defined(EFM32TG11B120F128IM64)
#include "efm32tg11b120f128im64.h"

#elif defined(EFM32TG11B120F128IQ48)
#include "efm32tg11b120f128iq48.h"

#elif defined(EFM32TG11B120F128IQ64)
#include "efm32tg11b120f128iq64.h"

#elif defined(EFM32TG11B140F64GM32)
#include "efm32tg11b140f64gm32.h"

#elif defined(EFM32TG11B140F64GM64)
#include "efm32tg11b140f64gm64.h"

#elif defined(EFM32TG11B140F64GQ48)
#include "efm32tg11b140f64gq48.h"

#elif defined(EFM32TG11B140F64GQ64)
#include "efm32tg11b140f64gq64.h"

#elif defined(EFM32TG11B140F64IM32)
#include "efm32tg11b140f64im32.h"

#elif defined(EFM32TG11B140F64IM64)
#include "efm32tg11b140f64im64.h"

#elif defined(EFM32TG11B140F64IQ48)
#include "efm32tg11b140f64iq48.h"

#elif defined(EFM32TG11B140F64IQ64)
#include "efm32tg11b140f64iq64.h"

#elif defined(EFM32TG11B320F128GM64)
#include "efm32tg11b320f128gm64.h"

#elif defined(EFM32TG11B320F128GQ48)
#include "efm32tg11b320f128gq48.h"

#elif defined(EFM32TG11B320F128GQ64)
#include "efm32tg11b320f128gq64.h"

#elif defined(EFM32TG11B320F128IM64)
#include "efm32tg11b320f128im64.h"

#elif defined(EFM32TG11B320F128IQ48)
#include "efm32tg11b320f128iq48.h"

#elif defined(EFM32TG11B320F128IQ64)
#include "efm32tg11b320f128iq64.h"

#elif defined(EFM32TG11B340F64GM64)
#include "efm32tg11b340f64gm64.h"

#elif defined(EFM32TG11B340F64GQ48)
#include "efm32tg11b340f64gq48.h"

#elif defined(EFM32TG11B340F64GQ64)
#include "efm32tg11b340f64gq64.h"

#elif defined(EFM32TG11B340F64IM64)
#include "efm32tg11b340f64im64.h"

#elif defined(EFM32TG11B340F64IQ48)
#include "efm32tg11b340f64iq48.h"

#elif defined(EFM32TG11B340F64IQ64)
#include "efm32tg11b340f64iq64.h"

#elif defined(EFM32TG11B520F128GM32)
#include "efm32tg11b520f128gm32.h"

#elif defined(EFM32TG11B520F128GM64)
#include "efm32tg11b520f128gm64.h"

#elif defined(EFM32TG11B520F128GM80)
#include "efm32tg11b520f128gm80.h"

#elif defined(EFM32TG11B520F128GQ48)
#include "efm32tg11b520f128gq48.h"

#elif defined(EFM32TG11B520F128GQ64)
#include "efm32tg11b520f128gq64.h"

#elif defined(EFM32TG11B520F128GQ80)
#include "efm32tg11b520f128gq80.h"

#elif defined(EFM32TG11B520F128IM32)
#include "efm32tg11b520f128im32.h"

#elif defined(EFM32TG11B520F128IM64)
#include "efm32tg11b520f128im64.h"

#elif defined(EFM32TG11B520F128IM80)
#include "efm32tg11b520f128im80.h"

#elif defined(EFM32TG11B520F128IQ48)
#include "efm32tg11b520f128iq48.h"

#elif defined(EFM32TG11B520F128IQ64)
#include "efm32tg11b520f128iq64.h"

#elif defined(EFM32TG11B520F128IQ80)
#include "efm32tg11b520f128iq80.h"

#elif defined(EFM32TG11B540F64GM32)
#include "efm32tg11b540f64gm32.h"

#elif defined(EFM32TG11B540F64GM64)
#include "efm32tg11b540f64gm64.h"

#elif defined(EFM32TG11B540F64GM80)
#include "efm32tg11b540f64gm80.h"

#elif defined(EFM32TG11B540F64GQ48)
#include "efm32tg11b540f64gq48.h"

#elif defined(EFM32TG11B540F64GQ64)
#include "efm32tg11b540f64gq64.h"

#elif defined(EFM32TG11B540F64GQ80)
#include "efm32tg11b540f64gq80.h"

#elif defined(EFM32TG11B540F64IM32)
#include "efm32tg11b540f64im32.h"

#elif defined(EFM32TG11B540F64IM64)
#include "efm32tg11b540f64im64.h"

#elif defined(EFM32TG11B540F64IM80)
#include "efm32tg11b540f64im80.h"

#elif defined(EFM32TG11B540F64IQ48)
#include "efm32tg11b540f64iq48.h"

#elif defined(EFM32TG11B540F64IQ64)
#include "efm32tg11b540f64iq64.h"

#elif defined(EFM32TG11B540F64IQ80)
#include "efm32tg11b540f64iq80.h"

#else
#error "em_device.h: PART NUMBER undefined"
#endif
#endif /* EM_DEVICE_H */
