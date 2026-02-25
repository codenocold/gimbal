#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

#include "main.h"

//
// Take in and set to correct values for Cortex-A systems with CPU cache
//
//#define SEGGER_RTT_CPU_CACHE_LINE_SIZE            (32)          // Largest cache line size (in bytes) in the current system
//#define SEGGER_RTT_UNCACHED_OFF                   (0xFB000000)  // Address alias where RTT CB and buffers can be accessed uncached

//
// Most common case:
// Up-channel 0: RTT
// Up-channel 1: SystemView
//
#ifndef SEGGER_RTT_MAX_NUM_UP_BUFFERS
    #define SEGGER_RTT_MAX_NUM_UP_BUFFERS (3) // Max. number of up-buffers (T->H) available on this target    (Default: 3)
#endif

//
// Most common case:
// Down-channel 0: RTT
// Down-channel 1: SystemView
//
#ifndef SEGGER_RTT_MAX_NUM_DOWN_BUFFERS
    #define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS (3) // Max. number of down-buffers (H->T) available on this target  (Default: 3)
#endif

#ifndef BUFFER_SIZE_UP
    #define BUFFER_SIZE_UP (512) // Size of the buffer for terminal output of target, up to host (Default: 1k)
#endif

#ifndef BUFFER_SIZE_DOWN
    #define BUFFER_SIZE_DOWN (32) // Size of the buffer for terminal input to target from host (Usually keyboard input) (Default: 16)
#endif

#ifndef SEGGER_RTT_PRINTF_BUFFER_SIZE
    #define SEGGER_RTT_PRINTF_BUFFER_SIZE (64u) // Size of buffer for RTT printf to bulk-send chars via RTT     (Default: 64)
#endif

#ifndef SEGGER_RTT_MODE_DEFAULT
    #define SEGGER_RTT_MODE_DEFAULT SEGGER_RTT_MODE_NO_BLOCK_SKIP // Mode for pre-initialized terminal channel (buffer 0)
#endif

/*********************************************************************
*
*       RTT memcpy configuration
*
*       memcpy() is good for large amounts of data,
*       but the overhead is big for small amounts, which are usually stored via RTT.
*       With SEGGER_RTT_MEMCPY_USE_BYTELOOP a simple byte loop can be used instead.
*
*       SEGGER_RTT_MEMCPY() can be used to replace standard memcpy() in RTT functions.
*       This is may be required with memory access restrictions,
*       such as on Cortex-A devices with MMU.
*/
#ifndef SEGGER_RTT_MEMCPY_USE_BYTELOOP
    #define SEGGER_RTT_MEMCPY_USE_BYTELOOP 0 // 0: Use memcpy/SEGGER_RTT_MEMCPY, 1: Use a simple byte-loop
#endif

//
// Example definition of SEGGER_RTT_MEMCPY to external memcpy with GCC toolchains and Cortex-A targets
//
//#if ((defined __SES_ARM) || (defined __CROSSWORKS_ARM) || (defined __GNUC__)) && (defined (__ARM_ARCH_7A__))
//  #define SEGGER_RTT_MEMCPY(pDest, pSrc, NumBytes)      SEGGER_memcpy((pDest), (pSrc), (NumBytes))
//#endif

//
// Target is not allowed to perform other RTT operations while string still has not been stored completely.
// Otherwise we would probably end up with a mixed string in the buffer.
// If using  RTT from within interrupts, multiple tasks or multi processors, define the SEGGER_RTT_LOCK() and SEGGER_RTT_UNLOCK() function here.
//
// SEGGER_RTT_MAX_INTERRUPT_PRIORITY can be used in the sample lock routines on Cortex-M3/4.
// Make sure to mask all interrupts which can send RTT data, i.e. generate SystemView events, or cause task switches.
// When high-priority interrupts must not be masked while sending RTT data, SEGGER_RTT_MAX_INTERRUPT_PRIORITY needs to be adjusted accordingly.
// (Higher priority = lower priority number)
// Default value for embOS: 128u
// Default configuration in FreeRTOS: configMAX_SYSCALL_INTERRUPT_PRIORITY: ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
// In case of doubt mask all interrupts: 1 << (8 - BASEPRI_PRIO_BITS) i.e. 1 << 5 when 3 bits are implemented in NVIC
// or define SEGGER_RTT_LOCK() to completely disable interrupts.
//
#ifndef SEGGER_RTT_MAX_INTERRUPT_PRIORITY
    #define SEGGER_RTT_MAX_INTERRUPT_PRIORITY (1)
#endif

#define SEGGER_RTT_LOCK() \
    {                     \
        __set_BASEPRI(SEGGER_RTT_MAX_INTERRUPT_PRIORITY);

#define SEGGER_RTT_UNLOCK() \
    __set_BASEPRI(0);       \
    }

#ifndef SEGGER_RTT_LOCK
    #define SEGGER_RTT_LOCK() // Lock RTT (nestable)   (i.e. disable interrupts)
#endif

#ifndef SEGGER_RTT_UNLOCK
    #define SEGGER_RTT_UNLOCK() // Unlock RTT (nestable) (i.e. enable previous interrupt lock state)
#endif

#endif
