/*
 * Copyright (c) 2020-23, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== power_xwrLx4xx_freertos.c ========
 */

#include <stdint.h>

/* driverlib header files */
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/tistdtypes.h>
#include <drivers/prcm.h>
#include <board/cpu.h>
#include <drivers/hw_include/cslr_soc.h>

#include <kernel/dpl/ClockP.h>
#include <kernel/nortos/dpl/m4/SysTickTimerP.h>

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#include <portmacro.h>
#include <drivers/power.h>
#include <drivers/power_xwrLx4xx.h>

extern Power_ConfigV1 Power_config;

/* bitmask of constraints that disallow LPDS */
#define LPDS_DISALLOWED (1 << POWER_DISALLOW_LPDS)

/* macro to pick two matching count values */
#define COUNT_WITHIN_TRESHOLD(a, b, c, th) \
        ((((b) - (a)) <= (th)) ? (b) : (c))

//#define TRUE    1
//#define FALSE   0


static volatile uint32_t idleTicks = 0;
unsigned long long ullSleepTime, ullIdleTime;

void Power_sleepPolicy(unsigned long long sleepTimeus)
{
#if (configUSE_TICKLESS_IDLE != 0)
    int i = 0;
    bool returnFromSleep = FALSE;
    bool returnFromIdle = FALSE;
    unsigned long constraintMask;
    unsigned long long ullLowPowerTimeBeforeSleep, ullLowPowerTimeAfterSleep, ullLowPowerTimeAfterIdle;
    unsigned long long count[3];
    unsigned long long remain;
    uint32_t LPDSlatency, sleepLatency, idleLatency;
    uint32_t LPDSTh, sleepTh, idleTh;
    TimerP_Params idleTimerParams;

    Power_setLowPowModeTaken(POWER_NONE);

    // Enter a critical section that will not effect interrupts
    // bringing the MCU out of sleep mode.
    vPortEnterCritical();

    // Query for Constraints
    constraintMask = Power_getConstraintMask();

    // Check if we are allowed to go to LPDS
    if ((constraintMask & LPDS_DISALLOWED) == 0)
    {
         // Read the current time from a time source that will remain
         // operational while the microcontroller is in a low power state.

         // Get the current RTC count, using the fast interface; to use the
         // fast interface the count must be read three times, and then
         // the value that matches on at least two of the reads is chosen

        for (i = 0; i < 3; i++)
        {
            count[i] =  PRCMSlowClkCtrGet();
        }

        ullLowPowerTimeBeforeSleep = COUNT_WITHIN_TRESHOLD(count[0], count[1], count[2], 1);

        // Stop the timer that is generating the tick interrupt
        SysTickTimerP_stop();


            // Check if LPDS is possible. All of the below are in MicroSeconds
            LPDSlatency = Power_getTransitionLatency(POWER_LPDS, Power_TOTAL_LATENCY);
            sleepLatency = Power_getTransitionLatency(POWER_SLEEP, Power_TOTAL_LATENCY);
            idleLatency = Power_getTransitionLatency(POWER_IDLE, Power_TOTAL_LATENCY);
            LPDSTh = Power_getThresholds(POWER_LPDS);
            sleepTh = Power_getThresholds(POWER_SLEEP);
            idleTh = Power_getThresholds(POWER_IDLE);
            if (sleepTimeus > LPDSTh) 
            {
                // If sleep counter is used, it is driven by slow clock
                remain = ((sleepTimeus - LPDSlatency) * 32768) / 1000000;

                if(Power_config.enableSleepCounterWakeupLPDS)
                {
                     PRCMLPDSIntervalSet(remain);
                }

                // Disable interrupts during ITM flush and restore
                CPUcpsid();

                // Now go to LPDS
                Power_sleep(POWER_LPDS);

                // re-enable interrupts
                CPUcpsie();
                
                Power_setLowPowModeTaken(POWER_LPDS);

                // set 'returnFromSleep' to TRUE
                returnFromSleep = TRUE;
            }
            else if (sleepTimeus > sleepTh)
            {
                remain = ((sleepTimeus - sleepLatency) * 32768) / 1000000;

                /* set the LPDS wakeup time interval */
                PRCMLPDSIntervalSet(remain);

                /* enable the wake source to be timer */
                PRCMLPDSWakeupSourceEnable(PRCM_LPDS_SLEEP_COUNTER \
                        | PRCM_LPDS_RTC_COUNTER);

                /* disable interrupts during ITM flush and restore */
                CPUcpsid();

                /* now go to Device Sleep Mode ... */
                Power_sleep(POWER_SLEEP);

                /* re-enable interrupts */
                CPUcpsie();

                Power_setLowPowModeTaken(POWER_SLEEP);

                /* set 'returnFromSleep' to TRUE*/
                returnFromSleep = TRUE;
            }
            else if (sleepTimeus > idleTh)
            {
                vPortExitCritical();
                // Configure RTI Counter 0 as Wakeup source from idle3 state
                TimerP_Params_init(&idleTimerParams);
                // In Idle3 state, only osc clock is enabled. Selecting Osc clock as source for RTI with prescale 1.
                idleTimerParams.inputPreScaler = 1;
                idleTimerParams.inputClkHz     = 40000000u;
                idleTimerParams.periodInUsec   = sleepTimeus - idleLatency;
                idleTimerParams.oneshotMode    = 0;
                idleTimerParams.enableOverflowInt = 1;
                TimerP_setup(CSL_APP_RTIA_U_BASE, &idleTimerParams);
                
                Power_sleep(POWER_IDLE);
                
                Power_setLowPowModeTaken(POWER_IDLE);

                returnFromIdle = TRUE;
            }
            else 
            {
                SysTickTimerP_start();

                vPortExitCritical();

                returnFromSleep = FALSE;
            }
    }
    else 
    {
        /* A constraint was set */
        vPortExitCritical();
    }

    if (returnFromSleep ) 
    {
        /*
         *  Determine how long the microcontroller was actually in a low
         *  power state for, which will be less than xExpectedIdleTime if the
         *  microcontroller was brought out of low power mode by an interrupt
         *  other than that configured by the vSetWakeTimeInterrupt() call.
         *  Note that the scheduler is suspended before
         *  portSUPPRESS_TICKS_AND_SLEEP() is called, and resumed when
         *  portSUPPRESS_TICKS_AND_SLEEP() returns.  Therefore no other
         *  tasks will execute until this function completes.
         */
        for (i = 0; i < 3; i++) 
        {
            count[i] =  PRCMSlowClkCtrGet();
        }
        ullLowPowerTimeAfterSleep =
                COUNT_WITHIN_TRESHOLD(count[0], count[1], count[2], 1);

        ullSleepTime = ullLowPowerTimeAfterSleep - ullLowPowerTimeBeforeSleep;

        ullSleepTime = ullSleepTime*1000;
        ullSleepTime = ullSleepTime/32768;

        /*
         *  Correct the kernels tick count to account for the time the
         *  microcontroller spent in its low power state.
         */
        vTaskStepTick((unsigned long)ullSleepTime);

        /* Restart the timer that is generating the tick interrupt. */
        SysTickTimerP_start();

        /*
         *  Exit the critical section - it might be possible to do this
         *  immediately after the prvSleep() calls.
         */
        vPortExitCritical();
    }
    else if (returnFromIdle)
    {
        for (i = 0; i < 3; i++) {
            count[i] =  PRCMSlowClkCtrGet();
        }
        ullLowPowerTimeAfterIdle =
                COUNT_WITHIN_TRESHOLD(count[0], count[1], count[2], 1);

        ullIdleTime = ullLowPowerTimeAfterIdle - ullLowPowerTimeBeforeSleep;

        ullIdleTime = ullIdleTime*1000;
        ullIdleTime = ullIdleTime/32768;

        /*
         *  Correct the kernels tick count to account for the time the
         *  microcontroller spent in its low power state.
         */
        vTaskStepTick((unsigned long)ullIdleTime);

        /* Restart the timer that is generating the tick interrupt. */
        SysTickTimerP_start();
    }
#endif
}

/*
 *  ======== Power_initPolicy ========
 */
void Power_initPolicy()
{
}

/* Tickless Hook */
void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
#if (configUSE_TICKLESS_IDLE != 0)
    unsigned long long time;
    idleTicks = xExpectedIdleTime;
    // convert ticks to microseconds
    time = ((idleTicks * 1000000)/configTICK_RATE_HZ);
    Power_idleFunc(time);
#endif
}
