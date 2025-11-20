/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
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
 *  ======== power_xwrLx4xx_nortos.c ========
 */

#include <stdint.h>

/* driverlib header files */
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/tistdtypes.h>
#include <drivers/prcm.h>
#include <board/cpu.h>

#include <kernel/dpl/ClockP.h>
#include <kernel/nortos/dpl/m4/SysTickTimerP.h>
#include <drivers/power.h>
#include <drivers/power_xwrLx4xx.h>

/* bitmask of constraints that disallow LPDS */
#define LPDS_DISALLOWED (1 << POWER_DISALLOW_LPDS)

/* macro to pick two matching count values */
#define COUNT_WITHIN_TRESHOLD(a, b, c, th) \
        ((((b) - (a)) <= (th)) ? (b) : (c))

//#define TRUE    1
//#define FALSE   0


//static volatile uint32_t idleTime = 0;

void Power_sleepPolicy()
{
#if 0
#if (configUSE_TICKLESS_IDLE != 0)
    int i = 0;
    bool returnFromSleep = FALSE;
    unsigned long constraintMask;
    unsigned long long ullLowPowerTimeBeforeSleep, ullLowPowerTimeAfterSleep;
    unsigned long long count[3];
    unsigned long long ullSleepTime;
    unsigned long long time;
    unsigned long long remain;
    unsigned long long LPDSlatency, sleepLatency;
    eSleepModeStatus eSleepStatus;

    /*
     *  Enter a critical section that will not effect interrupts
     *  bringing the MCU out of sleep mode.
     */
    vPortEnterCritical();

    /* query the declared constraints */
    constraintMask = Power_getConstraintMask();

    /* check if we are allowed to go to LPDS */
    if ((constraintMask & LPDS_DISALLOWED) == 0) {
        /*
         *  Read the current time from a time source that will remain
         *  operational while the microcontroller is in a low power state.
         */
        /*
         *  Get the current RTC count, using the fast interface; to use the
         *  fast interface the count must be read three times, and then
         *  the value that matches on at least two of the reads is chosen
         */
        for (i = 0; i < 3; i++) {
            count[i] =  PRCMSlowClkCtrFastGet();
        }
        ullLowPowerTimeBeforeSleep =
                COUNT_WITHIN_TRESHOLD(count[0], count[1], count[2], 1);

        /* Stop the timer that is generating the tick interrupt. */
        SysTickTimerP_stop();

        /* Ensure it is still ok to enter the sleep mode. */
        eSleepStatus = eTaskConfirmSleepModeStatus();

        if (eSleepStatus == eAbortSleep ) {
            /*
             *  A task has been moved out of the Blocked state since this
             *  macro was executed, or a context siwth is being held pending.
             *  Do not enter a sleep state.  Restart the tick and exit the
             *  critical section.
             */
            SysTickTimerP_stop();
            vPortExitCritical();

            returnFromSleep = FALSE;
        }
        else {
            /* convert ticks to microseconds */
            time = idleTime * configTICK_RATE_HZ;

            /* check if can go to LPDS */
            LPDSlatency = Power_getTransitionLatency(POWER_LPDS, Power_TOTAL_LATENCY);
            sleepLatency = Power_getTransitionLatency(POWER_SLEEP, Power_TOTAL_LATENCY);
            //TODO: Add disallow interburst idle into LPDS
            if (time > LPDSlatency) {
                remain = ((time - LPDSlatency) * 32768) / 1000000;

                /* set the LPDS wakeup time interval */
                PRCMLPDSIntervalSet(500u);
//                PRCMLPDSIntervalSet(remain);

                /* enable the wake source to be timer */
                PRCMLPDSWakeupSourceEnable(PRCM_LPDS_SLEEP_COUNTER \
                        | PRCM_LPDS_RTC_COUNTER);

                /* disable interrupts during ITM flush and restore */
                CPUcpsid();

                /* Flush any remaining log messages in the ITM */
//                ITM_flush();

                /* now go to LPDS ... */
                Power_sleep(POWER_LPDS);

                /* Restore ITM settings */
//                ITM_restore();

                /* re-enable interrupts */
                CPUcpsie();

                /* set 'returnFromSleep' to TRUE*/
                returnFromSleep = TRUE;
            }
            else if (time > sleepLatency)
            {
                remain = ((time - sleepLatency) * 32768) / 1000000;

                /* set the LPDS wakeup time interval */
//                PRCMLPDSIntervalSet(50u);
                PRCMLPDSIntervalSet(remain);

                /* enable the wake source to be timer */
                PRCMLPDSWakeupSourceEnable(PRCM_LPDS_SLEEP_COUNTER \
                        | PRCM_LPDS_RTC_COUNTER);

                /* disable interrupts during ITM flush and restore */
                CPUcpsid();

                /* Flush any remaining log messages in the ITM */
//                ITM_flush();

                /* now go to LPDS ... */
                Power_sleep(POWER_SLEEP);

                /* Restore ITM settings */
//                ITM_restore();

                /* re-enable interrupts */
                CPUcpsie();

                /* set 'returnFromSleep' to TRUE*/
                returnFromSleep = TRUE;
            }
            else {
                SysTickTimerP_start();
                vPortExitCritical();

                returnFromSleep = FALSE;
            }
        }
    }
    else {
        /* A constraint was set */
        vPortExitCritical();
    }

    if (returnFromSleep) {
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
        for (i = 0; i < 3; i++) {
            count[i] =  PRCMSlowClkCtrFastGet();
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
    else {

        /* disable interrupts during ITM flush and restore */
        CPUcpsid();
        /* Flush any remaining log messages in the ITM */
//        ITM_flush();
        PRCMSleepEnter();
        /* Restore ITM settings */
//        ITM_restore();
        /* re-enable interrupts */
        CPUcpsie();
    }
#endif
#endif
}

/*
 *  ======== Power_initPolicy ========
 */
void Power_initPolicy()
{
}
