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
 *  ======== power_xwrLx4xx.c ========
 */

#include <stdint.h>
#include <stdio.h>
/*
 * By default disable both asserts and log for this module.
 * This must be done before DebugP.h is included.
 */
#ifndef DebugP_ASSERT_ENABLED
#define DebugP_ASSERT_ENABLED 0
#endif
#ifndef DebugP_LOG_ENABLED
#define DebugP_LOG_ENABLED 0
#endif
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/utils/List.h>
#include <drivers/power.h>
#include <drivers/power_xwrLx4xx.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/prcm.h>
#include <drivers/pinmux.h>
#include <drivers/gpio.h>
#include <drivers/soc.h>
#include <board/cpu.h>
#include <drivers/mcspi.h>
#include <kernel/dpl/TimerP.h>

#define SYNCBARRIER() {          \
        __asm(" dsb \n"              \
                " isb \n");            \
}

/* Externs */
extern Power_ConfigV1 Power_config;

/* Module_State */
Power_ModuleState Power_module = {
        { NULL, NULL},  /* list */
        0,              /* constraintsMask */
        Power_ACTIVE,   /* state */
        /* dbRecords */
        {
                PRCM_CAN,
                PRCM_SPI,
                PRCM_QSPI,
                PRCM_TOPSS,
                PRCM_APP_RTI,
                PRCM_APP_WD,
                PRCM_APP_UART0,
                PRCM_APP_UART1,
                PRCM_I2C,
                PRCM_LIN,
                PRCM_ESM,
                PRCM_EDMA,
                PRCM_CRC,
                PRCM_PWM,
                PRCM_GIO,
                PRCM_HWASS
        },
        /* enablePolicy */
        FALSE,
        /* initialized */
        FALSE,
        /* refCount */
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        /* constraintCounts */
        { 0, 0, 0 },
        /* policyFxn */
        NULL
};

/* Low Power Mode that Framework got into */
static Power_SleepState lowPowStateTaken = POWER_NONE;

/* context save variable */
Power_SaveRegisters Power_contextSave;

/* Pin Parking Config: Functional mode 0. Output disabled. Override Input control from hardware and Disable Pull  */
uint32_t pinParkCfg = 0x000001F0;

HwiP_Object   idleHwiObj;
HwiP_Params   idleHwiPrms;
bool idleExited = FALSE;
int power_idleresumehook(uint_fast16_t eventType, uintptr_t eventArg, uintptr_t clientArg);
typedef void (*LPDSFunc)(void);

/* enter LPDS is an assembly function */
extern void Power_enterLPDS(LPDSFunc driverlibFunc);

/* internal functions */
static int_fast16_t notify(uint_fast16_t eventType);
static void restoreNVICRegs(void);
static void restorePeriphClocks(void);
static void saveNVICRegs(void);
static void Power_parkPins(void);
void Power_asmWfi();

/*
*    Wait for CPU Deep Sleep 
*/
void Power_asmWfi()
{
  __asm("    wfi\n");
}
/*
 *  ======== Power_getLowPowModeTaken ========
 *  Get the Low Power State Taken
 */

Power_SleepState Power_getLowPowModeTaken()
{
    return lowPowStateTaken;
}

/*
 *  ======== Power_setLowPowModeTaken ========
 *  Get the Low Power State Taken
 */

void Power_setLowPowModeTaken(Power_SleepState mode)
{
     lowPowStateTaken = mode;
}

/*
 *  ======== Power_disablePolicy ========
 *  Do not run the configured policy
 */
bool Power_disablePolicy(void)
{
    bool enablePolicy = Power_module.enablePolicy;
    Power_module.enablePolicy = FALSE;

    DebugP_log("Power: disable policy");

    return (enablePolicy);
}

/*
 *  ======== Power_enablePolicy ========
 *  Run the configured policy
 */
void Power_enablePolicy(void)
{
    Power_module.enablePolicy = TRUE;

    DebugP_log("Power: enable policy");
}

/*
 *  ======== Power_getConstraintMask ========
 *  Get a bitmask indicating the constraints that have been registered with
 *  Power.
 */
uint_fast32_t Power_getConstraintMask(void)
{
    return (Power_module.constraintMask);
}

/*
 *  ======== Power_getDependencyCount ========
 *  Get the count of dependencies that are currently declared upon a resource.
 */
int_fast16_t Power_getDependencyCount(uint_fast16_t resourceId)
{
    int_fast16_t status;

    if (resourceId >= POWER_NUMRESOURCES) {
        status = Power_EINVALIDINPUT;
    }
    else {
        status = Power_module.refCount[resourceId];
    }

    return (status);
}

/*
 *  ======== Power_getTransitionLatency ========
 *  Get the transition latency for a sleep state.  The latency is reported
 *  in units of microseconds.
 */
uint32_t Power_getTransitionLatency(Power_SleepState sleepState,
        Power_LatencyType type)
{
    uint32_t latency = 0;

    switch(sleepState)
    {
    case POWER_LPDS:

        if (type == Power_RESUME_LATENCY) {
            latency = Power_config.resumeLatencyForLPDS;
        }
        else {
            latency = Power_config.totalLatencyForLPDS;
        }

        break;

    case POWER_SLEEP:

        if (type == Power_RESUME_LATENCY) {
            latency = Power_config.resumeLatencyForSleep;
        }
        else {
            latency = Power_config.totalLatencyForSleep;
        }
        break;
    case POWER_IDLE:

        if (type == Power_RESUME_LATENCY) {
            latency = Power_config.resumeLatencyForIdle;
        }
        else {
            latency = Power_config.totalLatencyForIdle;
        }
        break;
    default:
        break;
    }

    return (latency);
}

/*
 *  ======== Power_getThresholds ========
 *  Get the thresholds for a sleep state.  The time is reported
 *  in units of microseconds.
 */
uint32_t Power_getThresholds(Power_SleepState sleepState)
{
    uint32_t latency = 0;

    switch(sleepState)
    {
        case POWER_LPDS:
            latency = Power_config.LPDSThreshold;
            break;
        case POWER_SLEEP:
            latency = Power_config.sleepThreshold;
            break;
        case POWER_IDLE:
            latency = Power_config.idleThreshold;
            break;
        default:
            break;
    }

    return (latency);
}
/*
 *  ======== Power_setTransitionLatency ========
 *  Get the transition latency for a sleep state.  The latency is reported
 *  in units of microseconds.
 */
void Power_setTransitionLatency(Power_SleepState sleepState,
        Power_LatencyType type, uint32_t latencyTime)
{
    switch(sleepState)
    {
    case POWER_LPDS:

        if (type == Power_RESUME_LATENCY) {
            Power_config.resumeLatencyForLPDS = latencyTime;
        }
        else {
            Power_config.totalLatencyForLPDS = latencyTime;
        }

        break;

    case POWER_SLEEP:

        if (type == Power_RESUME_LATENCY) {
            Power_config.resumeLatencyForSleep = latencyTime;
        }
        else {
            Power_config.totalLatencyForSleep = latencyTime;
        }
        break;

    case POWER_IDLE:

        if (type == Power_RESUME_LATENCY) {
            Power_config.resumeLatencyForIdle = latencyTime;
        }
        else {
            Power_config.totalLatencyForIdle = latencyTime;
        }
        break;

    default:
        break;
    }
}

/*
 *  ======== Power_setThresholds ========
 *  Set the thresholds for different sleep state.  The latency is
 *  in units of microseconds.
 */
void Power_setThresholds(Power_SleepState sleepState, uint32_t latencyTime)
{
    switch(sleepState)
    {
        case POWER_LPDS:
            Power_config.LPDSThreshold = latencyTime;
            break;
        case POWER_SLEEP:
            Power_config.sleepThreshold = latencyTime;
            break;
        case POWER_IDLE:
            Power_config.idleThreshold = latencyTime;
            break;
        default:
            break;
    }
}

/*
 *  ======== Power_getTransitionState ========
 *  Get the current sleep transition state.
 */
uint_fast16_t Power_getTransitionState(void)
{
    return (Power_module.state);
}

/*
 *  ======== Power_idleFunc ========
 *  Function needs to be plugged into the idle loop.
 *  It calls the configured policy function if the
 *  'enablePolicy' flag is set.
 */
void Power_idleFunc(unsigned long long sleepTimeus)
{
    if (Power_module.enablePolicy) {
        if (Power_module.policyFxn != NULL) {
            DebugP_log("Power: calling policy function (%p)",
                    (uintptr_t) Power_module.policyFxn);
            (*(Power_module.policyFxn))(sleepTimeus);
        }
    }
}

/*
 *  ======== Power_init ========
 */
int_fast16_t Power_init()
{
    uint32_t regVal;


    if (Power_module.initialized) 
    {
        /* if this function has already been called, just return */
    }
    else
    {
        /* set module state field 'initialized' to true */
        Power_module.initialized = TRUE;

        /* set the module state enablePolicy field */
        Power_module.enablePolicy = Power_config.enablePolicy;

        /* call the config policy init function if its not null */
        if (Power_config.policyInitFxn != NULL) {
            (*(Power_config.policyInitFxn))();
        }
        
        if(Power_config.enableGPIOSyncIOWakeupLPDS == (bool)TRUE)
        {
            Power_module.wakeupConfig.selectGpioSyncIOLpds = Power_config.selectGpioSyncIOLpds;
            if (POWER_GPIO_WAKEUP_LPDS == Power_config.selectGpioSyncIOLpds)
            {
                Power_module.wakeupConfig.wakeupGPIOEdgeLPDS =
                    Power_config.wakeupGPIOEdgeLPDS;
            }
            else
            {
                Power_module.wakeupConfig.wakeupSyncIOEdgeLPDS =
                    Power_config.wakeupSyncIOEdgeLPDS;
            }
        }
        if(Power_config.enableSPICSWakeupLPDS == (bool)TRUE)
    {
        Power_module.wakeupConfig.wakeupSpiEdgeLPDS =
                Power_config.wakeupSpiEdgeLPDS;
    }
    if(Power_config.enableUARTWakeupLPDS == (bool)TRUE)
    {
        Power_module.wakeupConfig.wakeupUartEdgeLPDS =
                Power_config.wakeupUartEdgeLPDS;
    }
    /* copy wakeup settings to module state */
        Power_module.wakeupConfig.enableSleepCounterWakeupLPDS =
                    Power_config.enableSleepCounterWakeupLPDS;
        Power_module.wakeupConfig.enableGPIOSyncIOWakeupLPDS =
                    Power_config.enableGPIOSyncIOWakeupLPDS;
        Power_module.wakeupConfig.enableUARTWakeupLPDS =
                Power_config.enableUARTWakeupLPDS;
        Power_module.wakeupConfig.enableSPICSWakeupLPDS =
                Power_config.enableSPICSWakeupLPDS;
        Power_module.wakeupConfig.enableRTCWakeupLPDS =
                Power_config.enableRTCWakeupLPDS;
        Power_module.wakeupConfig.enableFRCWakeupLPDS =
                Power_config.enableFRCWakeupLPDS;


        /* now configure these wakeup settings in the device... */
        Power_configureWakeup(&Power_module.wakeupConfig);

        /* copy the Power policy function to module state */
        Power_module.policyFxn = Power_config.policyFxn;

        /* spin if too many pins were specified in the pin park array */
        if (Power_config.numPins > POWER_NUMPINS) {
            uint32_t loop = 1U;
            while(loop != 0U){}
        }

        regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_RELEASE_PAUSE);
        HW_SET_FIELD32(regVal, CSL_TOP_PRCM_RELEASE_PAUSE_RELEASE_PAUSE_RELEASE_PAUSE,0x1);
        HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_RELEASE_PAUSE, regVal);
    }
    return (Power_SOK);
}

/*
 *  ======== Power_registerNotify ========
 *  Register a function to be called on a specific power event.
 */
int_fast16_t Power_registerNotify(Power_NotifyObj * pNotifyObj,
        uint_fast16_t eventTypes, Power_NotifyFxn notifyFxn, uintptr_t clientArg)
{
    int_fast16_t status = Power_SOK;

    /* check for NULL pointers  */
    if ((pNotifyObj == NULL) || (notifyFxn == NULL)) {
        status = Power_EINVALIDPOINTER;
    }

    else {
        /* fill in notify object elements */
        pNotifyObj->eventTypes = eventTypes;
        pNotifyObj->notifyFxn = notifyFxn;
        pNotifyObj->clientArg = clientArg;

        /* place notify object on event notification queue */
        List_put(&Power_module.notifyList, (List_Elem*)pNotifyObj);
    }

    DebugP_log(
            "Power: register notify (%p), eventTypes (0x%x), notifyFxn (%p)",
            (uintptr_t) pNotifyObj, eventTypes, (uintptr_t) notifyFxn);

    return (status);
}

/*
 *  ======== Power_releaseConstraint ========
 *  Release a previously declared constraint.
 */
int_fast16_t Power_releaseConstraint(uint_fast16_t constraintId)
{
    int_fast16_t status = Power_SOK;
    uintptr_t key;
    uint8_t count;

    /* first ensure constraintId is valid */
    if (constraintId >= POWER_NUMCONSTRAINTS) {
        status = Power_EINVALIDINPUT;
    }

    /* if constraintId is OK ... */
    else {

        /* disable interrupts */
        key = HwiP_disable();

        /* get the count of the constraint */
        count = Power_module.constraintCounts[constraintId];

        /* ensure constraint count is not already zero */
        if (count == 0U) {
            status = Power_EFAIL;
        }

        /* if not already zero ... */
        else {
            /* decrement the count */
            count--;

            /* save the updated count */
            Power_module.constraintCounts[constraintId] = count;

            /* if constraint count reaches zero, remove constraint from mask */
            if (count == 0U) {
                Power_module.constraintMask &= ~(1U << constraintId);
            }
        }

        /* restore interrupts */
        HwiP_restore(key);

        DebugP_log("Power: release constraint (%d)", constraintId);
    }

    return (status);
}

/*
 *  ======== Power_releaseDependency ========
 *  Release a previously declared dependency.
 */
int_fast16_t Power_releaseDependency(uint_fast16_t resourceId)
{
    int_fast16_t status = Power_SOK;
    uint8_t count;
    uint32_t id;
    uintptr_t key;

    /* first check that resourceId is valid */
    if (resourceId >= POWER_NUMRESOURCES) {
        status = Power_EINVALIDINPUT;
    }

    /* if resourceId is OK ... */
    else {

        /* disable interrupts */
        key = HwiP_disable();

        /* read the reference count */
        count = Power_module.refCount[resourceId];

        /* ensure dependency count is not already zero */
        if (count == 0U) {
            status = Power_EFAIL;
        }

        /* if not already zero ... */
        else {

            /* decrement the reference count */
            count--;

            /* if this was the last dependency being released.., */
            if (count == 0U) {
                /* deactivate this resource ... */
                id = Power_module.dbRecords[resourceId];

                /* disable clk to peripheral */
                PRCMPeripheralClkDisable(id, PRCM_GATE_CLK_ENABLE);
            }

            /* save the updated count */
            Power_module.refCount[resourceId] = count;
        }

        /* restore interrupts */
        HwiP_restore(key);

        DebugP_log("Power: release dependency (%d)", resourceId);
    }

    return (status);
}

/*
 *  ======== Power_setConstraint ========
 *  Declare an operational constraint.
 */
int_fast16_t Power_setConstraint(uint_fast16_t constraintId)
{
    int_fast16_t status = Power_SOK;
    uintptr_t key;

    /* ensure that constraintId is valid */
    if (constraintId >= POWER_NUMCONSTRAINTS) {
        status = Power_EINVALIDINPUT;
    }

    else {

        /* disable interrupts */
        key = HwiP_disable();

        /* set the specified constraint in the constraintMask */
        Power_module.constraintMask |= 1U << constraintId;

        /* increment the specified constraint count */
        Power_module.constraintCounts[constraintId]++;

        /* restore interrupts */
        HwiP_restore(key);

        DebugP_log("Power: set constraint (%d)", constraintId);
    }

    return (status);
}

/*
 *  ======== Power_setDependency ========
 *  Declare a dependency upon a resource.
 */
int_fast16_t Power_setDependency(uint_fast16_t resourceId)
{
    int_fast16_t status = Power_SOK;
    uint8_t count;
    uint32_t id;
    uintptr_t key;

    /* ensure resourceId is valid */
    if (resourceId >= POWER_NUMRESOURCES) {
        status = Power_EINVALIDINPUT;
    }

    /* resourceId is OK ... */
    else {

        /* disable interrupts */
        key = HwiP_disable();

        /* read and increment reference count */
        count = Power_module.refCount[resourceId];
        Power_module.refCount[resourceId] =  Power_module.refCount[resourceId] + 1U;

        /* if resource was NOT activated previously ... */
        if (count == 0U) {
            /* now activate this resource ... */
            id = Power_module.dbRecords[resourceId];

            /* enable the peripheral clock to the resource */
            PRCMPeripheralClkEnable(id,
                    PRCM_GATE_CLK_ENABLE);

            /* spin here until status returns TRUE */
            while(PRCMPeripheralStatusGet(id) == 0U) {
            }
        }

        /* restore interrupts */
        HwiP_restore(key);
        DebugP_log("Power: set dependency (%d)", resourceId);
    }

    return (status);
}

/*
 *  ======== Power_setPolicy ========
 *  Set the Power policy function
 */
void Power_setPolicy(Power_PolicyFxn policy)
{
    Power_module.policyFxn = policy;
}

static void Power_afterLPDS(void)
{
    /* restore NVIC registers */
    restoreNVICRegs();

    /* restore clock to those peripherals with dependecy set */
    restorePeriphClocks();
    
}


/*
 *  ======== Power_sleep ========
 */
int_fast16_t Power_sleep(uint_fast16_t sleepState)
{
    int_fast16_t status = Power_SOK;
    uint32_t preEvent;
    uint32_t postEvent;
    uint8_t pg_version;


    pg_version = SOC_getEfusePgVersion();

    /* first validate the sleep state */
    if ((sleepState != (uint_fast16_t)POWER_LPDS) && \
            (sleepState != (uint_fast16_t)POWER_SLEEP) && \
            (sleepState != (uint_fast16_t)POWER_IDLE)) 
    {
        status = Power_EINVALIDINPUT;
    }

    else if (Power_module.state == Power_ACTIVE)
    {
        if(sleepState == (uint_fast16_t)POWER_LPDS)
        {
            /* setup sleep vars */
            preEvent = POWER_ENTERING_LPDS;
            postEvent = POWER_AWAKE_LPDS;
            /* set transition state to entering Deep sleep */
            Power_module.state = Power_ENTERING_DEEPSLEEP;
        }
        else if (sleepState == (uint_fast16_t)POWER_SLEEP)
        {
            /* setup sleep vars */
            preEvent = POWER_ENTERING_SLEEP;
            postEvent = POWER_AWAKE_SLEEP;
            /* set transition state to entering Sleep */
            Power_module.state = Power_ENTERING_SLEEP;
        }
        else
        {
            preEvent = POWER_ENTERING_IDLE;
            postEvent = POWER_AWAKE_IDLE;
            /* set transition state to entering Sleep */
            Power_module.state = Power_ENTERING_IDLE;
        }

        /* signal all clients registered for pre-sleep notification */
        status = notify(preEvent);

        /* check for timeout or any other error */
        if (status != Power_SOK) {
            Power_module.state = Power_ACTIVE;
        }
        else
        {
            DebugP_log("Power: State (%d)", sleepState);

            /* invoke specific sequence to activate LPDS ...*/

        if(sleepState == (uint_fast16_t)POWER_LPDS)
        {
            // enable RAM retention
    
           
            PRCMSetSRAMRetention(Power_config.ramRetentionMaskLPDS, PRCM_SRAM_LPDS_RET);
            

                /* Consider functional values for dftrtagood and dftrtaon signals */
                PRCMSetPSCONDFTRTAOverride(false,0,false,0);

                /* call the enter LPDS hook function if configured */
                if (Power_config.enterLPDSHookFxn != NULL) 
                {
                    (*(Power_config.enterLPDSHookFxn))();
                }
                
                /* park pins, based upon board file definitions */
                if (Power_config.pinParkDefs != NULL)
                {
                    Power_parkPins();
                }
                
                if(pg_version == 1U)
                {
                    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PMS_BGAP_DIS_HIBERNATE, 0x1);
                }
                /* save the NVIC registers */
                saveNVICRegs();

                /* call sync barrier */
                SYNCBARRIER();
                
                /* now enter LPDS - function does not return... */
                Power_enterLPDS(PRCMLPDSEnter);
                
            }

            else if(sleepState == (uint_fast16_t)POWER_SLEEP)
            {
                /* save the NVIC registers */
                saveNVICRegs();

                /* call sync barrier */
                SYNCBARRIER();
                
                /* now enter LPDS - function does not return... */
                Power_enterLPDS(PRCMSleepEnter);
            
            }
            else
            {
                Power_Idle3();
            }
            
            /* return here after reset, from Power_resumeLPDS() */

            if(sleepState == (uint_fast16_t)POWER_LPDS)
            {
                //Enable the Debug back after Deep Sleep Mode
                uint32_t regVal;
                regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN);
                HW_SET_FIELD32(regVal,CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN_PSCON_TEST_DBG_PD_EN_OV_TEST_DBG_PD_IS_SLEEP, 0x0);
                HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN, regVal);

                // Initialize the FPU if floating point support is required.
                #ifdef __ARM_FP
                    volatile uint32_t* cpacr = (volatile uint32_t*)0xE000ED88U;
                    *cpacr |= (0xf0U << 16);
                #endif
                
                /* call the resume LPDS hook function if configured */
                if (Power_config.resumeLPDSHookFxn != NULL) 
                {
                    (*(Power_config.resumeLPDSHookFxn))();
                }

                Power_afterLPDS();
                
                /* set transition state to EXITING_DEEPSLEEP */
                Power_module.state = Power_EXITING_DEEPSLEEP;
            }
            
            /* return here after reset, from Power_resumeLPDS() */
            else if(sleepState == (uint_fast16_t)POWER_SLEEP)
            {
                /* restore NVIC registers */
                restoreNVICRegs();

                /* restore clock to those peripherals with dependecy set */
                restorePeriphClocks();

                /* call PRCMMCUInit() for any necessary post-LPDS restore */
                PRCMMCUInit();
                
                /* set transition state to EXITING_DEEPSLEEP */
                Power_module.state = Power_EXITING_SLEEP;
            }
            else
            {
                /*MISRA-C*/
            }
            /*
            * signal clients registered for post-sleep notification; for example,
            * a driver that needs to reinitialize its peripheral state, that was
            * lost during LPDS
            */
            status = notify(postEvent);
            
            /* now clear the transition state before re-enabling scheduler */
            Power_module.state = Power_ACTIVE;
        }
    }
    else 
    {
        status = Power_EBUSY;
    }

    return (status);
}

/*
 *  ======== Power_unregisterNotify ========
 *  Unregister for a power notification.
 *
 */
void Power_unregisterNotify(Power_NotifyObj * pNotifyObj)
{
    uintptr_t key;

    /* disable interrupts */
    key = HwiP_disable();

    /* remove notify object from its event queue */
    List_remove(&Power_module.notifyList, (List_Elem *)pNotifyObj);

    /* re-enable interrupts */
    HwiP_restore(key);

    DebugP_log("Power: unregister notify (%p)", (uintptr_t) pNotifyObj);
}

/*********************** 64XX-specific functions **************************/

/*
 *  ======== Power_configureWakeup ========
 *  Configure LPDS and shutdown wakeups; copy settings into driver state
 */
void Power_configureWakeup(Power_Wakeup *wakeup)
{
    PRCMLPDSWakeupSourceDisable(PRCM_LPDS_ALL_WAKEUPS);

     if(wakeup->enableUARTWakeupLPDS)
     {
         PRCMLPDSWakeupSourceEnable(PRCM_LPDS_UART_RX);
     }
     if(wakeup->enableSPICSWakeupLPDS)
     {
         PRCMLPDSWakeupSourceEnable(PRCM_LPDS_SPI_CHIP_SELECT);
     }
     if(wakeup->enableFRCWakeupLPDS)
     {
         PRCMLPDSWakeupSourceEnable(PRCM_LPDS_FRC_FRAME_START_INT);
     }
     if(wakeup->enableSleepCounterWakeupLPDS)
     {
         PRCMLPDSWakeupSourceEnable(PRCM_LPDS_SLEEP_COUNTER);
     }
     if(wakeup->enableRTCWakeupLPDS)
     {
         PRCMLPDSWakeupSourceEnable(PRCM_LPDS_RTC_COUNTER);
     }
    /* configure GPIO(WU_REQIN) as wakeup source for LPDS */
     if(wakeup->enableGPIOSyncIOWakeupLPDS)
     {
         PRCMLPDSWakeupSourceEnable(PRCM_LPDS_GPIO_SYNC_IO);
        if (POWER_GPIO_WAKEUP_LPDS == wakeup->selectGpioSyncIOLpds) 
        {
            PRCMLPDSWakeUpGPIOSelect(wakeup->wakeupGPIOEdgeLPDS);
            
        }
        else
        {
            PRCMLPDSWakeUpSYNCIOSelect(wakeup->wakeupSyncIOEdgeLPDS);
        }
     }
     if(wakeup->enableSPICSWakeupLPDS)
     {
         PRCMLPDSWakeUpSPISelect(wakeup->wakeupSpiEdgeLPDS);
     }
    if(wakeup->enableUARTWakeupLPDS)
     {
         PRCMLPDSWakeUpUartRxSelect(wakeup->wakeupUartEdgeLPDS);
     }
}


/*
 *  ======== Power_getWakeup ========
 *  Get the current LPDS and shutdown wakeup configuration
 */
void Power_getWakeup(Power_Wakeup *wakeup)
{
    *wakeup = Power_module.wakeupConfig;
}


/*
 *  ======== Power_reset ========
 *  Software reset of specific peripheral.
 */
int_fast16_t Power_reset(uint_fast16_t resourceId)
{
    int_fast16_t status = Power_SOK;
    uint32_t id;

    /* Ensure resourceId is valid */
    if (resourceId >= (uint_fast16_t)POWER_NUMRESOURCES) {
        status = Power_EINVALIDINPUT;
    }

    /* resourceId is OK ... */
    else {

        id = Power_module.dbRecords[resourceId];
        /* Reset the peripheral */
        PRCMPeripheralReset(id);
    }
    return (status);
}

/*************************internal functions ****************************/

/*
 *  ======== notify ========
 *  Note: When this function is called hardware interrupts are disabled
 */
static int_fast16_t notify(uint_fast16_t eventType)
{
    int_fast16_t notifyStatus;
    Power_NotifyFxn notifyFxn;
    uintptr_t clientArg;
    List_Elem *elem;
    int_fast16_t status = Power_SOK;

    /* if queue is empty, return immediately */
    if (!List_empty(&Power_module.notifyList)) {
        /* point to first client notify object */
        elem = List_head(&Power_module.notifyList);

        /* walk the queue and notify each registered client of the event */
        do {
            if ((((Power_NotifyObj *)elem)->eventTypes & eventType)!=0U) {
                /* pull params from notify object */
                notifyFxn = ((Power_NotifyObj *)elem)->notifyFxn;
                clientArg = ((Power_NotifyObj *)elem)->clientArg;

                /* call the client's notification function */
                notifyStatus = (int_fast16_t) (*(Power_NotifyFxn)notifyFxn)(
                        eventType, 0, clientArg);

                /* if client declared error stop all further notifications */
                if (notifyStatus != Power_NOTIFYDONE) {
                    status = Power_EFAIL;
                    break;
                }
            }

            /* get next element in the notification queue */
            elem = List_next(elem);

        } while (elem != NULL);
    }

    return status;
}

/*
 *  ======== restoreNVICRegs ========
 *  Restore the NVIC registers
 */
static void restoreNVICRegs(void)
{
    uint32_t i;
    uint32_t *base_reg_addr;

    /* Restore the NVIC control registers */
    HW_WR_REG32(NVIC_VTABLE, Power_contextSave.nvicRegs.vectorTable);
    HW_WR_REG32(NVIC_ACTLR, Power_contextSave.nvicRegs.auxCtrl);
    HW_WR_REG32(NVIC_APINT, Power_contextSave.nvicRegs.appInt);
    HW_WR_REG32(NVIC_INT_CTRL, Power_contextSave.nvicRegs.intCtrlState);
    HW_WR_REG32(NVIC_SYS_CTRL, Power_contextSave.nvicRegs.sysCtrl);
    HW_WR_REG32(NVIC_CFG_CTRL, Power_contextSave.nvicRegs.configCtrl);
    HW_WR_REG32(NVIC_SYS_PRI1, Power_contextSave.nvicRegs.sysPri1);
    HW_WR_REG32(NVIC_SYS_PRI2, Power_contextSave.nvicRegs.sysPri2);
    HW_WR_REG32(NVIC_SYS_PRI3, Power_contextSave.nvicRegs.sysPri3);
    HW_WR_REG32(NVIC_SYS_HND_CTRL, Power_contextSave.nvicRegs.sysHcrs);

    /* Systick registers */
    HW_WR_REG32(NVIC_ST_CTRL, Power_contextSave.nvicRegs.systickCtrl);
    HW_WR_REG32(NVIC_ST_RELOAD, Power_contextSave.nvicRegs.systickReload);
    HW_WR_REG32(NVIC_ST_CAL, Power_contextSave.nvicRegs.systickCalib);

    /* Restore the interrupt priority registers */
    base_reg_addr = (uint32_t *)NVIC_PRI0;
    for(i = 0; i < POWER_numNVICIntPriority; i++) {
        base_reg_addr[i] = Power_contextSave.nvicRegs.intPriority[i];
    }

    /* Restore the interrupt enable registers */
    base_reg_addr = (uint32_t *)NVIC_EN0;
    for(i = 0; i < POWER_numNVICSetEnableRegs; i++) {
        base_reg_addr[i] = Power_contextSave.nvicRegs.intSetEn[i];
    }

    /* Data and instruction sync barriers */
    SYNCBARRIER();
}

/*
 *  ======== restorePeriphClocks ========
 *  Restores the peripheral clocks that had dependency set
 */
static void restorePeriphClocks(void)
{
    uint32_t dependCount;
    uint32_t i;

    /* need to re-enable peripheral clocks to those with set dependency */
    for (i = 0; i < POWER_NUMRESOURCES; i++) {
        dependCount = Power_getDependencyCount(i);
        if (dependCount > 0U) {
            PRCMPeripheralClkEnable(Power_module.dbRecords[i],
                    PRCM_GATE_CLK_ENABLE);

            while(PRCMPeripheralStatusGet(Power_module.dbRecords[i]) == 0U) {
            }
        }
    }
}

/*
 *  ======== saveNVICRegs ========
 *  Save away the NVIC registers for LPDS mode.
 */
static void saveNVICRegs(void)
{
    uint32_t i;
    uint32_t *base_reg_addr;

    /* Save the NVIC control registers */
    Power_contextSave.nvicRegs.vectorTable = HW_RD_REG32(NVIC_VTABLE);
    Power_contextSave.nvicRegs.auxCtrl = HW_RD_REG32(NVIC_ACTLR);
    Power_contextSave.nvicRegs.intCtrlState = HW_RD_REG32(NVIC_INT_CTRL);
    Power_contextSave.nvicRegs.appInt = HW_RD_REG32(NVIC_APINT);
    Power_contextSave.nvicRegs.sysCtrl = HW_RD_REG32(NVIC_SYS_CTRL);
    Power_contextSave.nvicRegs.configCtrl = HW_RD_REG32(NVIC_CFG_CTRL);
    Power_contextSave.nvicRegs.sysPri1 = HW_RD_REG32(NVIC_SYS_PRI1);
    Power_contextSave.nvicRegs.sysPri2 = HW_RD_REG32(NVIC_SYS_PRI2);
    Power_contextSave.nvicRegs.sysPri3 = HW_RD_REG32(NVIC_SYS_PRI3);
    Power_contextSave.nvicRegs.sysHcrs = HW_RD_REG32(NVIC_SYS_HND_CTRL);

    /* Systick registers */
    Power_contextSave.nvicRegs.systickCtrl = HW_RD_REG32(NVIC_ST_CTRL);
    Power_contextSave.nvicRegs.systickReload = HW_RD_REG32(NVIC_ST_RELOAD);
    Power_contextSave.nvicRegs.systickCalib = HW_RD_REG32(NVIC_ST_CAL);

    /* Save the interrupt enable registers */
    base_reg_addr = (uint32_t *)NVIC_EN0;
    for (i = 0; i < POWER_numNVICSetEnableRegs; i++) {
        Power_contextSave.nvicRegs.intSetEn[i] = base_reg_addr[i];
    }

    /* Save the interrupt priority registers */
    base_reg_addr = (uint32_t *)NVIC_PRI0;
    for (i = 0; i < POWER_numNVICIntPriority; i++) {
        Power_contextSave.nvicRegs.intPriority[i] = base_reg_addr[i];
    }
}

/*
 *  ======== parkPins ========
 */
static void Power_parkPins(void)
{
    Power_ParkInfo parkInfo;
    uint32_t i;
    Pinmux_PerCfg_t pinCfg[2];
    pinCfg[1].offset = PINMUX_END;
    pinCfg[1].settings = PINMUX_END;

    DebugP_assert(Power_config.numPins < POWER_NUMPINS + 1);

    /* for each pin in the array ... */
    for (i = 0; i < Power_config.numPins; i++) {

        parkInfo = Power_config.pinParkDefs[i];

        /* skip this pin if "don't park" is specified */
        if (parkInfo.parkState == (uint32_t)POWER_DONT_PARK)
        {
            continue;
        }

        /* else, for all other pins */
        else
        {
            pinCfg[0].offset = parkInfo.pin;
            pinCfg[0].settings = pinParkCfg;
            Pinmux_config(pinCfg,PINMUX_DOMAIN_ID_MAIN);
        }
    }
}


// Wakeup interrupt handler
void Power_idleWicIsr(void *args)
{
    /* Clear the WIC Status register */
    HW_WR_REG32((CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_WIC_STAT_CLR), 0xFFFFFFFFU);
    idleExited =  TRUE;
}

/* Device Idle mode, IDLE 3 */
void Power_Idle3()
{
    uint32_t regVal;
    uint32_t loop = 1U;
    bool dPllLock = 0;
    uint32_t appClkCfg = 0, topssClkCfg = 0, efuseMMRClkStop = 0, psconDftRtaOvr = 0;
    
    // Check if Radar state is Idle
    if(PRCMIsRdrStateIdle() == TRUE)
    {
        // Consider functional values for dftrtagood and dftrtaon signals
        psconDftRtaOvr = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_DFTRTA_OVERRIDE);
        PRCMSetPSCONDFTRTAOverride(false,0,false,0);
        
        // Setup RTI Interrupt as Wakeup source from Idle by default.
        // Unmask all the interrupt sources for Wakeup from CPU deep Sleep.
        HW_WR_REG32((CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_WIC_CTRL), 0x0);

        // Clear the WIC Status register
        HW_WR_REG32((CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_WIC_STAT_CLR), 0xFFFFFFFFU);

        // Configure WIC interrupt for wakeup.
        HwiP_Params_init(&idleHwiPrms);
        idleHwiPrms.intNum = (CSL_APPSS_INTR_WIC_IRQ + 16U);
        idleHwiPrms.callback = Power_idleWicIsr;
        idleHwiPrms.isPulse = 0;
        idleHwiPrms.priority = 0;
        HwiP_construct(&idleHwiObj, &idleHwiPrms);
        
        // Read the APP Core Clock configuration
        appClkCfg = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_CPU_CLKCTL);

        // Select OSC for CPU  with Div 1
        HW_WR_REG32((CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_CPU_CLKCTL), 0x0);

        /* call the enter idle3 hook function if configured */
        if (Power_config.enteridle3HookFxn != NULL) 
        {
            (*(Power_config.enteridle3HookFxn))();
        }

        // Disable PLL DIG
        CSL_REG32_FINS((CSL_PLLDIG_CTRL_U_BASE + CSL_PLLDIG_CTRL_PLLDIG_EN), PLLDIG_CTRL_PLLDIG_EN_PLLDIG_EN_CFG_PLLDIG_EN, 0x000);

        // Read the TOPSS Clock Configuration
        topssClkCfg = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_TOPSS_CLKCTL);
        // Select OSC for TOPSS with Div 16
        HW_WR_REG32((CSL_APP_RCM_U_BASE + CSL_APP_RCM_TOPSS_CLKCTL), 0x0FFF0000);

        // Request CPU LPDS Entry
        regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_POWERMODE);
        HW_SET_FIELD32(regVal,CSL_APP_RCM_POWERMODE_POWERMODE_DEEPSLEEP, 0x0);
        HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_POWERMODE, regVal);

        regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_POWERMODE);
        HW_SET_FIELD32(regVal,CSL_APP_RCM_POWERMODE_POWERMODE_SLEEP, 0x0);
        HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_POWERMODE, regVal);

        // Disable Test DBG PD in sleep state
        regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN);
        HW_SET_FIELD32(regVal,CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN_PSCON_TEST_DBG_PD_EN_SEL_OV_TEST_DBG_PD_IS_SLEEP, 0x1);
        HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN, regVal);

        regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN);
        HW_SET_FIELD32(regVal,CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN_PSCON_TEST_DBG_PD_EN_OV_TEST_DBG_PD_IS_SLEEP, 0x1);
        HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN, regVal);
        
        // Read the CFG_MMR_CLKSTOP_OVERRIDE register
        efuseMMRClkStop = HW_RD_REG32(CSL_TOP_EFUSE_U_BASE + CSL_TOP_EFUSE_CFG_MMR_CLKSTOP_OVERRIDE);
        // Clock to all MMR registers are allowed only during VBUSP transaction
        HW_WR_REG32((CSL_TOP_EFUSE_U_BASE + CSL_TOP_EFUSE_CFG_MMR_CLKSTOP_OVERRIDE), 0x0);

        //Start the RTI timer for Wake-up
        TimerP_start(CSL_APP_RTIA_U_BASE);

        // Drive CPU clock by OSC/2
        HW_WR_REG32((CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_CPU_CLKCTL), 0x01110000);

        // ARM system control register for CPU deep Sleep
        regVal = HW_RD_REG32(CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_ADDR);
        HW_SET_FIELD32(regVal,CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_SLEEPDEEP, 0x1);
        HW_WR_REG32(CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_ADDR, regVal);

        // Wait for CPU Deep Sleep 
        Power_asmWfi();

        if(idleExited == (bool)TRUE)
        {
            idleExited = FALSE;

            // Stop the RTI timer
            TimerP_stop(CSL_APP_RTIA_U_BASE);
            
            // Re-enabling the Test Domain
            regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN);
            HW_SET_FIELD32(regVal,CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN_PSCON_TEST_DBG_PD_EN_OV_TEST_DBG_PD_IS_SLEEP, 0x0);
            HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN, regVal);
            
            // Restore the CFG_MMR_CLKSTOP_OVERRIDE register
            HW_WR_REG32((CSL_TOP_EFUSE_U_BASE + CSL_TOP_EFUSE_CFG_MMR_CLKSTOP_OVERRIDE),efuseMMRClkStop);
            HW_WR_REG32((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_DFTRTA_OVERRIDE), psconDftRtaOvr);

            // Enable PLL DIG
            CSL_REG32_FINS((CSL_PLLDIG_CTRL_U_BASE + CSL_PLLDIG_CTRL_PLLDIG_EN), PLLDIG_CTRL_PLLDIG_EN_PLLDIG_EN_CFG_PLLDIG_EN, 0x777);
            // Enable the PLL DIG Lock Monitor
            CSL_REG32_FINS((CSL_PLLDIG_CTRL_U_BASE + CSL_PLLDIG_CTRL_PLLDIG_EN), PLLDIG_CTRL_PLLDIG_EN_PLLDIG_EN_CFG_PLLDIG_LOCKMON_ENABLE, 0x777);
            // Wait for DIG PLL lock
            while (dPllLock == (bool)0U)
            {
                dPllLock = ((HW_RD_REG32(CSL_PLLDIG_CTRL_U_BASE + CSL_PLLDIG_CTRL_PLLDIG_STATUS) & \
                            CSL_PLLDIG_CTRL_PLLDIG_STATUS_PLLDIG_STATUS_PLLDIG_LOCKMON_MASK) >> \
                             CSL_PLLDIG_CTRL_PLLDIG_STATUS_PLLDIG_STATUS_PLLDIG_LOCKMON_SHIFT);
            }
            // Restore the APP core clock
            HW_WR_REG32((CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_CPU_CLKCTL), appClkCfg);
            
            // Restore the TOPSS clock
            HW_WR_REG32((CSL_APP_RCM_U_BASE + CSL_APP_RCM_TOPSS_CLKCTL), topssClkCfg);
            
             /* call the idle3 exit hook function if configured */
            if (Power_config.resumeidle3HookFxn != NULL) 
            {
                (*(Power_config.resumeidle3HookFxn))();
            }

        }
        else
        {
            //Illegal exit from Idle mode
            while(loop != 0U){;}
        }
    }
}

