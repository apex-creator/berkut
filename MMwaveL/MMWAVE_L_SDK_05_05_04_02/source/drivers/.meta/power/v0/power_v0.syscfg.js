
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/power/soc/power_${common.getSocName()}`);

let power_module_name = "/drivers/power/power";


let power_module = {
    displayName: "POWER",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/power/templates/power_v0_config.c.xdt",
            driver_init: "/drivers/power/templates/power_v0_init.c.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: power_module_name,
        },
    },
    maxInstances: soc.getConfigArr().length,
    defaultInstanceName: "CONFIG_POWER",
    config: [
        {
            name        : "enablePolicy",
            displayName : "Enable Policy",
            description : "Enable the power policy to run when the CPU is idle.",
            longDescription:`
    If enabled, the policy function will be invoked once for each pass of the
    idle loop.

    In addition to this static setting, the Power Policy can be dynamically
    enabled and disabled at runtime, via the [Power_enablePolicy()][1] and
    [Power_disablePolicy()][2] functions, respectively.

    [1]: /drivers/doxygen/html/_power_8h.html#ti_drivers_Power_Examples_enable
    "Enabling power policy"
    [2]: /drivers/doxygen/html/_power_8h.html#ti_drivers_Power_Examples_disable
    "Disabling power policy"
    `,
            onChange    : onChangeEnablePolicy,
            default     : false
        },
        {
            name        : "policyFunction",
            displayName : "Policy Function",
            description : "Power policy function called from the idle loop.",
            longDescription:`
    When enabled, this function is invoked in the idle loop, to opportunistically
    select and activate sleep states.

    One reference policy is provided:

    * __Power_sleepPolicy()__

    In addition to this static selection, the Power Policy can be
    dynamically changed at runtime, via the Power_setPolicy() API.
    `,
            onChange    : onChangePolicyFxn,
            default     : "Power_sleepPolicy",
            options     :
            [
                {
                    name: "Power_sleepPolicy",
                    description: "A reference power policy is provided which can"
                        + " transition the MCU from the active state to one of two"
                        + " sleep states: Low-Power Deep Sleep (LPDS) or Sleep.",
                    longDescription:`The policy looks at the estimated idle time
    remaining, and the active constraints, and determine which sleep state to
    transition to. The policy will give first preference to choosing LPDS, but if
    that is not appropriate (e.g., not enough idle time), it will choose Sleep.`
                },
                {
                    name: "Custom",
                    description: "Custom policies can be written, and specified"
                        + " via the Policy Custom Function configuration."
                }
            ]
        },
        {
            name        : "policyCustomFunction",
            displayName : "Policy Custom Function",
            description : "User provided Power policy function. Usage not typical.",
            default     : "customPolicyFxn",
            hidden      : true
        },
        {
            name        : "policyInitFunction",
            displayName : "Policy Init Function",
            description : "The initialization function for the Power policy.",
            longDescription:`
    This is an optional policy initialization function that will be called during
    application startup, during Power Manager initialization.
    `,
            default     : "Power_initPolicy",
            onChange    : onChangePolicyInitFxn,
            options     :
            [
                { name: "Power_initPolicy" },
                {
                    name: "Custom",
                    description: "Custom initialization policies can be written"
                        + "and specified via the Policy Init Custom Function"
                        + " configuration."
                }
            ]
        },
        {
            name        : "policyInitCustomFunction",
            displayName : "Policy Init Custom Function",
            description : "User provided Power policy initialization function. " +
                          "Usage not typical.",
            default     : "customPolicyInitFxn",
            hidden      : true
        },
        {
            name        : "enterLPDSHookFunction",
            displayName : "Enter LPDS Hook Function",
            description : "Optional hook function to call on entry into Low-Power"
                +" Deep Sleep (LPDS)",
            longDescription:`This function is called after any notifications are
    complete, and before any pins are parked, just before entry to LPDS.`,
            placeholder : "Enter a function name to enable",
            default     : "NULL"
        },
        {
            name        : "resumeLPDSHookFunction",
            displayName : "Resume LPDS Hook Function",
            description : "Optional hook function to call on resuming from"
                + " Low-Power Deep Sleep (LPDS)",
            longDescription:`This function is called early in the wake sequence,
    before any notification functions are run.`,
            placeholder : "Enter a function name to enable",
            default     : "NULL"
        },
        {
            name        : "enteridle3HookFunction",
            displayName : "Enter idle3 Hook Function",
            description : "Hook function to call on entry into"
                + " Idle3",
            longDescription:`This function is called after any notifications registerd are
    complete,just before entry to idle3. 
    User is expected to implement this function according to their power requirements like
    Clock Gating peripherals etc.`,
            placeholder : "Enter a function name to enable",
            default     : "NULL"
        },
        {
            name        : "resumeidle3HookFunction",
            displayName : "Resume idle3 Hook Function",
            description : "Hook function to call on resuming from"
                +" Idle3",
            longDescription:`This function is called early in the idle3 wake-up sequence,
    before any notification functions are run.
    User is expected to implement this function according to their use-case like
    ungating peripherals etc.`,
            placeholder : "Enter a function name to enable",
            default     : "NULL"
        },
        {
            name        : "enableSleepCounterWakeupLPDS",
            displayName : "Enable Sleep Counter Wakeup LPDS",
            description : "Enable Sleep Counter as a wakeup source for"
                + " Low-Power Deep Sleep (LPDS)",
            default     : false
        },
        {
            name        : "enableUartWakeupLPDS",
            displayName : "Enable UART Wakeup LPDS",
            description : "Enable UART as a wakeup source for"
                + " Low-Power Deep Sleep (LPDS)",
            default     : false,
            onChange: function (inst, ui) {
                let substate = (inst.enableUartWakeupLPDS == true)
                ui.wakeupUartEdgeLPDS.hidden = !substate;
                if(inst.enableUartWakeupLPDS == false)
                {
                    ui.wakeupUartEdgeLPDS.hidden = true;
                }
            },
        },
        {
            name        : "wakeupUartEdgeLPDS",
            displayName : "Wakeup UART RX edge for  LPDS",
            description : "UART RX Trigger Type for wakeup from LPDS",
            hidden      : true,
            default     : "FALL_EDGE",
            options     :
            [
                {
                    name: "FALL_EDGE",
                    description: "Wake up is active on a falling edge"
                },
                {
                    name: "RISE_EDGE",
                    description: "Wake up is active on a rising edge"
                }
            ]
        },
        {
            name        : "enableGPIOSYNCINWakeupLPDS",
            displayName : "Enable GPIO/SYNC_IN Wakeup LPDS",
            description : "Enable GPIO/SYNC_IN as a wakeup source for"
                + " Low-Power Deep Sleep (LPDS)",
            default     : false,
            onChange: function (inst, ui) {
                let substate = (inst.enableGPIOSYNCINWakeupLPDS == true)
                ui.selectGpioSyncioLPDS.hidden = !substate;
                if(inst.enableGPIOSYNCINWakeupLPDS == false)
                {
                    ui.wakeupGPIOEdgeLPDS.hidden = true;
                    ui.wakeupSYNCIOEdgeLPDS.hidden = true;
                }
            },
        },
        {
            name: "selectGpioSyncioLPDS",
            displayName: "Select GPIO or SYNC_IO LPDS",
            default: "POWER_GPIO_WAKEUP_NONE",
            hidden      : true,
            options: [
			    {
                    name: "POWER_GPIO_WAKEUP_NONE",
                    displayName: "NONE"
                },
                {
                    name: "POWER_GPIO_WAKEUP_LPDS",
                    displayName: "GPIO"
                },
                {
                    name: "POWER_SYNCIN_IO_WAKEUP_LPDS",
                    displayName: "SYNCIN_IO"
                }
            ],
            onChange: function (inst, ui) {
				if(inst.selectGpioSyncioLPDS != "POWER_GPIO_WAKEUP_NONE")
				{
                    let substate = (inst.selectGpioSyncioLPDS == "POWER_SYNCIN_IO_WAKEUP_LPDS")
                    ui.wakeupGPIOEdgeLPDS.hidden = substate;
                    ui.wakeupSYNCIOEdgeLPDS.hidden = !substate;
				}
				else
				{
                    ui.wakeupGPIOEdgeLPDS.hidden = true;
                    ui.wakeupSYNCIOEdgeLPDS.hidden = true;
                }
            },
        },
        {
            name        : "wakeupGPIOEdgeLPDS",
            displayName : "Wakeup GPIO Edge Type for LPDS",
            description : "GPIO Trigger Type for wakeup from LPDS",
            hidden      : true,
            default     : "FALL_EDGE",
            options     :
            [
                {
                    name: "FALL_EDGE",
                    description: "Wake up is active on a falling edge"
                },
                {
                    name: "RISE_EDGE",
                    description: "Wake up is active on a rising edge"
                }
            ]
        },
        {
            name        : "wakeupSYNCIOEdgeLPDS",
            displayName : "Wakeup SYNC_IO edge for  LPDS",
            description : "SYNC_IO Trigger Type for wakeup from LPDS",
            hidden      : true,
            default     : "FALL_EDGE",
            options     :
            [
                {
                    name: "FALL_EDGE",
                    description: "Wake up is active on a falling edge"
                },
                {
                    name: "RISE_EDGE",
                    description: "Wake up is active on a rising edge"
                }
            ]
        },
        {
            name        : "enableSpicsWakeupLPDS",
            displayName : "Enable SPICS Wakeup LPDS",
            description : "Enable SPICS as a wakeup source for"
                + " Low-Power Deep Sleep (LPDS)",
            default     : false,
            default     : false,
            onChange: function (inst, ui) {
                let substate = (inst.enableSpicsWakeupLPDS == true)
                ui.wakeupSpiEdgeLPDS.hidden = !substate;
                if(inst.enableSpicsWakeupLPDS == false)
                {
                    ui.wakeupSpiEdgeLPDS.hidden = true;
                }
            },
        },
        {
            name        : "wakeupSpiEdgeLPDS",
            displayName : "Wakeup SPI CS edge for  LPDS",
            description : "SPI CS Trigger Type for wakeup from LPDS",
            hidden      : true,
            default     : "FALL_EDGE",
            options     :
            [
                {
                    name: "FALL_EDGE",
                    description: "Wake up is active on a falling edge"
                },
                {
                    name: "RISE_EDGE",
                    description: "Wake up is active on a rising edge"
                }
            ]
        },
        {
            name        : "enableRtcWakeupLPDS",
            displayName : "Enable RTC Wakeup LPDS",
            description : "Enable RTC as a wakeup source for"
                + " Low-Power Deep Sleep (LPDS)",
            default     : false
        },
        {
            name        : "enableFrcWakeupLPDS",
            displayName : "Enable FRC Wakeup LPDS",
            description : "Enable FRC as a wakeup source for"
                + " Low-Power Deep Sleep (LPDS)",
            default     : false
        },
        {
            name         : "ramRetentionMaskLPDS",
            displayName  : "RAM Retention Mask LPDS",
            description  : "SRAM retention mask for Low-Power Deep Sleep (LPDS)."
                           + " These configs are set only after Power Framework is invoked.",
            default      : ["APP_PD_SRAM_CLUSTER_1","APP_PD_SRAM_CLUSTER_2","APP_PD_SRAM_CLUSTER_3",
                            "FEC_PD_SRAM_CLUSTER_1","FEC_PD_SRAM_CLUSTER_2","FEC_PD_SRAM_CLUSTER_3",
                            "HWA_PD_SRAM_CLUSTER_1","HWA_PD_SRAM_CLUSTER_2","HWA_PD_SRAM_CLUSTER_3"],
            minSelections: 0,
            options      :
            [
                { name: "APP_PD_SRAM_CLUSTER_1" },
                { name: "APP_PD_SRAM_CLUSTER_2" },
                { name: "APP_PD_SRAM_CLUSTER_3" },
                { name: "APP_PD_SRAM_CLUSTER_4" },
                { name: "APP_PD_SRAM_CLUSTER_5" },
                { name: "APP_PD_SRAM_CLUSTER_6" },
                { name: "FEC_PD_SRAM_CLUSTER_1" },
                { name: "FEC_PD_SRAM_CLUSTER_2" },
                { name: "FEC_PD_SRAM_CLUSTER_3" },
                { name: "HWA_PD_SRAM_CLUSTER_1" },
                { name: "HWA_PD_SRAM_CLUSTER_2" },
                { name: "HWA_PD_SRAM_CLUSTER_3" },
            ]
        },
        {
            name        : "thresholdForLPDS",
            displayName : "Threshold for LPDS",
            description : "The minimum time required to enter LPDS"
              + " (in units of microseconds).",
            longDescription:`
    The idle time available only after which entry to LPDS is considered.
    `,
            default     : 10000
        },
        {
            name        : "totalLatencyForLPDS",
            displayName : "Total Latency for LPDS",
            description : "The latency to reserve for entry to and exit from LPDS"
              + " (in units of microseconds).",
            longDescription:`
    There is a latency for the device to transtion into the LPDS sleep state, and to
    wake from LPDS to resume activity. The actual transtion latency will depend upon
    overall device state , as well as execution of notification functions that are 
    registered with the Power driver. This latency value can be changed (tuned) to meet specific
     application requirements.
    `,
            default     : 2000
        },
        {
            name        : "thresholdForSleep",
            displayName : "Threshold for Sleep",
            description : "The minimum time required to enter Sleep State"
              + " (in units of microseconds).",
            longDescription:`
    The idle time available only after which entry to Sleep State is considered.
    `,
            default     : 0xFFFFFFFF
        },
        {
            name        : "totalLatencyForSleep",
            displayName : "Total Latency for Sleep",
            description : "The latency to reserve for entry to and exit from LPDS"
              + " (in units of microseconds).",
            longDescription:`
    There is a latency for the device to transtion into the sleep state, and to
    wake from sleep to resume activity. The actual transtion latency will depend upon
    overall device state , as well as execution of notification functions that are 
    registered with the Power driver. This latency value can be changed (tuned) to meet specific
     application requirements.
    `,
            default     : 1500
        },
        {
            name        : "thresholdForIdle",
            displayName : "Threshold for Idle State",
            description : "The minimum time required to enter Idle State"
              + " (in units of microseconds).",
            longDescription:`
    The idle time available only after which entry to Idle State is considered.
    `,
            default     : 2000
        },
        {
            name        : "totalLatencyForIdle",
            displayName : "Total Latency for IDLE",
            description : "The latency to reserve for entry to and exit from LPDS"
              + " (in units of microseconds).",
            longDescription:`
    There is a latency for the device to transtion into the idle state, and to
    wake from idle state to resume activity. The actual transtion latency will depend upon
    overall device state , as well as execution of notification functions that are 
    registered with the Power driver. This latency value can be changed (tuned) to meet specific
     application requirements.
    `,
            default     : 1000
        },
    ],
    moduleInstances: addModuleInstances,
    validate: validate,
    moduleStatic: {
        modules: function(instance) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceConfig,
};

function onChangeEnablePolicy(inst, ui)
{
    onChangePolicyInitFxn(inst,ui);
    onChangePolicyFxn(inst,ui);
}

function onChangePolicyInitFxn(inst, ui)
{
    let subState = (inst.policyInitFunction !== 'Custom');
    ui.policyInitCustomFunction.hidden = subState;
}

/*
 *  ======== onChangePolicyFxn ========
 *  onChange callback function for the policyFunction config
 */
function onChangePolicyFxn(inst, ui)
{
    let subState = inst.policyFunction !== 'Custom';

    ui.policyCustomFunction.hidden = subState;
}

/*
 *  ======== moduleInstances ========
 *  Return a array of dependent modules for Power
 */
function moduleInstances(inst)
{
    return ([{
        name : 'parkPins',
        displayName: "Park Pins",
        description: "Pin Park States",
        moduleName : 'drivers/power/v0/power_v0_pins',
        collapsed : true
    }]);
}

function addModuleInstances(instance) {
    let modInstances = new Array();
    
    if(instance.selectGpioSyncioLPDS == "POWER_GPIO_WAKEUP_LPDS") {
        modInstances.push({
        name: "gpio",
        displayName: "GPIO Configuration",
        moduleName: "/drivers/power/v0/power_gpio",
        });
    }
    else if(instance.selectGpioSyncioLPDS == "POWER_SYNCIN_IO_WAKEUP_LPDS")
    {
        modInstances.push({
            name: "syncio",
            displayName: "Sync IO Configuration",
            moduleName: "/drivers/power/v0/power_syncin",
            collapsed : false,
            });

    }
    modInstances.push({
        name : 'parkPins',
        displayName: "Park Pins",
        description: "Pin Park States",
        moduleName : 'drivers/power/v0/power_v0_pins',
        collapsed : true});

    return modInstances;
}
/*
 *  ======== getClockFrequencies ========
 *  Return the value of the xwrLx4xx main CPU clock frequency
 */
/*
function getClockFrequencies(clock)
{
    return [ 40000000 ];
}
*/

function validate(inst, report)
{
    if (inst.enablePolicy) {
        if (inst.policyInitFunction === 'Custom') {
            common.validate.checkValidCName(inst, report, "policyInitFunction");
        }
        if (inst.policyFunction === 'Custom') {
            common.validate.checkValidCName(inst, report, "policyFunction");
        }
    }

    common.validate.checkValidCName(inst, report, "enterLPDSHookFunction");
    common.validate.checkValidCName(inst, report, "resumeLPDSHookFunction");
    common.validate.checkValidCName(inst, report, "enteridle3HookFunction");
    common.validate.checkValidCName(inst, report, "resumeidle3HookFunction");
}



function getConfigArr() {
    return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};

function getInterfaceName(inst) {
    return "power";
}

exports = power_module;
