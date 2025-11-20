
let common = system.getScript("/common");
let hwi = system.getScript("/kernel/dpl/hwi.js");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/mcspi/soc/mcspi_${common.getSocName()}`);

function getConfigArr() {
    return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let configArr = getConfigArr();
    let config = configArr.find(o => o.name === solution.peripheralName);

    return {
        ...config,
        ...moduleInstance,
    };
}


function getMaxChannels(inst) {
    if((inst.mode == "SINGLE_CONTROLLER") || (inst.mode == "PERIPHERAL")) {
        return 1;
    }
    else {
        return soc.getMaxChannels(inst);
    }
}

function pinmuxRequirements(inst) {

    let interfaceName = getInterfaceName(inst);

    let misoRequired = false;
    let mosiRequired = false;
    let txRequired = true;
    let rxRequired = true;

    switch (inst.trMode) {
        case "TX_RX":
            misoRequired = true;
            mosiRequired = true;
            break;
        case "TX_ONLY":
            if(inst.mode == "PERIPHERAL") {
                misoRequired = true;
                mosiRequired = false;
            }
            else {
                misoRequired = false;
                mosiRequired = true;
            }
            rxRequired = false;
            break;
        case "RX_ONLY":
            if(inst.mode == "PERIPHERAL") {
                misoRequired = false;
                mosiRequired = true;
            }
            else {
                misoRequired = true;
                mosiRequired = false;
            }
            txRequired = false;
            break;
    }

    let resources = [];
    resources.push( pinmux.getPinRequirements(interfaceName, "CLK", "SPI Clock Pin") );
    resources.push( pinmux.getPinRequirements(interfaceName, "MOSI", "SPI MOSI Pin") );
    resources.push( pinmux.getPinRequirements(interfaceName, "MISO", "SPI MISO Pin") );

    let spi = {
        name: interfaceName,
        displayName: "SPI Instance",
        interfaceName: interfaceName,
        resources: resources,
    };

    return [spi];
}

function getInterfaceName(inst) {

    return "MCSPI";
}

function getPeripheralPinNames(inst) {

    return [ "CLK", "MOSI", "MISO", "CS0", "CS1"];
}

function getClockEnableIds(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockIds;
}

let mcspi_module_name = "/drivers/mcspi/mcspi";

let mcspi_module = {
    displayName: "MCSPI",

    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/mcspi/templates/mcspi_v0_config.c.xdt",
            driver_init: "/drivers/mcspi/templates/mcspi_init.c.xdt",
            driver_deinit: "/drivers/mcspi/templates/mcspi_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/mcspi/templates/mcspi.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/mcspi/templates/mcspi_v0_open_close_config.c.xdt",
            driver_open: "/drivers/mcspi/templates/mcspi_open.c.xdt",
            driver_close: "/drivers/mcspi/templates/mcspi_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/mcspi/templates/mcspi_v0_open_close.h.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: mcspi_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: mcspi_module_name,
        },
    },
    defaultInstanceName: "CONFIG_MCSPI",
    config: getConfigurables(),
    validate: validate,
    moduleInstances: moduleInstances,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    sharedModuleInstances: addModuleInstances,
    getInstanceConfig,
    pinmuxRequirements,
    getMaxChannels,
    getInterfaceName,
    getPeripheralPinNames,
    getClockEnableIds,
    validatePinmux: validatePinmux
};

function addModuleInstances(instance) {
    let modInstances = new Array();
    
    if(instance.intrEnable == "DMA") {
        modInstances.push({
        name: "edmaDriver",
        displayName: "EDMA Configuration",
        moduleName: "/drivers/edma/edma",
        });
    }

    return modInstances;
}

function getConfigurables()
{
    let config = [];

    config.push(
        {
            name: "mode",
            displayName: "Mode of Operation",
            default: "SINGLE_CONTROLLER",
            options: [
                {
                    name: "SINGLE_CONTROLLER",
                    displayName: "Single Controller"
                },
                {
                    name: "MULTI_CONTROLLER",
                    displayName: "Multi Controller"
                },
                {
                    name: "PERIPHERAL",
                    displayName: "Single Peripheral"
                },
            ],
            description: "Controller/Peripheral or Single/Multi channel mode of operation",
            onChange: function (inst, ui) {
                /* Init delay applicable only for single controller mode */
                if((inst.advanced == true) &&
                   (inst.mode == "SINGLE_CONTROLLER")) {
                    ui.initDelay.hidden = false;
                }
                else {
                    ui.initDelay.hidden = true;
                }
                /* 3/4 pin mode applicable only in single channel mode */
                if(inst.mode == "SINGLE_CONTROLLER") {
                    ui.pinMode.hidden = false;
                }
                else {
                    ui.pinMode.hidden = true;
                }
            },
        },
        {
            name: "pinMode",
            displayName: "Pin Mode",
            default: 4,
            options: [
                {
                    name: 3,
                    displayName: "3 Pin Mode"
                },
                {
                    name: 4,
                    displayName: "4 Pin Mode"
                },
            ],
            description: "3 pin mode: Chip-select (CS) is not used and all related options to CS have no meaning. 4 pin mode: CS is used. This is selectable only in single channel mode",
        },
        {
            name: "trMode",
            displayName: "TR Mode",
            default: "TX_RX",
            options: [
                {
                    name: "TX_RX",
                    displayName: "TX and RX"
                },
                {
                    name: "RX_ONLY",
                    displayName: "RX Only"
                },
                {
                    name: "TX_ONLY",
                    displayName: "TX Only"
                },
            ],
            description: "Channel transmit/receive mode",
        },
        {
            name: "loopback",
            displayName: "LoopBack Enable",
            default: "DISABLE",
            options: [
                {
                    name: "DISABLE",
                    displayName: "LOOPBACK DISABLED"
                },
                {
                    name: "ENABLE",
                    displayName: "LOOPBACK ENABLED"
                },
            ],
            description: "PAD Level Loopback mode enable or disable",
        },
        {
            name: "inputSelect",
            displayName: "Input Select",
            default: "0",
            options: [
                {
                    name: "0",
                    displayName: "MISO"
                },
                {
                    name: "1",
                    displayName: "MOSI"
                },
            ],
            description: "Input selected on MOSI or MISO line",
        },
        {
            name: "dpe1",
            displayName: "MOSI TX Enable",
            default: "ENABLE",
            options: [
                {
                    name: "ENABLE",
                    displayName: "TX ENABLED"
                },
                {
                    name: "DISABLE",
                    displayName: "TX DISABLED"
                },
            ],
            description: "Transmission enable/disable for MOSI",
        },
        {
            name: "dpe0",
            displayName: "MISO TX Enable",
            default: "DISABLE",
            options: [
                {
                    name: "ENABLE",
                    displayName: "TX ENABLED"
                },
                {
                    name: "DISABLE",
                    displayName: "TX DISABLED"
                },
            ],
            description: "Transmission enable/disable for MISO",
        },
        {
            name: "intrEnable", /* Did not change name to avoid interface break */
            displayName: "Operating Mode",
            default: "INTERRUPT",
            hidden: false,
            options: [
                {
                    name: "POLLED",
                    displayName: "Polled Mode"
                },
                {
                    name: "INTERRUPT",
                    displayName: "Interrupt Mode"
                },
                {
                    name: "DMA",
                    displayName: "DMA Mode"
                },
            ],
            onChange: function (inst, ui) {
                if(inst.intrEnable == "POLLED") {
                    ui.intrPriority.hidden = true;
                    ui.transferMode.hidden = true;
                    ui.transferTimeout.hidden = true;
                }
                if((inst.intrEnable == "INTERRUPT") || (inst.intrEnable == "DMA")) {
                    ui.intrPriority.hidden = false;
                    ui.transferMode.hidden = false;
                    ui.transferTimeout.hidden = false;
                }
                if((inst.intrEnable == "DMA")){
                    ui.dmaEnable.hidden = false;
                }

            },
            description: "Driver Operating Mode. In case of DMA mode, Default TX Data feature is not supported"
        },
        {
            name: "dmaEnable", /* Did not change name to avoid interface break */
            displayName: "DMA Enabled",
            default: true,
            hidden: true,
        },
        {
            name: "intrPriority",
            displayName: "Interrupt Priority",
            default: 4,
            hidden: false,
            description: `Interrupt Priority: 0 (highest) to ${hwi.getHwiMaxPriority()} (lowest)`,
        },
        {
            name: "transferMode",
            displayName: "Transfer Mode",
            default: "BLOCKING",
            hidden: false,
            options: [
                {
                    name: "BLOCKING",
                    displayName: "Blocking"
                },
                {
                    name: "CALLBACK",
                    displayName: "Callback"
                },
            ],
            onChange: function (inst, ui) {
                if(inst.transferMode == "CALLBACK") {
                    ui.transferCallbackFxn.hidden = false;
                    if(inst.transferCallbackFxn == "NULL") {
                        /* Clear NULL entry as user need to provide a fxn */
                        inst.transferCallbackFxn = "";
                    }
                }
                else {
                    ui.transferCallbackFxn.hidden = true;
                    inst.transferCallbackFxn = "NULL";
                }
            },
            description: "This determines whether the driver operates synchronously or asynchronously",
        },
        {
            name: "transferCallbackFxn",
            displayName: "Transfer Callback",
            default: "NULL",
            hidden: true,
            description: "Transfer callback function when callback mode is selected",
        },
        {
            name: "transferTimeout",
            displayName: "Transfer Timeout",
            default: 0xFFFFFFFF,
            hidden: false,
            description: "Transfer timeout in system ticks. Provide 0xFFFFFFFF to wait forever",
            displayFormat: "hex",
        },
        /* Advanced parameters */
        {
            name: "advanced",
            displayName: "Show Advanced Config",
            default: false,
            onChange: function (inst, ui) {
                /* Init delay applicable only for single controller mode */
                if((inst.advanced == true) &&
                   (inst.mode == "SINGLE_CONTROLLER")) {
                    ui.initDelay.hidden = false;
                }
                else {
                    ui.initDelay.hidden = true;
                }
            },
        },
        /* Advance Instance attributes */
        {
            name: "initDelay",
            displayName: "Init Delay",
            default: "0",
            hidden: true,
            options: [
                {
                    name: "0",
                },
                {
                    name: "4",
                },
                {
                    name: "8",
                },
                {
                    name: "16",
                },
                {
                    name: "32",
                },
            ],
            description: "Initial delay for first transfer in bus clock cycles. Applicable only for single controller mode",
        },
    )

    return config;
}
/*
 *  ======== validate ========
 */
function validate(inst, report) {
    common.validate.checkNumberRange(inst, report, "transferTimeout", 0x0, 0xFFFFFFFF, "hex");
    common.validate.checkValidCName(inst, report, "transferCallbackFxn");
    if((inst.transferMode == "CALLBACK") &&
        ((inst.transferCallbackFxn == "NULL") ||
            (inst.transferCallbackFxn == ""))) {
        report.logError("Callback function MUST be provided for callback transfer mode", inst, "transferCallbackFxn");
    }
    if ((inst.useMcuDomainPeripherals) &&
        (inst.intrEnable == "DMA")) {
        report.logError("DMA is not supported in MCU Domain", inst, "useMcuDomainPeripherals");
    }
    common.validate.checkNumberRange(inst, report, "intrPriority", 0, hwi.getHwiMaxPriority(), "dec");
}

/*
 *  ======== moduleInstances ========
 */
function moduleInstances(inst) {
    let modInstances = new Array();

    let maxCh = getMaxChannels(inst);
    modInstances.push({
        name: "mcspiChannel",
        displayName: "MCSPI Channel Configuration",
        moduleName: '/drivers/mcspi/v0/mcspi_v0_channel',
        useArray: true,
        maxInstanceCount: maxCh,
        minInstanceCount: 1,
        defaultInstanceCount: 1,
        args: {
            interfaceName: getInterfaceName(inst),
        },
    });

    return (modInstances);
}

function getClockFrequencies(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockFrequencies;
}

function getModule()
{
    let module = mcspi_module;
    if(soc.isFrequencyDefined())
    {
        module.getClockFrequencies = getClockFrequencies;
    }
    return module;
}

function validatePinmux(inst, report) {
    if (inst.intrEnable == "DMA") {
        let instConfig = getInstanceConfig(inst);
        let rxEvtRsvd = false;
        let txEvtRsvd = false;
        for(let i = 0; i < inst.edmaDriver.edmaRmReservedDmaCh.length; i++) {
            if((inst.edmaDriver.edmaRmReservedDmaCh[i].startIndex <= instConfig.rxDmaEvt) &&
            (inst.edmaDriver.edmaRmReservedDmaCh[i].endIndex >= instConfig.rxDmaEvt)) {
                rxEvtRsvd = true;
            }
            if((inst.edmaDriver.edmaRmReservedDmaCh[i].startIndex <= instConfig.txDmaEvt) &&
            (inst.edmaDriver.edmaRmReservedDmaCh[i].endIndex >= instConfig.txDmaEvt)) {
                txEvtRsvd = true;
            }
        }
        if (rxEvtRsvd == false) {
            report.logInfo(`Allocate dma channel no: ${instConfig.rxDmaEvt} in "Reserved Dma Channel Resource Manager" in "EDMA" Configuration for event triggered transfers`, inst, "dmaEnable");
        }
        if (txEvtRsvd == false) {
            report.logInfo(`Allocate dma channel no: ${instConfig.txDmaEvt} in "Reserved Dma Channel Resource Manager" in "EDMA" Configuration for event triggered transfers`, inst, "dmaEnable");
        }
    }
}

exports = getModule();
