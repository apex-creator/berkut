let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/iomux/soc/iomux_${common.getSocName()}`);

let iomux_module_jtag_name = "/drivers/iomux/v0/iomux_jtag";

function getStaticConfigArr() {
    return system.getScript(`/drivers/iomux/soc/iomux_${common.getSocName()}`).getStaticConfigArr();
};

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let staticConfigArr = getStaticConfigArr();
    let staticConfig = staticConfigArr.find( o => o.name === solution.peripheralName);

    return {
        ...staticConfig,
        ...moduleInstance
    }
};


function getInterfaceName(inst) {

    return "JTAG";
};

function getPeripheralPinNames(inst) {

    return [ "TMS","TDI","TDO","TCK"];

};


function pinmuxRequirements(inst) {
    let interfaceName = getInterfaceName(inst);

    let resources = [];

    for (const ele of getPeripheralPinNames(inst)){
        resources.push( pinmux.getPinRequirements(interfaceName, ele));
    }

    let peripheral = {
        name          : interfaceName,
        displayName   : "JTAG instance",
        interfaceName : "JTAG",
        resources     : resources,
    };

    return [peripheral];
}


let iomux_module_jtag= {
    displayName: "iomux_jtag",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: iomux_module_jtag_name,
        },
    },
    defaultInstanceName: "CONFIG_JTAG",
    maxInstances: getStaticConfigArr().length,
    pinmuxRequirements,
    getStaticConfigArr,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
};

exports = iomux_module_jtag;