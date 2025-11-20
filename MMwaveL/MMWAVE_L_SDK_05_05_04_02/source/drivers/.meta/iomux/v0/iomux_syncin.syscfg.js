let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/iomux/soc/iomux_${common.getSocName()}`);

let iomux_module_sync_in_name = "/drivers/iomux/v0/iomux_syncin";

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

    return "SYNC_IN";
};

function getPeripheralPinNames(inst) {

    return [ "IN"];

};


function pinmuxRequirements(inst) {
    let interfaceName = getInterfaceName(inst);

    let resources = [];

    resources.push( pinmux.getPinRequirements(interfaceName, "IN"));

    let peripheral = {
        name          : interfaceName,
        displayName   : "SYNC_IN instance",
        interfaceName : interfaceName,
        resources     : resources,
    };

    return [peripheral];
}


let iomux_module_syncin= {
    displayName: "iomux_syncin",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: iomux_module_sync_in_name,
        },
    },
    defaultInstanceName: "CONFIG_SYNCIN",
    maxInstances: getStaticConfigArr().length,
    pinmuxRequirements,
    getStaticConfigArr,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
};

exports = iomux_module_syncin;