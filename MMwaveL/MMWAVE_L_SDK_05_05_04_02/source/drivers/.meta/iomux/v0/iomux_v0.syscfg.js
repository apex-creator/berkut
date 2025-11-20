let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/iomux/soc/iomux_${common.getSocName()}`);

let iomux_module_name = "/drivers/iomux/iomux";

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

    return "MDO";
};

function getPeripheralPinNames(inst) {

    return [ "CLK", "FRMCLK", "D0","D1", "D2", "D3"];

};

function pinmuxRequirements(inst) {
    let interfaceName = getInterfaceName(inst);

    let resources = [];

    resources.push( pinmux.getPinRequirements(interfaceName, "CLK", "Clk Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "FRMCLK", "Frame Clock Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "D0", "D0 Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "D1", "D1 Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "D2", "D2 Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "D3", "D3 Pin"));

    let peripheral = {
        name          : interfaceName,
        displayName   : "MDO Instance",
        interfaceName : interfaceName,
        resources     : resources,

    };

    return [peripheral];
}




let enable = 0;
let iomux_module = {
    displayName: "IOMUX",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: iomux_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: iomux_module_name,
        },
    },
    defaultInstanceName: "CONFIG_IOMUX",
    maxInstances: getStaticConfigArr().length,
    moduleInstances,
    getStaticConfigArr,
    getInstanceConfig,
    pinmuxRequirements,
    getInterfaceName,
    getPeripheralPinNames,
};

function moduleInstances(inst) {
    let modInstances = new Array();

    modInstances.push({
        name: "JTAG",
        displayName: "JTAG Configuration",
        moduleName: '/drivers/iomux/v0/iomux_jtag',
        useArray: true,
        maxInstanceCount: 1,
        minInstanceCount: 0,
        defaultInstanceCount: 0,
    });

    return (modInstances);
}

exports = iomux_module;