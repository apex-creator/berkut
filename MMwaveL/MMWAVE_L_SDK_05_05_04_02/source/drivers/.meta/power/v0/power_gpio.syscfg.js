let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");let soc = system.getScript(`/drivers/iomux/soc/iomux_${common.getSocName()}`);

let power_gpio_module = "/drivers/power/v0/power_gpio";

// function getStaticConfigArr() {
//     return system.getScript(`/drivers/iomux/soc/iomux_${common.getSocName()}`).getStaticConfigArr();
// };

function getInstanceConfig(moduleInstance) {
    // let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    // let staticConfigArr = getStaticConfigArr();
    // let staticConfig = staticConfigArr.find( o => o.name === solution.peripheralName);

    return {
    moduleInstance
    };
};


function getInterfaceName(inst) {

    return "WU_REQIN";
};

function getPeripheralPinNames(inst) {

    return [ "REQIN"];

};


function pinmuxRequirements(inst) {
    let interfaceName = getInterfaceName(inst);

    let resources = [];

    resources.push( pinmux.getPinRequirements(interfaceName, "REQIN"));

    let peripheral = {
        name          : interfaceName,
        displayName   : "GPIO instance",
        interfaceName : interfaceName,
        resources     : resources,
    };

    return [peripheral];
}


let power_module_gpio = {
    displayName: "power_gpio",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: power_gpio_module,
        },
    },
    defaultInstanceName: "CONFIG_GPIO",
    maxInstances: 1,
    pinmuxRequirements,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
};

exports = power_module_gpio;