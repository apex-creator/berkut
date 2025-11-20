
let common = system.getScript("/common");

function getConfigArr() {
    return system.getScript(`/drivers/hwa/soc/hwa_${common.getSocName()}`).getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);

    return {
        ...config,
        ...moduleInstance,
    };
};

function getClockEnableIds(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockIds;
}

let hwa_module_name = "/drivers/hwa/hwa";

let hwa_module = {
    displayName: "HWA",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/hwa/templates/hwa_config.c.xdt",
            driver_init: "/drivers/hwa/templates/hwa_init.c.xdt",
            driver_deinit: "/drivers/hwa/templates/hwa_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/hwa/templates/hwa.h.xdt",
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: hwa_module_name,
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_HWA",
    config: [
        common.ui.makeInstanceConfig(getConfigArr()),
    ],
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
    getClockEnableIds,
};

function validate(instance, report) {
    common.validate.checkSameInstanceName(instance, report);
}

exports = hwa_module;
