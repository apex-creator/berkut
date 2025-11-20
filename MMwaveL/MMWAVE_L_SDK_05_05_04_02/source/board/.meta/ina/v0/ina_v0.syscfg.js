
let common = system.getScript("/common");
let soc = system.getScript(`/board/ina/soc/ina_${common.getSocName()}`);

function getStaticConfigArr() {
    return system.getScript(`/board/ina/soc/ina_${common.getSocName()}`).getStaticConfigArr();
}

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};


function getInterfaceName(inst) {

    return "INA";
}


function getConfigurables()
{
    let config = [];

    config.push(

        {
            name: "reg0",
            displayName: "Register 0",
            default: "0x0020",
            description: "Register 0 config",
            options: [
                {
                    name: "0x0020",
                    displayName: "With Temp Comp"
                },
                {
                    name: "0x0000",
                    displayName: "No Temp Comp"
                },
            ],
        },
        {
            name: "reg1",
            displayName: "Register 1",
            default: "65535",
            description: "Register 1 config",
            options: [
                {
                    name: "65535",
                    displayName: "Enable All Modes"
                },
            ],
        },
        {
            name: "maxCurrent",
            displayName: "Max current (uA)",
            default: 524288,
            hidden: false,
            description: "Maximum current value in uAmpere",
        },
        {
            name: "shuntRes",
            displayName: "Shunt Resistance (mOhm)",
            default: 40,
            hidden: false,
            description: "Shunt resistor value in mOhm",
        },
        {
            name: "shuntTempCoeff",
            displayName: "Shunt Temperature Coefficient",
            default: 75,
            description: "Shunt temperature coefficient",
        },
        {
            name: "deviceType",
            displayName: "Device Type",
            default: "INA228",
            options: [
                {
                    name: "INA228",
                    displayName: "INA228"
                },
            ],
        },
        {
            name: "I2CTargetAddress",
            displayName: "Power Rail",
            default: "0x40",
            options: [
                {
                    name: "0x41",
                    displayName: "Dig 1.2V Rail"
                },
                {
                    name: "0x44",
                    displayName: "RF 1.2V Rail"
                },
                {
                    name: "0x40",
                    displayName: "1.8V Rail"
                },
                {
                    name: "0x45",
                    displayName: "3.3V Rail"
                },
            ],
            onChange: function(inst, ui) {
                if(inst.I2CTargetAddress == "0x40") {
                    inst.customI2CTargetAddress = 0x40;
                } else if(inst.I2CTargetAddress == "0x44") {
                    inst.customI2CTargetAddress = 0x44;
                } else if(inst.I2CTargetAddress == "0x41") {
                    inst.customI2CTargetAddress = 0x41;
                } else {
                    inst.customI2CTargetAddress = 0x45;
                }
            },
            
        },
        {
            name: "customI2CTargetAddress",
            displayName: "I2C Target address",
            default: 0x40,
            displayFormat: "hex",
            longDescription: `The default I2C target addresses are:\n
        Dig 1.2V Rail: 0x40\n
        1.8V Rail: 0x41\n
        RF 1.2V Rail: 0x44\n
        3.3V Rail is not currently used`,
        }
    )

    return config;

}

let ina_module_name = "/board/ina/ina";

let ina_module = {
    displayName: "INA",
    templates: {
        "/board/board/board_config.h.xdt": {
            board_config: "/board/ina/templates/ina_v0_config.h.xdt",
        },
        "/board/board/board_config.c.xdt": {
            board_config: "/board/ina/templates/ina_v0_config.c.xdt",
        },
    },

    defaultInstanceName: "CONFIG_INA",
    maxInstances: 4,
    config:  getConfigurables(),
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceConfig,
    getInterfaceName,
};



function getModule()
{
    let module = ina_module;
    return module;
}

exports = getModule();
