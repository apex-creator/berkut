
let common = system.getScript("/common");

const driverVer = {
    "crc": {
        version: "v0",
    },
    "edma": {
        version: "v0",
    },
    "epwm": {
        version: "v0",
    },
    "gpio": {
        version: "v0",
    },
    "hwa": {
        version: "v0",
    },
    "i2c": {
        version: "v0",
     },
    "iomux": {
        version: "v0",
     },
    "lin": {
         version: "v0",
     },
    "mcan": {
        version: "v0",
    },
    "mcspi": {
        version: "v0",
    },
    "power": {
        version: "v0",
    },
    "uart": {
        version: "v0",
    },
    "watchdog": {
        version: "v0",
    },
    "esm"     : {
        version: "v0",
    },
    "qspi"    : {
        version: "v0",
    }
};

const topModules_m4f = [
    "/drivers/crc/crc",
    "/drivers/edma/edma",
    "/drivers/epwm/epwm",
    "/drivers/esm/esm",
    "/drivers/gpio/gpio",
    "/drivers/hwa/hwa",
    "/drivers/i2c/i2c",
    "/drivers/iomux/iomux",
    "/drivers/lin/lin",
    "/drivers/mcan/mcan",
    "/drivers/mcspi/mcspi",
    "/drivers/power/power",
    "/drivers/qspi/qspi",
    "/drivers/uart/uart",
    "/drivers/watchdog/watchdog",
];

function getCpuID() {
    let corename_map = {
        "m4fss0-0" : "CSL_CORE_ID_M4FSS0_0",
    };

    return corename_map[common.getSelfSysCfgCoreName()];
}

exports = {
    getTopModules: function() {

        return topModules_m4f;
    },
    getDriverVer: function(driverName) {
        return driverVer[driverName].version;
    },
    getCpuID,
};
