let common = system.getScript("/common");

const gpio_config_m4fss = [
    {
        name            : "GPIO",
        baseAddr        : "CSL_APP_GIO_U_BASE",
        intrNumLow      : 16 + 7,
        intrNumHigh     : 16 + 6,
        clockIds        : [ "SOC_RcmPeripheralId_APPSS_GIO" ],
    },
];

function getConfigArr() {


    return gpio_config_m4fss;
}

exports = {
    getConfigArr,
};
