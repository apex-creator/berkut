
let common = system.getScript("/common");

let lin_func_clk = 40 * 1000 * 1000;

/* On M4F, interrupt number as specified in TRM is input to the NVIC but from M4 point of view there are 16 internal interrupts
 * and then the NVIC input interrupts start, hence we need to add +16 to the value specified by TRM */
const lin_config_m4fss = [
    {
        name            : "LIN0",
        baseAddr        : "CSL_APP_LIN_U_BASE",
        intrNum         : 16 + 10,
        clockIds        : [ "SOC_RcmPeripheralId_APPSS_LIN" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_APPSS_LIN",
                clkId   : "SOC_RcmPeripheralClockSource_OSC_CLK",
                clkRate : lin_func_clk,
            },
        ],
    },
];

function getConfigArr() {
    return lin_config_m4fss;
}

function getInterfaceName(instance) {
    return "LIN";
}

exports = {
    getConfigArr,
    getInterfaceName,
};
