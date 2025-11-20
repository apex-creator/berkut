let common = system.getScript("/common");

let mcan_func_clk = 80 * 1000 * 1000;

/* On M4F, interrupt number as specified in TRM is input to the NVIC but from M4 point of view there are 16 internal interrupts
 * and then the NVIC input interrupts start, hence we need to add +16 to the value specified by TRM */
const mcan_config_m4fss = [
    {
        name            : "MCAN0",
        baseAddr        : "CSL_APP_MCAN_MSG_RAM_U_BASE",
        intrNum         : 16 + 21,
        clockIds        : [ "SOC_RcmPeripheralId_APPSS_MCAN" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_APPSS_MCAN",
                clkId   : "SOC_RcmPeripheralClockSource_FAST_CLK",
                clkRate : mcan_func_clk,
            },
        ],
    },
];

function getConfigArr() {
    let mcan_config;

    mcan_config = mcan_config_m4fss;

    return mcan_config;
}

function getInterfaceName(inst) {

    return "MCAN";
}

exports = {
    getConfigArr,
    getInterfaceName,
};
