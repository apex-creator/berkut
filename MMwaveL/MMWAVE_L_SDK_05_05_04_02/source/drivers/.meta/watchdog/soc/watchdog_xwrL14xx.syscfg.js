
let common = system.getScript("/common");

let wdt_func_clk_freq = 40000000;

const watchdog_config_m4fss = [
    {
        name: "APP_WDT",
		wdtInstance: "0",
        baseAddr        : "CSL_APP_WD_U_BASE",
        inputClkFreq    : wdt_func_clk_freq,
        clockIds        : [ "SOC_RcmPeripheralId_APPSS_WDT" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_APPSS_WDT",
                clkId   : "SOC_RcmPeripheralClockSource_OSC_CLK",
                clkRate : wdt_func_clk_freq,
            },
        ],
    },
];

function getConfigArr() {
    return watchdog_config_m4fss;
}

const SOC_RcmClkSrcInfo = [
    {
        name: "SOC_RcmPeripheralClockSource_OSC_CLK",
        freq: 40000000,
        displayName: "OSC_CLK (40 MHz)"
    },
]

exports = {
    getConfigArr,
    SOC_RcmClkSrcInfo
};