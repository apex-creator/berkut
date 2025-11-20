let common = system.getScript("/common");


let mcspi_input_clk_freq = 40000000;

/* On M4F, interrupt number as specified in TRM is input to the NVIC but from M4 point of view there are 16 internal interrupts
 * and then the NVIC input interrupts start, hence we need to add +16 to the value specified by TRM */
const mcspi_config_m4f = [
    {
        name            : "MCSPIA",
        baseAddr        : "CSL_MCU_MCSPI0_CFG_BASE",
        inputClkFreq    : mcspi_input_clk_freq,
        intrNum         : 14 + 16,
        rxDmaEvt        : 0,
        txDmaEvt        : 1,
        clockIds        : [ "SOC_RcmPeripheralId_APPSS_MCSPIA" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_APPSS_MCSPIA",
                clkId   : "SOC_RcmPeripheralClockSource_OSC_CLK",
                clkRate : mcspi_input_clk_freq,
            },
        ],
    },
    {
        name            : "MCSPIB",
        baseAddr        : "CSL_MCU_MCSPI1_CFG_BASE",
        inputClkFreq    : mcspi_input_clk_freq,
        intrNum         : 28 + 16,
        rxDmaEvt        : 2,
        txDmaEvt        : 3,
        clockIds        : [ "SOC_RcmPeripheralId_APPSS_MCSPIB" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_APPSS_MCSPIB",
                clkId   : "SOC_RcmPeripheralClockSource_OSC_CLK",
                clkRate : mcspi_input_clk_freq,
            },
        ],
    },
];

function getMaxChannels(inst) {
    return 2;   /* max number of channels per MCSPI */
}

function getConfigArr() {

    return mcspi_config_m4f;
}

function isFrequencyDefined()
{
    return true;
}

exports = {
    getConfigArr,
    getMaxChannels,
    isFrequencyDefined,
};
