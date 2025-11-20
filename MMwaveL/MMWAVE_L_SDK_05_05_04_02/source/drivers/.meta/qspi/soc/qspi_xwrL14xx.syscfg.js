let qspi_input_clk_freq = 80000000;

const qspi_config_m4fss = [
    {
        name            : "QSPI0",
        baseAddr        : "CSL_APP_CFG_QSPI_U_BASE",
        memMapBaseAddr  : "CSL_APP_QSPI_EXT_FLASH_U_BASE",
        inputClkFreq    : qspi_input_clk_freq,
        intrNum         : 16 + 13,
        baudRateDiv     : 0,
        wrdLen          : 8,
        clockIds        : [ "SOC_RcmPeripheralId_APPSS_QSPI" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_APPSS_QSPI",
                clkId   : "SOC_RcmPeripheralClockSource_FAST_CLK",
                clkRate : qspi_input_clk_freq,
            },
        ],
    },
];

function getQspiPinName(inst, qspiPinName) {
    let pinName;
    switch (qspiPinName)
    {
        case "D0":
            pinName = "DOUT";
            break;
        case "D1":
            pinName = "DIN";
            break;
        case "D2":
            pinName = "DIN0";
            break;
        case "D3":
            pinName = "DIN1";
            break;
        case "CLK":
            pinName = "CLK";
            break;
        case "CS0":
        case "CS1":
        case "CS2":
        case "CS3":
            pinName = "CS";
            break;
    }
    return pinName;
}

function getInterfaceName(inst) {
    let interfaceName = "QSPI";
    return interfaceName;
}

function getDefaultConfig()
{
    return qspi_config_m4fss[0];
}

function getConfigArr() {
    return qspi_config_m4fss;
}

exports = {
    getDefaultConfig,
    getConfigArr,
    getInterfaceName,
    getQspiPinName,
};
