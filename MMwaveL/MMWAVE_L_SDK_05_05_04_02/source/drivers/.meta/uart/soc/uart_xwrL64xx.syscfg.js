let common = system.getScript("/common");

let uart_input_clk_freq = 40000000;

/* On M4F, interrupt number as specified in TRM is input to the NVIC but from M4 point of view there are 16 internal interrupts
 * and then the NVIC input interrupts start, hence we need to add +16 to the value specified by TRM */
const uart_config_m4fss = [
    {
        name            : "UARTA",
        baseAddr        : "CSL_APP_UART0_U_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 16 + 8,
        rxDmaEvt        : 4,
        txDmaEvt        : 5,
        clockIds        : [ "SOC_RcmPeripheralId_APPSS_UART0" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_APPSS_UART0",
                clkId   : "SOC_RcmPeripheralClockSource_OSC_CLK",
                clkRate : uart_input_clk_freq,
            },
        ],
    },
    {
        name            : "UARTB",
        baseAddr        : "CSL_APP_UART1_U_BASE",
        inputClkFreq    : uart_input_clk_freq,
        intrNum         : 16 + 62,
        rxDmaEvt        : 34,
        txDmaEvt        : 35,
        clockIds        : [ "SOC_RcmPeripheralId_APPSS_UART1" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_APPSS_UART1",
                clkId   : "SOC_RcmPeripheralClockSource_OSC_CLK",
                clkRate : uart_input_clk_freq,
            },
        ],
    }
];

function getConfigArr() {
    return uart_config_m4fss;
}

function getInterfaceName(inst) {
    return "UART";
}

exports = {
    getConfigArr,
    getInterfaceName,
};
