
let common = system.getScript("/common");

let epwm_func_clk = 80 * 1000 * 1000;


const staticConfig = [
    {
        name: "EPWMA",
        baseAddr: "CSL_APP_ETPWM_U_BASE",
        intrNum: 16 + 48,
        tripIntrNum: 16 + 49,
        isSyncoPresent: true,
        isSynciPresent: true,
		funcClk: epwm_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_MSS_RTIA" ],
		clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_RTIA",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : epwm_func_clk,
            },
        ],
       
    },	 
];
function getInterfaceName(inst) {

    let interfaceName;
   
        interfaceName = "EPWM";
         return interfaceName;

}

function getStaticConfigArr() {
    return staticConfig;
}



let soc = {
    getStaticConfigArr,
	getInterfaceName,
};

exports = soc;
