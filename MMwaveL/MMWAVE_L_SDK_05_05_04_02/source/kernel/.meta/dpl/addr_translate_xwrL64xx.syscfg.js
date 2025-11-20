
let common = system.getScript("/common");

let ratBaseAddr_m4f = 0x22400000;

let longDescription_m4f = 
`Refer to the device TRM for Address regions .`
    ;


function getRatBaseAddr() {
    
    let cpu = common.getSelfSysCfgCoreName();
    let ratAddr = ratBaseAddr_m4f;

    if(cpu.match(/m4f*/))
    {
        ratAddr = ratBaseAddr_m4f;
    }
    return ratAddr;
}

function getLongDescription() {

    let cpu = common.getSelfSysCfgCoreName();
    let longDescription = longDescription_m4f;

    if(cpu.match(/m4f*/))
    {
        longDescription = longDescription_m4f;
    }
    return longDescription;    
}

exports = {
    getRatBaseAddr,
    getLongDescription,
};

