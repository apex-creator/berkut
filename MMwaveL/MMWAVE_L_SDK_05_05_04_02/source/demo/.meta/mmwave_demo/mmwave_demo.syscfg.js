
let common = system.getScript("/common");
let soc = system.getScript(`/demo/soc/demo_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("mmwave_demo");

    return system.getScript(`/demo/mmwave_demo/${driverVer}/mmwave_demo_${driverVer}`);
}

exports = getModule();
