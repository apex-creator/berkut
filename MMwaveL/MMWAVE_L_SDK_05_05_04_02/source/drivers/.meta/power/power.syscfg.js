
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/soc/drivers_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("power");

    return system.getScript(`/drivers/power/${driverVer}/power_${driverVer}`);
}

exports = getModule();
