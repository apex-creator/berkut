
let common = system.getScript("/common");
let soc = system.getScript(`/board/soc/board_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("ina");

    return system.getScript(`/board/ina/${driverVer}/ina_${driverVer}`);
}

exports = getModule();
