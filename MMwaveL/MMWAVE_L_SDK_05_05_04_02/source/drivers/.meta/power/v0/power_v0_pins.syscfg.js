
/*
 * ======== Power64XXPins.syscfg.js ========
 * 64XX Pins module (owned by Power64XX)
 */

"use strict";

let pinTable = ["AA","AB","AC","AD","AE","AF","AG","AH","AI","AJ","AK","AL","AM","AN","AO","AP","AQ","AR","AS","AT","AU","AV","AW","AX"];
let pinOptions  = [
    {
        name: 'PARK',
        description: 'Park the Pin (Input and Output Disabled) during LPDS.' 
                      + 'User has to reconfig the PAD after exiting LPDS.'
    },
    {
        name: 'DONT_PARK',
        description: 'Do not park the pin. User PAD configuration is retained.'
    }
];

function resources()
{
    let resources = [
        {
            name          : 'changeAllPins',
            displayName   : 'Change All Pins',
            description   : "Use this configurable to update all pin"
                + " park states concurrently",
            onChange      : onchangeAllPins,
            default       : "DONT_PARK",
            options       : pinOptions
        }
    ];

    for (let idx = 0; idx < pinTable.length; idx++) {
        let pinNum = pinTable[idx];
        let defOpt = 'DONT_PARK';


        let description = "PAD_" + pinNum;
        if (system.deviceData.devicePins[pinTable[idx]]) {
            description = system.deviceData.devicePins[pinTable[idx]].designSignalName;
        }
        resources.push({
            name          : 'PAD_' + pinNum,
            displayName   : 'PAD_' + pinNum,
            description   : description,
            longDescription : description,
            default       : defOpt,
            options       : pinOptions
        });
    }
    return resources;
}

let config = resources();

function onchangeAllPins(inst, ui)
{
    let initVal = inst.changeAllPins;

    if (initVal === "DONT_PARK") {
        initVal = "DONT_PARK";
    }

    for (let idx = 0; idx < pinTable.length; idx++) {
        let pinName = "PAD_"+pinTable[idx];
        inst[pinName] = initVal;
    }
}

/* The device specific exports for the power module */
exports = {
    name         : 'parkPins',
    displayName  : 'Park Pins',
    description  : "Pin Park States",
    maxInstances : 1,
    config       : config,
};
