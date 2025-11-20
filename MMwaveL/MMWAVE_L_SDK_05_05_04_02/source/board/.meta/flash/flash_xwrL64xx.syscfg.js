let supported_spi_drivers = [
    {
        name: "qspi",
    },
];

let defaultFlashConfig = system.getScript("MX25V1635F.json");

function getDriverOptions()
{
    return supported_spi_drivers;
}

function getDefaultDriver()
{
    return supported_spi_drivers[0].name;
}

function getDefaultFlashName()
{
    return "MX25V1635F";
}

function getDefaultFlashConfig()
{
    return defaultFlashConfig;
}

function getDefaultProtocol()
{
    return { name : "1s_1s_4s", displayName : "1S-1S-4S" };
}

exports = {
    getDriverOptions,
    getDefaultDriver,
    getDefaultFlashName,
    getDefaultProtocol,
    getDefaultFlashConfig,
};
