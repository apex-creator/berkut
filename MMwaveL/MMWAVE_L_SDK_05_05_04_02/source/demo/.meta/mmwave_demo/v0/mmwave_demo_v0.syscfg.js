
let common = system.getScript("/common");
let soc = system.getScript(`/demo/mmwave_demo/soc/mmwave_demo_${common.getSocName()}`);
let nodeCmd = "node";
let res = null;

if(system.getOS() == "win")
{
    nodeCmd = "node.exe";
}

function getStaticConfigArr() {
    return system.getScript(`/demo/mmwave_demo/soc/mmwave_demo_${common.getSocName()}`).getStaticConfigArr();
}

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};


function getInterfaceName(inst) {

    return "MMWAVE_DEMO";
}

function getConfigurables()
{
    let config = [];

    config.push(

        {
            name: "CLI",
            displayName: "CLI Removal",
            default: "0",
            description: "Enable/Disable CLI Removal for a memory optimized demo",
            longDescription: `
    The CLI Removal option, if enabled, optimizes the demo application for memory by disabling CLI utility and certain features 
    dependent on CLI such as Quick Eval, ADC data logging and injection, Monitors. 
    This also helps in power saving by reducing the overall retention RAM and thereby, deep sleep power
    `,
            options: [
                {
                    name: "1",
                    displayName: "Enable"
                },
                {
                    name: "0",
                    displayName: "Disable"
                },
            ],
            onChange: function (inst, ui) {
                if(inst.CLI == "0") {
                    ui.DEVICE.hidden = true;
                    ui.userCliCfgFromFile.hidden = true;
                    ui.SPI_ADC_DATA_STREAMING.hidden = false;
                    ui.ENABLE_MONITORS.hidden = false;
                }
                else {
                    ui.DEVICE.hidden = false;
                    ui.userCliCfgFromFile.hidden = false;
                    ui.SPI_ADC_DATA_STREAMING.hidden = true;
                    ui.ENABLE_MONITORS.hidden = true;
                }
            },
        },
        {
            name: "DEVICE",
            displayName: "DEVICE SELECT",
            default: "0",
            description: "XWRL64XX/XWRL14XX",
            hidden:true,
            options: [
                {
                    name: "1",
                    displayName: "XWRL14XX"
                },
                {
                    name: "0",
                    displayName: "XWRL64XX"
                },
            ],
        },
        {
            name: "SPI_ADC_DATA_STREAMING",
            displayName: "ADC STREAMING via SPI",
            default: "0",
            description: "Enable/Disable ADC STREAMING via SPI",
            longDescription:`
    The Demo supports streaming of raw ADC data over SPI interface
    every frame during the frame idle time, if enabled.
            `,
            options: [
                {
                    name: "1",
                    displayName: "Enable"
                },
                {
                    name: "0",
                    displayName: "Disable"
                },
            ],
        },
        {
            name: "ENABLE_MONITORS",
            displayName: "MONITORS",
            default: "0",
            description: "Enable/Disable Monitors",
            longDescription: `
    mmWaveLink Monitor Module is responsible for providing APIs for FECSS analog monitor. If enabled, it supports the following monitors: 
    - APLL & SYNTH Monitors
    - RX and TX loopback Monitors
    - TX Power Monitor
    - TX Ballbreak Monitor
    - RX Saturation Live monitor
    - Synth Frequency Live Monitor
    - Internal Signal DCBIST Monitors
    - Rx Saturation Live Monitors and Synth Frequency Monitors, if enabled.
    `,
            options: [
                {
                    name: "1",
                    displayName: "Enable"
                },
                {
                    name: "0",
                    displayName: "Disable"
                },
            ],
        },
        {
            name: "channelCfg_rxChCtrlBitMask",
            default: "7",
            hidden: true,
        },
        {
            name: "channelCfg_txChCtrlBitMask",
            default: "3",
            hidden: true,
        },
        {
            name: "channelCfg_miscCtrl",
            default: "0",
            hidden: true,
        },
        {
            name: "chirpComnCfg_digOutputSampRate",
            default: "8",
            hidden: true,
        },
        {
            name: "chirpComnCfg_digOutputBitsSel",
            default: "0",
            hidden: true,
        },
        {
            name: "chirpComnCfg_dfeFirSel",
            default: "0",
            hidden: true,
        },
        {
            name: "chirpComnCfg_numOfAdcSamples",
            default: "256",
            hidden: true,
        },
        {
            name: "chirpComnCfg_chirpTxMimoPatSel",
            default: "4",
            hidden: true,
        },
        {
            name: "chirpComnCfg_chirpRampEndTime",
            default: "24.3",
            hidden: true,
        },
        {
            name: "chirpComnCfg_chirpRxHpfSel",
            default: "3",
            hidden: true,
        },
        {
            name: "chirpTimingCfg_chirpIdleTime",
            default: "28",
            hidden: true,
        },
        {
            name: "chirpTimingCfg_chirpAdcSkipSamples",
            default: "37",
            hidden: true,
        },
        {
            name: "chirpTimingCfg_chirpTxStartTime",
            default: "0",
            hidden: true,
        },
        {
            name: "chirpTimingCfg_chirpRfFreqSlope",
            default: "160",
            hidden: true,
        },
        {
            name: "chirpTimingCfg_chirpRfFreqStart_64xx",
            default: "58.0",
            hidden: true,
        },
        {
            name: "chirpTimingCfg_chirpRfFreqStart_14xx",
            default: "77.0",
            hidden: true,
        },
        {
            name: "frameCfg_numOfChirpsInBurst",
            default: "64",
            hidden: true,
        },
        {
            name: "frameCfg_numOfChirpsAccum",
            default: "0",
            hidden: true,
        },
        {
            name: "frameCfg_burstPeriodicity",
            default: "4000",
            hidden: true,
        },
        {
            name: "frameCfg_numOfBurstsInFrame",
            default: "1",
            hidden: true,
        },
        {
            name: "frameCfg_framePeriodicity",
            default: "100.0",
            hidden: true,
        },
        {
            name: "frameCfg_numOfFrames",
            default: "0",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt0_row",
            default: "0",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt0_col",
            default: "0",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt1_row",
            default: "1",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt1_col",
            default: "1",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt2_row",
            default: "0",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt2_col",
            default: "2",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt3_row",
            default: "0",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt3_col",
            default: "1",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt4_row",
            default: "1",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt4_col",
            default: "2",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt5_row",
            default: "0",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt5_col",
            default: "3",
            hidden: true,
        },
        {
            name: "antGeometryCfg_antDistX",
            default: "2.418",
            hidden: true,
        },
        {
            name: "antGeometryCfg_antDistZ",
            default: "2.418",
            hidden: true,
        },
        {
            name: "guiMonitor_pointCloud",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_rangeProfile",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_noiseProfile",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_rangeAzimuthHeatMap",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_rangeDopplerHeatMap",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_statsInfo",
            default: "1",
            hidden: true,
        },
        {
            name: "guiMonitor_presenceInfo",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_adcSamples",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_trackerInfo",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_microDopplerInfo",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_classifierInfo",
            default: "0",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_azimuthFftSize",
            default: "64",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_elevationFftSize",
            default: "32",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_motDetMode",
            default: "1",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_coherentDoppler",
            default: "0",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_numFrmPerMinorMotProc",
            default: "0",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_numMinorMotionChirpsPerFrame",
            default: "0",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_forceMinorMotionVelocityToZero",
            default: "0",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_minorMotionVelocityInclusionThr",
            default: "0",
            hidden: true,
        },
        {
            name: "cfarCfg_averageMode",
            default: "2",
            hidden: true,
        },
        {
            name: "cfarCfg_winLen",
            default: "8",
            hidden: true,
        },
        {
            name: "cfarCfg_guardLen",
            default: "4",
            hidden: true,
        },
        {
            name: "cfarCfg_noiseDiv",
            default: "3",
            hidden: true,
        },
        {
            name: "cfarCfg_cyclicMode",
            default: "0",
            hidden: true,
        },
        {
            name: "cfarCfg_thresholdScale",
            default: "12.0",
            hidden: true,
        },
        {
            name: "cfarCfg_peakGroupingEn",
            default: "0",
            hidden: true,
        },
        {
            name: "cfarCfg_sideLobeThreshold",
            default: "0.95",
            hidden: true,
        },
        {
            name: "cfarCfg_enableLocalMaxRange",
            default: "0",
            hidden: true,
        },
        {
            name: "cfarCfg_enableLocalMaxDoppler",
            default: "1",
            hidden: true,
        },
        {
            name: "cfarCfg_interpolateRange",
            default: "1",
            hidden: true,
        },
        {
            name: "cfarCfg_interpolateDoppler",
            default: "1",
            hidden: true,
        },
        {
            name: "cfarScndPassCfg_enabled",
            default: "1",
            hidden: true,
        },
        {
            name: "cfarScndPassCfg_averageMode",
            default: "2",
            hidden: true,
        },
        {
            name: "cfarScndPassCfg_winLen",
            default: "8",
            hidden: true,
        },
        {
            name: "cfarScndPassCfg_guardLen",
            default: "4",
            hidden: true,
        },
        {
            name: "cfarScndPassCfg_noiseDiv",
            default: "3",
            hidden: true,
        },
        {
            name: "cfarScndPassCfg_cyclicMode",
            default: "1",
            hidden: true,
        },
        {
            name: "cfarScndPassCfg_thresholdScale",
            default: "25.0",
            hidden: true,
        },
        {
            name: "cfarScndPassCfg_peakGroupingEn",
            default: "0",
            hidden: true,
        },
        {
            name: "aoaFovCfg_minAzimuthDeg",
            default: "-80",
            hidden: true,
        },
        {
            name: "aoaFovCfg_maxAzimuthDeg",
            default: "80",
            hidden: true,
        },
        {
            name: "aoaFovCfg_minElevationDeg",
            default: "-20",
            hidden: true,
        },
        {
            name: "aoaFovCfg_maxElevationDeg",
            default: "20",
            hidden: true,
        },
        {
            name: "rangeSelCfg_minMeters",
            default: "0.1",
            hidden: true,
        },
        {
            name: "rangeSelCfg_maxMeters",
            default: "10",
            hidden: true,
        },
        {
            name: "clutterRemoval_enabled",
            default: "0",
            hidden: true,
        },
        {
            name: "compRangeBiasAndRxChanPhase_rangeBias",
            default: "0.0",
            hidden: true,
        },
        {
            name: "lowPowerCfg_enable",
            default: "1",
            hidden: true,
        },
        {
            name: "factoryCalibCfg_saveEnable",
            default: "0",
            hidden: true,
        },
        {
            name: "factoryCalibCfg_restoreEnable",
            default: "0",
            hidden: true,
        },
        {
            name: "factoryCalibCfg_rxGain",
            default: "40",
            hidden: true,
        },
        {
            name: "factoryCalibCfg_txBackoff",
            default: "0",
            hidden: true,
        },
        {
            name: "factoryCalibCfg_flashOffset",
            default: "0x1FF000",
            hidden: true,
        },
        {
            name: "compressionCfg_enabled",
            default: "1",
            hidden: true,
        },
        {
            name: "compressionCfg_ratio",
            default: "0.5",
            hidden: true,
        },
        {
            name: "sensorStart_frameTrigMode",
            default: "0",
            hidden: true,
        },
        {
            name: "sensorStart_chirpStartSigLbEn",
            default: "0",
            hidden: true,
        },
        {
            name: "sensorStart_frameLivMonEn",
            default: "0",
            hidden: true,
        },
        {
            name: "sensorStart_frameTrigTimerVal",
            default: "0",
            hidden: true,
        },
        {
            name: "userCliCfgFromFile",
            displayName: "Load CLI Config From JSON",
            buttonText: "Load From JSON",
            fileFilter: ".json",
            pickDirectory: false,
            nonSerializable: true,
            hidden:true,
            onLaunch: (inst) => {
                let products=system.getProducts()
                let sdkPath = ""
                let copyScriptPath = ""
                if(system.getOS() == "win") {
                    sdkPath = products[0].path.split("\\.metadata\\product.json")[0];
                    copyScriptPath = sdkPath + "//source//demo//.meta//mmwave_demo//copyutil.js";
                } else {
                    sdkPath = products[0].path.split("/.metadata/product.json")[0];
                    copyScriptPath = sdkPath + "/source/demo/.meta/mmwave_demo/copyutil.js";
                }
                return {
                    command: nodeCmd,
                    args: [copyScriptPath, "$browsedFile", "$comFile"],
                    initialData: "initialData",
                    inSystemPath: true,
                };
            },
            onComplete: (inst, _ui, result) => {
                if(result.data === "error") {
                    inst.fname = "ERROR LOADING CONFIG";
                    return;
                } else if(result.data === "initialData") {
                    inst.fname = "ERROR RUNNING SCRIPT";
                } else {


                    try {
                        res = JSON.parse(result.data);
                        fillConfigs(inst,res);
                    } catch (e) {
                        inst.fname = "ERROR PARSING CONFIG";
                        return;
                    }
                    /* Fill up the configurables with data from JSON file */
                    return;
                }
            },
        },
             
    )
    return config;

}

function fillConfigs(inst,res)
{
    inst.channelCfg_rxChCtrlBitMask = res.channelCfg.rxChCtrlBitMask;
    inst.channelCfg_txChCtrlBitMask = res.channelCfg.txChCtrlBitMask;
    inst.channelCfg_miscCtrl = res.channelCfg.miscCtrl;

    inst.chirpComnCfg_digOutputSampRate = res.chirpComnCfg.digOutputSampRate;
    inst.chirpComnCfg_digOutputBitsSel = res.chirpComnCfg.digOutputBitsSel;
    inst.chirpComnCfg_dfeFirSel = res.chirpComnCfg.dfeFirSel;
    inst.chirpComnCfg_numOfAdcSamples = res.chirpComnCfg.numOfAdcSamples;
    inst.chirpComnCfg_chirpTxMimoPatSel = res.chirpComnCfg.chirpTxMimoPatSel;
    inst.chirpComnCfg_chirpRampEndTime = res.chirpComnCfg.chirpRampEndTime;
    inst.chirpComnCfg_chirpRxHpfSel = res.chirpComnCfg.chirpRxHpfSel;

    inst.chirpTimingCfg_chirpIdleTime = res.chirpTimingCfg.chirpIdleTime;
    inst.chirpTimingCfg_chirpAdcSkipSamples = res.chirpTimingCfg.chirpAdcSkipSamples;
    inst.chirpTimingCfg_chirpTxStartTime = res.chirpTimingCfg.chirpTxStartTime;
    inst.chirpTimingCfg_chirpRfFreqSlope = res.chirpTimingCfg.chirpRfFreqSlope;
    if(inst.DEVICE == "0")
    {
        inst.chirpTimingCfg_chirpRfFreqStart_64xx = res.chirpTimingCfg.chirpRfFreqStart;
    }
    else
    {
        inst.chirpTimingCfg_chirpRfFreqStart_14xx = res.chirpTimingCfg.chirpRfFreqStart;
    }
    inst.frameCfg_numOfChirpsInBurst = res.frameCfg.numOfChirpsInBurst;
    inst.frameCfg_numOfChirpsAccum = res.frameCfg.numOfChirpsAccum;
    inst.frameCfg_burstPeriodicity = res.frameCfg.burstPeriodicity;
    inst.frameCfg_numOfBurstsInFrame = res.frameCfg.numOfBurstsInFrame;
    inst.frameCfg_framePeriodicity = res.frameCfg.framePeriodicity;
    inst.frameCfg_numOfFrames = res.frameCfg.numOfFrames;

    inst.guiMonitor_pointCloud = res.guiMonitor.pointCloud;
    inst.guiMonitor_rangeProfile = res.guiMonitor.rangeProfile;
    inst.guiMonitor_noiseProfile = res.guiMonitor.noiseProfile;
    inst.guiMonitor_rangeAzimuthHeatMap = res.guiMonitor.rangeAzimuthHeatMap;
    inst.guiMonitor_rangeDopplerHeatMap = res.guiMonitor.rangeDopplerHeatMap;
    inst.guiMonitor_statsInfo = res.guiMonitor.statsInfo;
    inst.guiMonitor_presenceInfo = res.guiMonitor.presenceInfo;
    inst.guiMonitor_adcSamples = res.guiMonitor.adcSamples;
    inst.guiMonitor_trackerInfo = res.guiMonitor.trackerInfo;
    inst.guiMonitor_microDopplerInfo = res.guiMonitor.microDopplerInfo;
    inst.guiMonitor_classifierInfo = res.guiMonitor.classifierInfo;

    inst.sigProcChainCfg_azimuthFftSize = res.sigProcChainCfg.azimuthFftSize;
    inst.sigProcChainCfg_elevationFftSize = res.sigProcChainCfg.elevationFftSize;
    inst.sigProcChainCfg_motDetMode = res.sigProcChainCfg.motDetMode;
    inst.sigProcChainCfg_coherentDoppler = res.sigProcChainCfg.coherentDoppler;
    inst.sigProcChainCfg_numFrmPerMinorMotProc = res.sigProcChainCfg.numFrmPerMinorMotProc;
    inst.sigProcChainCfg_numMinorMotionChirpsPerFrame = res.sigProcChainCfg.numMinorMotionChirpsPerFrame;
    inst.sigProcChainCfg_forceMinorMotionVelocityToZero = res.sigProcChainCfg.forceMinorMotionVelocityToZero;
    inst.sigProcChainCfg_minorMotionVelocityInclusionThr = res.sigProcChainCfg.minorMotionVelocityInclusionThr;

    inst.antGeometryCfg_vAnt0_row = res.antGeometryCfg.vAnt0_row;
    inst.antGeometryCfg_vAnt0_col = res.antGeometryCfg.vAnt0_col;
    inst.antGeometryCfg_vAnt1_row = res.antGeometryCfg.vAnt1_row;
    inst.antGeometryCfg_vAnt1_col = res.antGeometryCfg.vAnt1_col;
    inst.antGeometryCfg_vAnt2_row = res.antGeometryCfg.vAnt2_row;
    inst.antGeometryCfg_vAnt2_col = res.antGeometryCfg.vAnt2_col;
    inst.antGeometryCfg_vAnt3_row = res.antGeometryCfg.vAnt3_row;
    inst.antGeometryCfg_vAnt3_col = res.antGeometryCfg.vAnt3_col;
    inst.antGeometryCfg_vAnt4_row = res.antGeometryCfg.vAnt4_row;
    inst.antGeometryCfg_vAnt4_col = res.antGeometryCfg.vAnt4_col;
    inst.antGeometryCfg_vAnt5_row = res.antGeometryCfg.vAnt5_row;
    inst.antGeometryCfg_vAnt5_col = res.antGeometryCfg.vAnt5_col;
    inst.antGeometryCfg_antDistX = res.antGeometryCfg.antDistX;
    inst.antGeometryCfg_antDistZ = res.antGeometryCfg.antDistZ;

    inst.cfarCfg_averageMode = res.cfarCfg.averageMode;
    inst.cfarCfg_winLen = res.cfarCfg.winLen;
    inst.cfarCfg_guardLen = res.cfarCfg.guardLen;
    inst.cfarCfg_noiseDiv = res.cfarCfg.noiseDiv;
    inst.cfarCfg_cyclicMode = res.cfarCfg.cyclicMode;
    inst.cfarCfg_thresholdScale = res.cfarCfg.thresholdScale;
    inst.cfarCfg_peakGroupingEn = res.cfarCfg.peakGroupingEn;
    inst.cfarCfg_sideLobeThreshold = res.cfarCfg.sideLobeThreshold;
    inst.cfarCfg_enableLocalMaxRange = res.cfarCfg.enableLocalMaxRange;
    inst.cfarCfg_enableLocalMaxDoppler = res.cfarCfg.enableLocalMaxDoppler;
    inst.cfarCfg_interpolateRange = res.cfarCfg.interpolateRange;
    inst.cfarCfg_interpolateDoppler = res.cfarCfg.interpolateDoppler;

    inst.cfarScndPassCfg_enabled = res.cfarScndPassCfg.enabled;
    inst.cfarScndPassCfg_averageMode = res.cfarScndPassCfg.averageMode;
    inst.cfarScndPassCfg_winLen = res.cfarScndPassCfg.winLen;
    inst.cfarScndPassCfg_guardLen = res.cfarScndPassCfg.guardLen;
    inst.cfarScndPassCfg_noiseDiv = res.cfarScndPassCfg.noiseDiv;
    inst.cfarScndPassCfg_cyclicMode = res.cfarScndPassCfg.cyclicMode;
    inst.cfarScndPassCfg_thresholdScale = res.cfarScndPassCfg.thresholdScale;
    inst.cfarScndPassCfg_peakGroupingEn = res.cfarScndPassCfg.peakGroupingEn;

    inst.aoaFovCfg_minAzimuthDeg = res.aoaFovCfg.minAzimuthDeg;
    inst.aoaFovCfg_maxAzimuthDeg = res.aoaFovCfg.maxAzimuthDeg;
    inst.aoaFovCfg_minElevationDeg = res.aoaFovCfg.minElevationDeg;
    inst.aoaFovCfg_maxElevationDeg = res.aoaFovCfg.maxElevationDeg;

    inst.rangeSelCfg_minMeters = res.rangeSelCfg.minMeters;
    inst.rangeSelCfg_maxMeters = res.rangeSelCfg.maxMeters;

    inst.clutterRemoval_enabled = res.clutterRemoval.enabled;

    inst.compRangeBiasAndRxChanPhase_rangeBias = res.compRangeBiasAndRxChanPhase.rangeBias;

    inst.lowPowerCfg_enable = res.lowPowerCfg.enable;

    inst.factoryCalibCfg_saveEnable = res.factoryCalibCfg.saveEnable;
    inst.factoryCalibCfg_restoreEnable = res.factoryCalibCfg.restoreEnable;
    inst.factoryCalibCfg_rxGain = res.factoryCalibCfg.rxGain;
    inst.factoryCalibCfg_txBackoff = res.factoryCalibCfg.txBackoff;
    inst.factoryCalibCfg_flashOffset = res.factoryCalibCfg.flashOffset;

    inst.compressionCfg_enabled = res.compressionCfg.enabled;
    inst.compressionCfg_ratio = res.compressionCfg.ratio;

    inst.sensorStart_frameTrigMode = res.sensorStart.frameTrigMode;
    inst.sensorStart_chirpStartSigLbEn = res.sensorStart.chirpStartSigLbEn;
    inst.sensorStart_frameLivMonEn = res.sensorStart.frameLivMonEn;
    inst.sensorStart_frameTrigTimerVal = res.sensorStart.frameTrigTimerVal;
}

let mmwave_demo_module_name = "/demo/mmwave_demo/mmwave_demo";

let mmwave_demo_module = {
    displayName: "MMWAVE_DEMO",
    templates: {
        "/demo/demo/cli_mmwave_demo_config.h.xdt": {
            cli_mmwave_demo_config: "/demo/mmwave_demo/templates/cli_mmwave_demo_config.h.xdt",
        },
    },

    defaultInstanceName: "CONFIG_MMWAVE_DEMO",
    maxInstances: 1,
    config:  getConfigurables(),
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceConfig,
    getInterfaceName,
};

function getModule()
{
    let module = mmwave_demo_module;
    return module;
}

exports = getModule();
