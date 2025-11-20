var EXAMPLES_MMWDEMOS =
[
    [ "Motion and Presence Detection OOB Demo", "MOTION_AND_PRESENCE_DETECTION_DEMO.html", [
      [ "Purpose and Scope", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md816", null ],
      [ "Supported Combinations", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#MOTION_AND_PRESENCE_DETECTION_DEMO_COMBOS", null ],
      [ "Supported Features", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md817", null ],
      [ "Using SysConfig", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md818", [
        [ "Custom Demo Code Options", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md819", [
          [ "Steps to enable/disable the supported features", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md820", null ],
          [ "CLI Removal Feature", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md821", null ]
        ] ]
      ] ],
      [ "Motion Detection Demo Setup", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md822", null ],
      [ "MIMO Modulation Schemes", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md823", null ],
      [ "Chirp Configuration Modes", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md824", null ],
      [ "Antenna Geometry", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md825", null ],
      [ "Signal Processing Chain", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md826", [
        [ "Motion Detection Modes", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md827", null ],
        [ "Dynamic Reconfig", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md828", null ]
      ] ],
      [ "Variable SNR detection thresholds", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#RANGE_ANGLE_SNR", [
        [ "SNR vs Range - Feature Description", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md829", null ],
        [ "SNR vs Angle - Feature Description", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md830", null ]
      ] ],
      [ "Rx Channel Gain/Offset Measurement and Compensation", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md831", [
        [ "Measurement Procedure Implementation", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md832", null ]
      ] ],
      [ "System execution flow", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md833", [
        [ "Initialization and Configuration", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md834", null ],
        [ "Steady state – low power mode disabled", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md835", null ],
        [ "Steady state – low power mode enabled", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md836", null ]
      ] ],
      [ "Task Model", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md837", [
        [ "Main Task", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md838", null ],
        [ "CLI Task", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md839", null ],
        [ "DPC Task", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md840", null ],
        [ "UART Task", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md841", null ],
        [ "ADC read data Task", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md842", null ],
        [ "Power Management Task", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md843", null ],
        [ "Timing Diagram", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md844", null ]
      ] ],
      [ "Benchmarks", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md845", null ],
      [ "Configuration (.cfg) File Format (CLI INTERFACE)", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md846", [
        [ "Sensor front-end parameters", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md847", null ],
        [ "Detection layer parameters", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md848", null ],
        [ "Motion/presence detection layer", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md849", null ],
        [ "Tracking layer parameters", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md850", null ],
        [ "Classification layer parameters", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md851", null ],
        [ "Profile switching parameters", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md852", null ]
      ] ],
      [ "UART and Output to the Host", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md853", [
        [ "Output TLV Description", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md854", [
          [ "Frame Header Structure", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md855", null ],
          [ "TLV Structure", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md856", null ],
          [ "Point Cloud TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md857", [
            [ "Floating point format", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md858", null ],
            [ "Fix-point format", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md861", null ]
          ] ],
          [ "Range Profile TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md862", null ],
          [ "Detection heatmap TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md863", null ],
          [ "Presence Detection TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md864", null ],
          [ "Stats TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md865", null ],
          [ "Group tracker data TLVs", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md866", [
            [ "List of tracking objects TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md867", null ],
            [ "Group Tracker indices array TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md868", null ]
          ] ],
          [ "Micro-Doppler TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md869", null ],
          [ "Features TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md870", null ],
          [ "Classifier Output TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md871", null ],
          [ "Quick-Eval TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md872", null ],
          [ "Rx channel compensation measurement output TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md873", null ],
          [ "Antenna symbols TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md874", null ]
        ] ]
      ] ],
      [ "Low Power Configuration (lowPowerCfg = 1)", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md875", [
        [ "Flow Diagram", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md876", null ],
        [ "Threshold and Latency time duration for Power module", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md877", null ]
      ] ],
      [ "Raw ADC data streaming", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md878", [
        [ "Steps for DCA based streaming", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md879", [
          [ "DCA1000EVM Setup", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md880", null ],
          [ "Power on/off sequence", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md881", null ],
          [ "Steps to Perform Raw ADC Data Streaming", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md882", null ],
          [ "Validation of the ADC data", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md883", null ]
        ] ],
        [ "SPI based streaming of Raw ADC Data", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md884", [
          [ "Building the demo with this feature enabled", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md885", null ],
          [ "Steps to Perform Raw ADC Data Streaming on xWRL6432 FCCSP device", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md886", null ],
          [ "Steps to Perform Raw ADC Data Streaming on xWRL6432 AOP device:", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md887", null ],
          [ "Validation of the ADC data", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md888", null ]
        ] ]
      ] ],
      [ "Steering Vector Tool Usage", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md889", [
        [ "Running the Tool", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md890", null ],
        [ "Input Parameters", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md891", null ],
        [ "Output", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md892", null ],
        [ "Examples", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md893", [
          [ "Example 1: Basic Usage", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md894", null ],
          [ "Example 2: Custom Spacing", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md895", null ]
        ] ],
        [ "Troubleshooting", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md896", null ]
      ] ],
      [ "Running in test mode", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md897", null ],
      [ "Steps to Run Monitors", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md898", null ],
      [ "Steps to Run the Example", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md899", null ],
      [ "Power Measurements with INA228", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md900", null ],
      [ "Memory Usage", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md901", null ]
    ] ],
    [ "Gesture Recognition Demo", "GESTURE_RECOGNITION_DEMO.html", [
      [ "Supported Combinations", "GESTURE_RECOGNITION_DEMO.html#GESTURE_RECOGNITION_DEMO_COMBOS", null ],
      [ "Using SDK with SysConfig", "GESTURE_RECOGNITION_DEMO.html#autotoc_md915", null ],
      [ "Steps to Run the Example", "GESTURE_RECOGNITION_DEMO.html#autotoc_md916", null ],
      [ "Configuration (.cfg) File Format (CLI INTERFACE)", "GESTURE_RECOGNITION_DEMO.html#autotoc_md917", [
        [ "Sensor front-end parameters", "GESTURE_RECOGNITION_DEMO.html#autotoc_md918", null ],
        [ "Signal Processing layer parameters", "GESTURE_RECOGNITION_DEMO.html#autotoc_md919", null ],
        [ "Presence Detect configuration parameters", "GESTURE_RECOGNITION_DEMO.html#autotoc_md920", null ]
      ] ],
      [ "Flow Diagram in Low Power Configuration (lowPowerCfg = 1)", "GESTURE_RECOGNITION_DEMO.html#autotoc_md921", null ]
    ] ],
    [ "Mmwave Demo", "MMWAVE_DEMO.html", [
      [ "Purpose and Scope", "MMWAVE_DEMO.html#autotoc_md922", null ],
      [ "Supported Combinations", "MMWAVE_DEMO.html#MMWAVE_DEMO_COMBOS", null ],
      [ "Supported Features", "MMWAVE_DEMO.html#autotoc_md923", null ],
      [ "Using SysConfig", "MMWAVE_DEMO.html#autotoc_md924", [
        [ "Custom Demo Code Options", "MMWAVE_DEMO.html#autotoc_md925", [
          [ "Steps to enable/disable the supported features", "MMWAVE_DEMO.html#autotoc_md926", null ],
          [ "CLI Removal Feature", "MMWAVE_DEMO.html#autotoc_md927", null ]
        ] ]
      ] ],
      [ "MMWAVE Demo Setup", "MMWAVE_DEMO.html#autotoc_md928", null ],
      [ "MIMO Modulation Schemes", "MMWAVE_DEMO.html#autotoc_md929", null ],
      [ "Antenna Geometry", "MMWAVE_DEMO.html#autotoc_md930", null ],
      [ "Signal Processing Chain", "MMWAVE_DEMO.html#autotoc_md931", [
        [ "Data Compression", "MMWAVE_DEMO.html#autotoc_md932", [
          [ "Range processing DPU", "MMWAVE_DEMO.html#autotoc_md933", null ],
          [ "Doppler Processing DPU", "MMWAVE_DEMO.html#autotoc_md934", null ],
          [ "AoA2D DPU", "MMWAVE_DEMO.html#autotoc_md935", null ]
        ] ]
      ] ],
      [ "Rx Channel Gain/Offset Measurement and Compensation", "MMWAVE_DEMO.html#autotoc_md936", [
        [ "Measurement Procedure Implementation", "MMWAVE_DEMO.html#autotoc_md937", null ]
      ] ],
      [ "System execution flow", "MMWAVE_DEMO.html#autotoc_md938", null ],
      [ "Task Model", "MMWAVE_DEMO.html#autotoc_md939", [
        [ "Main Task", "MMWAVE_DEMO.html#autotoc_md940", null ],
        [ "CLI Task", "MMWAVE_DEMO.html#autotoc_md941", null ],
        [ "DPC Task", "MMWAVE_DEMO.html#autotoc_md942", null ],
        [ "UART Task", "MMWAVE_DEMO.html#autotoc_md943", null ],
        [ "ADC read data Task", "MMWAVE_DEMO.html#autotoc_md944", null ],
        [ "Power Management Task", "MMWAVE_DEMO.html#autotoc_md945", null ],
        [ "Timing Diagram", "MMWAVE_DEMO.html#autotoc_md946", null ]
      ] ],
      [ "Memory Usage", "MMWAVE_DEMO.html#autotoc_md947", null ],
      [ "Benchmarks", "MMWAVE_DEMO.html#autotoc_md948", null ],
      [ "Configuration (.cfg) File Format (CLI INTERFACE)", "MMWAVE_DEMO.html#autotoc_md949", [
        [ "Sensor front-end parameters", "MMWAVE_DEMO.html#autotoc_md950", null ],
        [ "Detection layer parameters", "MMWAVE_DEMO.html#autotoc_md951", null ]
      ] ],
      [ "UART and Output to the Host", "MMWAVE_DEMO.html#autotoc_md952", [
        [ "Output TLV Description", "MMWAVE_DEMO.html#autotoc_md953", [
          [ "Frame Header Structure", "MMWAVE_DEMO.html#autotoc_md954", null ],
          [ "TLV Structure", "MMWAVE_DEMO.html#autotoc_md955", null ],
          [ "Point Cloud TLV", "MMWAVE_DEMO.html#autotoc_md956", [
            [ "Floating point format", "MMWAVE_DEMO.html#autotoc_md957", null ],
            [ "Fix-point format", "MMWAVE_DEMO.html#autotoc_md960", null ]
          ] ],
          [ "Range Profile TLV", "MMWAVE_DEMO.html#autotoc_md961", null ],
          [ "Range-Doppler Heatmap TLV", "MMWAVE_DEMO.html#autotoc_md962", null ],
          [ "Stats TLV", "MMWAVE_DEMO.html#autotoc_md963", null ],
          [ "Rx channel compensation measurement output TLV", "MMWAVE_DEMO.html#autotoc_md964", null ]
        ] ]
      ] ],
      [ "Low Power Configuration (lowPowerCfg = 1)", "MMWAVE_DEMO.html#autotoc_md965", [
        [ "Flow Diagram", "MMWAVE_DEMO.html#autotoc_md966", null ],
        [ "Threshold and Latency time duration for Power module", "MMWAVE_DEMO.html#autotoc_md967", null ]
      ] ],
      [ "Raw ADC data streaming", "MMWAVE_DEMO.html#autotoc_md968", [
        [ "Steps for DCA based streaming", "MMWAVE_DEMO.html#autotoc_md969", [
          [ "DCA1000EVM Setup", "MMWAVE_DEMO.html#autotoc_md970", null ],
          [ "Power on/off sequence", "MMWAVE_DEMO.html#autotoc_md971", null ],
          [ "Steps to Perform Raw ADC Data Streaming", "MMWAVE_DEMO.html#autotoc_md972", null ],
          [ "Validation of the ADC data", "MMWAVE_DEMO.html#autotoc_md973", null ]
        ] ],
        [ "SPI based streaming of Raw ADC Data", "MMWAVE_DEMO.html#autotoc_md974", [
          [ "Building the demo with this feature enabled", "MMWAVE_DEMO.html#autotoc_md975", null ],
          [ "Steps to Perform Raw ADC Data Streaming on xWRL6432 FCCSP device:", "MMWAVE_DEMO.html#autotoc_md976", null ],
          [ "Steps to Perform Raw ADC Data Streaming on xWRL6432 AOP device:", "MMWAVE_DEMO.html#autotoc_md977", null ],
          [ "Validation of the ADC data", "MMWAVE_DEMO.html#autotoc_md978", null ]
        ] ]
      ] ],
      [ "Running in test mode", "MMWAVE_DEMO.html#autotoc_md979", null ],
      [ "Steps to Run Monitors", "MMWAVE_DEMO.html#autotoc_md980", null ],
      [ "Steps to Run the Example", "MMWAVE_DEMO.html#autotoc_md981", null ],
      [ "Power Measurements with INA228", "MMWAVE_DEMO.html#autotoc_md982", null ]
    ] ],
    [ "Monitors", "MONITORS.html", [
      [ "Steps to run Monitors", "MONITORS.html#autotoc_md902", null ],
      [ "Steps to run Live Monitors", "MONITORS.html#autotoc_md903", null ],
      [ "Configuration (.cfg) File Format (CLI INTERFACE)", "MONITORS.html#autotoc_md904", [
        [ "Monitor parameters", "MONITORS.html#autotoc_md905", null ]
      ] ]
    ] ]
];