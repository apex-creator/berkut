# generateSteeringVectors - v1.0

The `generateSteeringVectors.exe` tool at **${SDK_INSTALL_PATH}/tools/steeringvector_generation_tool/v0** processes a CLI configuration file (`.cfg`) and generates steering vectors corresponding to the start frequency and bandwidth specified in the input configuration file.

---

## 🛠️ Running the Tool

1. Open Command Prompt 
2. Navigate to the folder containing the executable (**${SDK_INSTALL_PATH}/tools/steeringvector_generation_tool/v0**)
3. Run with arguments:

```
generateSteeringVectors.exe --filename "path\to\config.cfg" --azimSpacing 2.5 --elevSpacing 5
```

> **Note:** If the path contains spaces, enclose it in quotes as shown above.

---

## 🔧 Input Parameters

| Parameter | Required | Default | Description |
|------------------|----------|---------|-----------------------------------------------------------------------------|
| `--filename` | Yes | — | Path to the CLI configuration file. |
| `--azimSpacing` | No | 2.5 | Angular spacing (in degrees) between azimuth steering vectors. |
| `--elevSpacing` | No | 5 | Angular spacing (in degrees) between elevation steering vectors. |

---

## Output

On successful execution, the tool generates a `.c` file containing the steering vectors for the configured input.  
The output file is named as: `aoasv_table_6432.c` and is saved in the same location as the tool. Copy the contents of this file to **${SDK_INSTALL_PATH}/source/datapath/dpu/aossvcproc/v0/aoasv_table.c**, rebuild the datapath libraries and application.
---

## 📘 Examples

### Example 1: Basic Usage

```
generateSteeringVectors.exe --filename "config.cfg"
```

### Example 2: Custom Spacing

```
generateSteeringVectors.exe --filename "config.cfg" --azimSpacing 5 --elevSpacing 5
```

---

## ❗ Troubleshooting

- If you encounter **path-related errors**, use **absolute paths** for the config file.
- Ensure the `.cfg` file is properly formatted.
- For **relative paths**, ensure they are relative to the **location of the executable**.

---