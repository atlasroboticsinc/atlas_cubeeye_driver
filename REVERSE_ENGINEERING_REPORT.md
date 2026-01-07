# CubeEye I200D Reverse Engineering Report

**Date:** 2026-01-06
**Status:** Major breakthrough achieved

## Executive Summary

Successfully reverse-engineered the CubeEye I200D sensor communication protocol and discovered the actual raw data format. Key breakthroughs include:

1. **Identified UVC XU control structure** for sensor configuration
2. **Discovered data packing scheme** - Sub[3] and Sub[4] are 8-bit values in high byte
3. **Created working tools** for probing and configuring the sensor
4. **Confirmed calibration approach** using Sub[2] as primary depth data

---

## 1. Hardware Configuration

### Device Identification
- **USB ID:** 3674:0200 (meere company)
- **UVC Version:** 1.10
- **Frame Format:** 1600x241 YUYV (771,200 bytes)
- **Extension Unit ID:** 3
- **Available Controls:** Selectors 1-5

### UVC Extension Unit Controls

| Selector | Size | Purpose |
|----------|------|---------|
| 1 | 20 bytes | Device info query (serial, cal ver, fw ver) |
| 2 | 20 bytes | Register access (read/write sensor registers) |
| 3 | 263 bytes | Large data block (calibration write?) |
| 4 | 260 bytes | Large data block (calibration read?) |
| 5 | 8 bytes | Short command/status |

### Discovered Command Format

**Selector 1 (Info Query):**
```
Byte[0]: 0x02 (query command)
Byte[3]: Query type
  - 0x00: Status
  - 0x01: Serial number (ASCII)
  - 0x02: Calibration version
  - 0x03: Firmware version
```

**Device Info Captured:**
- Serial: `I200DU2509000349`
- Cal Version: `0x06D3` (1747)
- FW Version: `0x03`

---

## 2. Raw Data Format Breakthrough

### Sub-pixel Layout (1600 pixels per row → 320 spatial pixels × 5 sub-pixels)

| Channel | Bits Used | Value Range | Description |
|---------|-----------|-------------|-------------|
| Sub[0] | 9 bits | 0-509 | Black level reference 1 |
| Sub[1] | 9 bits | 0-509 | Black level reference 2 |
| Sub[2] | 16 bits | 0-65520 | **Primary depth/intensity data** |
| Sub[3] | 8 bits (HIGH) | 0-255 × 256 | Secondary data (99.8% mod 256) |
| Sub[4] | 8 bits (HIGH) | 0-255 × 256 | Amplitude (99.8% mod 256) |

### Critical Discovery

**Sub[3] and Sub[4] are 8-bit values stored in the high byte!**

```cpp
// Wrong (what we were doing):
uint16_t amp = raw_sub4;  // Treating as 16-bit

// Correct:
uint8_t amp = raw_sub4 >> 8;  // Extract 8-bit value from high byte
```

This explains why our calibration had errors - we were treating 8-bit quantized values as 16-bit.

---

## 3. Tools Created

### probe_xu
Queries all UVC XU controls and displays their values.
```bash
sudo ./build/probe_xu /dev/video0
```

### set_mode
Sets sensor mode via XU Selector 1.
```bash
sudo ./build/set_mode /dev/video0 <mode>
```

### reg_access
Read/write sensor registers via XU Selector 2.
```bash
sudo ./build/reg_access /dev/video0 read <addr>
sudo ./build/reg_access /dev/video0 write <addr> <value>
```

### v4l2_hook (Updated)
Now captures UVC XU commands with data payloads.
```bash
LD_PRELOAD=./build/libv4l2_hook.so ./sdk_app
```

---

## 4. SDK Traffic Analysis

Captured actual SDK initialization sequence:

```
# Device identification
Selector 1: 02 00 00 01  → Serial number query
Selector 1: 02 00 00 02  → Cal version query
Selector 1: 02 00 00 03  → FW version query

# Sensor configuration
Selector 2: 00 04 00 81  → Read/config register 0x0004
Selector 2: 00 10 00 0d  → Read/config register 0x0010
Selector 2: 01 01 00 d0  → Write 0xD0 to register 0x0001

# Stream control
Selector 2: 01 02 94 00 01  → Enable streaming
Selector 2: 01 02 94 00 00  → Disable streaming

# Status
Selector 5: 01 80 00 16 XX  → Status command (byte 4 toggles)
```

---

## 5. Calibration Status

### Current Calibration (3-point ruler test)
Using Sub[2] as primary data, calibrated at 0.5m, 1.0m, 1.5m:

| Distance | Sub[2] Mean | Predicted | Error |
|----------|-------------|-----------|-------|
| 500mm | 25,038 | 462mm | -38mm |
| 1000mm | 30,456 | 1,160mm | +160mm |
| 1500mm | 32,152 | 1,378mm | -122mm |

**MAE: ~107mm**

### Why the Error?
We were averaging Sub[2] with the 8-bit-quantized Sub[3], which added noise.

### Next Calibration Step
Use only Sub[2] (full 16-bit precision) and the correctly extracted Sub[4]>>8 for amplitude filtering.

---

## 6. Phase Mode Investigation

### Finding
The sensor does NOT appear to output raw I/Q phase data in its current mode:
- Sub[2] ≈ Sub[3] (after correcting for 8-bit packing)
- Phase calculations give constant ~45° regardless of depth
- This suggests pre-processed depth encoding, not raw phase

### Possible Explanations
1. Sensor configured in "depth output" mode rather than "raw phase" mode
2. SDK performs internal phase-to-depth conversion before output
3. Would need to find the mode switch command in Selector 2/4

---

## 7. Files Modified/Created

### Source Files
- `src/depth_processor.h` - Updated with correct data format documentation
- `src/depth_processor.cpp` - Three processing methods
- `src/v4l2_hook.c` - Added UVC XU command logging
- `src/probe_xu.c` - NEW: UVC XU control probe tool
- `src/set_mode.c` - NEW: Mode setting tool
- `src/reg_access.c` - NEW: Register access tool

### Analysis Scripts
- `deep_analysis.py` - Comprehensive data analysis
- `analyze_subpixels.py` - Bit pattern and phase analysis
- `validate_models.py` - Calibration model validation
- `capture_usb.sh` - USB traffic capture script

---

## 8. Recommended Next Steps

### Immediate (High Priority)
1. **Re-calibrate using only Sub[2]** - Don't average with quantized Sub[3]
2. **Use Sub[4]>>8 for amplitude filtering** - Correct 8-bit extraction
3. **Capture more calibration points** - 0.25m intervals from 0.3m to 2.0m

### Investigation (Medium Priority)
4. **Explore Selector 3/4** - Large data blocks may contain calibration tables
5. **Find raw phase mode command** - May need to write specific register values
6. **Dump calibration data** - Use `reg_access` to read sensor EEPROM

### Long-term
7. **Implement proper phase calculation** - If raw mode can be enabled
8. **Validate against SDK output** - Fix benchmark_capture to get valid SDK depth
9. **Create ROS2 driver** - Based on working depth processor

---

## 9. Technical Notes

### UVC XU Query Types
```c
#define UVC_SET_CUR  0x01
#define UVC_GET_CUR  0x81
#define UVC_GET_MIN  0x82
#define UVC_GET_MAX  0x83
#define UVC_GET_RES  0x84
#define UVC_GET_LEN  0x85
#define UVC_GET_INFO 0x86
#define UVC_GET_DEF  0x87
```

### IOCTL Code
```c
#define UVCIOC_CTRL_QUERY  _IOWR('u', 0x21, struct uvc_xu_control_query)
// In hex: 0xC0107521
```

---

## Conclusion

This reverse engineering session made significant progress:

✅ Identified the correct UVC XU control structure
✅ Discovered the 8-bit packing in Sub[3]/Sub[4]
✅ Created tools for direct sensor access
✅ Captured real SDK traffic patterns
⏳ Calibration accuracy needs improvement with correct data extraction
⏳ Raw phase mode access still under investigation

The depth processor should now use **Sub[2] directly** (not averaged with Sub[3]) and extract amplitude as **Sub[4] >> 8**.
