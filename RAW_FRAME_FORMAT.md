# CubeEye I200D Raw Frame Format Analysis

## Frame Structure

**Total Size:** 771,200 bytes = 241 lines × 3200 bytes/line

```
Line 0:     Header (embedded data, 3200 bytes)
Lines 1-240: Pixel data (240 lines × 320 pixels × 10 bytes/pixel)
```

## Header Line (Line 0)

First 32 bytes example: `02000003f5000000037920003c00180f0000000303030303ff19073d08280000`

Contains:
- Mode/config bytes
- Temperature data
- Calibration info
- Frame counter

## Pixel Format (10 bytes per pixel)

Each output pixel is represented by 10 raw bytes (5 × uint16 sub-pixels):

| Sub-pixel | Bytes | Description |
|-----------|-------|-------------|
| Sub[0] | 0-1 | Amplitude low (uint16 LE) |
| Sub[1] | 2-3 | Amplitude low (same as Sub[0]) |
| Sub[2] | 4-5 | (Amplitude << 8) | Depth_low_byte |
| Sub[3] | 6-7 | Amplitude << 8 |
| Sub[4] | 8-9 | Overflow/cycle counter |

### Observed Pattern

For a pixel with amplitude `A` and raw depth component `D`:

```
Sub[0] = A (0-511 range)
Sub[1] = A (identical to Sub[0])
Sub[2] = (A << 8) | D  (where D = Sub[2] & 0xFF)
Sub[3] = A << 8
Sub[4] = Cycle/overflow indicator
```

### Sample Data

```
Pixel [0,0]:
  Sub[0]:     8       Sub[2] high=8, low=48   Sub[4]: 8448

Pixel [60,80]:
  Sub[0]:     4       Sub[2] high=4, low=19   Sub[4]: 256

Pixel [120,160]:
  Sub[0]:    44       Sub[2] high=44, low=35  Sub[4]: 256
  (SDK depth ≈ 3200mm)
```

## Depth Extraction Hypothesis

The depth appears to be encoded as:
- **8-bit phase component** in Sub[2] low byte
- **Cycle counter** in Sub[4] (for depths > single phase wrap)

**Tentative formula:**
```python
phase_raw = Sub[2] & 0xFF
cycle = Sub[4] >> 8  # or some function of Sub[4]
depth = phase_raw + cycle * wavelength_mm
```

Where wavelength depends on modulation frequency (81 MHz → ~1850mm max unambiguous range)

## The Paradox

**Problem:** SDK outputs 3200mm without calling phase unwrap functions.

**Possible explanations:**

1. **Sensor performs on-chip unwrapping** using multiple frequencies internally
2. **The raw frame already contains pre-computed depth** in a packed format
3. **The cycle counter (Sub[4])** provides the extended range information

## Next Steps to Resolve

1. **Capture synchronized raw + SDK depth** for exact correlation
2. **Test if depth = Sub[2]&0xFF + (Sub[4]>>8) * wavelength**
3. **Check if different distances produce different Sub[4] values**
4. **Examine SDK gradient table to understand the correction applied**

## SDK Processing Pipeline (Confirmed)

```
Raw Frame (771,200 bytes)
    │
    ▼
[readFrameProc] - Reads from V4L2/USB
    │
    ▼
[imageProcessProc] - Main processing (NO unwrapping called)
    │
    ├── frame_cast_basic16u() - Just validates, no transformation
    ├── FlyingPixelFilter2() - Removes edge artifacts
    ├── Gradient table lookup at offset 0x468 - Polynomial correction
    └── Depth range filter - Clips values
    │
    ▼
Depth Output (640×480 uint16, 2x interpolated)
```

## Key Finding

The SDK's `ChineseReminderDualDepthUnwrapper::unwrap` function **EXISTS** in the library but is **NOT CALLED** during normal frame processing. This strongly suggests:

1. The sensor delivers already-unwrapped depth in the raw frame
2. The 5-subpixel format contains pre-computed depth data
3. The SDK only applies post-processing corrections (gradient table, filters)

## Statistics from Actual Capture

```
Sub[0]: min=0, max=511,   mean=58.9   (amplitude)
Sub[1]: min=0, max=508,   mean=58.9   (amplitude duplicate)
Sub[2]: min=0, max=65534, mean=15061  (packed amplitude+depth)
Sub[3]: min=0, max=65280, mean=15020  (amplitude << 8)
Sub[4]: min=0, max=65280, mean=15856  (cycle/control)
```
