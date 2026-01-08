#!/usr/bin/env python3
"""
Extract FPPN tables from calibration page dump.
FPPN is 320x240 int16 values per frequency = 153,600 bytes.
"""

import os
import struct
import numpy as np

CAL_DIR = "./cal_scan_output"
OUTPUT_DIR = "./extracted_calibration"

def read_page(page_num):
    """Read a calibration page file."""
    filename = f"{CAL_DIR}/page_{page_num:04x}.bin"
    if not os.path.exists(filename):
        return None
    with open(filename, "rb") as f:
        data = f.read()
    # Skip 4-byte header (00 20 HH LL) and 2-byte data header (01 00)
    return data[6:]  # Return data starting at byte 6

def analyze_page_ranges():
    """Find contiguous page ranges that might be FPPN."""
    pages = []
    for f in os.listdir(CAL_DIR):
        if f.startswith("page_") and f.endswith(".bin"):
            page_num = int(f[5:9], 16)
            pages.append(page_num)
    pages.sort()

    # Find ranges with phase-like data (values around 0xfd00 = -768)
    fppn_pages = []
    for p in pages:
        data = read_page(p)
        if data and len(data) >= 10:
            # Check if values look like FPPN (16-bit values in range 0xfa00-0xff00)
            vals = struct.unpack(">5H", data[:10])  # Big-endian uint16
            if all(0xf800 <= v <= 0xffff or v < 0x0100 for v in vals):
                fppn_pages.append(p)

    return pages, fppn_pages

def extract_fppn(page_start, num_pages, output_name):
    """Extract FPPN data from consecutive pages."""
    fppn_data = bytearray()

    for i in range(num_pages):
        page_num = page_start + i * 2  # Pages seem to be even-numbered
        data = read_page(page_num)
        if data:
            fppn_data.extend(data)
        else:
            print(f"  Missing page 0x{page_num:04X}")
            fppn_data.extend(b'\x00' * 254)  # Pad with zeros

    # Save raw data
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    output_path = f"{OUTPUT_DIR}/{output_name}.bin"
    with open(output_path, "wb") as f:
        f.write(fppn_data)
    print(f"Saved {len(fppn_data)} bytes to {output_path}")

    # Convert to numpy array for analysis
    # FPPN values are big-endian int16
    if len(fppn_data) >= 153600:
        fppn_array = np.frombuffer(fppn_data[:153600], dtype='>i2')  # Big-endian int16
        fppn_2d = fppn_array.reshape(240, 320)

        # Save as numpy file
        np_path = f"{OUTPUT_DIR}/{output_name}.npy"
        np.save(np_path, fppn_2d)
        print(f"Saved numpy array (240x320) to {np_path}")

        # Print stats
        print(f"  Min: {fppn_2d.min()}, Max: {fppn_2d.max()}, Mean: {fppn_2d.mean():.1f}")
        return fppn_2d

    return None

def main():
    print("FPPN Extraction Tool")
    print("=" * 40)

    all_pages, fppn_pages = analyze_page_ranges()
    print(f"Total pages: {len(all_pages)}")
    print(f"FPPN-like pages: {len(fppn_pages)}")

    if fppn_pages:
        print(f"\nFPPN page ranges:")
        # Group into contiguous ranges
        ranges = []
        start = fppn_pages[0]
        prev = start
        for p in fppn_pages[1:]:
            if p - prev > 4:  # Gap in pages
                ranges.append((start, prev))
                start = p
            prev = p
        ranges.append((start, prev))

        for i, (s, e) in enumerate(ranges):
            count = (e - s) // 2 + 1
            print(f"  Range {i+1}: 0x{s:04X} - 0x{e:04X} ({count} pages, ~{count*254} bytes)")

    # Extract FPPN data
    # Based on the scan, FPPN1 appears to start at 0x021A and FPPN2 at 0x031A
    print("\n" + "=" * 40)
    print("Extracting FPPN tables...")

    # FPPN needs 153600 bytes = 600 pages of 256 bytes
    # But our pages are 254 bytes of data each, so need ~605 pages

    # Try extracting from page 0x021A onwards
    print("\nFPPN1 (starting at page 0x021A):")
    fppn1 = extract_fppn(0x021A, 303, "fppn1")  # 303 even pages needed for 76800 int16 values...

    # The pages might not be exactly 600 consecutive
    # Let me try a different approach - combine all FPPN-like pages

    print("\nCombining all FPPN-like pages:")
    all_fppn = bytearray()
    for p in sorted(fppn_pages):
        data = read_page(p)
        if data:
            all_fppn.extend(data)

    output_path = f"{OUTPUT_DIR}/fppn_combined.bin"
    with open(output_path, "wb") as f:
        f.write(all_fppn)
    print(f"Saved {len(all_fppn)} bytes combined FPPN data")

    # Analyze the combined data
    if len(all_fppn) >= 153600:
        fppn_array = np.frombuffer(all_fppn[:153600], dtype='>i2')
        fppn_2d = fppn_array.reshape(240, 320)
        np.save(f"{OUTPUT_DIR}/fppn_combined.npy", fppn_2d)
        print(f"Combined FPPN: Min={fppn_2d.min()}, Max={fppn_2d.max()}, Mean={fppn_2d.mean():.1f}")

if __name__ == "__main__":
    main()
