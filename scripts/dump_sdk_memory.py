#!/usr/bin/env python3
"""
Dump SDK memory to find exact FPPN array.

This script attaches to a running SDK process and searches for FPPN data.
Can also be used to compare our extracted FPPN with what SDK has in memory.

Usage:
  1. Start benchmark_capture in background: ./benchmark_capture 100 &
  2. Get PID: pgrep benchmark_capture
  3. Run this script: sudo python3 dump_sdk_memory.py <PID>
"""

import sys
import os
import struct
import re
from pathlib import Path

# FPPN characteristics
WIDTH = 320
HEIGHT = 240
FPPN_SIZE = WIDTH * HEIGHT * 2  # 153600 bytes
FPPN_VALUE_MIN = -950
FPPN_VALUE_MAX = -450

OUTPUT_DIR = Path("/home/cmericli/development/atlas/code/cubeeye_nano_driver/sdk_fppn_dump")


def parse_maps(pid):
    """Parse /proc/pid/maps to get memory regions."""
    regions = []
    with open(f"/proc/{pid}/maps", "r") as f:
        for line in f:
            match = re.match(r'([0-9a-f]+)-([0-9a-f]+)\s+(\S+)\s+\S+\s+\S+\s+\S+\s*(.*)', line)
            if match:
                start = int(match.group(1), 16)
                end = int(match.group(2), 16)
                perms = match.group(3)
                path = match.group(4).strip()
                regions.append({
                    'start': start,
                    'end': end,
                    'perms': perms,
                    'path': path
                })
    return regions


def is_fppn_value(val, endian='<'):
    """Check if value is in FPPN range."""
    return FPPN_VALUE_MIN < val < FPPN_VALUE_MAX


def search_region(mem_fd, start, end, endian='<'):
    """Search a memory region for FPPN array."""
    results = []

    try:
        mem_fd.seek(start)
        # Read in chunks
        chunk_size = 1024 * 1024  # 1MB
        offset = 0
        buffer = b''

        while start + offset < end:
            read_size = min(chunk_size, end - start - offset)
            try:
                mem_fd.seek(start + offset)
                data = mem_fd.read(read_size)
                if not data:
                    break
                buffer = buffer[-FPPN_SIZE:] + data  # Keep overlap for array spanning chunks

                # Search for FPPN pattern in buffer
                found = search_fppn_in_buffer(buffer, start + offset - len(buffer) + len(data), endian)
                results.extend(found)

                offset += read_size
            except OSError:
                offset += 4096  # Skip unreadable page

    except Exception as e:
        pass

    return results


def search_fppn_in_buffer(data, base_addr, endian='<'):
    """Search buffer for FPPN-like arrays."""
    results = []

    if len(data) < FPPN_SIZE:
        return results

    # Quick scan: look for runs of FPPN-like values
    i = 0
    while i < len(data) - FPPN_SIZE:
        # Quick check: first 10 values
        quick_count = 0
        for j in range(10):
            if i + j*2 + 1 >= len(data):
                break
            val = struct.unpack(f'{endian}h', data[i+j*2:i+j*2+2])[0]
            if is_fppn_value(val):
                quick_count += 1

        if quick_count >= 8:  # 80% of first 10 values
            # Full check
            fppn_count = 0
            total = FPPN_SIZE // 2

            for j in range(total):
                if i + j*2 + 1 >= len(data):
                    break
                val = struct.unpack(f'{endian}h', data[i+j*2:i+j*2+2])[0]
                if is_fppn_value(val):
                    fppn_count += 1

            coverage = 100.0 * fppn_count / total
            if coverage > 95:
                addr = base_addr + i
                results.append({
                    'address': addr,
                    'coverage': coverage,
                    'data': data[i:i+FPPN_SIZE],
                    'endian': endian
                })
                i += FPPN_SIZE  # Skip past this match
            else:
                i += 2
        else:
            i += 2

    return results


def analyze_fppn(data, endian='<'):
    """Analyze FPPN data."""
    values = []
    for i in range(0, len(data), 2):
        val = struct.unpack(f'{endian}h', data[i:i+2])[0]
        values.append(val)

    fppn_vals = [v for v in values if is_fppn_value(v)]
    non_fppn = [(i, v) for i, v in enumerate(values) if not is_fppn_value(v)]

    print(f"  Total values: {len(values)}")
    print(f"  FPPN-like: {len(fppn_vals)} ({100*len(fppn_vals)/len(values):.1f}%)")

    if fppn_vals:
        mean = sum(fppn_vals) / len(fppn_vals)
        print(f"  Min: {min(fppn_vals)}, Max: {max(fppn_vals)}, Mean: {mean:.1f}")

    if non_fppn:
        print(f"  Non-FPPN positions ({len(non_fppn)}):")
        for idx, val in non_fppn[:20]:
            row = idx // WIDTH
            col = idx % WIDTH
            print(f"    [{row},{col}]: {val}")


def compare_with_extracted(sdk_data, extracted_path, endian='<'):
    """Compare SDK FPPN with our extracted FPPN."""
    if not os.path.exists(extracted_path):
        print(f"  Extracted file not found: {extracted_path}")
        return

    with open(extracted_path, "rb") as f:
        extracted = f.read()

    if len(extracted) != len(sdk_data):
        print(f"  Size mismatch: SDK={len(sdk_data)}, Extracted={len(extracted)}")
        return

    # Compare byte by byte
    differences = []
    for i in range(0, len(sdk_data), 2):
        sdk_val = struct.unpack(f'{endian}h', sdk_data[i:i+2])[0]
        ext_val = struct.unpack(f'{endian}h', extracted[i:i+2])[0]

        if sdk_val != ext_val:
            idx = i // 2
            row = idx // WIDTH
            col = idx % WIDTH
            differences.append({
                'index': idx,
                'row': row,
                'col': col,
                'sdk': sdk_val,
                'extracted': ext_val,
                'diff': sdk_val - ext_val
            })

    if not differences:
        print("  EXACT MATCH! SDK and extracted FPPN are identical!")
    else:
        print(f"  Found {len(differences)} differences:")
        for d in differences[:30]:
            print(f"    [{d['row']:3d},{d['col']:3d}]: SDK={d['sdk']:5d}, Ext={d['extracted']:5d}, Diff={d['diff']:+5d}")

        if len(differences) > 30:
            print(f"    ... and {len(differences) - 30} more")

        # Statistics on differences
        rows_with_diff = set(d['row'] for d in differences)
        print(f"\n  Rows with differences: {sorted(rows_with_diff)}")


def main():
    if len(sys.argv) < 2:
        print(f"Usage: sudo {sys.argv[0]} <PID>")
        print("  PID: Process ID of running benchmark_capture")
        sys.exit(1)

    pid = int(sys.argv[1])
    print(f"Searching for FPPN in process {pid}")

    # Check we have access
    if not os.path.exists(f"/proc/{pid}/mem"):
        print(f"Cannot access /proc/{pid}/mem - run with sudo")
        sys.exit(1)

    regions = parse_maps(pid)
    print(f"Found {len(regions)} memory regions")

    # Focus on heap and libCubeEye data sections
    interesting_regions = []
    for r in regions:
        if 'r' in r['perms']:  # Readable
            if '[heap]' in r['path'] or 'libCubeEye' in r['path']:
                interesting_regions.append(r)

    print(f"Searching {len(interesting_regions)} interesting regions")

    all_results = []

    with open(f"/proc/{pid}/mem", "rb") as mem:
        for r in interesting_regions:
            size = (r['end'] - r['start']) / 1024 / 1024
            print(f"  {hex(r['start'])}-{hex(r['end'])} ({size:.1f} MB) - {r['path']}")

            # Search both little-endian and big-endian
            for endian in ['<', '>']:
                results = search_region(mem, r['start'], r['end'], endian)
                all_results.extend(results)

    if not all_results:
        print("\nNo FPPN arrays found in targeted regions.")
        print("Trying exhaustive search of all readable memory...")

        with open(f"/proc/{pid}/mem", "rb") as mem:
            for r in regions:
                if 'r' in r['perms'] and r['end'] - r['start'] >= FPPN_SIZE:
                    for endian in ['<', '>']:
                        results = search_region(mem, r['start'], r['end'], endian)
                        all_results.extend(results)

    print(f"\n{'='*60}")
    print(f"Found {len(all_results)} potential FPPN arrays")

    if all_results:
        OUTPUT_DIR.mkdir(exist_ok=True)

        for i, result in enumerate(all_results):
            endian_name = "LE" if result['endian'] == '<' else "BE"
            print(f"\n[{i}] Address: {hex(result['address'])}, Coverage: {result['coverage']:.1f}%, Endian: {endian_name}")
            analyze_fppn(result['data'], result['endian'])

            # Save
            filename = OUTPUT_DIR / f"sdk_fppn_{i}_{endian_name}_{hex(result['address'])}.bin"
            with open(filename, "wb") as f:
                f.write(result['data'])
            print(f"  Saved: {filename}")

            # Compare with our extraction
            extracted_file = "/home/cmericli/development/atlas/code/cubeeye_nano_driver/extracted_calibration/fppn_320x240_le.bin"
            if result['endian'] == '<' and os.path.exists(extracted_file):
                print("\n  Comparing with our extracted FPPN:")
                compare_with_extracted(result['data'], extracted_file, result['endian'])


if __name__ == "__main__":
    main()
