#!/usr/bin/env python3
"""
GDB Python script to dump EXACT FPPN array from SDK memory.

This script:
1. Runs the SDK benchmark_capture program
2. Waits for camera initialization (FPPN loaded)
3. Searches memory for FPPN array pattern
4. Dumps the exact array the SDK uses

Usage: gdb -x dump_sdk_fppn.py
"""

import gdb
import struct
import os

OUTPUT_DIR = "/home/cmericli/development/atlas/code/cubeeye_nano_driver/sdk_fppn_dump"

# FPPN characteristics we know:
# - 320x240 = 76800 int16 values = 153600 bytes
# - Values typically in range -950 to -450
# - Mean around -670
FPPN_SIZE = 320 * 240 * 2  # 153600 bytes
FPPN_VALUE_MIN = -950
FPPN_VALUE_MAX = -450


class FPPNFinder:
    """Search process memory for FPPN array."""

    def __init__(self):
        self.found_arrays = []

    def search_memory_region(self, start, end):
        """Search a memory region for FPPN-like data."""
        try:
            inf = gdb.selected_inferior()
            # Read in chunks to avoid huge memory allocations
            chunk_size = 1024 * 1024  # 1MB chunks

            addr = start
            while addr < end - FPPN_SIZE:
                try:
                    # Read enough for full FPPN array check
                    read_size = min(chunk_size + FPPN_SIZE, end - addr)
                    data = bytes(inf.read_memory(addr, read_size))

                    # Search for FPPN pattern in this chunk
                    self._search_chunk(data, addr)

                    addr += chunk_size
                except gdb.MemoryError:
                    addr += 4096  # Skip unreadable page

        except Exception as e:
            print(f"Error searching {hex(start)}-{hex(end)}: {e}")

    def _search_chunk(self, data, base_addr):
        """Search chunk for FPPN-like int16 arrays."""
        # Look for runs of FPPN-like values
        # We need at least 320 consecutive FPPN values (one row)
        MIN_RUN = 320 * 2  # bytes

        i = 0
        while i < len(data) - MIN_RUN:
            # Check if this could be start of FPPN row
            fppn_count = 0
            for j in range(0, MIN_RUN, 2):
                if i + j + 1 >= len(data):
                    break
                val = struct.unpack('<h', data[i+j:i+j+2])[0]
                if FPPN_VALUE_MIN < val < FPPN_VALUE_MAX:
                    fppn_count += 1

            # If >90% of first row is FPPN-like, check full array
            if fppn_count > 288:  # 90% of 320
                self._check_full_array(data, i, base_addr)
                i += FPPN_SIZE  # Skip past this potential match
            else:
                i += 2

    def _check_full_array(self, data, offset, base_addr):
        """Check if this is a full FPPN array."""
        if offset + FPPN_SIZE > len(data):
            return

        array_data = data[offset:offset + FPPN_SIZE]

        # Count FPPN-like values in full array
        fppn_count = 0
        total_count = FPPN_SIZE // 2

        for i in range(0, FPPN_SIZE, 2):
            val = struct.unpack('<h', array_data[i:i+2])[0]
            if FPPN_VALUE_MIN < val < FPPN_VALUE_MAX:
                fppn_count += 1

        coverage = 100.0 * fppn_count / total_count

        if coverage > 95:  # >95% FPPN values
            addr = base_addr + offset
            print(f"\n[FPPN FOUND] Address: {hex(addr)}, Coverage: {coverage:.1f}%")
            self.found_arrays.append({
                'address': addr,
                'coverage': coverage,
                'data': array_data
            })


class SearchMemoryCommand(gdb.Command):
    """GDB command to search for FPPN array."""

    def __init__(self):
        super(SearchMemoryCommand, self).__init__("search-fppn", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        print("Searching process memory for FPPN array...")
        print(f"Looking for: {FPPN_SIZE} bytes of int16 values in range [{FPPN_VALUE_MIN}, {FPPN_VALUE_MAX}]")

        finder = FPPNFinder()

        # Get memory mappings
        mappings = gdb.execute("info proc mappings", to_string=True)

        for line in mappings.split('\n'):
            parts = line.split()
            if len(parts) >= 5:
                try:
                    start = int(parts[0], 16)
                    end = int(parts[1], 16)
                    perms = parts[4] if len(parts) > 4 else ""

                    # Only search readable regions
                    if end - start >= FPPN_SIZE and 'r' in perms:
                        # Focus on heap and library data sections
                        if '[heap]' in line or 'libCubeEye' in line or '.so' in line:
                            print(f"Searching: {hex(start)}-{hex(end)} ({perms})")
                            finder.search_memory_region(start, end)
                except ValueError:
                    continue

        print(f"\nFound {len(finder.found_arrays)} potential FPPN arrays")

        if finder.found_arrays:
            # Save the best match (highest coverage)
            best = max(finder.found_arrays, key=lambda x: x['coverage'])
            print(f"Best match: {hex(best['address'])} with {best['coverage']:.1f}% coverage")

            os.makedirs(OUTPUT_DIR, exist_ok=True)

            # Save as binary
            with open(f"{OUTPUT_DIR}/sdk_fppn_exact.bin", "wb") as f:
                f.write(best['data'])
            print(f"Saved to {OUTPUT_DIR}/sdk_fppn_exact.bin")

            # Analyze the data
            self._analyze_fppn(best['data'])

    def _analyze_fppn(self, data):
        """Analyze the extracted FPPN data."""
        values = []
        for i in range(0, len(data), 2):
            val = struct.unpack('<h', data[i:i+2])[0]
            values.append(val)

        import statistics
        fppn_vals = [v for v in values if FPPN_VALUE_MIN < v < FPPN_VALUE_MAX]
        non_fppn_vals = [v for v in values if not (FPPN_VALUE_MIN < v < FPPN_VALUE_MAX)]

        print(f"\nFPPN Analysis:")
        print(f"  Total values: {len(values)}")
        print(f"  FPPN-like values: {len(fppn_vals)} ({100*len(fppn_vals)/len(values):.1f}%)")
        print(f"  Non-FPPN values: {len(non_fppn_vals)}")

        if fppn_vals:
            print(f"  Min: {min(fppn_vals)}")
            print(f"  Max: {max(fppn_vals)}")
            print(f"  Mean: {statistics.mean(fppn_vals):.1f}")
            print(f"  Std: {statistics.stdev(fppn_vals):.1f}")

        if non_fppn_vals:
            print(f"\n  Non-FPPN values (first 20):")
            for i, v in enumerate(non_fppn_vals[:20]):
                idx = values.index(v)
                row = idx // 320
                col = idx % 320
                print(f"    [{row},{col}]: {v}")


class DumpAllFPPNCommand(gdb.Command):
    """Dump all potential FPPN arrays found."""

    def __init__(self):
        super(DumpAllFPPNCommand, self).__init__("dump-all-fppn", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        print("Exhaustive FPPN search - checking all heap memory...")

        finder = FPPNFinder()

        # Get heap boundaries
        mappings = gdb.execute("info proc mappings", to_string=True)

        for line in mappings.split('\n'):
            if '[heap]' in line:
                parts = line.split()
                start = int(parts[0], 16)
                end = int(parts[1], 16)
                print(f"Heap: {hex(start)}-{hex(end)} ({(end-start)/1024/1024:.1f} MB)")
                finder.search_memory_region(start, end)
                break

        # Also check BSS/data sections of libCubeEye
        for line in mappings.split('\n'):
            if 'libCubeEye' in line and 'rw' in line:
                parts = line.split()
                start = int(parts[0], 16)
                end = int(parts[1], 16)
                print(f"libCubeEye data: {hex(start)}-{hex(end)}")
                finder.search_memory_region(start, end)

        if finder.found_arrays:
            os.makedirs(OUTPUT_DIR, exist_ok=True)
            for i, arr in enumerate(finder.found_arrays):
                filename = f"{OUTPUT_DIR}/fppn_candidate_{i}_{hex(arr['address'])}.bin"
                with open(filename, "wb") as f:
                    f.write(arr['data'])
                print(f"Saved: {filename}")


# Register commands
SearchMemoryCommand()
DumpAllFPPNCommand()

print("\n" + "=" * 60)
print("SDK FPPN Memory Dump Tool")
print("=" * 60)
print("""
Commands:
  run 1             - Run benchmark_capture (capture 1 frame)
  search-fppn       - Search memory for FPPN array
  dump-all-fppn     - Dump all potential FPPN candidates

Workflow:
  1. Run 'run 1' to start the program
  2. Wait for camera initialization
  3. Press Ctrl+C to interrupt
  4. Run 'search-fppn' to find and dump FPPN array

Output directory: """ + OUTPUT_DIR)
