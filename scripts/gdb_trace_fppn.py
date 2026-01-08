#!/usr/bin/env python3
"""
GDB Python script to trace FPPN loading in CubeEye SDK.
Usage: gdb -x gdb_trace_fppn.py ./benchmark_capture
"""

import gdb
import struct

class CalibrationBreakpoint(gdb.Breakpoint):
    """Breakpoint to trace readCalibrationData calls."""

    def __init__(self, addr, name):
        super(CalibrationBreakpoint, self).__init__("*" + hex(addr))
        self.name = name
        self.call_count = 0

    def stop(self):
        self.call_count += 1
        # Get arguments from registers (x86_64 calling convention)
        # RDI = this, RSI = address pointer, RDX = output vector
        try:
            rdi = int(gdb.parse_and_eval("$rdi"))
            rsi = int(gdb.parse_and_eval("$rsi"))
            rdx = int(gdb.parse_and_eval("$rdx"))

            # Read the address value (first 4 bytes at RSI)
            addr_val = gdb.selected_inferior().read_memory(rsi, 4)
            addr = struct.unpack('<I', addr_val)[0]

            print(f"\n[TRACE] {self.name} called (#{self.call_count})")
            print(f"  Address parameter: 0x{addr:08x}")
            print(f"  Output vector at: 0x{rdx:016x}")
        except Exception as e:
            print(f"[TRACE] {self.name} - error reading args: {e}")

        return False  # Don't stop, continue execution


class IoctlBreakpoint(gdb.Breakpoint):
    """Breakpoint to trace ioctl calls (UVC XU commands)."""

    def __init__(self):
        super(IoctlBreakpoint, self).__init__("ioctl")
        self.xu_calls = []

    def stop(self):
        try:
            fd = int(gdb.parse_and_eval("$rdi"))
            request = int(gdb.parse_and_eval("$rsi"))
            arg = int(gdb.parse_and_eval("$rdx"))

            # UVCIOC_CTRL_QUERY = 0xc0107521
            if request == 0xc0107521:
                # Read uvc_xu_control_query structure
                data = gdb.selected_inferior().read_memory(arg, 16)
                unit, selector, query, size = struct.unpack('<BBHI', data[:8])
                data_ptr = struct.unpack('<Q', data[8:16])[0]

                if selector == 4:  # Calibration data selector
                    # Read the query data
                    query_data = gdb.selected_inferior().read_memory(data_ptr, min(size, 16))
                    query_hex = ' '.join(f'{b:02x}' for b in query_data)

                    query_names = {1: "SET_CUR", 2: "GET_CUR", 3: "GET_MIN", 4: "GET_MAX"}
                    qname = query_names.get(query, f"QUERY_{query}")

                    print(f"[XU] Selector 4 {qname}: unit={unit}, size={size}")
                    print(f"     Data: {query_hex}")

                    if query == 1:  # SET_CUR - page request
                        if size >= 4:
                            page_num = struct.unpack('>H', query_data[2:4])[0]
                            print(f"     Page request: 0x{page_num:04x}")

                    self.xu_calls.append({
                        'query': query,
                        'selector': selector,
                        'size': size,
                        'data': bytes(query_data)
                    })
        except Exception as e:
            pass  # Many ioctl calls won't be UVC XU

        return False


class FPPNTracer:
    """Main tracer class."""

    def __init__(self):
        self.breakpoints = []

    def setup(self):
        print("Setting up FPPN tracer...")

        # Get base address of libCubeEye.so (will be set after run)
        # These are offsets from the library base
        self.cal_offsets = {
            'readCalibrationData': 0x21c6a0,
            'loadCalibrationData': 0x2349c0,
            'loadCalibrationDataFileSource': 0x237dc0,
        }

        # Set breakpoint on ioctl
        self.ioctl_bp = IoctlBreakpoint()
        self.breakpoints.append(self.ioctl_bp)

        print("Tracer ready. Run the program with 'run 1'")
        print("After it loads libCubeEye.so, use 'info sharedlibrary' to find base address")


# Create and setup tracer when script loads
tracer = FPPNTracer()

def setup_lib_breakpoints():
    """Call this after libCubeEye.so is loaded to set function breakpoints."""
    try:
        # Get library base address
        output = gdb.execute("info sharedlibrary libCubeEye", to_string=True)
        for line in output.split('\n'):
            if 'libCubeEye' in line:
                parts = line.split()
                if len(parts) >= 2:
                    base = int(parts[0], 16)
                    print(f"libCubeEye.so loaded at 0x{base:x}")

                    # Set breakpoints at function addresses
                    for name, offset in tracer.cal_offsets.items():
                        addr = base + offset
                        bp = CalibrationBreakpoint(addr, name)
                        tracer.breakpoints.append(bp)
                        print(f"  Breakpoint: {name} at 0x{addr:x}")
                    return
    except Exception as e:
        print(f"Error setting up library breakpoints: {e}")

# Define GDB command to set up after library load
class SetupLibBreakpoints(gdb.Command):
    """Set up breakpoints after libCubeEye.so is loaded."""

    def __init__(self):
        super(SetupLibBreakpoints, self).__init__("setup-cubeeye", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        setup_lib_breakpoints()

SetupLibBreakpoints()

print("\n=== FPPN Tracer Loaded ===")
print("Commands:")
print("  run 1           - Run program (capture 1 frame)")
print("  setup-cubeeye   - Set up library breakpoints (after library loads)")
print("  continue        - Continue after breakpoint")
