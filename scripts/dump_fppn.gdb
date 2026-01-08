# GDB script to dump FPPN tables from running SDK
#
# Usage:
#   1. Start the SDK capture program
#   2. In another terminal: gdb -p $(pgrep -f benchmark_capture) -x dump_fppn.gdb
#
# The script will wait for FPPN to be loaded, then dump it.

set pagination off
set logging file fppn_dump.log
set logging on

# FPPN is stored at I200S object offset 0x4020 (vector<int16>)
# Size: 0x25800 = 153600 bytes = 320*240*2 per frequency
# Two tables: FPPN1 (freq1) and FPPN2 (freq2)

define dump_fppn
    printf "Looking for FPPN data...\n"

    # Method 1: Break on FPPN log message
    # The SDK logs "[Load FPPN1]" when loading
    # We can break on Log::d calls that reference this string

    # Method 2: Find I200S object and dump offset 0x4020
    # This requires knowing the I200S object address

    # For now, print instructions
    printf "\n=== FPPN Dump Instructions ===\n"
    printf "1. Find I200S object address (search for vtable)\n"
    printf "2. Examine: x/4gx <addr>+0x4020\n"
    printf "   First pointer = data buffer\n"
    printf "3. Dump: dump binary memory fppn1.bin <data_ptr> <data_ptr>+153600\n"
    printf "\n"
    printf "Expected FPPN size: 153600 bytes (320*240*2)\n"
    printf "===\n"
end

define find_cubeeye_strings
    printf "Searching for CubeEye-related strings in memory...\n"
    # This searches the process memory for FPPN-related strings
    find /b 0x00400000, 0x7fffffffffff, 'f', 'p', 'p', 'n'
end

define dump_memory_range
    if $argc != 3
        printf "Usage: dump_memory_range <addr> <size> <filename>\n"
    else
        printf "Dumping %d bytes from %p to %s\n", $arg1, $arg0, $arg2
        dump binary memory $arg2 $arg0 $arg0+$arg1
    end
end

# Commands to run when script loads
printf "\n=== CubeEye FPPN Dump Script ===\n"
printf "Commands available:\n"
printf "  dump_fppn           - Instructions for dumping FPPN\n"
printf "  find_cubeeye_strings - Search for FPPN strings\n"
printf "  dump_memory_range <addr> <size> <file> - Dump memory\n"
printf "\n"
printf "Quick method:\n"
printf "  1. Run: info proc mappings\n"
printf "  2. Find heap address range\n"
printf "  3. Search: find /b <heap_start>, <heap_end>, 0x00, 0x25, 0x80, 0x00\n"
printf "     (This is the FPPN vector size: 0x25800)\n"
printf "===\n\n"
