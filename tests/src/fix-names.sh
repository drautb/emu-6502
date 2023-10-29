#!/usr/bin/env bash

# After running the assembler in dosbox, run this script to
# fix the names and put the files in the right places.

mv 65C02_~1.BIN 65C02_extended_opcodes_test.bin
mv 65C02_~1.LST 65C02_extended_opcodes_test.lst

mv 6502_F~1.BIN 6502_functional_test.bin
mv 6502_F~1.LST 6502_functional_test.lst

mv 6502_I~1.BIN 6502_interrupt_test.bin
mv 6502_I~1.LST 6502_interrupt_test.lst

mv ./*.bin ../binaries/
mv ./*.lst ../listings/