use std::path::PathBuf;

use emu_6502::emulator::Emulator;

/**
 * Runs functional tests from https://github.com/Klaus2m5/6502_65C02_functional_tests
 */

fn run_binary(emulator: &mut Emulator, binary: String, start_mem: usize, start_pc: usize) {
    let path_str = format!("{}/tests/binaries/{}", env!("CARGO_MANIFEST_DIR"), binary);
    emulator.load_binary(PathBuf::from(path_str), start_mem);
    emulator.override_program_counter(start_pc);
    emulator.unpause();

    while !emulator.is_paused() {
        emulator.clock_tick();
    }
}

fn assert_pc(emulator: &Emulator, expected_pc: usize) {
    let actual = emulator.cpu().program_counter();
    assert_eq!(
        actual, expected_pc,
        "Actual: {:#06X} Expected: {:#06X}",
        actual, expected_pc
    );
}

#[test]
fn functional_test() {
    let mut emulator = Emulator::new();
    run_binary(
        &mut emulator,
        "6502_functional_test.bin".to_string(),
        0,
        0x400,
    );

    assert_pc(&emulator, 0x3469);
}

#[test]
fn extended_opcodes_test() {
    let mut emulator = Emulator::new();
    run_binary(
        &mut emulator,
        "65C02_extended_opcodes_test.bin".to_string(),
        0,
        0x400,
    );

    assert_pc(&emulator, 0x24F1);
}

#[test]
fn interrupt_test() {
    let mut emulator = Emulator::new();
    emulator.configure_interrupts(0xBFFC, 0b0000_0001, 0b0000_00010);

    run_binary(
        &mut emulator,
        "6502_interrupt_test.bin".to_string(),
        0xA,
        0x400,
    );

    assert_pc(&emulator, 0x06F5);
}
