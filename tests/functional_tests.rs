use std::path::PathBuf;

use emu_6502::emulator::Emulator;

/**
 * Runs functional tests from https://github.com/Klaus2m5/6502_65C02_functional_tests
 */

fn load_binary(emulator: &mut Emulator, binary: String) {
    let path_str = format!("{}/tests/binaries/{}", env!("CARGO_MANIFEST_DIR"), binary);
    emulator.load_binary(PathBuf::from(path_str), 0);
}

#[test]
fn functional_test() {
    let mut emulator = Emulator::new();
    load_binary(&mut emulator, "6502_functional_test.bin".to_string());
    emulator.override_program_counter(0x400);
    emulator.unpause();

    while !emulator.is_paused() {
        emulator.clock_tick();
    }

    assert_eq!(emulator.cpu().program_counter(), 0x3469); // Success marker
}
