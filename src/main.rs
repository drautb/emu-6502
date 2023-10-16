use emu_6502::emulator::Emulator;
use emu_6502::frontend::show_frontend;

fn main() -> Result<(), eframe::Error> {
    let emulator = Emulator::new();
    show_frontend(&emulator)
}
