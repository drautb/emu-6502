use std::sync::{Arc, Mutex};
use std::thread;

use emu_6502::{emulator::Emulator, frontend::Frontend};

fn main() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        initial_window_size: Some(eframe::egui::vec2(1280.0, 800.0)),
        ..Default::default()
    };

    let emulator = Arc::new(Mutex::new(Emulator::new()));

    let background_emulator = Arc::clone(&emulator);
    thread::spawn(move || {
        let emulator = background_emulator;
        loop {
            let mut emulator = emulator.lock().unwrap();

            let old_pc = emulator.cpu().program_counter();
            if !emulator.is_paused() {
                emulator.step_cpu();

                // Pause when PC doesn't change, which indicates HCF
                // https://en.wikipedia.org/wiki/Halt_and_Catch_Fire_(computing)
                if emulator.cpu().program_counter() == old_pc {
                    emulator.pause();
                }
            }
        }
    });

    eframe::run_native(
        "EMU 6502",
        options,
        Box::new(move |_cc| Box::<Frontend>::new(Frontend::new(emulator))),
    )
}
