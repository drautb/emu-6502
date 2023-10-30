#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

use std::sync::{Arc, Mutex};

#[cfg(not(target_arch = "wasm32"))]
use std::thread;

use emu_6502::{emulator::Emulator, frontend::Frontend};

#[cfg(not(target_arch = "wasm32"))]
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
            emulator.clock_tick();
        }
    });

    eframe::run_native(
        "EMU 6502",
        options,
        Box::new(move |_cc| Box::<Frontend>::new(Frontend::new(emulator))),
    )
}

#[cfg(target_arch = "wasm32")]
fn main() {
    // Redirect `log` message to `console.log` and friends:
    // eframe::WebLogger::init(log::LevelFilter::Debug).ok();

    let web_options = eframe::WebOptions::default();

    let emulator = Arc::new(Mutex::new(Emulator::new()));

    let background_emulator = Arc::clone(&emulator);
    wasm_bindgen_futures::spawn_local(async {
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

    wasm_bindgen_futures::spawn_local(async {
        eframe::WebRunner::new()
            .start(
                "emu_6502", // hardcode it
                web_options,
                Box::new(|_cc| Box::<Frontend>::new(Frontend::new(emulator))),
            )
            .await
            .expect("Failed to start eframe");
    });
}
