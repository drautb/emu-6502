use emu_6502::frontend::Frontend;

fn main() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        initial_window_size: Some(eframe::egui::vec2(1280.0, 800.0)),
        ..Default::default()
    };

    eframe::run_native(
        "EMU 6502",
        options,
        Box::new(|_cc| Box::<Frontend>::default()),
    )
}
