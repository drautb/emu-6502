use crate::cpu::Cpu;
use crate::emulator::Emulator;
use crate::Memory;
use eframe::egui;
use eframe::egui::{Button, Ui, Vec2};
use eframe::epaint::Color32;
use egui_extras::Column;

const STATUS_FLAG_SIZE: Vec2 = Vec2::new(26.0, 26.0);
const STATUS_FLAG_CLEAR: Color32 = Color32::from_gray(64);
const STATUS_FLAG_SET: Color32 = Color32::from_rgb(44, 108, 34);

pub struct Frontend {}

pub fn show_frontend(emulator: &Emulator) -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        initial_window_size: Some(egui::vec2(1280.0, 800.0)),
        ..Default::default()
    };

    let cpu = emulator.cpu();
    let memory = emulator.memory();
    eframe::run_simple_native("EMU-6502", options, move |ctx, _frame| {
        egui::CentralPanel::default().show(ctx, |_ui| {
            let mut open = true;
            egui::Window::new("CPU")
                .open(&mut open)
                .fixed_size(Vec2::new(200.0, 1.0))
                .show(ctx, |ui| {
                    show_cpu_window(ui, &cpu);
                });
        });

        egui::CentralPanel::default().show(ctx, |_ui| {
            let mut open = true;
            egui::Window::new("Memory")
                .open(&mut open)
                .fixed_size(Vec2::new(200.0, 1.0))
                .show(ctx, |ui| {
                    show_memory_window(ui, &memory);
                });
        });
    })
}

pub fn show_cpu_window(ui: &mut Ui, cpu: &Cpu) {
    ui.heading("Registers");

    let table = egui_extras::TableBuilder::new(ui)
        .striped(true)
        .cell_layout(egui::Layout::right_to_left(eframe::emath::Align::Center))
        .column(Column::initial(150.0))
        .column(Column::initial(50.0))
        .column(Column::initial(50.0))
        .column(Column::initial(40.0))
        .column(Column::initial(140.0));

    let table = table.header(20.0, |mut header| {
        header.col(|ui| {
            ui.strong("");
        });
        header.col(|ui| {
            ui.strong("(Abbrev)");
        });
        header.col(|ui| {
            ui.strong("Hex");
        });
        header.col(|ui| {
            ui.strong("Dec");
        });
        header.col(|ui| {
            ui.strong("Bin");
        });
    });

    table.body(|mut body| {
        body.row(18.0, |mut row| {
            row.col(|ui| {
                ui.monospace("Program Counter");
            });
            row.col(|ui| {
                ui.monospace("pc");
            });

            let pc = cpu.program_counter();
            row.col(|ui| {
                ui.monospace(format!("{:#06X}", pc));
            });
            row.col(|ui| {
                ui.monospace(format!("{:5}", pc));
            });
            row.col(|ui| {
                ui.monospace(format!(
                    "{:04b} {:04b} {:04b} {:04b}",
                    pc >> 12,
                    pc >> 8 & 15,
                    pc >> 4 & 15,
                    pc & 15
                ));
            });
        });

        show_register(
            &mut body,
            "Instruction Register",
            "ir",
            cpu.instruction_register(),
        );
        show_register(&mut body, "Accumulator", "a", cpu.accumulator());
        show_register(&mut body, "X", "x", cpu.x_register());
        show_register(&mut body, "Y", "y", cpu.y_register());
        show_register(&mut body, "Stack Pointer", "s", cpu.stack_pointer());
        show_register(&mut body, "Status", "p", cpu.status());
    });

    ui.separator();

    ui.heading("Status Flags");
    ui.horizontal_centered(|ui| {
        show_status_flag(ui, "N", cpu.get_status_neg());
        show_status_flag(ui, "V", cpu.get_status_overflow());
        show_status_flag(ui, "_", false);
        show_status_flag(ui, "B", cpu.get_status_brk());
        show_status_flag(ui, "D", cpu.get_status_dec());
        show_status_flag(ui, "I", cpu.get_status_interrupt_disable());
        show_status_flag(ui, "Z", cpu.get_status_zero());
        show_status_flag(ui, "C", cpu.get_status_carry());
    });
}

fn show_register(body: &mut egui_extras::TableBody, label: &str, abbrev: &str, val: u8) {
    body.row(18.0, |mut row| {
        row.col(|ui| {
            ui.monospace(label);
        });
        row.col(|ui| {
            ui.monospace(abbrev);
        });
        row.col(|ui| {
            ui.monospace(format!("{:#04X}", val));
        });
        row.col(|ui| {
            ui.monospace(format!("{:5}", val));
        });
        row.col(|ui| {
            ui.monospace(format!("{:04b} {:04b}", val >> 4, val & 15));
        });
    });
}

fn show_status_flag(ui: &mut Ui, label: &str, set: bool) {
    ui.add_enabled(
        false,
        Button::new(label).min_size(STATUS_FLAG_SIZE).fill(if set {
            STATUS_FLAG_SET
        } else {
            STATUS_FLAG_CLEAR
        }),
    );
}

pub fn show_memory_window(ui: &mut Ui, mem: &Memory) {
    egui::Grid::new("memory_grid")
        .num_columns(3)
        .spacing([40.0, 4.0])
        .striped(true)
        .show(ui, |ui| {
            for r in 0..20 {
                dump_memory_row(ui, r, mem);
            }
        });
}

fn dump_memory_row(ui: &mut Ui, row: u16, mem: &Memory) {
    ui.monospace(format!("{:08x}", row * 16));
    let mut hex_line = String::new();
    let mut ascii_line = String::new();
    ascii_line.push('|');
    for b in 0..16 {
        let byte: u8 = mem[((row * 16) + b) as usize];
        hex_line.push_str(format!("{:02x} ", byte).as_str());
        if b == 7 {
            hex_line.push(' ');
        }
        let c: char = byte as char;
        if c.is_ascii_control() {
            ascii_line.push('.');
        } else {
            ascii_line.push(c);
        }
    }
    ascii_line.push('|');

    ui.monospace(hex_line);
    ui.monospace(ascii_line);
    ui.end_row();
}
