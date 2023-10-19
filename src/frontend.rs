use std::cmp;

use crate::cpu::Cpu;
use crate::emulator::Emulator;
use crate::Memory;
use eframe::egui::{self, TextEdit};
use eframe::egui::{Button, Ui, Vec2};
use eframe::epaint::Color32;
use egui_extras::{Column, TableBuilder};

const STATUS_FLAG_SIZE: Vec2 = Vec2::new(26.0, 26.0);
const STATUS_FLAG_CLEAR: Color32 = Color32::from_gray(64);
const STATUS_FLAG_SET: Color32 = Color32::from_rgb(44, 108, 34);

#[derive(Default)]
pub struct Frontend {
    emulator: Emulator,
    memory_offset: u16,
}

impl eframe::App for Frontend {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |_ui| {
            let mut open = true;
            egui::Window::new("CPU")
                .open(&mut open)
                .fixed_size(Vec2::new(200.0, 1.0))
                .show(ctx, |ui| {
                    self.show_cpu_window(ui, &self.emulator.cpu());
                });
        });

        egui::CentralPanel::default().show(ctx, |_ui| {
            let mut open = true;
            egui::Window::new("Memory")
                .open(&mut open)
                .fixed_size(Vec2::new(200.0, 1.0))
                .show(ctx, |ui| {
                    self.show_memory_window(ui, &self.emulator.memory());
                });
        });
    }
}

impl Frontend {
    pub fn show_cpu_window(&self, ui: &mut Ui, cpu: &Cpu) {
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

            self.show_register(
                &mut body,
                "Instruction Register",
                "ir",
                cpu.instruction_register(),
            );
            self.show_register(&mut body, "Accumulator", "a", cpu.accumulator());
            self.show_register(&mut body, "X", "x", cpu.x_register());
            self.show_register(&mut body, "Y", "y", cpu.y_register());
            self.show_register(&mut body, "Stack Pointer", "s", cpu.stack_pointer());
            self.show_register(&mut body, "Status", "p", cpu.status());
        });

        ui.separator();

        ui.heading("Status Flags");
        ui.horizontal_centered(|ui| {
            self.show_status_flag(ui, "N", cpu.get_status_neg());
            self.show_status_flag(ui, "V", cpu.get_status_overflow());
            self.show_status_flag(ui, "_", false);
            self.show_status_flag(ui, "B", cpu.get_status_brk());
            self.show_status_flag(ui, "D", cpu.get_status_dec());
            self.show_status_flag(ui, "I", cpu.get_status_interrupt_disable());
            self.show_status_flag(ui, "Z", cpu.get_status_zero());
            self.show_status_flag(ui, "C", cpu.get_status_carry());
        });
    }

    fn show_register(&self, body: &mut egui_extras::TableBody, label: &str, abbrev: &str, val: u8) {
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

    fn show_status_flag(&self, ui: &mut Ui, label: &str, set: bool) {
        ui.add_enabled(
            false,
            Button::new(label).min_size(STATUS_FLAG_SIZE).fill(if set {
                STATUS_FLAG_SET
            } else {
                STATUS_FLAG_CLEAR
            }),
        );
    }

    pub fn show_memory_window(&mut self, ui: &mut Ui, mem: &Memory) {
        let table = TableBuilder::new(ui)
            .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
            .column(Column::exact(210.0))
            .column(Column::exact(180.0))
            .column(Column::exact(210.0));
        table.body(|mut body| {
            body.row(24.0, |mut row| {
                row.col(|_| {});
                row.col(|ui| {
                    ui.horizontal(|ui| {
                        if ui.button("⏪").clicked() {
                            self.memory_offset -= cmp::min(0x100, self.memory_offset);
                        }
                        if ui.button("◀").clicked() {
                            self.memory_offset -= cmp::min(0x10, self.memory_offset);
                        }
                        ui.monospace("0x");

                        let mut memory_offset_str = format!("{:X}", self.memory_offset);
                        ui.add(
                            TextEdit::singleline(&mut memory_offset_str)
                                .desired_width(40.0)
                                .char_limit(4)
                                .clip_text(false),
                        );
                        self.memory_offset =
                            match u16::from_str_radix(memory_offset_str.as_str(), 16) {
                                Ok(val) => val,
                                _ => self.memory_offset,
                            };

                        if ui.button("▶").clicked() {
                            self.memory_offset += cmp::min(0x10, 0xFFFF - self.memory_offset);
                        }
                        if ui.button("⏩").clicked() {
                            self.memory_offset += cmp::min(0x100, 0xFFFF - self.memory_offset);
                        }
                    });
                });
                row.col(|_| {});
            });
        });

        ui.separator();

        egui::Grid::new("memory_grid")
            .num_columns(3)
            .spacing([40.0, 4.0])
            .striped(true)
            .show(ui, |ui| {
                let start_row = self.memory_offset / 0x10;
                for r in start_row..start_row + 20 {
                    self.dump_memory_row(ui, r, mem);
                }
            });
    }

    fn dump_memory_row(&self, ui: &mut Ui, row: u16, mem: &Memory) {
        ui.monospace(format!("{:08X}", row * 16));
        let mut hex_line = String::new();
        let mut ascii_line = String::new();
        ascii_line.push('|');
        for b in 0..16 {
            let byte: u8 = mem[((row * 16) + b) as usize];
            hex_line.push_str(format!("{:02X} ", byte).as_str());
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
}
