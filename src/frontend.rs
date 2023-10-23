use std::cmp;
use std::thread::sleep;
use std::time::Duration;

use crate::cpu::parse_instruction;
use crate::emulator::Emulator;
use eframe::egui::{self, TextEdit};
use eframe::egui::{Button, Ui, Vec2};
use eframe::epaint::Color32;
use egui_extras::{Column, TableBuilder};

const STATUS_FLAG_SIZE: Vec2 = Vec2::new(26.0, 26.0);
const STATUS_FLAG_CLEAR: Color32 = Color32::from_gray(64);
const STATUS_FLAG_SET: Color32 = Color32::from_rgb(44, 108, 34);

const MEMORY_ROWS: u16 = 20;
const LAST_ROW_START_ADDRESS: u16 = 0xFFFF - 0x10 * (MEMORY_ROWS - 1);
const LAST_ROW: u16 = LAST_ROW_START_ADDRESS / 0x10;

#[derive(Default)]
pub struct Frontend {
    // The emulator
    emulator: Emulator,

    // Selected memory cell in viewer
    selected_memory: u16,

    // Where binary originates in memory (start address)
    binary_orig: u16,

    // True if the next instruction is loaded into the IR, but hasn't executed yet.
    instruction_loaded: bool,

    // PC override string
    pc_override: String,

    // Run continuously or in single step mode
    continuous: bool,
}

impl eframe::App for Frontend {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if self.continuous {
            sleep(Duration::from_millis(10));
            self.emulator.step_cpu();
            self.selected_memory = self.emulator.cpu().program_counter() as u16;
        }

        egui::CentralPanel::default().show(ctx, |_ui| {
            let mut open = true;
            egui::Window::new("CPU")
                .open(&mut open)
                .fixed_size(Vec2::new(200.0, 1.0))
                .show(ctx, |ui| {
                    self.show_cpu_window(ui);
                });
        });

        egui::CentralPanel::default().show(ctx, |_ui| {
            let mut open = true;
            egui::Window::new("Memory")
                .open(&mut open)
                .fixed_size(Vec2::new(200.0, 1.0))
                .show(ctx, |ui| {
                    self.show_memory_window(ui);
                });
        });
    }
}

impl Frontend {
    pub fn show_cpu_window(&mut self, ui: &mut Ui) {
        self.show_cpu_controls(ui);

        ui.separator();

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

        let cpu = self.emulator.cpu();
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

            let mut ir_label = String::new();
            if self.instruction_loaded {
                ir_label.push('*');
            }
            ir_label.push_str("Instruction Register");

            self.show_register(&mut body, &ir_label, "ir", cpu.instruction_register());
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

    fn show_cpu_controls(&mut self, ui: &mut Ui) {
        ui.push_id(1, |ui| {
            let table = TableBuilder::new(ui)
                .cell_layout(egui::Layout::right_to_left(egui::Align::Center))
                .column(Column::auto())
                .column(Column::auto().at_least(175.0))
                .column(Column::auto().at_least(175.0));
            table.body(|mut body| {
                body.row(30.0, |mut row| {
                    row.col(|ui| {
                        ui.horizontal(|ui| {
                            if ui.button("▶").on_hover_text("Run continuously").clicked() {
                                self.continuous = true;
                            }

                            if ui
                                .button("⏸")
                                .on_hover_text("Pause for single steps")
                                .clicked()
                            {
                                self.continuous = false;
                            }

                            if ui
                                .button("Step")
                                .on_hover_text("Execute next instruction")
                                .clicked()
                            {
                                self.emulator.step_cpu();
                                self.instruction_loaded = false;
                            };

                            if ui
                                .button("Load")
                                .on_hover_text("Load next instruction")
                                .clicked()
                            {
                                self.emulator.load_next_instruction();
                                self.instruction_loaded = true;
                            };

                            if ui.button("🔃").on_hover_text("Reset").clicked() {
                                self.emulator.reset_cpu();
                                self.instruction_loaded = false;
                            };
                        });
                    });

                    row.col(|ui| {
                        ui.horizontal(|ui| {
                            let pc_override =
                                match usize::from_str_radix(self.pc_override.as_str(), 16) {
                                    Ok(val) => val,
                                    _ => 0,
                                };

                            if ui.button("Override PC").clicked() {
                                self.emulator.override_program_counter(pc_override);
                            }

                            ui.add(
                                TextEdit::singleline(&mut self.pc_override)
                                    .desired_width(40.0)
                                    .char_limit(4)
                                    .clip_text(false),
                            );
                            ui.monospace("0x");
                        });
                    });

                    row.col(|ui| {
                        ui.monospace(self.hover_text(self.emulator.cpu().instruction_register()));
                    });
                });
            });
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
                ui.monospace(format!("{:#04X}", val))
                    .on_hover_text(self.hover_text(val));
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

    pub fn show_memory_window(&mut self, ui: &mut Ui) {
        let table = TableBuilder::new(ui)
            .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
            .column(Column::auto())
            .column(Column::exact(50.0))
            .column(Column::auto());
        table.body(|mut body| {
            body.row(30.0, |mut row| {
                row.col(|ui| {
                    ui.horizontal(|ui| {
                        ui.label("Orig:");

                        let mut binary_orig_str = format!("{:X}", self.binary_orig);
                        ui.add(
                            TextEdit::singleline(&mut binary_orig_str)
                                .desired_width(40.0)
                                .char_limit(4)
                                .clip_text(false),
                        );
                        self.binary_orig = match u16::from_str_radix(binary_orig_str.as_str(), 16) {
                            Ok(val) => val,
                            _ => self.selected_memory,
                        };

                        if ui.button("Load Binary").clicked() {
                            if let Some(path) = rfd::FileDialog::new().pick_file() {
                                self.emulator.load_binary(path, self.binary_orig.into());
                            }
                        }
                    });
                });

                row.col(|_| {});

                row.col(|ui| {
                    ui.horizontal(|ui| {
                        if ui.button("⏮").clicked() {
                            self.selected_memory = 0;
                        }
                        if ui.button("⏪").clicked() {
                            self.selected_memory -= cmp::min(0x100, self.selected_memory);
                        }
                        if ui.button("◀").clicked() {
                            self.selected_memory -= cmp::min(0x10, self.selected_memory);
                        }
                        if ui.button("-1").clicked() {
                            self.selected_memory -= cmp::min(0x1, self.selected_memory);
                        }
                        ui.monospace("0x");

                        let mut memory_offset_str = format!("{:X}", self.selected_memory);
                        ui.add(
                            TextEdit::singleline(&mut memory_offset_str)
                                .desired_width(40.0)
                                .char_limit(4)
                                .clip_text(false),
                        );
                        self.selected_memory =
                            match u16::from_str_radix(memory_offset_str.as_str(), 16) {
                                Ok(val) => val,
                                _ => self.selected_memory,
                            };

                        if ui.button("+1").clicked() {
                            self.selected_memory += cmp::min(0x1, 0xFFFF - self.selected_memory);
                        }
                        if ui.button("▶").clicked() {
                            self.selected_memory += cmp::min(0x10, 0xFFFF - self.selected_memory);
                        }
                        if ui.button("⏩").clicked() {
                            self.selected_memory += cmp::min(0x100, 0xFFFF - self.selected_memory);
                        }
                        if ui.button("⏭").clicked() {
                            self.selected_memory = 0xFFFF;
                        }

                        ui.separator();

                        if ui.button("PC").clicked() {
                            self.selected_memory = self.emulator.cpu().program_counter() as u16;
                        }
                    });
                });
            });
        });

        ui.separator();

        egui::Grid::new("memory_grid")
            .num_columns(3)
            .spacing([20.0, 4.0])
            .striped(true)
            .show(ui, |ui| {
                ui.monospace("");
                ui.horizontal(|ui| {
                    for b in 0..16 {
                        ui.label(
                            egui::RichText::new(format!("{:02X}", b).as_str())
                                .strong()
                                .monospace(),
                        );
                        if b == 7 {
                            ui.monospace(' '.to_string());
                        }
                    }
                });
                ui.monospace("");
                ui.end_row();
                let start_row = self.compute_start_row();
                for r in start_row..start_row + MEMORY_ROWS {
                    self.dump_memory_row(ui, r);
                }
            });
    }

    fn dump_memory_row(&mut self, ui: &mut Ui, row: u16) {
        ui.label(
            egui::RichText::new(format!("{:04X}", row * 16))
                .strong()
                .monospace(),
        );

        let mem = self.emulator.memory();
        let mut ascii_str = String::new();
        ui.horizontal(|ui| {
            for b in 0..16 {
                let addr = (row * 16) + b;
                let byte: u8 = mem[addr as usize];
                let mut text = egui::RichText::new(format!("{:02X}", byte).as_str()).monospace();

                // Highlight the currently selected memory location as well as the stack and other well-known locations.
                if addr == self.selected_memory {
                    text = text.color(Color32::GOLD);
                } else if addr >> 8 == 0x01 {
                    text = text.color(Color32::LIGHT_YELLOW);
                } else if addr == 0xFFFA || addr == 0xFFFB {
                    text = text.color(Color32::LIGHT_RED);
                } else if addr == 0xFFFC || addr == 0xFFFD {
                    text = text.color(Color32::LIGHT_GREEN);
                } else if addr == 0xFFFE || addr == 0xFFFF {
                    text = text.color(Color32::LIGHT_BLUE);
                }

                if addr as usize == self.emulator.cpu().program_counter() {
                    text = text.background_color(Color32::DARK_GREEN);
                }

                if ui
                    .add(egui::Label::new(text).sense(egui::Sense::click()).sense(egui::Sense::hover()))
                    .on_hover_text(format!("{:08b}", byte))
                    .clicked()
                {
                    self.selected_memory = addr;
                }

                if b == 7 {
                    ui.monospace(' '.to_string());
                }

                let c: char = byte as char;
                if (0x20..=0x7E).contains(&byte) {
                    ascii_str.push(c);
                } else {
                    ascii_str.push('.');
                }
            }
        });

        ui.monospace(ascii_str);
        ui.end_row();
    }

    fn compute_start_row(&self) -> u16 {
        let start_row = (self.selected_memory - cmp::min(self.selected_memory, 0x90)) / 0x10;
        cmp::min(start_row, LAST_ROW)
    }

    fn hover_text(&self, val: u8) -> String {
        match parse_instruction(val) {
            Some(instruction) => {
                format!("{:?}", instruction)
            }
            None => "INVALID".to_string(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compute_start_row() {
        let mut frontend = Frontend::default();

        frontend.selected_memory = 0;
        assert_eq!(frontend.compute_start_row(), 0);

        frontend.selected_memory = 0x90;
        assert_eq!(frontend.compute_start_row(), 0);

        frontend.selected_memory = 0xA0;
        assert_eq!(frontend.compute_start_row(), 1);

        frontend.selected_memory = 0xF0;
        assert_eq!(frontend.compute_start_row(), 6);

        frontend.selected_memory = 0xFE30;
        assert_eq!(frontend.compute_start_row(), 4058);

        frontend.selected_memory = 0xFE40;
        assert_eq!(frontend.compute_start_row(), 4059);

        frontend.selected_memory = 0xFF40;
        assert_eq!(frontend.compute_start_row(), 4075);

        frontend.selected_memory = 0xFF50;
        assert_eq!(frontend.compute_start_row(), 4076);

        frontend.selected_memory = 0xFFFF;
        assert_eq!(frontend.compute_start_row(), 4076);
    }
}
