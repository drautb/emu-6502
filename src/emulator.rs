use std::fs::File;
use std::io::prelude::*;
use std::io::BufReader;
use std::path::PathBuf;

use crate::cpu::Cpu;
use crate::Memory;

pub struct Emulator {
    cpu: Cpu,
    memory: Memory,

    paused: bool,
    step_count: u64,
    pc_breakpoints: Vec<u16>,
    step_breakpoints: Vec<u64>,
}

impl Default for Emulator {
    fn default() -> Self {
        Self::new()
    }
}

impl Emulator {
    pub fn new() -> Self {
        Emulator {
            cpu: Cpu::new(),
            memory: [0; 65_536],
            paused: true,
            step_count: 0,
            pc_breakpoints: vec![],
            step_breakpoints: vec![],
        }
    }

    pub fn cpu(&self) -> &Cpu {
        &self.cpu
    }

    pub fn memory(&self) -> &Memory {
        &self.memory
    }

    pub fn reset_cpu(&mut self) {
        self.cpu.reset(&self.memory);
        self.step_count = 0;
    }

    pub fn clock_tick(&mut self) {
        let old_pc = self.cpu().program_counter();
        if !self.is_paused() {
            self.step_cpu();

            // Pause when PC doesn't change, which indicates HCF
            // https://en.wikipedia.org/wiki/Halt_and_Catch_Fire_(computing)
            if self.cpu().program_counter() == old_pc {
                self.pause();
            }
        }
    }

    pub fn load_next_instruction(&mut self) {
        self.cpu.load_instruction(&self.memory);
    }

    pub fn step_cpu(&mut self) {
        self.cpu.step(&mut self.memory);
        self.step_count += 1;

        let pc = self.cpu.program_counter() as u16;
        if self.pc_breakpoints.contains(&pc) || self.step_breakpoints.contains(&self.step_count) {
            self.pause();
        }
    }

    pub fn override_program_counter(&mut self, new_pc: usize) {
        self.cpu.override_program_counter(new_pc);
    }

    pub fn is_paused(&self) -> bool {
        self.paused
    }

    pub fn pause(&mut self) {
        self.paused = true;
    }

    pub fn unpause(&mut self) {
        self.paused = false;
    }

    pub fn step_count(&self) -> u64 {
        self.step_count
    }

    pub fn pc_breakpoints(&self) -> &Vec<u16> {
        &self.pc_breakpoints
    }

    pub fn add_pc_breakpoint(&mut self, new_breakpoint: u16) {
        self.pc_breakpoints.push(new_breakpoint);
    }

    pub fn remove_pc_breakpoint(&mut self, idx: usize) {
        self.pc_breakpoints.remove(idx);
    }

    pub fn step_breakpoints(&self) -> &Vec<u64> {
        &self.step_breakpoints
    }

    pub fn add_step_breakpoint(&mut self, new_breakpoint: u64) {
        self.step_breakpoints.push(new_breakpoint);
    }

    pub fn remove_step_breakpoint(&mut self, idx: usize) {
        self.step_breakpoints.remove(idx);
    }

    /**
     * Load a binary into memory, starting at the given start address.
     */
    pub fn load_binary(&mut self, filepath: PathBuf, start_address: usize) {
        let file = BufReader::new(File::open(filepath).expect("Failed to open file!"));
        for (idx, byte_result) in file.bytes().enumerate() {
            if let Result::Ok(byte) = byte_result {
                self.memory[start_address + idx] = byte;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn load_binary() {
        let mut emulator = Emulator::new();

        emulator.load_binary(PathBuf::from("test_programs/blink.out"), 0x200);

        assert_eq!(emulator.memory()[0x200], 0xA9);
        assert_eq!(emulator.memory()[0x201], 0xFF);
        assert_eq!(emulator.memory()[0x202], 0x8D);
        assert_eq!(emulator.memory()[0x210], 0x0A);
    }
}
