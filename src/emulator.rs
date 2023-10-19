use std::fs::File;
use std::io::prelude::*;
use std::io::BufReader;
use std::path::PathBuf;

use crate::cpu::Cpu;
use crate::Memory;

pub struct Emulator {
    cpu: Cpu,
    memory: Memory,
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
