use crate::Memory;

use super::cpu::Cpu;

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

    pub fn cpu(&self) -> Cpu {
        self.cpu
    }

    pub fn memory(&self) -> Memory {
        self.memory
    }
}
