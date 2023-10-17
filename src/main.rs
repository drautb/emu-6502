use emu_6502::cpu::Cpu;

use std::io::{stdin, stdout, Read, Write};

fn pause() {
    let mut stdout = stdout();
    stdout.write_all(b"Press any key to continue...").unwrap();
    stdout.flush().unwrap();
    stdin().read_exact(&mut [0]).unwrap();
}

fn main() {
    let mut cpu = Cpu::new();
    let mut mem = [0; 65_536];

    loop {
        cpu.step(&mut mem);
        println!("{}", cpu);
        pause();
    }
}
