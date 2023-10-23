use clap::Parser;
use emu_6502::emulator::Emulator;

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Emu6502Args {
    #[arg(short, long)]
    binary_path: std::path::PathBuf,

    #[arg(short, long, default_value = "0")]
    start_address: usize,

    #[arg(short, long, default_value = "0")]
    initial_pc: usize,
}

fn main() {
    let args = Emu6502Args::parse();

    let mut emulator = Emulator::new();
    emulator.load_binary(args.binary_path, args.start_address);
    emulator.reset_cpu();

    if args.initial_pc > 0 {
        emulator.override_program_counter(args.initial_pc);
    }

    let mut step_count: u64 = 0;
    let mut pc_changed = true;
    let mut pc = emulator.cpu().program_counter();
    while pc_changed {
        emulator.step_cpu();
        step_count += 1;
        if pc == emulator.cpu().program_counter() {
            pc_changed = false;
        }
        pc = emulator.cpu().program_counter();
    }

    println!("PC did not change. Step count: {}", step_count);
    println!("{}", emulator.cpu());
}
