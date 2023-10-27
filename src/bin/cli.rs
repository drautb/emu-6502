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

    emulator.unpause();
    while !emulator.is_paused() {
        emulator.clock_tick();
    }

    println!("PC did not change. Step count: {}", emulator.step_count());
    println!("{}", emulator.cpu());
}
