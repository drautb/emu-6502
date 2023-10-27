use clap::Parser;
use emu_6502::emulator::Emulator;

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Emu6502Args {
    #[arg(short, long)]
    binary_path: std::path::PathBuf,

    #[arg(short, long, default_value = "0", value_parser = parse_usize)]
    start_address: usize,

    #[arg(short, long, default_value = "0", value_parser = parse_usize)]
    pc: usize,

    #[arg(short, long, default_value = "0", value_parser = parse_usize)]
    interrupt_address: usize,

    #[arg(short = 'r', long, default_value = "0", value_parser = parse_u8)]
    irq_mask: u8,

    #[arg(short, long, default_value = "0", value_parser = parse_u8)]
    nmi_mask: u8,
}

fn parse_usize(arg: &str) -> Result<usize, <usize as num_traits::Num>::FromStrRadixErr> {
    parse_bin_hex_dec::<usize>(arg)
}

fn parse_u8(arg: &str) -> Result<u8, <u8 as num_traits::Num>::FromStrRadixErr> {
    parse_bin_hex_dec::<u8>(arg)
}

fn parse_bin_hex_dec<T: num_traits::Num>(
    arg: &str,
) -> Result<T, <T as num_traits::Num>::FromStrRadixErr> {
    if let Some(raw) = arg.strip_prefix("0x") {
        T::from_str_radix(raw, 16)
    } else if let Some(raw) = arg.strip_prefix("0b") {
        T::from_str_radix(raw, 2)
    } else {
        T::from_str_radix(arg, 10)
    }
}

fn main() {
    let args = Emu6502Args::parse();

    let mut emulator = Emulator::new();

    println!("Loading binary: {}", args.binary_path.display());
    println!("  Start address: {:#06X}", args.start_address);
    emulator.load_binary(args.binary_path, args.start_address);

    println!(
        "Configuring interrupts at address: {:#06X}",
        args.interrupt_address
    );
    println!("  IRQ mask: {:#010b}", args.irq_mask);
    println!("  NMI mask: {:#010b}", args.nmi_mask);
    emulator.configure_interrupts(args.interrupt_address, args.irq_mask, args.nmi_mask);

    println!("Resetting CPU...");
    emulator.reset_cpu();

    if args.pc > 0 {
        println!("Overriding initial PC: {:#06X}", args.pc);
        emulator.override_program_counter(args.pc);
    }

    println!("Beginning execution...");
    emulator.unpause();
    while !emulator.is_paused() {
        emulator.clock_tick();
    }

    println!("Emulator Paused. Step count: {}", emulator.step_count());
    println!("{}", emulator.cpu());
}
