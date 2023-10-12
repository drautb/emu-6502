#![allow(non_camel_case_types)]

/**
 * https://www.westerndesigncenter.com/wdc/documentation/w65c02s.pdf
 * https://eater.net/datasheets/w65c02s.pdf
 * https://www.pagetable.com/c64ref/6502/?cpu=65c02s
 */
use std::fmt;
use std::num::Wrapping;
use std::ops::Index;
use std::ops::IndexMut;

#[derive(Debug)]
enum AddressMode {
    ACC,
    IMMEDIATE,
    ABS,
    AIX,
    AIY,
    AI,
    AII,
    ZP,
    ZPIX,
    ZPIY,
    ZPI,
    ZPII,
    ZPIIY,
}

#[derive(Debug)]
enum Instruction {
    ADC(u8, AddressMode), // ADd memory to accumulator with Carry
    AND(u8, AddressMode), // AND memory with accumulator
    ASL(u8, AddressMode), // Arithmetic Shift one bit Left, memory or accumulator

    BBR(u8, u8), // Branch on Bit Reset
    BBS(u8, u8), // Branch on Bit Set

    BCC(u8), // Branch on Carry Clear (Pc=0)
    BCS(u8), // Branch on Carry Set (Pc=1)
    BEQ(u8), // Branch if EQual (Pz=1)

    BIT(u8, AddressMode), // BIt Test

    BMI(u8), // Branch if result MInus (Pn=1)
    BNE(u8), // Branch if result Not Equal (Pz=0)
    BPL(u8), // Branch if result PLus (Pn=0)
    BRA(u8), // BRanch Always
    BRK(u8), // BReaK instruction
    BVC(u8), // Branch on oVerflow Clear (Pv=0)
    BVS(u8), // Branch on oVerflow Set (Pv=1)

    CLC(u8), // CLear Carry flag
    CLD(u8), // CLear Decimal mode
    CLI(u8), // CLear Interrupt disable bit
    CLV(u8), // CLear oVerflow flag

    CMP(u8, AddressMode), // CoMPare memory and accumulator

    CPX(u8, AddressMode), // ComPare memory and X register

    CPY(u8, AddressMode), // ComPare memory and Y register

    DEC(u8, AddressMode), // DECrement memory or accumulate by one

    DEX(u8), // DEcrement X by one
    DEY(u8), // DEcrement Y by one

    EOR(u8, AddressMode), // Exclusive OR memory with accumulate

    INC(u8, AddressMode), // INCrement memory or accumulate by one

    INX(u8), // INcrement X register by one
    INY(u8), // INcrement Y register by one

    JMP(u8, AddressMode), // JuMP to new location

    JSR(u8), // Jump to new location Saving Return (Jump to SubRoutine)

    LDA(u8, AddressMode), // LoaD Accumulator with memory

    LDX(u8, AddressMode), // LoaD the X register with memory

    LDY(u8, AddressMode), // LoaD the Y register with memory

    LSR(u8, AddressMode), // Logical Shift one bit Right memory or accumulator

    NOP(u8), // No OPeration

    ORA(u8, AddressMode), // "OR" memory with Accumulator

    PHA(u8), // PusH Accumulator on stack
    PHP(u8), // PusH Processor status on stack
    PHX(u8), // PusH X register on stack
    PHY(u8), // PusH Y register on stack
    PLA(u8), // PuLl Accumulator from stack
    PLP(u8), // PuLl Processor status from stack
    PLX(u8), // PuLl X register from stack
    PLY(u8), // PuLl Y register from stack

    RMB(u8, u8), // Reset Memory Bit

    ROL(u8, AddressMode), // ROtate one bit Left memory or accumulator

    ROR(u8, AddressMode), // ROtate one bit Right memory or accumulator

    RTI(u8), // ReTurn from Interrupt
    RTS(u8), // ReTurn from Subroutine

    SBC(u8, AddressMode), // SuBtract memory from accumulator with borrow (Carry bit)

    SEC(u8), // SEt Carry
    SED(u8), // SEt Decimal mode
    SEI(u8), // SEt Interrupt disable status

    SMB(u8, u8), // Set Memory Bit

    STA(u8, AddressMode), // STore Accumulator in memory

    STP(u8), // SToP mode

    STX(u8, AddressMode), // STore the X register in memory

    STY(u8, AddressMode), // STore the Y register in memory

    STZ(u8, AddressMode), // STore Zero in memory

    TAX(u8), // Transfer the Accumulator to the X register
    TAY(u8), // Transfer the Accumulator to the Y register

    TRB(u8, AddressMode), // Test and Reset memory Bit

    TSB(u8, AddressMode), // Test and Set memory Bit

    TSX(u8), // Transfer the Stack pointer to the X register
    TXA(u8), // Transfer the X register to the Accumulator
    TXS(u8), // Transfer the X register to the Stack pointer register
    TYA(u8), // Transfer Y register to the Accumulator

    WAI(u8), // WAit for Interrupt
}

fn load_instruction(opcode: u8) -> Instruction {
    match opcode {
        0x69 => Instruction::ADC(opcode, AddressMode::IMMEDIATE),
        0x6D => Instruction::ADC(opcode, AddressMode::ABS),
        0x7D => Instruction::ADC(opcode, AddressMode::AIX),
        0x79 => Instruction::ADC(opcode, AddressMode::AIY),
        0x65 => Instruction::ADC(opcode, AddressMode::ZP),
        0x75 => Instruction::ADC(opcode, AddressMode::ZPIX),
        0x72 => Instruction::ADC(opcode, AddressMode::ZPI),
        0x61 => Instruction::ADC(opcode, AddressMode::ZPII),
        0x71 => Instruction::ADC(opcode, AddressMode::ZPIIY),

        0x29 => Instruction::AND(opcode, AddressMode::IMMEDIATE),
        0x2D => Instruction::AND(opcode, AddressMode::ABS),
        0x3D => Instruction::AND(opcode, AddressMode::AIX),
        0x39 => Instruction::AND(opcode, AddressMode::AIY),
        0x25 => Instruction::AND(opcode, AddressMode::ZP),
        0x35 => Instruction::AND(opcode, AddressMode::ZPIX),
        0x32 => Instruction::AND(opcode, AddressMode::ZPI),
        0x21 => Instruction::AND(opcode, AddressMode::ZPII),
        0x31 => Instruction::AND(opcode, AddressMode::ZPIIY),

        0x0A => Instruction::ASL(opcode, AddressMode::ACC),
        0x0E => Instruction::ASL(opcode, AddressMode::ABS),
        0x1E => Instruction::ASL(opcode, AddressMode::AIX),
        0x06 => Instruction::ASL(opcode, AddressMode::ZP),
        0x16 => Instruction::ASL(opcode, AddressMode::ZPIX),

        0x0F => Instruction::BBR(opcode, 0),
        0x1F => Instruction::BBR(opcode, 1),
        0x2F => Instruction::BBR(opcode, 2),
        0x3F => Instruction::BBR(opcode, 3),
        0x4F => Instruction::BBR(opcode, 4),
        0x5F => Instruction::BBR(opcode, 5),
        0x6F => Instruction::BBR(opcode, 6),
        0x7F => Instruction::BBR(opcode, 7),

        0x8F => Instruction::BBS(opcode, 0),
        0x9F => Instruction::BBS(opcode, 1),
        0xAF => Instruction::BBS(opcode, 2),
        0xBF => Instruction::BBS(opcode, 3),
        0xCF => Instruction::BBS(opcode, 4),
        0xDF => Instruction::BBS(opcode, 5),
        0xEF => Instruction::BBS(opcode, 6),
        0xFF => Instruction::BBS(opcode, 7),

        0x90 => Instruction::BCC(opcode),
        0xB0 => Instruction::BCS(opcode),
        0xF0 => Instruction::BEQ(opcode),

        0x89 => Instruction::BIT(opcode, AddressMode::IMMEDIATE),
        0x2C => Instruction::BIT(opcode, AddressMode::ABS),
        0x3C => Instruction::BIT(opcode, AddressMode::AIX),
        0x24 => Instruction::BIT(opcode, AddressMode::ZP),
        0x34 => Instruction::BIT(opcode, AddressMode::ZPIX),

        0x30 => Instruction::BMI(opcode),
        0xD0 => Instruction::BNE(opcode),
        0x10 => Instruction::BPL(opcode),
        0x80 => Instruction::BRA(opcode),
        0x00 => Instruction::BRK(opcode),
        0x50 => Instruction::BVC(opcode),
        0x70 => Instruction::BVS(opcode),

        0x18 => Instruction::CLC(opcode),
        0xD8 => Instruction::CLD(opcode),
        0x58 => Instruction::CLI(opcode),
        0xB8 => Instruction::CLV(opcode),

        0xC9 => Instruction::CMP(opcode, AddressMode::IMMEDIATE),
        0xCD => Instruction::CMP(opcode, AddressMode::ABS),
        0xDD => Instruction::CMP(opcode, AddressMode::AIX),
        0xD9 => Instruction::CMP(opcode, AddressMode::AIY),
        0xC5 => Instruction::CMP(opcode, AddressMode::ZP),
        0xD5 => Instruction::CMP(opcode, AddressMode::ZPIX),
        0xD2 => Instruction::CMP(opcode, AddressMode::ZPI),
        0xC1 => Instruction::CMP(opcode, AddressMode::ZPII),
        0xD1 => Instruction::CMP(opcode, AddressMode::ZPIIY),

        0xE0 => Instruction::CPX(opcode, AddressMode::IMMEDIATE),
        0xEC => Instruction::CPX(opcode, AddressMode::ABS),
        0xE4 => Instruction::CPX(opcode, AddressMode::ZP),

        0xC0 => Instruction::CPY(opcode, AddressMode::IMMEDIATE),
        0xCC => Instruction::CPY(opcode, AddressMode::ABS),
        0xC4 => Instruction::CPY(opcode, AddressMode::ZP),

        0x3A => Instruction::DEC(opcode, AddressMode::ACC),
        0xCE => Instruction::DEC(opcode, AddressMode::ABS),
        0xDE => Instruction::DEC(opcode, AddressMode::AIX),
        0xC6 => Instruction::DEC(opcode, AddressMode::ZP),
        0xD6 => Instruction::DEC(opcode, AddressMode::ZPIX),

        0xCA => Instruction::DEX(opcode),
        0x88 => Instruction::DEY(opcode),

        0x49 => Instruction::EOR(opcode, AddressMode::IMMEDIATE),
        0x4D => Instruction::EOR(opcode, AddressMode::ABS),
        0x5D => Instruction::EOR(opcode, AddressMode::AIX),
        0x59 => Instruction::EOR(opcode, AddressMode::AIY),
        0x45 => Instruction::EOR(opcode, AddressMode::ZP),
        0x55 => Instruction::EOR(opcode, AddressMode::ZPIX),
        0x52 => Instruction::EOR(opcode, AddressMode::ZPI),
        0x41 => Instruction::EOR(opcode, AddressMode::ZPII),
        0x51 => Instruction::EOR(opcode, AddressMode::ZPIIY),

        0x1A => Instruction::INC(opcode, AddressMode::ACC),
        0xEE => Instruction::INC(opcode, AddressMode::ABS),
        0xFE => Instruction::INC(opcode, AddressMode::AIX),
        0xE6 => Instruction::INC(opcode, AddressMode::ZP),
        0xF6 => Instruction::INC(opcode, AddressMode::ZPIX),

        0xE8 => Instruction::INX(opcode),
        0xC8 => Instruction::INY(opcode),

        0x4C => Instruction::JMP(opcode, AddressMode::ABS),
        0x6C => Instruction::JMP(opcode, AddressMode::AI),
        0x7C => Instruction::JMP(opcode, AddressMode::AII),

        0x20 => Instruction::JSR(opcode),

        0xA9 => Instruction::LDA(opcode, AddressMode::IMMEDIATE),
        0xAD => Instruction::LDA(opcode, AddressMode::ABS),
        0xBD => Instruction::LDA(opcode, AddressMode::AIX),
        0xB9 => Instruction::LDA(opcode, AddressMode::AIY),
        0xA5 => Instruction::LDA(opcode, AddressMode::ZP),
        0xB5 => Instruction::LDA(opcode, AddressMode::ZPIX),
        0xB2 => Instruction::LDA(opcode, AddressMode::ZPI),
        0xA1 => Instruction::LDA(opcode, AddressMode::ZPII),
        0xB1 => Instruction::LDA(opcode, AddressMode::ZPIIY),

        0xA2 => Instruction::LDX(opcode, AddressMode::IMMEDIATE),
        0xAE => Instruction::LDX(opcode, AddressMode::ABS),
        0xBE => Instruction::LDX(opcode, AddressMode::AIY),
        0xA6 => Instruction::LDX(opcode, AddressMode::ZP),
        0xB6 => Instruction::LDX(opcode, AddressMode::ZPIY),

        0xA0 => Instruction::LDY(opcode, AddressMode::IMMEDIATE),
        0xAC => Instruction::LDY(opcode, AddressMode::ABS),
        0xBC => Instruction::LDY(opcode, AddressMode::AIX),
        0xA4 => Instruction::LDY(opcode, AddressMode::ZP),
        0xB4 => Instruction::LDY(opcode, AddressMode::ZPIX),

        0x4A => Instruction::LSR(opcode, AddressMode::ACC),
        0x4E => Instruction::LSR(opcode, AddressMode::ABS),
        0x5E => Instruction::LSR(opcode, AddressMode::AIX),
        0x46 => Instruction::LSR(opcode, AddressMode::ZP),
        0x56 => Instruction::LSR(opcode, AddressMode::ZPIX),

        0xEA => Instruction::NOP(opcode),

        0x09 => Instruction::ORA(opcode, AddressMode::IMMEDIATE),
        0x0D => Instruction::ORA(opcode, AddressMode::ABS),
        0x1D => Instruction::ORA(opcode, AddressMode::AIX),
        0x19 => Instruction::ORA(opcode, AddressMode::AIY),
        0x05 => Instruction::ORA(opcode, AddressMode::ZP),
        0x15 => Instruction::ORA(opcode, AddressMode::ZPIX),
        0x12 => Instruction::ORA(opcode, AddressMode::ZPI),
        0x01 => Instruction::ORA(opcode, AddressMode::ZPII),
        0x11 => Instruction::ORA(opcode, AddressMode::ZPIIY),

        0x48 => Instruction::PHA(opcode),
        0x08 => Instruction::PHP(opcode),
        0xDA => Instruction::PHX(opcode),
        0x5A => Instruction::PHY(opcode),
        0x68 => Instruction::PLA(opcode),
        0x28 => Instruction::PLP(opcode),
        0xFA => Instruction::PLX(opcode),
        0x7A => Instruction::PLY(opcode),

        0x07 => Instruction::RMB(opcode, 0),
        0x17 => Instruction::RMB(opcode, 1),
        0x27 => Instruction::RMB(opcode, 2),
        0x37 => Instruction::RMB(opcode, 3),
        0x47 => Instruction::RMB(opcode, 4),
        0x57 => Instruction::RMB(opcode, 5),
        0x67 => Instruction::RMB(opcode, 6),
        0x77 => Instruction::RMB(opcode, 7),

        0x2A => Instruction::ROL(opcode, AddressMode::ACC),
        0x2E => Instruction::ROL(opcode, AddressMode::ABS),
        0x3E => Instruction::ROL(opcode, AddressMode::AIX),
        0x26 => Instruction::ROL(opcode, AddressMode::ZP),
        0x36 => Instruction::ROL(opcode, AddressMode::ZPIX),

        0x6A => Instruction::ROR(opcode, AddressMode::ACC),
        0x6E => Instruction::ROR(opcode, AddressMode::ABS),
        0x7E => Instruction::ROR(opcode, AddressMode::AIX),
        0x66 => Instruction::ROR(opcode, AddressMode::ZP),
        0x76 => Instruction::ROR(opcode, AddressMode::ZPIX),

        0x40 => Instruction::RTI(opcode),
        0x60 => Instruction::RTS(opcode),

        0xE9 => Instruction::SBC(opcode, AddressMode::IMMEDIATE),
        0xED => Instruction::SBC(opcode, AddressMode::ABS),
        0xFD => Instruction::SBC(opcode, AddressMode::AIX),
        0xF9 => Instruction::SBC(opcode, AddressMode::AIY),
        0xE5 => Instruction::SBC(opcode, AddressMode::ZP),
        0xF5 => Instruction::SBC(opcode, AddressMode::ZPIX),
        0xF2 => Instruction::SBC(opcode, AddressMode::ZPI),
        0xE1 => Instruction::SBC(opcode, AddressMode::ZPII),
        0xF1 => Instruction::SBC(opcode, AddressMode::ZPIIY),

        0x38 => Instruction::SEC(opcode),
        0xF8 => Instruction::SED(opcode),
        0x78 => Instruction::SEI(opcode),

        0x87 => Instruction::SMB(opcode, 0),
        0x97 => Instruction::SMB(opcode, 1),
        0xA7 => Instruction::SMB(opcode, 2),
        0xB7 => Instruction::SMB(opcode, 3),
        0xC7 => Instruction::SMB(opcode, 4),
        0xD7 => Instruction::SMB(opcode, 5),
        0xE7 => Instruction::SMB(opcode, 6),
        0xF7 => Instruction::SMB(opcode, 7),

        0x8D => Instruction::STA(opcode, AddressMode::ABS),
        0x9D => Instruction::STA(opcode, AddressMode::AIX),
        0x99 => Instruction::STA(opcode, AddressMode::AIY),
        0x85 => Instruction::STA(opcode, AddressMode::ZP),
        0x95 => Instruction::STA(opcode, AddressMode::ZPIX),
        0x92 => Instruction::STA(opcode, AddressMode::ZPI),
        0x81 => Instruction::STA(opcode, AddressMode::ZPII),
        0x91 => Instruction::STA(opcode, AddressMode::ZPIIY),

        0xDB => Instruction::STP(opcode),

        0x8E => Instruction::STX(opcode, AddressMode::ABS),
        0x86 => Instruction::STX(opcode, AddressMode::ZP),
        0x96 => Instruction::STX(opcode, AddressMode::ZPIY),

        0x8C => Instruction::STY(opcode, AddressMode::ABS),
        0x84 => Instruction::STY(opcode, AddressMode::ZP),
        0x94 => Instruction::STY(opcode, AddressMode::ZPIX),

        0x9C => Instruction::STZ(opcode, AddressMode::ABS),
        0x9E => Instruction::STZ(opcode, AddressMode::AIX),
        0x64 => Instruction::STZ(opcode, AddressMode::ZP),
        0x74 => Instruction::STZ(opcode, AddressMode::ZPIX),

        0xAA => Instruction::TAX(opcode),
        0xA8 => Instruction::TAY(opcode),

        0x1C => Instruction::TRB(opcode, AddressMode::ABS),
        0x14 => Instruction::TRB(opcode, AddressMode::ZP),

        0x0C => Instruction::TSB(opcode, AddressMode::ABS),
        0x04 => Instruction::TSB(opcode, AddressMode::ZP),

        0xBA => Instruction::TSX(opcode),
        0x8A => Instruction::TXA(opcode),
        0x9A => Instruction::TXS(opcode),
        0x98 => Instruction::TYA(opcode),

        0xCB => Instruction::WAI(opcode),

        _ => {
            println!("Unrecognized opcode! {}", opcode);
            panic!("Unrecognized opcode!")
        }
    }
}

const PN_MASK: u8 = 0b10000000;
const PV_MASK: u8 = 0b01000000;
// const PB_MASK: u8 = 0b00010000;
// const PD_MASK: u8 = 0b00001000;
// const PI_MASK: u8 = 0b00000100;
const PZ_MASK: u8 = 0b00000010;
const PC_MASK: u8 = 0b00000001;

fn inc_wrap(n: u8) -> u8 {
    add_wrap(n, 1)
}

fn add_wrap(n1: u8, n2: u8) -> u8 {
    (Wrapping(n1) + Wrapping(n2)).0
}

#[derive(PartialEq)]
pub struct Cpu {
    ir: u8, // instruction register
    a: u8,  // accumulator
    x: u8,  // index registers
    y: u8,
    p: u8,     // processor status
    pc: usize, // program counter
    s: u8,     // stack pointer
}

impl fmt::Debug for Cpu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Cpu")
            .field("ir", &format_args!("{:#02X}", self.ir))
            .field("a", &self.a)
            .field("x", &self.x)
            .field("y", &self.y)
            .field("p", &format_args!("{:08b}", self.p))
            .field("pc", &format_args!("{:#06X}", self.pc))
            .field("s", &self.s)
            .finish()
    }
}

impl fmt::Display for Cpu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "\n| ----------------------------------------- |\n\
        | PC | {:#06X} | {:5} | {:04b} {:04b} {:04b} {:04b} |\n\
        | ----------------------------------------- |\n\
        | IR |   {:#04X} | {:5} | {:04b} {:04b}           |\n\
        |  A |   {:#04X} | {:5} | {:04b} {:04b}           |\n\
        |  X |   {:#04X} | {:5} | {:04b} {:04b}           |\n\
        |  Y |   {:#04X} | {:5} | {:04b} {:04b}           |\n\
        |  S |   {:#04X} | {:5} | {:04b} {:04b}           |\n\
        | ----------------------------------------- |\n\
        |  P | NV_BDIZC                             |\n\
        |    | {:08b}                             |\n\
        | ----------------------------------------- |\n",
            self.pc,
            self.pc,
            self.pc >> 12,
            self.pc >> 8 & 15,
            self.pc >> 4 & 15,
            self.pc & 15,
            self.ir,
            self.ir,
            self.ir >> 4,
            self.ir & 15,
            self.a,
            self.a,
            self.a >> 4,
            self.a & 15,
            self.x,
            self.x,
            self.x >> 4,
            self.x & 15,
            self.y,
            self.y,
            self.y >> 4,
            self.y & 15,
            self.s,
            self.s,
            self.s >> 4,
            self.s & 15,
            self.p
        )
    }
}

impl Cpu {
    fn instruction_length(address_mode: AddressMode) -> usize {
        match address_mode {
            AddressMode::ACC => 1,
            AddressMode::IMMEDIATE => 2,
            AddressMode::ABS => 3,
            AddressMode::AIX => 3,
            AddressMode::AIY => 3,
            AddressMode::AI => 3,
            AddressMode::AII => 3,
            AddressMode::ZP => 2,
            AddressMode::ZPIX => 2,
            AddressMode::ZPIY => 2,
            AddressMode::ZPI => 2,
            AddressMode::ZPII => 2,
            AddressMode::ZPIIY => 2,
        }
    }

    pub fn new() -> Self {
        Cpu {
            ir: 0,
            a: 0,
            x: 0,
            y: 0,
            p: 0,
            pc: 0,
            s: 0,
        }
    }

    pub fn reset(&mut self) {
        self.ir = 0;
        self.a = 0;
        self.x = 0;
        self.y = 0;
        self.p = 0;
        self.pc = 0;
        self.s = 0;
    }

    pub fn step<M, R>(&mut self, rom: &R, mem: &mut M)
    where
        R: Index<usize, Output = u8>,
        M: IndexMut<usize, Output = u8>,
    {
        let opcode = rom[self.pc];
        let instruction = load_instruction(opcode);
        self.ir = opcode;
        match instruction {
            Instruction::ADC(_, address_mode) => {
                let n1 = self.a;
                let n2 = self.resolve_operand(&address_mode, rom, mem);
                self.a = add_wrap(n1, n2);
                self.update_status_nz(self.a);
                self.update_status_c(self.a, n1);
                self.update_status_v(self.a, n1, n2);
                self.update_pc(address_mode);
            }

            Instruction::AND(_, address_mode) => {
                self.a &= self.resolve_operand(&address_mode, rom, mem);
                self.update_status_nz(self.a);
                self.update_pc(address_mode);
            }

            Instruction::ASL(_, AddressMode::ACC) => {
                self.p = (self.p & !PC_MASK) | (self.a >> 7);
                self.a <<= 1;
                self.update_status_nz(self.a);
                self.update_pc(AddressMode::ACC);
            }

            Instruction::ASL(_, address_mode) => {
                let resolved_addr = self.resolve_operand_addr(&address_mode, rom, mem);
                let val = mem[resolved_addr];
                self.p = (self.p & !PC_MASK) | (val >> 7);
                let result = val << 1;
                mem[resolved_addr] = result;
                self.update_status_nz(result);
                self.update_pc(address_mode);
            }

            Instruction::BBR(_, bit) => {
                let test = mem[self.resolve_zp(rom)];
                let offset = self.third_byte_operand(rom);
                if test & (1 << bit) == 0 {
                    self.pc += offset as usize;
                } else {
                    self.pc += 3;
                }
            }

            Instruction::BBS(_, bit) => {
                let test = mem[self.resolve_zp(rom)];
                let offset = self.third_byte_operand(rom);
                if test & (1 << bit) > 0 {
                    self.pc += offset as usize;
                } else {
                    self.pc += 3;
                }
            }

            Instruction::BCC(_) => {
                self.pc += if self.p & PC_MASK == 0 {
                    self.second_byte_operand(rom) as usize
                } else {
                    2
                }
            }

            Instruction::BCS(_) => {
                self.pc += if self.p & PC_MASK > 0 {
                    self.second_byte_operand(rom) as usize
                } else {
                    2
                }
            }

            Instruction::BEQ(_) => {
                self.pc += if self.p & PZ_MASK > 0 {
                    self.second_byte_operand(rom) as usize
                } else {
                    2
                }
            }

            Instruction::INX(_) => {
                self.x = inc_wrap(self.x);
                self.update_status_nz(self.x);
                self.incr_pc();
            }

            Instruction::INY(_) => {
                self.y = inc_wrap(self.y);
                self.update_status_nz(self.y);
                self.incr_pc();
            }

            Instruction::JMP(_, AddressMode::ABS) => self.pc = self.two_byte_operand(rom) as usize,
            Instruction::JMP(_, AddressMode::AI) => {
                let new_pc_addr = self.two_byte_operand(rom) as usize;
                self.pc = self.deref_mem(mem, new_pc_addr) as usize;
            }
            Instruction::JMP(_, AddressMode::AII) => {
                let new_pc_addr: usize = (self.two_byte_operand(rom) + self.x as u16) as usize;
                self.pc = self.deref_mem(mem, new_pc_addr) as usize;
            }

            Instruction::LDA(_, AddressMode::IMMEDIATE) => {
                self.a = self.second_byte_operand(rom);
                self.update_pc(AddressMode::IMMEDIATE);
            }

            Instruction::NOP(_) => {
                self.incr_pc();
            }

            Instruction::STA(_, AddressMode::ABS) => {
                let addr = self.two_byte_operand(rom);
                mem[addr as usize] = self.a;
                self.update_pc(AddressMode::ABS);
            }

            instruction => {
                println!("Not implemented: {:?}", instruction);
                todo!();
            }
        }
    }

    fn resolve_operand_addr<R, M>(&mut self, address_mode: &AddressMode, rom: &R, mem: &M) -> usize
    where
        R: Index<usize, Output = u8>,
        M: IndexMut<usize, Output = u8>,
    {
        match address_mode {
            AddressMode::ABS => self.resolve_abs(rom),
            AddressMode::AIX => self.resolve_aix(rom),
            AddressMode::AIY => self.resolve_aiy(rom),
            AddressMode::ZP => self.resolve_zp(rom),
            AddressMode::ZPIX => self.resolve_zpix(rom),
            AddressMode::ZPI => self.resolve_zpi(rom, mem),
            AddressMode::ZPII => self.resolve_zpii(rom, mem),
            AddressMode::ZPIIY => self.resolve_zpiiy(rom, mem),
            _ => {
                println!(
                    "Unable to resolve operand address for mode {:?}",
                    address_mode
                );
                panic!("Unable to resolve operand address for mode");
            }
        }
    }

    fn resolve_operand<R, M>(&mut self, address_mode: &AddressMode, rom: &R, mem: &M) -> u8
    where
        R: Index<usize, Output = u8>,
        M: IndexMut<usize, Output = u8>,
    {
        match address_mode {
            AddressMode::IMMEDIATE => self.second_byte_operand(rom),
            _ => mem[self.resolve_operand_addr(address_mode, rom, mem)],
        }
    }

    fn second_byte_operand<R>(&self, rom: &R) -> u8
    where
        R: Index<usize, Output = u8>,
    {
        rom[self.pc + 1]
    }

    fn third_byte_operand<R>(&self, rom: &R) -> u8
    where
        R: Index<usize, Output = u8>,
    {
        rom[self.pc + 2]
    }

    fn two_byte_operand<R>(&self, rom: &R) -> u16
    where
        R: Index<usize, Output = u8>,
    {
        let op_l: u16 = rom[self.pc + 1].into();
        let op_h: u16 = rom[self.pc + 2].into();
        (op_h << 8) | op_l
    }

    fn resolve_abs<R>(&mut self, rom: &R) -> usize
    where
        R: Index<usize, Output = u8>,
    {
        self.two_byte_operand(rom) as usize
    }

    fn resolve_aix<R>(&self, rom: &R) -> usize
    where
        R: Index<usize, Output = u8>,
    {
        (self.two_byte_operand(rom) + self.x as u16) as usize
    }

    fn resolve_aiy<R>(&self, rom: &R) -> usize
    where
        R: Index<usize, Output = u8>,
    {
        (self.two_byte_operand(rom) + self.y as u16) as usize
    }

    fn resolve_zp<R>(&self, rom: &R) -> usize
    where
        R: Index<usize, Output = u8>,
    {
        self.second_byte_operand(rom) as usize
    }

    fn resolve_zpix<R>(&self, rom: &R) -> usize
    where
        R: Index<usize, Output = u8>,
    {
        (self.second_byte_operand(rom) + self.x) as usize
    }

    fn resolve_zpi<R, M>(&self, rom: &R, mem: &M) -> usize
    where
        R: Index<usize, Output = u8>,
        M: IndexMut<usize, Output = u8>,
    {
        let indirect_address = self.second_byte_operand(rom);
        let operand_address: u16 = self.deref_mem(mem, indirect_address as usize);
        operand_address as usize
    }

    fn resolve_zpii<R, M>(&self, rom: &R, mem: &M) -> usize
    where
        R: Index<usize, Output = u8>,
        M: IndexMut<usize, Output = u8>,
    {
        let indirect_address = self.second_byte_operand(rom) + self.x;
        let operand_address: u16 = self.deref_mem(mem, indirect_address as usize);
        operand_address as usize
    }

    fn resolve_zpiiy<R, M>(&self, rom: &R, mem: &M) -> usize
    where
        R: Index<usize, Output = u8>,
        M: IndexMut<usize, Output = u8>,
    {
        // Deref the zero page pointer
        let zp = self.second_byte_operand(rom);
        let indirect_base = self.deref_mem(mem, zp as usize);

        // Add y to the address found
        let indirect_address = indirect_base + self.y as u16;

        // Deref new address to get operand
        indirect_address as usize
    }

    fn deref_mem<M>(&self, mem: &M, addr: usize) -> u16
    where
        M: IndexMut<usize, Output = u8>,
    {
        let new_addr_l: u16 = mem[addr as usize].into();
        let new_addr_h: u16 = mem[(addr + 1) as usize].into();
        (new_addr_h << 8) | new_addr_l
    }

    fn update_status_nz(&mut self, value: u8) {
        // Negative
        self.p = (PN_MASK & value) | (!PN_MASK & self.p);

        // Zero
        if value == 0 {
            self.p |= PZ_MASK;
        } else {
            self.p &= !PZ_MASK;
        }
    }

    fn update_status_c(&mut self, result: u8, original: u8) {
        // Carry
        if original > result {
            self.p |= PC_MASK;
        } else {
            self.p &= !PC_MASK;
        }
    }

    fn update_status_v(&mut self, result: u8, n1: u8, n2: u8) {
        // oVerflow
        if n1 & PN_MASK == n2 & PN_MASK && n1 & PN_MASK != result & PN_MASK {
            self.p |= PV_MASK;
        } else {
            self.p &= !PV_MASK;
        }
    }

    fn update_pc(&mut self, address_mode: AddressMode) {
        self.pc += Cpu::instruction_length(address_mode);
    }

    fn incr_pc(&mut self) {
        self.pc += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn setup() -> (Cpu, [u8; 65_536]) {
        (Cpu::new(), [0; 65_536])
    }

    #[test]
    fn reset_cpu() {
        let mut cpu = Cpu {
            ir: 1,
            a: 2,
            x: 3,
            y: 4,
            p: 5,
            pc: 6,
            s: 7,
        };

        cpu.reset();

        assert_eq!(cpu, Cpu::new());
    }

    mod adc_tests {
        use super::*;

        #[test]
        fn adc() {
            let (mut cpu, mut mem) = setup();
            cpu.a = 40;
            cpu.p = PC_MASK | PZ_MASK | PN_MASK | PV_MASK; // These should all be cleared
            let rom = vec![0x69, 2];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x69,
                    pc: 2,
                    a: 42,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn adc_carry() {
            let (mut cpu, mut mem) = setup();
            cpu.a = 255;
            let rom = vec![0x69, 1];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x69,
                    pc: 2,
                    a: 0,
                    p: PZ_MASK | PC_MASK,
                    ..Cpu::new()
                }
            )
        }
    }

    mod and_tests {
        use super::*;

        #[test]
        fn and_immediate() {
            let (mut cpu, mut mem) = setup();
            cpu.a = 0b00111100;
            let rom = vec![0x29, 0b00001111];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x29,
                    pc: 2,
                    a: 0b00001100,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn and_immediate_pn() {
            let (mut cpu, mut mem) = setup();
            cpu.a = 0b10000000;
            let rom = vec![0x29, 0b10001111];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x29,
                    pc: 2,
                    a: 0b10000000,
                    p: 0b10000000,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn and_immediate_pz() {
            let (mut cpu, mut mem) = setup();
            cpu.a = 0;
            let rom = vec![0x29, 0xFF];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x29,
                    pc: 2,
                    a: 0,
                    p: 0b00000010,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn and_abs() {
            let (mut cpu, mut mem) = setup();
            cpu.a = 0b10001111;
            mem[0xABCD] = 0b10111100;
            let rom = vec![0x2D, 0xCD, 0xAB];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x2D,
                    pc: 3,
                    a: 0b10001100,
                    p: 0b10000000,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn and_aix() {
            let (mut cpu, mut mem) = setup();
            cpu.a = 0b10001111;
            cpu.x = 10;
            mem[0xABCA] = 0b10111100;
            let rom = vec![0x3D, 0xC0, 0xAB];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x3D,
                    pc: 3,
                    a: 0b10001100,
                    x: 10,
                    p: 0b10000000,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn and_aiy() {
            let (mut cpu, mut mem) = setup();
            cpu.a = 0b10001111;
            cpu.y = 10;
            mem[0xABCA] = 0b10111100;
            let rom = vec![0x39, 0xC0, 0xAB];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x39,
                    pc: 3,
                    a: 0b10001100,
                    y: 10,
                    p: 0b10000000,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn and_zp() {
            let (mut cpu, mut mem) = setup();
            cpu.a = 0b10001111;
            mem[0x00CD] = 0b10111100;
            let rom = vec![0x25, 0xCD];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x25,
                    pc: 2,
                    a: 0b10001100,
                    p: 0b10000000,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn and_zpix() {
            let (mut cpu, mut mem) = setup();
            cpu.a = 0b10001111;
            cpu.x = 10;
            mem[0x00CA] = 0b10111100;
            let rom = vec![0x35, 0xC0];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x35,
                    pc: 2,
                    a: 0b10001100,
                    x: 10,
                    p: 0b10000000,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn and_zpi() {
            let (mut cpu, mut mem) = setup();
            cpu.a = 0b10001111;
            mem[0x00CD] = 0x57;
            mem[0x00CE] = 0x43;
            mem[0x4357] = 0b10111100;
            let rom = vec![0x32, 0xCD];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x32,
                    pc: 2,
                    a: 0b10001100,
                    p: 0b10000000,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn and_zpii() {
            let (mut cpu, mut mem) = setup();
            cpu.a = 0b10001111;
            cpu.x = 10;
            mem[0x00CA] = 0x57;
            mem[0x00CB] = 0x43;
            mem[0x4357] = 0b10111100;
            let rom = vec![0x21, 0xC0];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x21,
                    pc: 2,
                    a: 0b10001100,
                    x: 10,
                    p: 0b10000000,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn and_zpiiy() {
            let (mut cpu, mut mem) = setup();
            cpu.a = 0b10001111;
            cpu.y = 10;
            mem[0x00C0] = 0x00;
            mem[0x00C1] = 0xFF;
            mem[0xFF0A] = 0b10111100;
            let rom = vec![0x31, 0xC0];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x31,
                    pc: 2,
                    a: 0b10001100,
                    y: 10,
                    p: 0b10000000,
                    ..Cpu::new()
                }
            )
        }
    }

    mod asl_tests {
        use super::*;

        #[test]
        fn asl_acc() {
            let (mut cpu, mut mem) = setup();
            cpu.a = 0b10100000;
            let rom = vec![0x0A, 0x0A, 0x0A];

            cpu.step(&rom, &mut mem);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x0A,
                    pc: 1,
                    a: 0b01000000,
                    p: 0b00000001,
                    ..Cpu::new()
                }
            );

            cpu.step(&rom, &mut mem);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x0A,
                    pc: 2,
                    a: 0b10000000,
                    p: 0b10000000,
                    ..Cpu::new()
                }
            );

            cpu.step(&rom, &mut mem);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x0A,
                    pc: 3,
                    a: 0b00000000,
                    p: 0b00000011,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn asl_abs() {
            let (mut cpu, mut mem) = setup();
            mem[0xABCD] = 0b10100000;
            let rom = vec![0x0E, 0xCD, 0xAB, 0x0E, 0xCD, 0xAB, 0x0E, 0xCD, 0xAB];

            cpu.step(&rom, &mut mem);
            assert_eq!(mem[0xABCD], 0b01000000);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x0E,
                    pc: 3,
                    p: 0b00000001,
                    ..Cpu::new()
                }
            );

            cpu.step(&rom, &mut mem);
            assert_eq!(mem[0xABCD], 0b10000000);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x0E,
                    pc: 6,
                    p: 0b10000000,
                    ..Cpu::new()
                }
            );

            cpu.step(&rom, &mut mem);
            assert_eq!(mem[0xABCD], 0b00000000);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x0E,
                    pc: 9,
                    p: 0b00000011,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn asl_zp() {
            let (mut cpu, mut mem) = setup();
            mem[0x00CD] = 0b10100000;
            let rom = vec![0x06, 0xCD, 0x06, 0xCD, 0x06, 0xCD];

            cpu.step(&rom, &mut mem);
            assert_eq!(mem[0x00CD], 0b01000000);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x06,
                    pc: 2,
                    p: 0b00000001,
                    ..Cpu::new()
                }
            );

            cpu.step(&rom, &mut mem);
            assert_eq!(mem[0x00CD], 0b10000000);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x06,
                    pc: 4,
                    p: 0b10000000,
                    ..Cpu::new()
                }
            );

            cpu.step(&rom, &mut mem);
            assert_eq!(mem[0x00CD], 0b00000000);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x06,
                    pc: 6,
                    p: 0b00000011,
                    ..Cpu::new()
                }
            );
        }
    }

    mod branch_tests {
        use super::*;

        #[test]
        fn bbr_no_branch() {
            for i in 0..8 {
                let (mut cpu, mut mem) = setup();
                mem[0x00CD] = 0xFF;
                let opcode = 0x0F + (i * 0x10);
                let rom = vec![opcode, 0xCD, 0xBB];

                cpu.step(&rom, &mut mem);

                assert_eq!(
                    cpu,
                    Cpu {
                        ir: opcode,
                        pc: 3,
                        ..Cpu::new()
                    }
                )
            }
        }

        #[test]
        fn bbr_branch() {
            for i in 0..8 {
                let (mut cpu, mut mem) = setup();
                mem[0x00CD] = 0x00;
                let opcode = 0x0F + (i * 0x10);
                let rom = vec![opcode, 0xCD, 0xBB];

                cpu.step(&rom, &mut mem);

                assert_eq!(
                    cpu,
                    Cpu {
                        ir: opcode,
                        pc: 0xBB,
                        ..Cpu::new()
                    }
                )
            }
        }

        #[test]
        fn bbs_no_branch() {
            for i in 0..8 {
                let (mut cpu, mut mem) = setup();
                mem[0x00CD] = 0x00;
                let opcode = 0x8F + (i * 0x10);
                let rom = vec![opcode, 0xCD, 0xBB];

                cpu.step(&rom, &mut mem);

                assert_eq!(
                    cpu,
                    Cpu {
                        ir: opcode,
                        pc: 3,
                        ..Cpu::new()
                    }
                )
            }
        }

        #[test]
        fn bbs_branch() {
            for i in 0..8 {
                let (mut cpu, mut mem) = setup();
                mem[0x00CD] = 0xFF;
                let opcode = 0x8F + (i * 0x10);
                let rom = vec![opcode, 0xCD, 0xBB];

                cpu.step(&rom, &mut mem);

                assert_eq!(
                    cpu,
                    Cpu {
                        ir: opcode,
                        pc: 0xBB,
                        ..Cpu::new()
                    }
                )
            }
        }

        #[test]
        fn bcc_no_branch() {
            let (mut cpu, mut mem) = setup();
            cpu.p = PC_MASK;
            let rom = vec![0x90, 0xCD];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x90,
                    pc: 2,
                    p: PC_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bcc_branch() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0x90, 0xCD];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x90,
                    pc: 0xCD,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bcs_no_branch() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0xB0, 0xCD];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xB0,
                    pc: 2,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bcs_branch() {
            let (mut cpu, mut mem) = setup();
            cpu.p = PC_MASK;
            let rom = vec![0xB0, 0xCD];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xB0,
                    pc: 0xCD,
                    p: PC_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn beq_no_branch() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0xF0, 0xCD];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xF0,
                    pc: 2,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn beq_branch() {
            let (mut cpu, mut mem) = setup();
            cpu.p = PZ_MASK;
            let rom = vec![0xF0, 0xCD];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xF0,
                    pc: 0xCD,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            )
        }
    }

    mod inc_tests {
        use super::*;

        #[test]
        fn inx() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0xE8];
            cpu.x = 41;

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xE8,
                    pc: 1,
                    x: 42,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn inx_zero() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0xE8];
            cpu.x = u8::MAX;

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xE8,
                    pc: 1,
                    x: 0,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn inx_reset_zero() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0xE8];
            cpu.x = 5;
            cpu.p = PZ_MASK;

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xE8,
                    pc: 1,
                    x: 6,
                    p: 0b00000000,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn inx_neg() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0xE8];
            cpu.x = 127;

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xE8,
                    pc: 1,
                    x: 128,
                    p: 0b10000000,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn inx_reset_neg() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0xE8];
            cpu.x = u8::MAX;
            cpu.p = PN_MASK;

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xE8,
                    pc: 1,
                    x: 0,
                    p: 0b00000010,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn iny() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0xC8];
            cpu.y = 40;

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xC8,
                    pc: 1,
                    y: 41,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn iny_zero() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0xC8];
            cpu.y = u8::MAX;

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xC8,
                    pc: 1,
                    y: 0,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn iny_reset_zero() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0xC8];
            cpu.y = 5;
            cpu.p = PZ_MASK;

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xC8,
                    pc: 1,
                    y: 6,
                    p: 0b00000000,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn iny_neg() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0xC8];
            cpu.y = 127;

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xC8,
                    pc: 1,
                    y: 128,
                    p: 0b10000000,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn iny_reset_neg() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0xC8];
            cpu.y = u8::MAX;
            cpu.p = PN_MASK;

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xC8,
                    pc: 1,
                    y: 0,
                    p: 0b00000010,
                    ..Cpu::new()
                }
            )
        }
    }

    mod jmp_tests {
        use super::*;

        #[test]
        fn jmp_abs() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0x4C, 0x0A, 0x80];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x4C,
                    pc: 0x800A,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn jmp_ai() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0x6C, 0x0A, 0x80];
            mem[0x800A] = 0xCD;
            mem[0x800B] = 0xAB;

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x6C,
                    pc: 0xABCD,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn jmp_aii() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0x7C, 0x00, 0x80];

            cpu.x = 6;
            mem[0x8006] = 0xCD;
            mem[0x8007] = 0xAB;

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x7C,
                    pc: 0xABCD,
                    x: 6,
                    ..Cpu::new()
                }
            );
        }
    }

    mod lda_tests {
        use super::*;

        #[test]
        fn lda_immediate() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0xA9, 0xED];

            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xA9,
                    a: 0xED,
                    pc: 2,
                    ..Cpu::new()
                }
            );
        }
    }

    #[test]
    fn nop() {
        let (mut cpu, mut mem) = setup();
        let rom = vec![0xEA];

        cpu.step(&rom, &mut mem);

        assert_eq!(
            cpu,
            Cpu {
                ir: 0xEA,
                pc: 1,
                ..Cpu::new()
            }
        );
    }

    mod sta_tests {
        use super::*;

        #[test]
        fn sta_a() {
            let (mut cpu, mut mem) = setup();
            let rom = vec![0x8D, 0x02, 0x60];

            cpu.a = 57;
            cpu.step(&rom, &mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x8D,
                    a: 57,
                    pc: 3,
                    ..Cpu::new()
                }
            );

            assert_eq!(mem[0x6002], 57);
        }
    }

    mod cpu_status_tests {
        use super::*;

        #[test]
        fn overflow_flag() {
            let mut cpu = Cpu::new();

            cpu.update_status_v(0, 0, 0);
            assert_eq!(cpu.p, 0);

            cpu.update_status_v(0b10000001, 0, 0);
            assert_eq!(cpu.p, PV_MASK);

            cpu.update_status_v(0, 0b10000001, 0);
            assert_eq!(cpu.p, 0);

            cpu.update_status_v(0, 0, 0b10000001);
            assert_eq!(cpu.p, 0);

            cpu.update_status_v(0b10000001, 0b10000001, 0);
            assert_eq!(cpu.p, 0);

            cpu.update_status_v(0b10000001, 0, 0b10000001);
            assert_eq!(cpu.p, 0);

            cpu.update_status_v(0, 0b10000001, 0b10000001);
            assert_eq!(cpu.p, PV_MASK);

            cpu.update_status_v(0b10000001, 0b10000001, 0b10000001);
            assert_eq!(cpu.p, 0);
        }
    }
}
