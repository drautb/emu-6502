#![allow(non_camel_case_types)]
#![allow(clippy::upper_case_acronyms)]

/**
 * https://www.westerndesigncenter.com/wdc/documentation/w65c02s.pdf
 * https://eater.net/datasheets/w65c02s.pdf
 * https://www.pagetable.com/c64ref/6502/?cpu=65c02s
 */
use std::fmt;
use std::num::Wrapping;

use crate::Memory;

#[derive(Debug)]
pub enum AddressMode {
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
pub enum Instruction {
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

    INVALID, // Unrecognized opcode
}

fn parse_instruction_internal(opcode: u8) -> Instruction {
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

        _ => Instruction::INVALID,
    }
}

fn parse_instruction_or_panic(opcode: u8) -> Instruction {
    match parse_instruction_internal(opcode) {
        Instruction::INVALID => {
            println!("Unrecognized opcode! {}", opcode);
            panic!("Unrecognized opcode!");
        }
        other => other,
    }
}

pub fn parse_instruction(opcode: u8) -> Option<Instruction> {
    match parse_instruction_internal(opcode) {
        Instruction::INVALID => None,
        instruction => Some(instruction),
    }
}

const PN_MASK: u8 = 0b10000000;
const PV_MASK: u8 = 0b01000000;
const P5_MASK: u8 = 0b00100000;
const PB_MASK: u8 = 0b00010000;
const PD_MASK: u8 = 0b00001000;
const PI_MASK: u8 = 0b00000100;
const PZ_MASK: u8 = 0b00000010;
const PC_MASK: u8 = 0b00000001;

const PNV_MASK: u8 = PN_MASK | PV_MASK;

// const NMI_VECTOR: usize = 0xFFFA;
const RESET_VECTOR: usize = 0xFFFC;
const IRQ_VECTOR: usize = 0xFFFE;

fn inc_wrap(n: u8) -> u8 {
    add_wrap(n, 1)
}

fn dec_wrap(n: u8) -> u8 {
    sub_wrap(n, 1)
}

fn add_wrap(n1: u8, n2: u8) -> u8 {
    (Wrapping(n1) + Wrapping(n2)).0
}

fn sub_wrap(n1: u8, n2: u8) -> u8 {
    (Wrapping(n1) - Wrapping(n2)).0
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
            .field("a", &format_args!("{:#04X}", self.a))
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

impl Default for Cpu {
    fn default() -> Self {
        Self::new()
    }
}

impl Cpu {
    pub fn new() -> Self {
        Cpu {
            ir: 0,
            a: 0,
            x: 0,
            y: 0,
            p: 0,
            pc: 0,
            s: 0xFF,
        }
    }

    pub fn reset(&mut self, mem: &Memory) {
        self.ir = 0;
        self.a = 0;
        self.x = 0;
        self.y = 0;
        self.p = 0;
        self.pc = self.deref_mem(mem, RESET_VECTOR) as usize;
        self.s = 0xFF;
    }

    pub fn load_instruction(&mut self, mem: &Memory) {
        self.ir = mem[self.pc];
    }

    pub fn step(&mut self, mem: &mut Memory) {
        self.load_instruction(mem);
        let instruction = parse_instruction_or_panic(self.ir);
        match instruction {
            Instruction::ADC(_, address_mode) => {
                let n1 = self.a;
                let n2 = self.resolve_operand(&address_mode, mem);
                if self.p & PD_MASK > 0 {
                    self.adc_dec(n1, n2);
                } else {
                    self.adc_bin(n1, n2);
                }
                self.update_pc(address_mode);
            }

            Instruction::SBC(_, address_mode) => {
                let n1 = self.a;
                let n2 = self.resolve_operand(&address_mode, mem);
                if self.p & PD_MASK > 0 {
                    self.sbc_dec(n1, n2);
                } else {
                    self.adc_bin(n1, !n2);
                }
                self.update_pc(address_mode);
            }

            Instruction::AND(_, address_mode) => {
                self.a &= self.resolve_operand(&address_mode, mem);
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
                let resolved_addr = self.resolve_operand_addr(&address_mode, mem);
                let val = mem[resolved_addr];
                self.p = (self.p & !PC_MASK) | (val >> 7);
                let result = val << 1;
                mem[resolved_addr] = result;
                self.update_status_nz(result);
                self.update_pc(address_mode);
            }

            Instruction::BBR(_, bit) => {
                let test = mem[self.resolve_zp(mem)];
                let offset = self.third_byte_operand(mem) as i8;
                self.pc += 3;
                if test & (1 << bit) == 0 {
                    self.update_pc_relative(offset);
                }
            }
            Instruction::BBS(_, bit) => {
                let test = mem[self.resolve_zp(mem)];
                let offset = self.third_byte_operand(mem) as i8;
                self.pc += 3;
                if test & (1 << bit) > 0 {
                    self.update_pc_relative(offset);
                }
            }

            Instruction::BCC(_) => {
                let offset = self.second_byte_operand(mem) as i8;
                self.pc += 2;
                if self.p & PC_MASK == 0 {
                    self.update_pc_relative(offset);
                }
            }
            Instruction::BCS(_) => {
                let offset = self.second_byte_operand(mem) as i8;
                self.pc += 2;
                if self.p & PC_MASK > 0 {
                    self.update_pc_relative(offset);
                }
            }

            Instruction::BEQ(_) => {
                let offset = self.second_byte_operand(mem) as i8;
                self.pc += 2;
                if self.p & PZ_MASK > 0 {
                    self.update_pc_relative(offset);
                }
            }

            Instruction::BIT(_, address_mode) => {
                let operand = self.resolve_operand(&address_mode, mem);
                self.p = (self.p & !PNV_MASK) | (operand & PNV_MASK); // Set N and V equal to operand bits 7 and 6 respectively
                self.update_status_z(self.a & operand);
                self.update_pc(address_mode)
            }

            Instruction::BMI(_) => {
                let offset = self.second_byte_operand(mem) as i8;
                self.pc += 2;
                if self.p & PN_MASK > 0 {
                    self.update_pc_relative(offset);
                }
            }
            Instruction::BNE(_) => {
                let offset = self.second_byte_operand(mem) as i8;
                self.pc += 2;
                if self.p & PZ_MASK == 0 {
                    self.update_pc_relative(offset);
                }
            }
            Instruction::BPL(_) => {
                let offset = self.second_byte_operand(mem) as i8;
                self.pc += 2;
                if self.p & PN_MASK == 0 {
                    self.update_pc_relative(offset);
                }
            }

            Instruction::BRA(_) => {
                let offset = self.second_byte_operand(mem) as i8;
                self.pc += 2;
                self.update_pc_relative(offset);
            }

            Instruction::BRK(_) => {
                let return_address = self.pc + 2;
                self.push_stack(mem, (return_address >> 8) as u8);
                self.push_stack(mem, return_address as u8);
                self.push_stack(mem, self.p | P5_MASK | PB_MASK);
                self.set_status(PI_MASK);
                self.clear_status(PD_MASK);
                // BRK uses the vector at $FFFE-$FFFF - http://6502.org/tutorials/interrupts.html#2.2
                self.pc = self.deref_mem(mem, IRQ_VECTOR) as usize;
            }

            Instruction::BVC(_) => {
                let offset = self.second_byte_operand(mem) as i8;
                self.pc += 2;
                if self.p & PV_MASK == 0 {
                    self.update_pc_relative(offset);
                }
            }
            Instruction::BVS(_) => {
                let offset = self.second_byte_operand(mem) as i8;
                self.pc += 2;
                if self.p & PV_MASK > 0 {
                    self.update_pc_relative(offset);
                }
            }

            Instruction::CLC(_) => {
                self.clear_status(PC_MASK);
                self.incr_pc();
            }
            Instruction::CLD(_) => {
                self.clear_status(PD_MASK);
                self.incr_pc();
            }
            Instruction::CLI(_) => {
                self.clear_status(PI_MASK);
                self.incr_pc();
            }
            Instruction::CLV(_) => {
                self.clear_status(PV_MASK);
                self.incr_pc();
            }

            Instruction::CMP(_, address_mode) => {
                self.cmp_register(mem, address_mode, self.a);
            }
            Instruction::CPX(_, address_mode) => {
                self.cmp_register(mem, address_mode, self.x);
            }
            Instruction::CPY(_, address_mode) => {
                self.cmp_register(mem, address_mode, self.y);
            }

            Instruction::DEC(_, AddressMode::ACC) => self.a = self.dec_register(self.a),
            Instruction::DEC(_, address_mode) => {
                let resolved_addr = self.resolve_operand_addr(&address_mode, mem);
                let val = mem[resolved_addr];
                let result = dec_wrap(val);
                mem[resolved_addr] = result;
                self.update_status_nz(result);
                self.update_pc(address_mode);
            }
            Instruction::DEX(_) => self.x = self.dec_register(self.x),
            Instruction::DEY(_) => self.y = self.dec_register(self.y),

            Instruction::EOR(_, address_mode) => {
                let operand = self.resolve_operand(&address_mode, mem);
                self.a ^= operand;
                self.update_status_nz(self.a);
                self.update_pc(address_mode);
            }

            Instruction::INC(_, AddressMode::ACC) => self.a = self.inc_register(self.a),
            Instruction::INC(_, address_mode) => {
                let resolved_addr = self.resolve_operand_addr(&address_mode, mem);
                let val = mem[resolved_addr];
                let result = inc_wrap(val);
                mem[resolved_addr] = result;
                self.update_status_nz(result);
                self.update_pc(address_mode);
            }
            Instruction::INX(_) => self.x = self.inc_register(self.x),
            Instruction::INY(_) => self.y = self.inc_register(self.y),

            Instruction::JMP(_, AddressMode::ABS) => self.pc = self.two_byte_operand(mem) as usize,
            Instruction::JMP(_, AddressMode::AI) => {
                let new_pc_addr = self.two_byte_operand(mem) as usize;
                self.pc = self.deref_mem(mem, new_pc_addr) as usize;
            }
            Instruction::JMP(_, AddressMode::AII) => {
                let new_pc_addr: usize = (self.two_byte_operand(mem) + self.x as u16) as usize;
                self.pc = self.deref_mem(mem, new_pc_addr) as usize;
            }
            Instruction::JSR(_) => {
                let return_address = self.pc + 2;
                self.push_stack(mem, (return_address >> 8) as u8);
                self.push_stack(mem, return_address as u8);
                self.pc = self.two_byte_operand(mem) as usize;
            }

            Instruction::LDA(_, mode) => self.a = self.load_register(mem, mode),
            Instruction::LDX(_, mode) => self.x = self.load_register(mem, mode),
            Instruction::LDY(_, mode) => self.y = self.load_register(mem, mode),

            Instruction::LSR(_, AddressMode::ACC) => {
                self.p = (self.p & !PC_MASK) | (self.a & PC_MASK);
                self.a >>= 1;
                self.update_status_nz(self.a);
                self.update_pc(AddressMode::ACC);
            }
            Instruction::LSR(_, address_mode) => {
                let resolved_addr = self.resolve_operand_addr(&address_mode, mem);
                let val = mem[resolved_addr];
                self.p = (self.p & !PC_MASK) | (val & PC_MASK);
                let result = val >> 1;
                mem[resolved_addr] = result;
                self.update_status_nz(result);
                self.update_pc(address_mode);
            }

            Instruction::NOP(_) | Instruction::STP(_) | Instruction::WAI(_) => {
                self.incr_pc();
            }

            Instruction::ORA(_, address_mode) => {
                let operand = self.resolve_operand(&address_mode, mem);
                self.a |= operand;
                self.update_status_nz(self.a);
                self.update_pc(address_mode);
            }

            Instruction::PHA(_) => self.push_stack_inst(mem, self.a),
            Instruction::PHP(_) => self.push_stack_inst(mem, self.p | P5_MASK | PB_MASK),
            Instruction::PHX(_) => self.push_stack_inst(mem, self.x),
            Instruction::PHY(_) => self.push_stack_inst(mem, self.y),
            Instruction::PLA(_) => {
                self.a = self.pop_stack(mem);
                self.update_status_nz(self.a);
                self.incr_pc();
            }
            Instruction::PLP(_) => {
                let mut tmp_p = self.pop_stack(mem);
                tmp_p &= !(P5_MASK | PB_MASK) | self.p;
                self.p = tmp_p;
                self.incr_pc();
            }
            Instruction::PLX(_) => {
                self.x = self.pop_stack(mem);
                self.update_status_nz(self.x);
                self.incr_pc();
            }
            Instruction::PLY(_) => {
                self.y = self.pop_stack(mem);
                self.update_status_nz(self.y);
                self.incr_pc();
            }

            Instruction::RMB(_, bit) => {
                let mask = 1 << bit;
                let resolved_addr = self.resolve_operand_addr(&AddressMode::ZP, mem);
                mem[resolved_addr] &= !mask;
                self.update_pc(AddressMode::ZP);
            }

            Instruction::ROL(_, AddressMode::ACC) => {
                let new_carry = self.a >> 7;
                self.a = (self.a << 1) | (self.p & PC_MASK);
                self.p = (self.p & !PC_MASK) | (new_carry & PC_MASK);
                self.update_status_nz(self.a);
                self.update_pc(AddressMode::ACC);
            }
            Instruction::ROL(_, address_mode) => {
                let resolved_addr = self.resolve_operand_addr(&address_mode, mem);
                let val = mem[resolved_addr];
                let new_carry = val >> 7;
                let new_val = (val << 1) | self.p & PC_MASK;
                self.p = (self.p & !PC_MASK) | (new_carry & PC_MASK);
                mem[resolved_addr] = new_val;
                self.update_status_nz(new_val);
                self.update_pc(address_mode);
            }

            Instruction::ROR(_, AddressMode::ACC) => {
                let new_carry = self.a & PC_MASK;
                self.a = (self.a >> 1) | ((self.p & PC_MASK) << 7);
                self.p = (self.p & !PC_MASK) | (new_carry & PC_MASK);
                self.update_status_nz(self.a);
                self.update_pc(AddressMode::ACC);
            }
            Instruction::ROR(_, address_mode) => {
                let resolved_addr = self.resolve_operand_addr(&address_mode, mem);
                let val = mem[resolved_addr];
                let new_carry = val & PC_MASK;
                let new_val = (val >> 1) | ((self.p & PC_MASK) << 7);
                self.p = (self.p & !PC_MASK) | (new_carry & PC_MASK);
                mem[resolved_addr] = new_val;
                self.update_status_nz(new_val);
                self.update_pc(address_mode);
            }

            Instruction::RTI(_) => {
                self.p = self.pop_stack(mem);
                let pcl: u16 = self.pop_stack(mem) as u16;
                let pch: u16 = self.pop_stack(mem) as u16;
                self.pc = ((pch << 8) | pcl) as usize;
            }

            Instruction::RTS(_) => {
                let pcl: u16 = self.pop_stack(mem) as u16;
                let pch: u16 = self.pop_stack(mem) as u16;
                self.pc = ((pch << 8) | pcl) as usize + 1;
            }

            Instruction::SEC(_) => {
                self.set_status(PC_MASK);
                self.incr_pc();
            }
            Instruction::SED(_) => {
                self.set_status(PD_MASK);
                self.incr_pc();
            }
            Instruction::SEI(_) => {
                self.set_status(PI_MASK);
                self.incr_pc();
            }

            Instruction::SMB(_, bit) => {
                let mask = 1 << bit;
                let resolved_addr = self.resolve_operand_addr(&AddressMode::ZP, mem);
                mem[resolved_addr] |= mask;
                self.update_pc(AddressMode::ZP);
            }

            Instruction::STA(_, mode) => self.store_register(mem, mode, self.a),
            Instruction::STX(_, mode) => self.store_register(mem, mode, self.x),
            Instruction::STY(_, mode) => self.store_register(mem, mode, self.y),
            Instruction::STZ(_, mode) => self.store_register(mem, mode, 0),

            Instruction::TAX(_) => {
                self.x = self.a;
                self.update_status_nz(self.x);
                self.incr_pc();
            }
            Instruction::TAY(_) => {
                self.y = self.a;
                self.update_status_nz(self.y);
                self.incr_pc();
            }

            Instruction::TRB(_, address_mode) => {
                let resolved_addr = self.resolve_operand_addr(&address_mode, mem);
                mem[resolved_addr] &= !self.a;
                self.update_status_z(mem[resolved_addr]);
                self.update_pc(address_mode);
            }
            Instruction::TSB(_, address_mode) => {
                let resolved_addr = self.resolve_operand_addr(&address_mode, mem);
                mem[resolved_addr] |= self.a;
                self.update_status_z(mem[resolved_addr]);
                self.update_pc(address_mode);
            }

            Instruction::TSX(_) => {
                self.x = self.s;
                self.update_status_nz(self.x);
                self.incr_pc();
            }
            Instruction::TXA(_) => {
                self.a = self.x;
                self.update_status_nz(self.a);
                self.incr_pc();
            }
            Instruction::TXS(_) => {
                self.s = self.x;
                self.incr_pc();
            }
            Instruction::TYA(_) => {
                self.a = self.y;
                self.update_status_nz(self.a);
                self.incr_pc();
            }

            instruction => {
                println!("Not implemented: {:?}", instruction);
                todo!();
            }
        }
    }

    pub fn program_counter(&self) -> usize {
        self.pc
    }

    pub fn override_program_counter(&mut self, new_pc: usize) {
        self.pc = new_pc
    }

    pub fn instruction_register(&self) -> u8 {
        self.ir
    }

    pub fn accumulator(&self) -> u8 {
        self.a
    }

    pub fn x_register(&self) -> u8 {
        self.x
    }

    pub fn y_register(&self) -> u8 {
        self.y
    }

    pub fn status(&self) -> u8 {
        self.p
    }

    pub fn stack_pointer(&self) -> u8 {
        self.s
    }

    pub fn get_status_neg(&self) -> bool {
        (PN_MASK & self.p) != 0
    }

    pub fn get_status_overflow(&self) -> bool {
        (PV_MASK & self.p) != 0
    }

    pub fn get_status_brk(&self) -> bool {
        (PB_MASK & self.p) != 0
    }

    pub fn get_status_dec(&self) -> bool {
        (PD_MASK & self.p) != 0
    }

    pub fn get_status_interrupt_disable(&self) -> bool {
        (PI_MASK & self.p) != 0
    }

    pub fn get_status_zero(&self) -> bool {
        (PZ_MASK & self.p) != 0
    }

    pub fn get_status_carry(&self) -> bool {
        (PC_MASK & self.p) != 0
    }

    fn resolve_operand_addr(&mut self, address_mode: &AddressMode, mem: &Memory) -> usize {
        match address_mode {
            AddressMode::ABS => self.resolve_abs(mem),
            AddressMode::AIX => self.resolve_aix(mem),
            AddressMode::AIY => self.resolve_aiy(mem),
            AddressMode::ZP => self.resolve_zp(mem),
            AddressMode::ZPIX => self.resolve_zpix(mem),
            AddressMode::ZPIY => self.resolve_zpiy(mem),
            AddressMode::ZPI => self.resolve_zpi(mem),
            AddressMode::ZPII => self.resolve_zpii(mem),
            AddressMode::ZPIIY => self.resolve_zpiiy(mem),
            _ => {
                println!(
                    "Unable to resolve operand address for mode {:?}",
                    address_mode
                );
                panic!("Unable to resolve operand address for mode");
            }
        }
    }

    fn resolve_operand(&mut self, address_mode: &AddressMode, mem: &Memory) -> u8 {
        match address_mode {
            AddressMode::IMMEDIATE => self.second_byte_operand(mem),
            _ => mem[self.resolve_operand_addr(address_mode, mem)],
        }
    }

    fn second_byte_operand(&self, mem: &Memory) -> u8 {
        mem[self.pc + 1]
    }

    fn third_byte_operand(&self, mem: &Memory) -> u8 {
        mem[self.pc + 2]
    }

    fn two_byte_operand(&self, mem: &Memory) -> u16 {
        let op_l: u16 = mem[self.pc + 1].into();
        let op_h: u16 = mem[self.pc + 2].into();
        (op_h << 8) | op_l
    }

    fn resolve_abs(&mut self, mem: &Memory) -> usize {
        self.two_byte_operand(mem) as usize
    }

    fn resolve_aix(&self, mem: &Memory) -> usize {
        (self.two_byte_operand(mem) + self.x as u16) as usize
    }

    fn resolve_aiy(&self, mem: &Memory) -> usize {
        (self.two_byte_operand(mem) + self.y as u16) as usize
    }

    fn resolve_zp(&self, mem: &Memory) -> usize {
        self.second_byte_operand(mem) as usize
    }

    fn resolve_zpix(&self, mem: &Memory) -> usize {
        (add_wrap(self.second_byte_operand(mem), self.x)) as usize
    }

    fn resolve_zpiy(&self, mem: &Memory) -> usize {
        (add_wrap(self.second_byte_operand(mem), self.y)) as usize
    }

    fn resolve_zpi(&self, mem: &Memory) -> usize {
        let indirect_address = self.second_byte_operand(mem);
        let operand_address: u16 = self.deref_mem(mem, indirect_address as usize);
        operand_address as usize
    }

    fn resolve_zpii(&self, mem: &Memory) -> usize {
        let indirect_address = add_wrap(self.second_byte_operand(mem), self.x);
        let operand_address: u16 = self.deref_mem(mem, indirect_address as usize);
        operand_address as usize
    }

    fn resolve_zpiiy(&self, mem: &Memory) -> usize {
        // Deref the zero page pointer
        let zp = self.second_byte_operand(mem);
        let indirect_base = self.deref_mem(mem, zp as usize);

        // Add y to the address found
        let indirect_address = indirect_base + self.y as u16;

        // Deref new address to get operand
        indirect_address as usize
    }

    fn deref_mem(&self, mem: &Memory, addr: usize) -> u16 {
        let new_addr_l: u16 = mem[addr].into();
        let new_addr_h: u16 = mem[addr + 1].into();
        (new_addr_h << 8) | new_addr_l
    }

    fn update_status_nz(&mut self, value: u8) {
        // Negative
        self.p = (PN_MASK & value) | (!PN_MASK & self.p);
        self.update_status_z(value);
    }

    fn update_status_z(&mut self, value: u8) {
        if value == 0 {
            self.p |= PZ_MASK;
        } else {
            self.p &= !PZ_MASK;
        }
    }

    fn update_status_v(&mut self, result: u8, n1: u8, n2: u8) {
        // oVerflow
        if n1 & PN_MASK == n2 & PN_MASK && n1 & PN_MASK != result & PN_MASK {
            self.set_status(PV_MASK);
        } else {
            self.clear_status(PV_MASK);
        }
    }

    fn clear_status(&mut self, mask: u8) {
        self.p &= !mask;
    }

    fn set_status(&mut self, mask: u8) {
        self.p |= mask
    }

    fn update_pc(&mut self, address_mode: AddressMode) {
        self.pc += Cpu::instruction_length(address_mode);
    }

    fn update_pc_relative(&mut self, offset: i8) {
        self.pc = ((self.pc as i16) + offset as i16) as usize;
    }

    fn incr_pc(&mut self) {
        self.pc += 1;
    }

    fn cmp_register(&mut self, mem: &Memory, address_mode: AddressMode, register: u8) {
        let operand = self.resolve_operand(&address_mode, mem);
        let result = sub_wrap(register, operand);
        self.update_status_nz(result);
        if operand <= register {
            self.set_status(PC_MASK);
        } else {
            self.clear_status(PC_MASK);
        }
        self.update_pc(address_mode);
    }

    fn inc_register(&mut self, register: u8) -> u8 {
        let result = inc_wrap(register);
        self.update_status_nz(result);
        self.incr_pc();
        result
    }

    fn dec_register(&mut self, register: u8) -> u8 {
        let result = dec_wrap(register);
        self.update_status_nz(result);
        self.incr_pc();
        result
    }

    fn load_register(&mut self, mem: &Memory, address_mode: AddressMode) -> u8 {
        let value = self.resolve_operand(&address_mode, mem);
        self.update_status_nz(value);
        self.update_pc(address_mode);
        value
    }

    fn store_register(&mut self, mem: &mut Memory, address_mode: AddressMode, val: u8) {
        let resolved_addr = self.resolve_operand_addr(&address_mode, mem);
        mem[resolved_addr] = val;
        self.update_pc(address_mode);
    }

    fn push_stack_inst(&mut self, mem: &mut Memory, val: u8) {
        self.push_stack(mem, val);
        self.incr_pc();
    }

    fn push_stack(&mut self, mem: &mut Memory, val: u8) {
        mem[(self.s as u16 + 0x100) as usize] = val;
        self.s = dec_wrap(self.s);
    }

    fn pop_stack(&mut self, mem: &mut Memory) -> u8 {
        self.s = inc_wrap(self.s);
        mem[(self.s as u16 + 0x100) as usize]
    }

    fn adc_bin(&mut self, n1: u8, n2: u8) {
        self.a = add_wrap(add_wrap(n1, n2), self.p & PC_MASK);
        let total = n1 as u16 + n2 as u16 + (self.p & PC_MASK) as u16;
        if total > 255 {
            self.set_status(PC_MASK);
        } else {
            self.clear_status(PC_MASK);
        }
        self.update_status_nz(self.a);
        self.update_status_v(self.a, n1, n2);
    }

    fn adc_dec(&mut self, n1: u8, n2: u8) {
        let n1 = self.bin_to_dec(n1);
        let n2 = self.bin_to_dec(n2);
        let bin_result = n1 as u16 + n2 as u16 + (self.p & PC_MASK) as u16;
        if bin_result > 99 {
            self.set_status(PC_MASK);
        } else {
            self.clear_status(PC_MASK);
        }
        self.a = self.dec_to_bin(bin_result);
        self.update_status_nz(self.a);
        self.update_status_v(self.a, n1, n2);
    }

    fn sbc_dec(&mut self, n1: u8, n2: u8) {
        let n1 = self.bin_to_dec(n1);
        let n2 = self.bin_to_dec(n2);
        let carry = (!self.p & PC_MASK) as u16 as i16;
        let mut bin_result = n1 as i16 - n2 as i16 - carry;
        if bin_result < 0 {
            bin_result += 100;
            self.clear_status(PC_MASK);
        } else {
            self.set_status(PC_MASK);
        }
        self.a = self.dec_to_bin(bin_result as u16);
        self.update_status_nz(self.a);
        self.update_status_v(self.a, n1, n2);
    }

    fn bin_to_dec(&self, val: u8) -> u8 {
        (((val & 0b1111_0000) >> 4) * 10) + (val & 0b0000_1111)
    }

    fn dec_to_bin(&self, val: u16) -> u8 {
        let hundreds = (val / 100) * 100;
        let tens = ((val - hundreds) / 10) * 10;
        let ones = val - hundreds - tens;
        ((tens / 10) << 4 | ones) as u8
    }

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
}

#[cfg(test)]
mod tests {
    use super::*;

    fn setup(program: Vec<u8>) -> (Cpu, [u8; 65_536]) {
        let mut mem = [0; 65_536];
        for (i, instruction) in program.iter().enumerate() {
            mem[i] = instruction.clone();
        }
        (Cpu::new(), mem)
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
        let mem = [0; 65_536];

        cpu.reset(&mem);

        assert_eq!(cpu, Cpu::new());
    }

    mod adc_tests {
        use super::*;

        #[test]
        fn adc() {
            let (mut cpu, mut mem) = setup(vec![0x69, 2]);
            cpu.a = 40;
            cpu.p = PZ_MASK | PN_MASK | PV_MASK; // These should all be cleared

            cpu.step(&mut mem);

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
        fn adc_zero_plus_zero() {
            let (mut cpu, mut mem) = setup(vec![0x69, 0]);
            cpu.a = 0;
            cpu.p = PC_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x69,
                    pc: 2,
                    a: 1,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn adc_zero_plus_ff_vzc() {
            let (mut cpu, mut mem) = setup(vec![0x69, 0xFF]);
            cpu.a = 0;
            cpu.p = PV_MASK | PZ_MASK | PC_MASK;

            cpu.step(&mut mem);

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

        #[test]
        fn adc_carry() {
            let (mut cpu, mut mem) = setup(vec![0x69, 1]);
            cpu.a = 255;

            cpu.step(&mut mem);

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

        #[test]
        fn adc_decimal() {
            let (mut cpu, mut mem) = setup(vec![0x69, 0x12]);
            cpu.a = 0x34;
            cpu.p = PD_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x69,
                    pc: 2,
                    a: 0x46,
                    p: PD_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn adc_decimal_set_carry() {
            let (mut cpu, mut mem) = setup(vec![0x69, 0x81]);
            cpu.a = 0x92;
            cpu.p = PD_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x69,
                    pc: 2,
                    a: 0x73,
                    p: PD_MASK | PC_MASK,
                    ..Cpu::new()
                }
            )
        }
    }

    mod and_tests {
        use super::*;

        #[test]
        fn and_immediate() {
            let (mut cpu, mut mem) = setup(vec![0x29, 0b00001111]);
            cpu.a = 0b00111100;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0x29, 0b10001111]);
            cpu.a = 0b10000000;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0x29, 0xFF]);
            cpu.a = 0;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0x2D, 0xCD, 0xAB]);
            cpu.a = 0b10001111;
            mem[0xABCD] = 0b10111100;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0x3D, 0xC0, 0xAB]);
            cpu.a = 0b10001111;
            cpu.x = 10;
            mem[0xABCA] = 0b10111100;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0x39, 0xC0, 0xAB]);
            cpu.a = 0b10001111;
            cpu.y = 10;
            mem[0xABCA] = 0b10111100;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0x25, 0xCD]);
            cpu.a = 0b10001111;
            mem[0x00CD] = 0b10111100;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0x35, 0xC0]);
            cpu.a = 0b10001111;
            cpu.x = 10;
            mem[0x00CA] = 0b10111100;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0x32, 0xCD]);
            cpu.a = 0b10001111;
            mem[0x00CD] = 0x57;
            mem[0x00CE] = 0x43;
            mem[0x4357] = 0b10111100;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0x21, 0xC0]);
            cpu.a = 0b10001111;
            cpu.x = 10;
            mem[0x00CA] = 0x57;
            mem[0x00CB] = 0x43;
            mem[0x4357] = 0b10111100;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0x31, 0xC0]);
            cpu.a = 0b10001111;
            cpu.y = 10;
            mem[0x00C0] = 0x00;
            mem[0x00C1] = 0xFF;
            mem[0xFF0A] = 0b10111100;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0x0A, 0x0A, 0x0A]);
            cpu.a = 0b10100000;

            cpu.step(&mut mem);
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

            cpu.step(&mut mem);
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

            cpu.step(&mut mem);
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
            let rom = vec![0x0E, 0xCD, 0xAB, 0x0E, 0xCD, 0xAB, 0x0E, 0xCD, 0xAB];
            let (mut cpu, mut mem) = setup(rom);
            mem[0xABCD] = 0b10100000;

            cpu.step(&mut mem);
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

            cpu.step(&mut mem);
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

            cpu.step(&mut mem);
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
            let rom = vec![0x06, 0xCD, 0x06, 0xCD, 0x06, 0xCD];
            let (mut cpu, mut mem) = setup(rom);
            mem[0x00CD] = 0b10100000;

            cpu.step(&mut mem);
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

            cpu.step(&mut mem);
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

            cpu.step(&mut mem);
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
                let opcode = 0x0F + (i * 0x10);
                let rom = vec![opcode, 0xCD, 0xBB];
                let (mut cpu, mut mem) = setup(rom);
                mem[0x00CD] = 0xFF;

                cpu.step(&mut mem);

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
        fn bbr_branch_forward() {
            for i in 0..8 {
                let opcode = 0x0F + (i * 0x10);
                let rom = vec![opcode, 0xCD, 5];
                let (mut cpu, mut mem) = setup(rom);
                mem[0x00CD] = 0x00;

                cpu.step(&mut mem);

                assert_eq!(
                    cpu,
                    Cpu {
                        ir: opcode,
                        pc: 8,
                        ..Cpu::new()
                    }
                )
            }
        }

        #[test]
        fn bbr_branch_backward() {
            for i in 0..8 {
                let opcode = 0x0F + (i * 0x10);
                let rom = vec![
                    0xEA, 0xEA, 0xEA, opcode, 0xCD, 0b11111011, // -5 in 2's comp
                    0xEA,
                ];
                let (mut cpu, mut mem) = setup(rom);
                cpu.pc = 3;
                mem[0x00CD] = 0x00;

                cpu.step(&mut mem);

                assert_eq!(
                    cpu,
                    Cpu {
                        ir: opcode,
                        pc: 1,
                        ..Cpu::new()
                    }
                )
            }
        }

        #[test]
        fn bbs_no_branch() {
            for i in 0..8 {
                let opcode = 0x8F + (i * 0x10);
                let rom = vec![opcode, 0xCD, 0xBB];
                let (mut cpu, mut mem) = setup(rom);
                mem[0x00CD] = 0x00;

                cpu.step(&mut mem);

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
        fn bbs_branch_forward() {
            for i in 0..8 {
                let opcode = 0x8F + (i * 0x10);
                let rom = vec![opcode, 0xCD, 5];
                let (mut cpu, mut mem) = setup(rom);
                mem[0x00CD] = 0xFF;

                cpu.step(&mut mem);

                assert_eq!(
                    cpu,
                    Cpu {
                        ir: opcode,
                        pc: 8,
                        ..Cpu::new()
                    }
                )
            }
        }

        #[test]
        fn bbs_branch_backward() {
            for i in 0..8 {
                let opcode = 0x8F + (i * 0x10);
                let rom = vec![
                    0xEA, 0xEA, 0xEA, opcode, 0xCD, 0b11111011, // -5 in 2's comp
                    0xEA,
                ];
                let (mut cpu, mut mem) = setup(rom);
                cpu.pc = 3;
                mem[0x00CD] = 0xFF;

                cpu.step(&mut mem);

                assert_eq!(
                    cpu,
                    Cpu {
                        ir: opcode,
                        pc: 1,
                        ..Cpu::new()
                    }
                )
            }
        }

        #[test]
        fn bcc_no_branch() {
            let (mut cpu, mut mem) = setup(vec![0x90, 0xCD]);
            cpu.p = PC_MASK;

            cpu.step(&mut mem);

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
        fn bcc_branch_forward() {
            let (mut cpu, mut mem) = setup(vec![0x90, 5]);

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x90,
                    pc: 7,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bcc_branch_backward() {
            let (mut cpu, mut mem) = setup(vec![
                0xEA, 0xEA, 0xEA, 0x90, 0b11111100, // -4 in 2's comp
                0xEA,
            ]);
            cpu.pc = 3;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x90,
                    pc: 1,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bcs_no_branch() {
            let (mut cpu, mut mem) = setup(vec![0xB0, 0xCD]);

            cpu.step(&mut mem);

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
        fn bcs_branch_forward() {
            let (mut cpu, mut mem) = setup(vec![0xB0, 5]);
            cpu.p = PC_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xB0,
                    pc: 7,
                    p: PC_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bcs_branch_backward() {
            let (mut cpu, mut mem) = setup(vec![
                0xEA, 0xEA, 0xEA, 0xB0, 0b11111100, // -4 in 2's comp
                0xEA,
            ]);
            cpu.p = PC_MASK;
            cpu.pc = 3;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xB0,
                    pc: 1,
                    p: PC_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn beq_no_branch() {
            let (mut cpu, mut mem) = setup(vec![0xF0, 0xCD]);

            cpu.step(&mut mem);

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
        fn beq_branch_forward() {
            let (mut cpu, mut mem) = setup(vec![0xF0, 5]);
            cpu.p = PZ_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xF0,
                    pc: 7,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn beq_branch_backward() {
            let (mut cpu, mut mem) = setup(vec![
                0xEA, 0xEA, 0xEA, 0xF0, 0b11111100, // -4 in 2's comp
                0xEA,
            ]);
            cpu.p = PZ_MASK;
            cpu.pc = 3;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xF0,
                    pc: 1,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bmi_no_branch() {
            let (mut cpu, mut mem) = setup(vec![0x30, 0xCD]);

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x30,
                    pc: 2,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bmi_branch_forward() {
            let (mut cpu, mut mem) = setup(vec![0x30, 5]);
            cpu.p = PN_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x30,
                    pc: 7,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bmi_branch_backward() {
            let (mut cpu, mut mem) = setup(vec![
                0xEA, 0xEA, 0xEA, 0x30, 0b11111100, // -4 in 2's comp
                0xEA,
            ]);
            cpu.p = PN_MASK;
            cpu.pc = 3;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x30,
                    pc: 1,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bne_no_branch() {
            let (mut cpu, mut mem) = setup(vec![0xD0, 0xCD]);
            cpu.p = PZ_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xD0,
                    pc: 2,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bne_branch_foward() {
            let (mut cpu, mut mem) = setup(vec![0xD0, 5]);

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xD0,
                    pc: 7,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bne_branch_backward() {
            let (mut cpu, mut mem) = setup(vec![
                0xEA, 0xEA, 0xEA, 0xD0, 0b11111100, // -4 in 2's complement
                0xEA,
            ]);
            cpu.pc = 3;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xD0,
                    pc: 1,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bpl_no_branch() {
            let (mut cpu, mut mem) = setup(vec![0x10, 0xCD]);
            cpu.p = PN_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x10,
                    pc: 2,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bpl_branch_forward() {
            let (mut cpu, mut mem) = setup(vec![0x10, 5]);

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x10,
                    pc: 7,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bpl_branch_backward() {
            let (mut cpu, mut mem) = setup(vec![
                0xEA, 0xEA, 0xEA, 0x10, 0b11111100, // -4 in 2's comp
                0xEA,
            ]);
            cpu.pc = 3;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x10,
                    pc: 1,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bra_branch_forward() {
            let (mut cpu, mut mem) = setup(vec![0x80, 5]);

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x80,
                    pc: 7,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bra_branch_backward() {
            let (mut cpu, mut mem) = setup(vec![
                0xEA, 0xEA, 0xEA, 0x80, 0b11111100, // -4 in 2's comp
                0xEA,
            ]);
            cpu.pc = 3;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x80,
                    pc: 1,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bvc_no_branch() {
            let (mut cpu, mut mem) = setup(vec![0x50, 0xCD]);
            cpu.p = PV_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x50,
                    pc: 2,
                    p: PV_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bvc_branch_forward() {
            let (mut cpu, mut mem) = setup(vec![0x50, 5]);

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x50,
                    pc: 7,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bvc_branch_backward() {
            let (mut cpu, mut mem) = setup(vec![
                0xEA, 0xEA, 0xEA, 0x50, 0b11111100, // -4 in 2's comp
                0xEA,
            ]);
            cpu.pc = 3;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x50,
                    pc: 1,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bvs_no_branch() {
            let (mut cpu, mut mem) = setup(vec![0x70, 0xCD]);

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x70,
                    pc: 2,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bvs_branch_forward() {
            let (mut cpu, mut mem) = setup(vec![0x70, 5]);
            cpu.p = PV_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x70,
                    pc: 7,
                    p: PV_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bvs_branch_backward() {
            let (mut cpu, mut mem) = setup(vec![
                0xEA, 0xEA, 0xEA, 0x70, 0b11111100, // -4 in 2's comp
                0xEA,
            ]);
            cpu.pc = 3;
            cpu.p = PV_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x70,
                    pc: 1,
                    p: PV_MASK,
                    ..Cpu::new()
                }
            )
        }
    }

    mod bit_tests {
        use super::*;

        #[test]
        fn bit_zero() {
            let (mut cpu, mut mem) = setup(vec![0x89, 0b0000_0000]);
            cpu.p = PN_MASK | PV_MASK; // Should get cleared

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x89,
                    pc: 2,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn bit_nonzero() {
            let (mut cpu, mut mem) = setup(vec![0x89, 0b1100_0001]);
            cpu.a = 0b0000_0001;
            cpu.p = PZ_MASK; // Should get cleared

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x89,
                    pc: 2,
                    a: 1,
                    p: PN_MASK | PV_MASK,
                    ..Cpu::new()
                }
            )
        }
    }

    mod clear_tests {
        use super::*;

        #[test]
        fn clc() {
            let (mut cpu, mut mem) = setup(vec![0x18]);
            cpu.p = PC_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x18,
                    pc: 1,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn cld() {
            let (mut cpu, mut mem) = setup(vec![0xD8]);
            cpu.p = PD_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xD8,
                    pc: 1,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn cli() {
            let (mut cpu, mut mem) = setup(vec![0x58]);
            cpu.p = PI_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x58,
                    pc: 1,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn clv() {
            let (mut cpu, mut mem) = setup(vec![0xB8]);
            cpu.p = PV_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xB8,
                    pc: 1,
                    ..Cpu::new()
                }
            )
        }
    }

    mod cmp_tests {
        use super::*;

        #[test]
        fn cmp_equal() {
            let (mut cpu, mut mem) = setup(vec![0xC9, 10]);
            cpu.a = 10;
            cpu.p = PN_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xC9,
                    pc: 2,
                    a: 10,
                    p: PZ_MASK | PC_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn cmp_equal_zero() {
            let (mut cpu, mut mem) = setup(vec![0xC9, 0]);
            cpu.a = 0;
            cpu.p = PN_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xC9,
                    pc: 2,
                    a: 0,
                    p: PZ_MASK | PC_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn cmp_less_than() {
            let (mut cpu, mut mem) = setup(vec![0xC9, 9]);
            cpu.a = 10;
            cpu.p = PZ_MASK | PN_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xC9,
                    pc: 2,
                    a: 10,
                    p: PC_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn cmp_greater_than() {
            let (mut cpu, mut mem) = setup(vec![0xC9, 11]);
            cpu.a = 10;
            cpu.p = PZ_MASK | PC_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xC9,
                    pc: 2,
                    a: 10,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn cpx_equal() {
            let (mut cpu, mut mem) = setup(vec![0xE0, 10]);
            cpu.x = 10;
            cpu.p = PN_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xE0,
                    pc: 2,
                    x: 10,
                    p: PZ_MASK | PC_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn cpx_less_than() {
            let (mut cpu, mut mem) = setup(vec![0xE0, 9]);
            cpu.x = 10;
            cpu.p = PZ_MASK | PN_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xE0,
                    pc: 2,
                    x: 10,
                    p: PC_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn cpx_greater_than() {
            let (mut cpu, mut mem) = setup(vec![0xE0, 11]);
            cpu.x = 10;
            cpu.p = PZ_MASK | PC_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xE0,
                    pc: 2,
                    x: 10,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn cpy_equal() {
            let (mut cpu, mut mem) = setup(vec![0xC0, 10]);
            cpu.y = 10;
            cpu.p = PN_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xC0,
                    pc: 2,
                    y: 10,
                    p: PZ_MASK | PC_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn cpy_less_than() {
            let (mut cpu, mut mem) = setup(vec![0xC0, 9]);
            cpu.y = 10;
            cpu.p = PZ_MASK | PN_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xC0,
                    pc: 2,
                    y: 10,
                    p: PC_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn cpy_greater_than() {
            let (mut cpu, mut mem) = setup(vec![0xC0, 11]);
            cpu.y = 10;
            cpu.p = PZ_MASK | PC_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xC0,
                    pc: 2,
                    y: 10,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            )
        }
    }

    mod dec_tests {
        use super::*;

        #[test]
        fn dec_acc() {
            let (mut cpu, mut mem) = setup(vec![0x3A]);
            cpu.a = 10;
            cpu.p = PZ_MASK | PN_MASK; // Should get cleared

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x3A,
                    pc: 1,
                    a: 9,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn dec_acc_zero() {
            let (mut cpu, mut mem) = setup(vec![0x3A]);
            cpu.a = 1;
            cpu.p = PN_MASK; // Should get cleared

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x3A,
                    pc: 1,
                    a: 0,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn dec_acc_neg() {
            let (mut cpu, mut mem) = setup(vec![0x3A]);
            cpu.a = 0;
            cpu.p = PZ_MASK; // Should get cleared

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x3A,
                    pc: 1,
                    a: 255,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn dec_abs() {
            let (mut cpu, mut mem) = setup(vec![0xCE, 0xCD, 0xAB]);
            cpu.p = PZ_MASK | PN_MASK; // Should get cleared
            mem[0xABCD] = 10;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xCE,
                    pc: 3,
                    ..Cpu::new()
                }
            );
            assert_eq!(mem[0xABCD], 9);
        }

        #[test]
        fn dec_abs_zero() {
            let (mut cpu, mut mem) = setup(vec![0xCE, 0xCD, 0xAB]);
            cpu.p = PN_MASK; // Should get cleared
            mem[0xABCD] = 1;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xCE,
                    pc: 3,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
            assert_eq!(mem[0xABCD], 0);
        }

        #[test]
        fn dec_abs_neg() {
            let (mut cpu, mut mem) = setup(vec![0xCE, 0xCD, 0xAB]);
            cpu.p = PZ_MASK; // Should get cleared
            mem[0xABCD] = 0;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xCE,
                    pc: 3,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
            assert_eq!(mem[0xABCD], 255);
        }

        #[test]
        fn dex() {
            let (mut cpu, mut mem) = setup(vec![0xCA]);
            cpu.p = PZ_MASK | PN_MASK; // Should get cleared
            cpu.x = 10;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xCA,
                    pc: 1,
                    x: 9,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn dex_zero() {
            let (mut cpu, mut mem) = setup(vec![0xCA]);
            cpu.p = PN_MASK; // Should get cleared
            cpu.x = 1;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xCA,
                    pc: 1,
                    p: PZ_MASK,
                    x: 0,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn dex_neg() {
            let (mut cpu, mut mem) = setup(vec![0xCA]);
            cpu.p = PZ_MASK; // Should get cleared
            cpu.x = 0;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xCA,
                    pc: 1,
                    p: PN_MASK,
                    x: 255,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn dey() {
            let (mut cpu, mut mem) = setup(vec![0x88]);
            cpu.p = PZ_MASK | PN_MASK; // Should get cleared
            cpu.y = 10;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x88,
                    pc: 1,
                    y: 9,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn dey_zero() {
            let (mut cpu, mut mem) = setup(vec![0x88]);
            cpu.p = PN_MASK; // Should get cleared
            cpu.y = 1;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x88,
                    pc: 1,
                    p: PZ_MASK,
                    y: 0,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn dey_neg() {
            let (mut cpu, mut mem) = setup(vec![0x88]);
            cpu.p = PZ_MASK; // Should get cleared
            cpu.y = 0;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x88,
                    pc: 1,
                    p: PN_MASK,
                    y: 255,
                    ..Cpu::new()
                }
            );
        }
    }

    mod eor_tests {
        use super::*;

        #[test]
        fn eor_zero() {
            let (mut cpu, mut mem) = setup(vec![0x49, 0b11111111]);
            cpu.p = PN_MASK; // Should get cleared
            cpu.a = 0b11111111;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x49,
                    pc: 2,
                    a: 0,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn eor_neg() {
            let (mut cpu, mut mem) = setup(vec![0x49, 0b0110_0110]);
            cpu.p = PZ_MASK; // Should get cleared
            cpu.a = 0b1001_1001;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x49,
                    pc: 2,
                    a: 0b11111111,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }
    }

    mod inc_tests {
        use super::*;

        #[test]
        fn inc_acc() {
            let (mut cpu, mut mem) = setup(vec![0x1A]);
            cpu.a = 41;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x1A,
                    pc: 1,
                    a: 42,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn inc_acc_zero() {
            let (mut cpu, mut mem) = setup(vec![0x1A]);
            cpu.a = u8::MAX;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x1A,
                    pc: 1,
                    a: 0,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn inc_acc_reset_zero() {
            let (mut cpu, mut mem) = setup(vec![0x1A]);
            cpu.a = 5;
            cpu.p = PZ_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x1A,
                    pc: 1,
                    a: 6,
                    p: 0b00000000,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn inc_abs() {
            let (mut cpu, mut mem) = setup(vec![0xEE, 0xCD, 0xAB]);
            mem[0xABCD] = 41;

            cpu.step(&mut mem);

            assert_eq!(mem[0xABCD], 42);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xEE,
                    pc: 3,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn inc_abs_zero() {
            let (mut cpu, mut mem) = setup(vec![0xEE, 0xCD, 0xAB]);
            mem[0xABCD] = u8::MAX;

            cpu.step(&mut mem);

            assert_eq!(mem[0xABCD], 0);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xEE,
                    pc: 3,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn inc_abs_reset_zero() {
            let (mut cpu, mut mem) = setup(vec![0xEE, 0xCD, 0xAB]);
            mem[0xABCD] = 5;
            cpu.p = PZ_MASK;

            cpu.step(&mut mem);

            assert_eq!(mem[0xABCD], 6);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xEE,
                    pc: 3,
                    p: 0b00000000,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn inx() {
            let (mut cpu, mut mem) = setup(vec![0xE8]);
            cpu.x = 41;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0xE8]);
            cpu.x = u8::MAX;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0xE8]);
            cpu.x = 5;
            cpu.p = PZ_MASK;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0xE8]);
            cpu.x = 127;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0xE8]);
            cpu.x = u8::MAX;
            cpu.p = PN_MASK;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0xC8]);
            cpu.y = 40;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0xC8]);
            cpu.y = u8::MAX;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0xC8]);
            cpu.y = 5;
            cpu.p = PZ_MASK;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0xC8]);
            cpu.y = 127;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0xC8]);
            cpu.y = u8::MAX;
            cpu.p = PN_MASK;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0x4C, 0x0A, 0x80]);

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0x6C, 0x0A, 0x80]);
            mem[0x800A] = 0xCD;
            mem[0x800B] = 0xAB;

            cpu.step(&mut mem);

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
            let (mut cpu, mut mem) = setup(vec![0x7C, 0x00, 0x80]);

            cpu.x = 6;
            mem[0x8006] = 0xCD;
            mem[0x8007] = 0xAB;

            cpu.step(&mut mem);

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

        #[test]
        fn jsr() {
            // Pad rom with no-ops to get the PC to an interesting location
            let mut rom = vec![0xEA; 300];
            rom.extend(vec![0x20, 0xCD, 0xAB]);
            let (mut cpu, mut mem) = setup(rom);
            cpu.s = 0xFF;

            for _ in 0..301 {
                cpu.step(&mut mem);
            }

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x20,
                    pc: 0xABCD,
                    s: 0xFD,
                    ..Cpu::new()
                }
            );

            // 0x012E = 302nd byte - end of JSR instruction
            assert_eq!(mem[0x01FF], 0x01);
            assert_eq!(mem[0x01FE], 0x2E);
        }

        #[test]
        fn rts() {
            let (mut cpu, mut mem) = setup(vec![0x60]);
            cpu.s = 0xFD;
            mem[0x01FF] = 0x01;
            mem[0x01FE] = 0x2E;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x60,
                    pc: 0x012F,
                    s: 0xFF,
                    ..Cpu::new()
                }
            );
        }
    }

    mod ld_tests {
        use super::*;

        #[test]
        fn lda_immediate() {
            let (mut cpu, mut mem) = setup(vec![0xA9, 0xED]);
            cpu.p = PZ_MASK; // Should be cleared

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xA9,
                    a: 0xED,
                    pc: 2,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn lda_abs() {
            let (mut cpu, mut mem) = setup(vec![0xAD, 0xCD, 0xAB]);
            cpu.p = PN_MASK; // Should be cleared
            mem[0xABCD] = 0;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xAD,
                    a: 0,
                    pc: 3,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn ldx_immediate() {
            let (mut cpu, mut mem) = setup(vec![0xA2, 0xED]);
            cpu.p = PZ_MASK; // Should be cleared

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xA2,
                    x: 0xED,
                    pc: 2,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn ldx_abs() {
            let (mut cpu, mut mem) = setup(vec![0xAE, 0xCD, 0xAB]);
            cpu.p = PN_MASK; // Should be cleared
            mem[0xABCD] = 0;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xAE,
                    x: 0,
                    pc: 3,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn ldx_zpiy() {
            let (mut cpu, mut mem) = setup(vec![0xB6, 0xC0]);
            cpu.p = PN_MASK; // Should be cleared
            cpu.y = 10;
            mem[0x00CA] = 42;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xB6,
                    pc: 2,
                    x: 42,
                    y: 10,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn ldy_immediate() {
            let (mut cpu, mut mem) = setup(vec![0xA0, 0xED]);
            cpu.p = PZ_MASK; // Should be cleared

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xA0,
                    y: 0xED,
                    pc: 2,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn ldy_abs() {
            let (mut cpu, mut mem) = setup(vec![0xAC, 0xCD, 0xAB]);
            cpu.p = PN_MASK; // Should be cleared
            mem[0xABCD] = 0;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xAC,
                    y: 0,
                    pc: 3,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn ldy_zpix() {
            let (mut cpu, mut mem) = setup(vec![0xB4, 0xC0]);
            cpu.p = PN_MASK; // Should be cleared
            cpu.x = 10;
            mem[0x00CA] = 42;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xB4,
                    pc: 2,
                    x: 10,
                    y: 42,
                    ..Cpu::new()
                }
            );
        }
    }

    mod lsr_tests {
        use super::*;

        #[test]
        fn lsr_acc() {
            let (mut cpu, mut mem) = setup(vec![0x4A]);
            cpu.p = PN_MASK | PZ_MASK | PC_MASK; // Should be cleared
            cpu.a = 0b1111_0000;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x4A,
                    pc: 1,
                    a: 0b0111_1000,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn lsr_acc_zero() {
            let (mut cpu, mut mem) = setup(vec![0x4A]);
            cpu.p = PN_MASK; // Should be cleared
            cpu.a = 0b0000_0001;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x4A,
                    pc: 1,
                    a: 0,
                    p: PZ_MASK | PC_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn lsr_abs() {
            let (mut cpu, mut mem) = setup(vec![0x4E, 0xCD, 0xAB]);
            cpu.p = PN_MASK | PZ_MASK | PC_MASK; // Should be cleared
            mem[0xABCD] = 0b1111_0000;

            cpu.step(&mut mem);

            assert_eq!(mem[0xABCD], 0b0111_1000);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x4E,
                    pc: 3,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn lsr_abs_zero() {
            let (mut cpu, mut mem) = setup(vec![0x4E, 0xCD, 0xAB]);
            cpu.p = PN_MASK; // Should be cleared
            mem[0xABCD] = 0b0000_0001;

            cpu.step(&mut mem);

            assert_eq!(mem[0xABCD], 0);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x4E,
                    pc: 3,
                    p: PZ_MASK | PC_MASK,
                    ..Cpu::new()
                }
            );
        }
    }

    mod nop_tests {
        use super::*;

        #[test]
        fn nop() {
            let (mut cpu, mut mem) = setup(vec![0xEA]);

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xEA,
                    pc: 1,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn stp() {
            let (mut cpu, mut mem) = setup(vec![0xDB]);

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xDB,
                    pc: 1,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn wai() {
            let (mut cpu, mut mem) = setup(vec![0xCB]);

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xCB,
                    pc: 1,
                    ..Cpu::new()
                }
            );
        }
    }

    mod ora_tests {
        use super::*;

        #[test]
        fn ora_immediate() {
            let (mut cpu, mut mem) = setup(vec![0x09, 0b0011_1100]);
            cpu.a = 0b0000_0001;
            cpu.p = PN_MASK | PZ_MASK; // Should be cleared

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x09,
                    pc: 2,
                    a: 0b0011_1101,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn ora_immediate_zero() {
            let (mut cpu, mut mem) = setup(vec![0x09, 0]);
            cpu.a = 0;
            cpu.p = PN_MASK; // Should be cleared

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x09,
                    pc: 2,
                    a: 0,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn ora_immediate_neg() {
            let (mut cpu, mut mem) = setup(vec![0x09, 0b1000_0000]);
            cpu.a = 0b0000_0001;
            cpu.p = PZ_MASK; // Should be cleared

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x09,
                    pc: 2,
                    a: 0b1000_0001,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }
    }

    mod push_pull_stack_tests {
        use super::*;

        #[test]
        fn pha() {
            let (mut cpu, mut mem) = setup(vec![0x48]);
            cpu.a = 42;

            cpu.step(&mut mem);

            assert_eq!(mem[0x01FF], 42);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x48,
                    pc: 1,
                    a: 42,
                    s: 0xFE,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn php() {
            let (mut cpu, mut mem) = setup(vec![0x08]);
            cpu.p = 0b11001100;

            cpu.step(&mut mem);

            assert_eq!(mem[0x01FF], 0b11111100);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x08,
                    pc: 1,
                    p: 0b11001100,
                    s: 0xFE,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn php_overflow() {
            let (mut cpu, mut mem) = setup(vec![0x08]);
            cpu.p = 0b11001100;
            cpu.s = 0;

            cpu.step(&mut mem);

            assert_eq!(mem[0x0100], 0b11111100);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x08,
                    pc: 1,
                    p: 0b11001100,
                    s: 0xFF,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn phx() {
            let (mut cpu, mut mem) = setup(vec![0xDA]);
            cpu.x = 42;

            cpu.step(&mut mem);

            assert_eq!(mem[0x01FF], 42);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xDA,
                    pc: 1,
                    x: 42,
                    s: 0xFE,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn phy() {
            let (mut cpu, mut mem) = setup(vec![0x5A]);
            cpu.y = 42;

            cpu.step(&mut mem);

            assert_eq!(mem[0x01FF], 42);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x5A,
                    pc: 1,
                    y: 42,
                    s: 0xFE,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn pla_zero() {
            let (mut cpu, mut mem) = setup(vec![0x68]);
            cpu.a = 42;
            cpu.s = 0xFE;
            cpu.p = PN_MASK;
            mem[0x01FF] = 0;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x68,
                    pc: 1,
                    a: 0,
                    s: 0xFF,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn pla_neg() {
            let (mut cpu, mut mem) = setup(vec![0x68]);
            cpu.s = 0xFE;
            cpu.p = PZ_MASK;
            mem[0x01FF] = 200;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x68,
                    pc: 1,
                    a: 200,
                    s: 0xFF,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn plp() {
            let (mut cpu, mut mem) = setup(vec![0x28]);
            cpu.s = 0xFE;
            cpu.p = 63;
            mem[0x01FF] = 42;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x28,
                    pc: 1,
                    s: 0xFF,
                    p: 42,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn plp_underflow() {
            let (mut cpu, mut mem) = setup(vec![0x28]);
            cpu.s = 0xFF;
            cpu.p = 63;
            mem[0x0100] = 42;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x28,
                    pc: 1,
                    s: 0x00,
                    p: 42,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn plx_zero() {
            let (mut cpu, mut mem) = setup(vec![0xFA]);
            cpu.x = 42;
            cpu.s = 0xFE;
            cpu.p = PN_MASK;
            mem[0x01FF] = 0;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xFA,
                    pc: 1,
                    x: 0,
                    s: 0xFF,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn plx_neg() {
            let (mut cpu, mut mem) = setup(vec![0xFA]);
            cpu.s = 0xFE;
            cpu.p = PZ_MASK;
            mem[0x01FF] = 200;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xFA,
                    pc: 1,
                    x: 200,
                    s: 0xFF,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn ply_zero() {
            let (mut cpu, mut mem) = setup(vec![0x7A]);
            cpu.y = 42;
            cpu.s = 0xFE;
            cpu.p = PN_MASK;
            mem[0x01FF] = 0;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x7A,
                    pc: 1,
                    y: 0,
                    s: 0xFF,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn ply_neg() {
            let (mut cpu, mut mem) = setup(vec![0x7A]);
            cpu.s = 0xFE;
            cpu.p = PZ_MASK;
            mem[0x01FF] = 200;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x7A,
                    pc: 1,
                    y: 200,
                    s: 0xFF,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }
    }

    mod rmb_tests {
        use super::*;

        #[test]
        fn rmb_all_bits() {
            for i in 0..8 {
                let opcode = 0x07 + (i * 0x10);
                let rom = vec![opcode, 0xCD];
                let (mut cpu, mut mem) = setup(rom);
                mem[0x00CD] = 0xFF;

                cpu.step(&mut mem);

                assert_eq!(mem[0x00CD], 0xFF ^ (1 << i));
                assert_eq!(
                    cpu,
                    Cpu {
                        ir: opcode,
                        pc: 2,
                        ..Cpu::new()
                    }
                )
            }
        }
    }

    mod rotate_tests {
        use super::*;

        #[test]
        fn rol_acc_zero() {
            let (mut cpu, mut mem) = setup(vec![0x2A]);
            cpu.p = PN_MASK; // Should be cleared
            cpu.a = 0b1000_0000;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x2A,
                    pc: 1,
                    a: 0,
                    p: PC_MASK | PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn rol_acc_neg() {
            let (mut cpu, mut mem) = setup(vec![0x2A]);
            cpu.p = PZ_MASK | PC_MASK; // Should be cleared
            cpu.a = 0b0100_0000;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x2A,
                    pc: 1,
                    a: 0b1000_0001,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn rol_abs_zero() {
            let (mut cpu, mut mem) = setup(vec![0x2E, 0xCD, 0xAB]);
            cpu.p = PN_MASK; // Should be cleared
            mem[0xABCD] = 0b1000_0000;

            cpu.step(&mut mem);

            assert_eq!(mem[0xABCD], 0);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x2E,
                    pc: 3,
                    p: PC_MASK | PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn rol_abs_neg() {
            let (mut cpu, mut mem) = setup(vec![0x2E, 0xCD, 0xAB]);
            cpu.p = PZ_MASK | PC_MASK; // Should be cleared
            mem[0xABCD] = 0b0100_0000;

            cpu.step(&mut mem);

            assert_eq!(mem[0xABCD], 0b1000_0001);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x2E,
                    pc: 3,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn ror_acc_zero() {
            let (mut cpu, mut mem) = setup(vec![0x6A]);
            cpu.p = PN_MASK; // Should be cleared
            cpu.a = 0b0000_0001;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x6A,
                    pc: 1,
                    a: 0,
                    p: PC_MASK | PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn ror_acc_neg() {
            let (mut cpu, mut mem) = setup(vec![0x6A]);
            cpu.p = PZ_MASK | PC_MASK; // Should be cleared
            cpu.a = 0b0000_0010;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x6A,
                    pc: 1,
                    a: 0b1000_0001,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn ror_abs_zero() {
            let (mut cpu, mut mem) = setup(vec![0x6E, 0xCD, 0xAB]);
            cpu.p = PN_MASK; // Should be cleared
            mem[0xABCD] = 0b0000_0001;

            cpu.step(&mut mem);

            assert_eq!(mem[0xABCD], 0);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x6E,
                    pc: 3,
                    p: PC_MASK | PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn ror_abs_neg() {
            let (mut cpu, mut mem) = setup(vec![0x6E, 0xCD, 0xAB]);
            cpu.p = PZ_MASK | PC_MASK; // Should be cleared
            mem[0xABCD] = 0b0000_0010;

            cpu.step(&mut mem);

            assert_eq!(mem[0xABCD], 0b1000_0001);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x6E,
                    pc: 3,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }
    }

    mod sbc_test {
        use super::*;

        #[test]
        fn sbc_immediate() {
            let (mut cpu, mut mem) = setup(vec![0xE9, 0xFF]);
            cpu.a = 0;
            cpu.p = PC_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xE9,
                    pc: 2,
                    a: 1,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn sbc_immediate_zero_minus_zero_with_carry() {
            let (mut cpu, mut mem) = setup(vec![0xE9, 0]);
            cpu.a = 0;
            cpu.p = PV_MASK | PZ_MASK | PC_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xE9,
                    pc: 2,
                    a: 0,
                    p: PZ_MASK | PC_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn sbc_decimal_99_minus_0() {
            let (mut cpu, mut mem) = setup(vec![0xE9, 0]);
            cpu.a = 0x99;
            cpu.p = PN_MASK | PV_MASK | PD_MASK | PC_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xE9,
                    pc: 2,
                    a: 0x99,
                    p: PN_MASK | PV_MASK | PD_MASK | PC_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn sbc_decimal_0_minus_0() {
            let (mut cpu, mut mem) = setup(vec![0xE9, 0]);
            cpu.a = 0x0;
            cpu.p = PV_MASK | PD_MASK | PZ_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xE9,
                    pc: 2,
                    a: 0x99,
                    p: PN_MASK | PV_MASK | PD_MASK,
                    ..Cpu::new()
                }
            )
        }

        #[test]
        fn sbc_decimal_12_minus_21() {
            let (mut cpu, mut mem) = setup(vec![0xE9, 0x21]);
            cpu.a = 0x12;
            cpu.p = PD_MASK | PC_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xE9,
                    pc: 2,
                    a: 0x91,
                    p: PN_MASK | PV_MASK | PD_MASK,
                    ..Cpu::new()
                }
            )
        }
    }

    mod set_tests {
        use super::*;

        #[test]
        fn sec() {
            let (mut cpu, mut mem) = setup(vec![0x38]);

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x38,
                    pc: 1,
                    p: PC_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn sed() {
            let (mut cpu, mut mem) = setup(vec![0xF8]);

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xF8,
                    pc: 1,
                    p: PD_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn sei() {
            let (mut cpu, mut mem) = setup(vec![0x78]);

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x78,
                    pc: 1,
                    p: PI_MASK,
                    ..Cpu::new()
                }
            );
        }
    }

    mod smb_tests {
        use super::*;

        #[test]
        fn smb_all_bits() {
            for i in 0..8 {
                let opcode = 0x87 + (i * 0x10);
                let rom = vec![opcode, 0xCD];
                let (mut cpu, mut mem) = setup(rom);
                mem[0x00CD] = 0;

                cpu.step(&mut mem);

                assert_eq!(mem[0x00CD], 1 << i);
                assert_eq!(
                    cpu,
                    Cpu {
                        ir: opcode,
                        pc: 2,
                        ..Cpu::new()
                    }
                )
            }
        }
    }

    mod store_tests {
        use super::*;

        #[test]
        fn sta_abs() {
            let (mut cpu, mut mem) = setup(vec![0x8D, 0x02, 0x60]);

            cpu.a = 57;
            cpu.step(&mut mem);

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

        #[test]
        fn sta_zpii() {
            let (mut cpu, mut mem) = setup(vec![0x81, 0xC0]);
            cpu.x = 10;
            cpu.a = 57;
            mem[0x00CA] = 0x57;
            mem[0x00CB] = 0x43;
            mem[0x4357] = 0;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x81,
                    a: 57,
                    x: 10,
                    pc: 2,
                    ..Cpu::new()
                }
            );

            assert_eq!(mem[0x4357], 57);
        }

        #[test]
        fn stx_abs() {
            let (mut cpu, mut mem) = setup(vec![0x8E, 0x02, 0x60]);

            cpu.x = 57;
            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x8E,
                    x: 57,
                    pc: 3,
                    ..Cpu::new()
                }
            );

            assert_eq!(mem[0x6002], 57);
        }

        #[test]
        fn sty_zp() {
            let (mut cpu, mut mem) = setup(vec![0x84, 0xCD]);
            cpu.y = 57;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x84,
                    y: 57,
                    pc: 2,
                    ..Cpu::new()
                }
            );

            assert_eq!(mem[0x00CD], 57);
        }

        #[test]
        fn stz_aix() {
            let (mut cpu, mut mem) = setup(vec![0x9E, 0xC0, 0xAB]);
            cpu.x = 5;
            mem[0xABC5] = 42;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x9E,
                    x: 5,
                    pc: 3,
                    ..Cpu::new()
                }
            );

            assert_eq!(mem[0xABC5], 0);
        }
    }

    mod transfer_tests {
        use super::*;

        #[test]
        fn tax_zero() {
            let (mut cpu, mut mem) = setup(vec![0xAA]);

            cpu.a = 0;
            cpu.p = PN_MASK;
            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xAA,
                    a: 0,
                    x: 0,
                    pc: 1,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn tax_neg() {
            let (mut cpu, mut mem) = setup(vec![0xAA]);

            cpu.a = 200;
            cpu.p = PZ_MASK;
            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xAA,
                    a: 200,
                    x: 200,
                    pc: 1,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn tay_zero() {
            let (mut cpu, mut mem) = setup(vec![0xA8]);

            cpu.a = 0;
            cpu.p = PN_MASK;
            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xA8,
                    a: 0,
                    y: 0,
                    pc: 1,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn tay_neg() {
            let (mut cpu, mut mem) = setup(vec![0xA8]);

            cpu.a = 200;
            cpu.p = PZ_MASK;
            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xA8,
                    a: 200,
                    y: 200,
                    pc: 1,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn tsx_zero() {
            let (mut cpu, mut mem) = setup(vec![0xBA]);

            cpu.s = 0;
            cpu.p = PN_MASK;
            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xBA,
                    pc: 1,
                    p: PZ_MASK,
                    s: 0,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn tsx_neg() {
            let (mut cpu, mut mem) = setup(vec![0xBA]);

            cpu.s = 200;
            cpu.p = PZ_MASK;
            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0xBA,
                    s: 200,
                    x: 200,
                    pc: 1,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn txa_zero() {
            let (mut cpu, mut mem) = setup(vec![0x8A]);

            cpu.p = PN_MASK;
            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x8A,
                    pc: 1,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn txa_neg() {
            let (mut cpu, mut mem) = setup(vec![0x8A]);

            cpu.x = 200;
            cpu.p = PZ_MASK;
            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x8A,
                    a: 200,
                    x: 200,
                    pc: 1,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn txs_zero() {
            let (mut cpu, mut mem) = setup(vec![0x9A]);

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x9A,
                    pc: 1,
                    s: 0,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn txs_neg() {
            let (mut cpu, mut mem) = setup(vec![0x9A]);

            cpu.x = 200;
            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x9A,
                    s: 200,
                    x: 200,
                    pc: 1,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn tya_zero() {
            let (mut cpu, mut mem) = setup(vec![0x98]);

            cpu.p = PN_MASK;
            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x98,
                    pc: 1,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn tya_neg() {
            let (mut cpu, mut mem) = setup(vec![0x98]);

            cpu.y = 200;
            cpu.p = PZ_MASK;
            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x98,
                    a: 200,
                    y: 200,
                    pc: 1,
                    p: PN_MASK,
                    ..Cpu::new()
                }
            );
        }
    }

    mod test_and_set_or_reset_tests {
        use super::*;

        #[test]
        fn trb() {
            let (mut cpu, mut mem) = setup(vec![0x1C, 0xCD, 0xAB]);
            mem[0xABCD] = 0b0011_1100;
            cpu.a = 0b0000_1111;
            cpu.p = PZ_MASK;

            cpu.step(&mut mem);

            assert_eq!(mem[0xABCD], 0b0011_0000);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x1C,
                    pc: 3,
                    a: 0b0000_1111,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn trb_zero() {
            let (mut cpu, mut mem) = setup(vec![0x1C, 0xCD, 0xAB]);
            mem[0xABCD] = 0b0000_1100;
            cpu.a = 0b0000_1111;

            cpu.step(&mut mem);

            assert_eq!(mem[0xABCD], 0);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x1C,
                    pc: 3,
                    a: 0b0000_1111,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn tsb() {
            let (mut cpu, mut mem) = setup(vec![0x0C, 0xCD, 0xAB]);
            mem[0xABCD] = 0b0011_1100;
            cpu.a = 0b0000_1111;
            cpu.p = PZ_MASK;

            cpu.step(&mut mem);

            assert_eq!(mem[0xABCD], 0b0011_1111);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x0C,
                    pc: 3,
                    a: 0b0000_1111,
                    ..Cpu::new()
                }
            );
        }

        #[test]
        fn tsb_zero() {
            let (mut cpu, mut mem) = setup(vec![0x0C, 0xCD, 0xAB]);
            cpu.p = PZ_MASK;

            cpu.step(&mut mem);

            assert_eq!(mem[0xABCD], 0);
            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x0C,
                    pc: 3,
                    p: PZ_MASK,
                    ..Cpu::new()
                }
            );
        }
    }

    mod interrupt_tests {
        use super::*;

        #[test]
        fn brk() {
            // Pad rom with no-ops to get the PC to an interesting location
            let mut rom = vec![0xEA; 300];
            rom.extend(vec![0x00]);
            let (mut cpu, mut mem) = setup(rom);
            cpu.p = PZ_MASK | PN_MASK | PD_MASK;
            mem[0xFFFE] = 0xCD;
            mem[0xFFFF] = 0xAB;

            for _ in 0..301 {
                cpu.step(&mut mem);
            }

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x00,
                    pc: 0xABCD,
                    s: 0xFC,
                    p: PZ_MASK | PN_MASK | PI_MASK,
                    ..Cpu::new()
                }
            );

            // 0x012D = 301st byte - end of BRK instruction
            assert_eq!(mem[0x01FF], 0x01);
            assert_eq!(mem[0x01FE], 0x2E);
            assert_eq!(mem[0x01FD], PZ_MASK | PN_MASK | PD_MASK | PB_MASK | P5_MASK);
        }

        #[test]
        fn rti() {
            let (mut cpu, mut mem) = setup(vec![0x40]);
            cpu.p = PZ_MASK | PN_MASK | PI_MASK;
            cpu.s = 0xFC;
            mem[0x01FF] = 0x01;
            mem[0x01FE] = 0x2D;
            mem[0x01FD] = PZ_MASK | PN_MASK;

            cpu.step(&mut mem);

            assert_eq!(
                cpu,
                Cpu {
                    ir: 0x40,
                    pc: 0x012D,
                    s: 0xFF,
                    p: PZ_MASK | PN_MASK,
                    ..Cpu::new()
                }
            );
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

    #[test]
    fn test_bin_to_dec() {
        let cpu = Cpu::new();

        assert_eq!(cpu.bin_to_dec(0x99), 99);
    }

    #[test]
    fn test_dec_to_bin() {
        let cpu = Cpu::new();

        assert_eq!(cpu.dec_to_bin(46), 0x46);
        assert_eq!(cpu.dec_to_bin(173), 0x73);
    }
}
