#![no_std]

use defmt::{info, Format};

use embassy_rp::dma::{AnyChannel, Channel};
use embassy_rp::pio::{
    Common, Config, FifoJoin, Instance, PioPin, ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::{clocks, Peri};
use embassy_time::Timer;

use fixed::types::U24F8;
use fixed_macro::fixed;
use smart_leds::{RGB8, colors as colours};
use core::mem::transmute;

// For all colour options, see https://docs.rs/smart-leds/0.4.0/smart_leds/colors/index.html
// These are the only ones I need.
#[derive(Copy, Clone, Format, PartialEq)]
#[repr(u8)]
pub enum Colour {
    ORANGE,
    BLUE,
    RED,
    GREEN,
    WHITE,
}

impl Colour {
    pub fn from_integer(v: u8) -> Self {
	match v {
	    0 => Self::ORANGE,
	    1 => Self::BLUE,
	    2 => Self::RED,
	    3 => Self::GREEN,
	    4 => Self::WHITE,
	    _ => panic!("Unknown value: {}", v),
	}
    }
}

impl From<u8> for Colour {
    fn from(t: u8) -> Colour {
	assert!(Self::ORANGE as u8 <= t && t <= Self::WHITE as u8);
	unsafe { transmute(t) }
    }
}

// ================================================================================

pub struct Ws2812<'d, P: Instance, const S: usize> {
    dma: Peri<'d, AnyChannel>,
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize> Ws2812<'d, P, S> {
    pub fn new(
        pio: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        dma: Peri<'d, impl Channel>,
        pin: Peri<'d, impl PioPin + 'd>,
    ) -> Self {
        // Setup sm0

        // prepare the PIO program
        let side_set = pio::SideSet::new(false, 1, false);
        let mut a: pio::Assembler<{ pio::RP2040_MAX_PROGRAM_SIZE }> =
            pio::Assembler::new_with_side_set(side_set);

        const T1: u8 = 2; // start bit
        const T2: u8 = 5; // data bit
        const T3: u8 = 3; // stop bit
        const CYCLES_PER_BIT: u32 = (T1 + T2 + T3) as u32;

        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut do_zero = a.label();
        a.set_with_side_set(pio::SetDestination::PINDIRS, 1, 0);
        a.bind(&mut wrap_target);
        // Do stop bit
        a.out_with_delay_and_side_set(pio::OutDestination::X, 1, T3 - 1, 0);
        // Do start bit
        a.jmp_with_delay_and_side_set(pio::JmpCondition::XIsZero, &mut do_zero, T1 - 1, 1);
        // Do data bit = 1
        a.jmp_with_delay_and_side_set(pio::JmpCondition::Always, &mut wrap_target, T2 - 1, 1);
        a.bind(&mut do_zero);
        // Do data bit = 0
        a.nop_with_delay_and_side_set(T2 - 1, 0);
        a.bind(&mut wrap_source);

        let prg = a.assemble_with_wrap(wrap_source, wrap_target);
        let mut cfg = Config::default();

        // Pin config
        let out_pin = pio.make_pio_pin(pin);
        cfg.set_out_pins(&[&out_pin]);
        cfg.set_set_pins(&[&out_pin]);

        cfg.use_program(&pio.load_program(&prg), &[&out_pin]);

        // Clock config, measured in kHz to avoid overflows
        // TODO: CLOCK_FREQ should come from embassy_rp
        let clock_freq = U24F8::from_num(clocks::clk_sys_freq() / 1000);
        let ws2812_freq = fixed!(800: U24F8);
        let bit_freq = ws2812_freq * CYCLES_PER_BIT;
        cfg.clock_divider = clock_freq / bit_freq;

        // FIFO config
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 24,
            direction: ShiftDirection::Left,
        };

        sm.set_config(&cfg);
        sm.set_enable(true);

        Self {
            dma: dma.into(),
            sm,
        }
    }

    pub async fn write(&mut self, colors: &[RGB8; 1]) {
        // Precompute the word bytes from the colors
        let mut words = [0u32; 1];
        for i in 0..1 {
            let word = (u32::from(colors[i].r) << 24)
                | (u32::from(colors[i].g) << 16)
                | (u32::from(colors[i].b) << 8);
            words[i] = word;
        }

        // DMA transfer
        self.sm
            .tx()
            .dma_push(self.dma.reborrow(), &words, false)
            .await;

        Timer::after_micros(55).await;
    }

    pub async fn set_colour(&mut self, colour: Colour) {
        info!("Setting LED colour: {}", colour);

        let c: [RGB8; 1];
        match colour {
	    Colour::ORANGE => c = [(colours::YELLOW).into()],
	    Colour::BLUE   => c = [(colours::BLUE).into()],
	    Colour::RED    => c = [(colours::RED).into()],
	    Colour::GREEN  => c = [(colours::GREEN).into()],
	    Colour::WHITE  => c = [(colours::WHITE).into()],
	}

        self.write(&c).await;
    }
}
