#![no_std]
#![no_main]

extern crate panic_halt;

use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::hal::delay::Sleep;
use hifive1::{self, pin, Led};
use riscv_rt::entry;

struct Board {
    pub sleep: Sleep,
    pub leds: (hifive1::RED, hifive1::GREEN, hifive1::BLUE),
}

impl Board {
    pub fn new() -> Board {
        let dr = DeviceResources::take().unwrap();
        let p = dr.peripherals;
        let pins = dr.pins;

        let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());
        hifive1::stdout::configure(
            p.UART0,
            pin!(pins, uart0_tx),
            pin!(pins, uart0_rx),
            115_200.bps(),
            clocks,
        );
        let leds = hifive1::rgb(pin!(pins, led_red), pin!(pins, led_green), pin!(pins, led_blue));

        let clint = dr.core_peripherals.clint;
        let sleep = Sleep::new(clint.mtimecmp, clocks);

        let mut result = Board {
            sleep,
            leds,
        };

        result.light(false, false, false);
        result
    }

    pub fn light(&mut self, red: bool, green: bool, blue: bool) {
        if red {
            self.leds.0.on();
        } else {
            self.leds.0.off();
        }

        if green {
            self.leds.1.on();
        } else {
            self.leds.1.off();
        }

        if blue {
            self.leds.2.on();
        } else {
            self.leds.2.off();
        }
    }

    pub fn delay(&mut self, delay: u32) {
        self.sleep.delay_ms(delay);
    }
}

#[entry]
fn main() -> ! {
    let mut board = Board::new();

    let mut index = 1;
    loop {
        if index % 2 == 1 {
            board.light(index & 0b0010 > 0, index & 0b0100 > 0, index & 0b1000 > 0);
        } else {
            board.light(false, false, false);
        }
        board.delay(500);
        index = index % 15 + 1;
    }
}
