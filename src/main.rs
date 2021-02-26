#![no_std]
#![no_main]

extern crate panic_halt;

use hifive1::hal::{
    delay::Sleep, e310x::I2C0, gpio::gpio0::Pin12, gpio::gpio0::Pin13, gpio::NoInvert,
    gpio::Output, gpio::Regular, gpio::IOF0, i2c::I2c, i2c::Speed, DeviceResources,
};
use hifive1::hal::{gpio::gpio0::Pin11, prelude::*};
use hifive1::{self, pin, Led};
use riscv_rt::entry;
use shared_bus::{BusManagerSimple, I2cProxy};
use ssd1306::{
    displaysize::DisplaySize128x64, mode::TerminalMode, prelude::I2CInterface, I2CDIBuilder,
};

type HiFiveI2c = I2c<I2C0, (Pin12<IOF0<NoInvert>>, Pin13<IOF0<NoInvert>>)>;

type DisplayInterface<'b> = I2CInterface<I2cProxy<'b, shared_bus::NullMutex<HiFiveI2c>>>;

type Terminal<'b> = TerminalMode<DisplayInterface<'b>, DisplaySize128x64>;

#[allow(unused)]
struct Board {
    sleep: Sleep,
    leds: (hifive1::RED, hifive1::GREEN, hifive1::BLUE),
    i2c: BusManagerSimple<HiFiveI2c>,
    out: Pin11<Output<Regular<NoInvert>>>,
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
        let leds = hifive1::rgb(
            pin!(pins, led_red),
            pin!(pins, led_green),
            pin!(pins, led_blue),
        );
        let sda = pin!(pins, i2c0_sda).into_iof0();
        let scl = pin!(pins, i2c0_scl).into_iof0();
        let i2c = BusManagerSimple::new(I2c::new(p.I2C0, sda, scl, Speed::Normal, clocks));
        let out = pin!(pins, dig17).into_output();

        let clint = dr.core_peripherals.clint;
        let sleep = Sleep::new(clint.mtimecmp, clocks);

        let mut result = Board {
            sleep,
            leds,
            i2c,
            out,
        };

        result.light(false, false, false);
        result
    }

    pub fn display(&self) -> Terminal {
        let iface = I2CDIBuilder::new().init(self.i2c.acquire_i2c());
        let mut display: TerminalMode<_, _> = ssd1306::Builder::new().connect(iface).into();
        display.init().unwrap();
        let _ = display.clear();
        display
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
    let mut display = board.display();
    display.write_str("Hello world").unwrap();

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
