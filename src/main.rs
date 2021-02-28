#![no_std]
#![no_main]

extern crate panic_halt;

use bme280::BME280;
use embedded_hal::blocking::delay::DelayMs;
use heapless::String;
use hifive1::hal::{
    delay::Sleep, e310x::I2C0, gpio::gpio0::Pin12, gpio::gpio0::Pin13, gpio::NoInvert,
    gpio::Output, gpio::Regular, gpio::IOF0, i2c::I2c, i2c::Speed, DeviceResources,
};
use hifive1::hal::{gpio::gpio0::Pin11, prelude::*};
use hifive1::{self, pin, Led};
use riscv_rt::entry;
use shared_bus::BusManagerSimple;
use ssd1306::{
    displaysize::DisplaySize128x64, mode::TerminalMode, prelude::I2CInterface, I2CDIBuilder,
};

// Type aliases

type HiFiveI2c = I2c<I2C0, (Pin12<IOF0<NoInvert>>, Pin13<IOF0<NoInvert>>)>;

type I2cProxy<'b> = shared_bus::I2cProxy<'b, shared_bus::NullMutex<HiFiveI2c>>;

type Terminal<'b> = TerminalMode<I2CInterface<I2cProxy<'b>>, DisplaySize128x64>;

// Peripherals

struct Leds(hifive1::RED, hifive1::GREEN, hifive1::BLUE);

impl Leds {
    fn new(red: hifive1::RED, green: hifive1::GREEN, blue: hifive1::BLUE) -> Leds {
        let mut result = Leds(red, green, blue);
        result.light(false, false, false);
        result
    }

    fn light(&mut self, red: bool, green: bool, blue: bool) {
        if red {
            self.0.on();
        } else {
            self.0.off();
        }

        if green {
            self.1.on();
        } else {
            self.1.off();
        }

        if blue {
            self.2.on();
        } else {
            self.2.off();
        }
    }
}

struct I2cBus(BusManagerSimple<HiFiveI2c>);

impl I2cBus {
    fn display(&self) -> Terminal {
        let iface = I2CDIBuilder::new().init(self.0.acquire_i2c());
        let mut display: TerminalMode<_, _> = ssd1306::Builder::new().connect(iface).into();
        display.init().unwrap();
        let _ = display.clear();
        display
    }

    fn temperature_sensor<D: DelayMs<u8>>(&self, delay: &mut D) -> BME280<I2cProxy> {
        let mut bme = BME280::new_primary(self.0.acquire_i2c());
        let result = bme.init(delay);
        result.unwrap();
        bme
    }
}

// Board

#[allow(unused)]
struct Board {
    sleep: Sleep,
    leds: Leds,
    i2c: I2cBus,
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
        let (red, green, blue) = hifive1::rgb(
            pin!(pins, led_red),
            pin!(pins, led_green),
            pin!(pins, led_blue),
        );
        let leds = Leds::new(red, green, blue);
        let sda = pin!(pins, i2c0_sda).into_iof0();
        let scl = pin!(pins, i2c0_scl).into_iof0();
        let i2c = I2cBus(BusManagerSimple::new(I2c::new(
            p.I2C0,
            sda,
            scl,
            Speed::Normal,
            clocks,
        )));
        let out = pin!(pins, dig17).into_output();

        let clint = dr.core_peripherals.clint;
        let sleep = Sleep::new(clint.mtimecmp, clocks);

        Board {
            sleep,
            leds,
            i2c,
            out,
        }
    }
}

#[entry]
fn main() -> ! {
    let mut board = Board::new();
    let mut display = board.i2c.display();

    display.write_str("Hello world").unwrap();

    // let mut lines: [String<heapless::consts::U32>; 2] = [String::new(), String::new()];

    let mut index = 1;
    loop {
        // if let Ok(msr) = temp.measure(&mut board.sleep) {
        //     lines.iter_mut().for_each(String::clear);
        //     write!(lines[0], "t {:.1}, h {:.1}%", msr.temperature, msr.humidity).unwrap();
        //     display.clear().unwrap();
        //     display.write_str(lines[0].as_str()).unwrap();
        // }

        if index % 2 == 1 {
            board
                .leds
                .light(index & 0b0010 > 0, index & 0b0100 > 0, index & 0b1000 > 0);
        } else {
            board.leds.light(false, false, false);
        }
        board.sleep.delay_ms(1000u32);
        index = index % 15 + 1;
    }
}
