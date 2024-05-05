#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use rp2040_hal as hal;

use hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac::{self, interrupt},
    sio::Sio,
    watchdog::Watchdog,
};

use usb_device::{class_prelude::*, prelude::*};
use usbd_hid::{
    descriptor::{generator_prelude::*, KeyboardReport},
    hid_class::HIDClass,
};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[rtic::app(device = rp2040_hal::pac, peripherals = true, dispatchers = [I2C0_IRQ])]
mod app {
    use embedded_hal::digital::{InputPin, OutputPin};
    use rp2040_hal as hal;

    use hal::{
        clocks::init_clocks_and_plls,
        fugit::ExtU64,
        gpio,
        timer::{monotonic::Monotonic, Alarm0},
        Sio, Watchdog,
    };

    const XOSC_CRYSTAL_FREQ: u32 = 12_000_000u32;

    #[shared]
    struct Shared {
        led: gpio::Pin<gpio::bank0::Gpio25, gpio::FunctionSioOutput, gpio::PullNone>,
    }

    #[local]
    struct Local {}

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Monotonic<Alarm0>;

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset doesn't release hardware spinlocks
        // Release them here to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }

        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(c.device.SIO);
        let pins = gpio::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let mut led = pins.gpio25.reconfigure();
        led.set_low().unwrap();

        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets, &clocks);
        let alarm = timer.alarm_0().unwrap();
        blink_led::spawn_after(500.millis()).unwrap();

        (
            Shared { led },
            Local {},
            init::Monotonics(Monotonic::new(timer, alarm)),
        )
    }

    #[task(shared = [led], local = [tog: bool = true])]
    fn blink_led(mut c: blink_led::Context) {
        if *c.local.tog {
            c.shared.led.lock(|l| l.set_high().unwrap());
        } else {
            c.shared.led.lock(|l| l.set_low().unwrap());
        }

        *c.local.tog = !*c.local.tog;

        blink_led::spawn_after(500.millis()).unwrap();
    }
}
