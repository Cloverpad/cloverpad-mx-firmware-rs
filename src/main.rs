#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::digital::OutputPin;
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

pub const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// The USB device driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

#[rp2040_hal::entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Setup the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // NOTE: Safe as interrupts haven't started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Setup the USB HID Class Device driver, providing mouse reports
    // let usb_hid = HIDClass::new(bus_ref, NkroReport::desc(), 1);
    let usb_hid = HIDClass::new(bus_ref, KeyboardReport::desc(), 1);
    unsafe {
        // NOTE: Safe as interrupts haven't started yet
        USB_HID = Some(usb_hid);
    }

    // Setup the USB device description
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x1404, 0x1404))
        .strings(&[StringDescriptors::default()
            .manufacturer("Cloverpad")
            .product("Cloverpad MX")
            .serial_number("CLP-MX-01")])
        .unwrap()
        .build();
    unsafe {
        // NOTE: Safe as interrupts haven't started yet
        USB_DEVICE = Some(usb_dev);
    }

    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    }

    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut led_pin = pins.gpio25.into_push_pull_output();

    const PRESSED: KeyboardReport = KeyboardReport {
        modifier: 0,
        reserved: 0,
        leds: 0,
        keycodes: [0x04, 0, 0, 0, 0, 0],
    };

    const RELEASED: KeyboardReport = KeyboardReport {
        modifier: 0,
        reserved: 0,
        leds: 0,
        keycodes: [0; 6],
    };

    loop {
        led_pin.set_high().unwrap();
        push_hid_report(&PRESSED).ok().unwrap_or(0);
        delay.delay_ms(1000);

        led_pin.set_low().unwrap();
        push_hid_report(&RELEASED).ok().unwrap_or(0);
        delay.delay_ms(1000);
    }
}

fn push_hid_report<IR: AsInputReport>(report: &IR) -> Result<usize, usb_device::UsbError> {
    // NOTE: Requires critical section, as we are modifying the USB_HID global variable
    critical_section::with(|_| unsafe {
        // With interrupts being disabled, retrieve the global variable and send as a HID report
        USB_HID.as_mut().map(|hid| hid.push_input(report))
    })
    .unwrap()
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();

    if usb_dev.poll(&mut [usb_hid]) {
        // The OS may send a report to the keypad, e.g. setting NumLock LED
        // We don't need to process this, so read + discard any output
        let mut buf = [0; 64];
        let _ = usb_hid.pull_raw_output(&mut buf);
    }
}
