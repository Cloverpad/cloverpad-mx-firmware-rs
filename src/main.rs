#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[rtic::app(device = rp2040_hal::pac, peripherals = true, dispatchers = [I2C0_IRQ])]
mod app {
    use defmt::*;
    use defmt_rtt as _;
    use panic_probe as _;

    use embedded_hal::digital::OutputPin;
    use rp2040_hal as hal;

    use hal::{
        clocks::init_clocks_and_plls,
        fugit::ExtU64,
        gpio,
        timer::{monotonic::Monotonic, Alarm0},
        Sio, Watchdog,
    };

    use usb_device::{class_prelude::*, prelude::*};
    use usbd_hid::{
        descriptor::{generator_prelude::*, AsInputReport, KeyboardReport},
        hid_class::HIDClass,
    };

    const XOSC_CRYSTAL_FREQ: u32 = 12_000_000u32;

    #[shared]
    struct Shared {
        led: gpio::Pin<gpio::bank0::Gpio25, gpio::FunctionSioOutput, gpio::PullNone>,
        usb_device: UsbDevice<'static, rp2040_hal::usb::UsbBus>,
        hid_keyboard: HIDClass<'static, rp2040_hal::usb::UsbBus>,
    }

    #[local]
    struct Local {
        key_report: KeyboardReport,
    }

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Monotonic<Alarm0>;

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        info!("Initialising app");

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

        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets, &clocks);
        let sio = Sio::new(c.device.SIO);
        let pins = gpio::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // Setup the USB driver
        // NOTE: The USB bus allocator needs a static lifetime so we can store references in shared struct
        static mut USB_BUS_ALLOCATOR: Option<UsbBusAllocator<rp2040_hal::usb::UsbBus>> = None;
        let usb_bus_allocator = unsafe {
            USB_BUS_ALLOCATOR = Some(UsbBusAllocator::new(hal::usb::UsbBus::new(
                c.device.USBCTRL_REGS,
                c.device.USBCTRL_DPRAM,
                clocks.usb_clock,
                true,
                &mut resets,
            )));

            USB_BUS_ALLOCATOR.as_ref().unwrap()
        };

        // Setup the HID class driver, providing keyboard reports
        let hid_keyboard = HIDClass::new(&usb_bus_allocator, KeyboardReport::desc(), 1);

        // Setup the USB device description
        let usb_device = UsbDeviceBuilder::new(&usb_bus_allocator, UsbVidPid(0x1404, 0x1404))
            .strings(&[StringDescriptors::default()
                .manufacturer("Cloverpad")
                .product("Cloverpad MX")
                .serial_number("CLP-MX-01")])
            .unwrap()
            .build();

        let mut led = pins.gpio25.reconfigure();
        led.set_low().unwrap();

        let alarm = timer.alarm_0().unwrap();
        toggle_key::spawn_after(500.millis()).unwrap();

        (
            Shared {
                led,
                usb_device,
                hid_keyboard,
            },
            Local {
                key_report: KeyboardReport::default(),
            },
            init::Monotonics(Monotonic::new(timer, alarm)),
        )
    }

    #[task(binds = USBCTRL_IRQ, shared = [usb_device, hid_keyboard])]
    fn poll_usb(c: poll_usb::Context) {
        let poll_usb::SharedResources {
            usb_device,
            hid_keyboard,
        } = c.shared;

        (usb_device, hid_keyboard).lock(|dev, hid| {
            if dev.poll(&mut [hid]) {
                // The OS may send a report to the keypad, e.g. setting NumLock LED
                // We don't need to process this, so read + discard it
                let mut throwaway = [0; 64];
                let _ = hid.pull_raw_output(&mut throwaway);
            }
        })
    }

    #[task(shared = [led, hid_keyboard], local = [key_report, tog: bool = true])]
    fn toggle_key(mut c: toggle_key::Context) {
        if *c.local.tog {
            c.shared.led.lock(|l| l.set_high().unwrap());
            c.local.key_report.keycodes[0] = 0x04;
        } else {
            c.shared.led.lock(|l| l.set_low().unwrap());
            c.local.key_report.keycodes[0] = 0x00;
        }

        c.shared.hid_keyboard.lock(|h| {
            push_hid_report(h, c.local.key_report)
                .ok()
                .unwrap_or_default()
        });

        *c.local.tog = !*c.local.tog;

        toggle_key::spawn_after(500.millis()).unwrap();
    }

    fn push_hid_report<U: usb_device::bus::UsbBus, IR: AsInputReport>(
        hid: &mut HIDClass<U>,
        report: &IR,
    ) -> Result<usize, usb_device::UsbError> {
        hid.push_input(report)
    }
}
