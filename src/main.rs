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

    use embedded_hal::digital::{InputPin, OutputPin};

    use rp2040_hal::{
        clocks::init_clocks_and_plls,
        fugit::ExtU64,
        gpio,
        timer::{monotonic::Monotonic, Alarm0},
        Sio, Watchdog,
    };

    use usb_device::{class_prelude::*, prelude::*};
    use usbd_hid::{
        descriptor::{generator_prelude::*, KeyboardReport},
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
        button: gpio::Pin<gpio::bank0::Gpio18, gpio::FunctionSioInput, gpio::PullUp>,
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
            rp2040_hal::sio::spinlock_reset();
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

        let mut timer = rp2040_hal::Timer::new(c.device.TIMER, &mut resets, &clocks);
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
            USB_BUS_ALLOCATOR = Some(UsbBusAllocator::new(rp2040_hal::usb::UsbBus::new(
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

        let button = pins.gpio18.reconfigure();

        let alarm = timer.alarm_0().unwrap();
        blinky::spawn_after(500.millis()).unwrap();

        (
            Shared {
                led,
                usb_device,
                hid_keyboard,
            },
            Local {
                button,
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

    #[idle(shared = [hid_keyboard], local = [button, key_report, pressed: bool = false])]
    fn idle(mut c: idle::Context) -> ! {
        let idle::LocalResources {
            button,
            key_report,
            pressed,
        } = c.local;

        loop {
            if button.is_high().unwrap() && !*pressed {
                *pressed = true;
                key_report.keycodes[0] = 0x04;

                c.shared
                    .hid_keyboard
                    .lock(|hid| hid.push_input(key_report).unwrap_or_default());
            } else if button.is_low().unwrap() && *pressed {
                *pressed = false;
                key_report.keycodes[0] = 0x00;

                c.shared
                    .hid_keyboard
                    .lock(|hid| hid.push_input(key_report).unwrap_or_default());
            }
        }
    }

    #[task(shared = [led], local = [tog: bool = true])]
    fn blinky(mut c: blinky::Context) {
        if *c.local.tog {
            c.shared.led.lock(|l| l.set_high().unwrap());
        } else {
            c.shared.led.lock(|l| l.set_low().unwrap());
        }

        *c.local.tog = !*c.local.tog;

        blinky::spawn_after(500.millis()).unwrap();
    }
}
