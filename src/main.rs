#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[rtic::app(device = rp2040_hal::pac, peripherals = true)]
mod app {
    use defmt::*;
    use defmt_rtt as _;
    use panic_probe as _;

    use embedded_hal::digital::InputPin;
    use rp2040_hal::{
        clocks::init_clocks_and_plls,
        fugit::MillisDurationU64,
        gpio,
        timer::{monotonic::Monotonic, Alarm0, Instant},
        Sio, Watchdog,
    };

    use usb_device::{class_prelude::*, prelude::*};
    use usbd_hid::{
        descriptor::{generator_prelude::*, KeyboardReport, KeyboardUsage},
        hid_class::HIDClass,
    };

    const XOSC_CRYSTAL_FREQ: u32 = 12_000_000u32;
    const DEBOUNCE_DOWN: MillisDurationU64 = MillisDurationU64::millis(5);
    const DEBOUNCE_UP: MillisDurationU64 = MillisDurationU64::millis(5);

    struct KeyState {
        pin: gpio::Pin<gpio::DynPinId, gpio::FunctionSioInput, gpio::PullUp>,
        last_update: Instant,
        keycode: KeyboardUsage,
    }

    #[shared]
    struct Shared {
        hid_keyboard: HIDClass<'static, rp2040_hal::usb::UsbBus>,
    }

    #[local]
    struct Local {
        // USB IRQ
        usb_device: UsbDevice<'static, rp2040_hal::usb::UsbBus>,
        hid_discard_buf: [u8; 64],

        // "Idle" Loop
        key_report: KeyboardReport,
        key_states: [KeyState; 3],
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
        let alarm = timer.alarm_0().unwrap();

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

        // If K1 is low during boot, reset to bootloader instead
        let mut k1_pin = pins.gpio25.reconfigure();
        if k1_pin.is_low().unwrap() {
            rp2040_hal::rom_data::reset_to_usb_boot(0, 0);

            loop {
                cortex_m::asm::wfi();
            }
        }

        let key_states = [
            KeyState {
                pin: k1_pin.into_dyn_pin(),
                last_update: Instant::from_ticks(0),
                keycode: KeyboardUsage::KeyboardZz,
            },
            KeyState {
                pin: pins.gpio24.reconfigure().into_dyn_pin(),
                last_update: Instant::from_ticks(0),
                keycode: KeyboardUsage::KeyboardXx,
            },
            KeyState {
                pin: pins.gpio23.reconfigure().into_dyn_pin(),
                last_update: Instant::from_ticks(0),
                keycode: KeyboardUsage::KeyboardCc,
            },
        ];

        (
            Shared { hid_keyboard },
            Local {
                // USB IRQ
                usb_device,
                hid_discard_buf: [0; 64],

                // "Idle" Loop
                key_report: KeyboardReport::default(),
                key_states,
            },
            init::Monotonics(Monotonic::new(timer, alarm)),
        )
    }

    #[task(binds = USBCTRL_IRQ, shared = [hid_keyboard], local = [usb_device, hid_discard_buf])]
    fn poll_usb(mut c: poll_usb::Context) {
        let poll_usb::LocalResources {
            usb_device,
            hid_discard_buf,
        } = c.local;

        c.shared.hid_keyboard.lock(|hid| {
            if usb_device.poll(&mut [hid]) {
                // The OS may send a report to the keypad, e.g. setting NumLock LED
                // We don't need to process this, so read + discard it
                let _ = hid.pull_raw_output(hid_discard_buf);
            }
        })
    }

    #[idle(shared = [hid_keyboard], local = [key_report, key_states])]
    fn idle(mut c: idle::Context) -> ! {
        let idle::LocalResources {
            key_report,
            key_states,
        } = c.local;

        loop {
            for (i, key_state) in key_states.into_iter().enumerate() {
                let timestamp = monotonics::now();

                if key_state.pin.is_low().unwrap()
                    && timestamp - key_state.last_update >= DEBOUNCE_UP
                {
                    key_report.keycodes[i] = key_state.keycode as u8;
                } else if key_state.pin.is_high().unwrap()
                    && timestamp - key_state.last_update >= DEBOUNCE_DOWN
                {
                    key_report.keycodes[i] = 0x00;
                }

                c.shared
                    .hid_keyboard
                    .lock(|hid| hid.push_input(key_report).unwrap_or_default());
            }
        }
    }
}
