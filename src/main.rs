#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use rtic_monotonics::{rp2040_timer_monotonic, Monotonic};

use embedded_hal::digital::InputPin;
use rp2040_hal::{
    clocks::init_clocks_and_plls, fugit::MillisDurationU64, gpio, timer::Instant, Sio, Watchdog,
};

use usb_device::{class_prelude::*, prelude::*};
use usbd_hid::{
    descriptor::{generator_prelude::*, KeyboardReport, KeyboardUsage},
    hid_class::HIDClass,
};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

rp2040_timer_monotonic!(Mono);

#[rtic::app(device = rp2040_hal::pac, peripherals = true)]
mod app {
    use super::*;

    const XOSC_CRYSTAL_FREQ: u32 = 12_000_000u32;
    const DEBOUNCE_DOWN: MillisDurationU64 = MillisDurationU64::millis(5);
    const DEBOUNCE_UP: MillisDurationU64 = MillisDurationU64::millis(5);

    struct KeyState {
        read_mask: u32,
        last_update: Instant,
        keycode: KeyboardUsage,
    }

    #[shared]
    struct Shared {
        key_report: KeyboardReport,
    }

    #[local]
    struct Local {
        // USB IRQ
        usb_device: UsbDevice<'static, rp2040_hal::usb::UsbBus>,
        hid_keyboard: HIDClass<'static, rp2040_hal::usb::UsbBus>,

        // "Idle" Loop
        key_states: [KeyState; 3],
    }

    #[init(local = [usb_bus: Option<UsbBusAllocator<rp2040_hal::usb::UsbBus>> = None])]
    fn init(c: init::Context) -> (Shared, Local) {
        defmt::info!("Initialising app");

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

        Mono::start(c.device.TIMER, &mut resets);

        let sio = Sio::new(c.device.SIO);
        let pins = gpio::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // Setup the USB driver with static lifetime
        *c.local.usb_bus = Some(UsbBusAllocator::new(rp2040_hal::usb::UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        )));

        let usb_bus_allocator = c.local.usb_bus.as_ref().unwrap();

        // Setup the HID class driver, providing keyboard reports
        let hid_keyboard = HIDClass::new_ep_in(usb_bus_allocator, KeyboardReport::desc(), 1);

        // Setup the USB device description
        let usb_device = UsbDeviceBuilder::new(usb_bus_allocator, UsbVidPid(0x1404, 0x1404))
            .strings(&[StringDescriptors::default()
                .manufacturer("Cloverpad")
                .product("Cloverpad MX")
                .serial_number("CLP-MX-01")])
            .unwrap()
            .build();

        // If K1 is low during boot, reset to bootloader instead
        let mut k1_pin = pins.gpio25.into_pull_up_input();
        let k2_pin = pins.gpio24.into_pull_up_input();
        let k3_pin = pins.gpio23.into_pull_up_input();

        k1_pin.set_schmitt_enabled(true);
        k2_pin.set_schmitt_enabled(true);
        k3_pin.set_schmitt_enabled(true);

        if k1_pin.is_low().unwrap() {
            rp2040_hal::rom_data::reset_to_usb_boot(0, 0);

            loop {
                cortex_m::asm::wfi();
            }
        }

        let key_states = [
            KeyState {
                read_mask: 1 << k1_pin.id().num,
                last_update: Instant::from_ticks(0),
                keycode: KeyboardUsage::KeyboardZz,
            },
            KeyState {
                read_mask: 1 << k2_pin.id().num,
                last_update: Instant::from_ticks(0),
                keycode: KeyboardUsage::KeyboardXx,
            },
            KeyState {
                read_mask: 1 << k3_pin.id().num,
                last_update: Instant::from_ticks(0),
                keycode: KeyboardUsage::KeyboardCc,
            },
        ];

        (
            Shared {
                key_report: KeyboardReport::default(),
            },
            Local {
                // USB IRQ
                usb_device,
                hid_keyboard,

                // "Idle" Loop
                key_states,
            },
        )
    }

    #[task(binds = USBCTRL_IRQ, shared = [key_report], local = [usb_device, hid_keyboard])]
    fn poll_usb(mut c: poll_usb::Context) {
        let poll_usb::LocalResources {
            usb_device,
            hid_keyboard,
            ..
        } = c.local;

        if usb_device.poll(&mut [hid_keyboard]) {
            c.shared.key_report.lock(|kr| {
                let _ = hid_keyboard.push_input(kr);
            });
        }
    }

    #[idle(shared = [key_report], local = [key_states])]
    fn idle(c: idle::Context) -> ! {
        let mut key_report = c.shared.key_report;
        let key_states = c.local.key_states;

        loop {
            let timestamp = Mono::now();
            let pin_states = rp2040_hal::Sio::read_bank0();

            for (i, key_state) in key_states.into_iter().enumerate() {
                let pressed = (pin_states & key_state.read_mask) == 0;

                if pressed && timestamp - key_state.last_update >= DEBOUNCE_UP {
                    key_report.lock(|kr| kr.keycodes[i] = key_state.keycode as u8);
                    key_state.last_update = timestamp;
                } else if !pressed && timestamp - key_state.last_update >= DEBOUNCE_DOWN {
                    key_report.lock(|kr| kr.keycodes[i] = 0x00);
                    key_state.last_update = timestamp;
                }
            }
        }
    }
}
