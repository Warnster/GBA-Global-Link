//! # Pico USB Serial Example
//!
//! Creates a USB Serial device on a Pico board, with the USB driver running in
//! the main thread.
//!
//! This will create a USB Serial device echoing anything it receives. Incoming
//! ASCII characters are converted to upercase, so you can tell it is working
//! and not just local-echo!
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

mod serial_usb;
mod uart_link;
mod relay;

mod comms;
use comms::{Router, Spi};

// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

use rp_pico::hal::multicore::{Multicore, Stack};

// USB Device support
use usb_device::class_prelude::*;

// UART to the ESP32 (Switch-side) — heartbeat / relay link on GP0/GP1.
use core::fmt::Write as _;
use cortex_m::prelude::{
    _embedded_hal_watchdog_Watchdog as _, _embedded_hal_watchdog_WatchdogEnable as _,
};
use fugit::{ExtU32, RateExtU32};
use rp_pico::hal::uart::{DataBits, StopBits, UartConfig, UartPeripheral};
use rp_pico::hal::Clock as _;

/// Tiny no_std buffer for building a heartbeat line without alloc.
struct HbBuf {
    buf: [u8; 40],
    len: usize,
}
impl core::fmt::Write for HbBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            if self.len < self.buf.len() {
                self.buf[self.len] = b;
                self.len += 1;
            }
        }
        Ok(())
    }
}

static mut CORE1_STACK: Stack<4096> = Stack::new();

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then echoes any characters
/// received over USB Serial.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Split out the pins we need before core1 takes the GBA-link ones (GP2-5).
    let uart_tx = pins.gpio0.into_function::<hal::gpio::FunctionUart>();
    let uart_rx = pins.gpio1.into_function::<hal::gpio::FunctionUart>();
    let (g2, g3, g4, g5) = (pins.gpio2, pins.gpio3, pins.gpio4, pins.gpio5);

    // UART0 to the ESP32: GP0 = TX, GP1 = RX, 115200 8N1. Handed to uart_link so core1
    // (the Router) can forward RFU slots to the ESP32 alongside USB.
    let uart = UartPeripheral::new(pac.UART0, (uart_tx, uart_rx), &mut pac.RESETS)
        .enable(
            UartConfig::new(115_200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    uart_link::init(uart);

    // Timer for the 1 Hz heartbeat.
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    serial_usb::init(usb_bus);

    let _ = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        let spi = Spi::new(g2, g3, g4, g5);
        Router::new(spi).run();
    });

    // Announce once, then heartbeat every ~1s over the UART (liveness even with no GBA
    // attached; the real RFU slots are forwarded from core1 via uart_link::send32).
    uart_link::send(b"GBA-FW up\r\n");
    let mut last = timer.get_counter().ticks();
    let mut n: u32 = 0;
    let mut last_dump = last;

    // Link-drop recovery via hardware watchdog (replaces the per-bit CLK_TIMEOUT that broke
    // login timing). We feed it below ONLY while core 1's SPI heartbeat keeps advancing, so
    // a hung transfer_bit (frozen clock: link dropped / room comm error) stops the feed and
    // the chip reboots + re-logins. During a live session (a transfer every ~16ms) it is fed
    // continuously, well inside the window, so no spurious resets.
    //
    // The tolerated no-clock gap is FEED_WINDOW_US; a real session clocks every ~16ms, but
    // the Union Room has brief legitimate pauses (menu transitions, host<->search cycling)
    // that must NOT reboot us and drop a live link. 1.2s was too aggressive and dropped an
    // in-room connection, so this is ~5s: forgiving of pauses, still recovers a truly dead
    // link before a human would (and the GBA itself only gives up after ~3s).
    watchdog.pause_on_debug(false);
    watchdog.start(1_500.millis());
    let mut spi_hb = comms::Spi::tx_count();
    let mut spi_hb_time = last;
    // Only gate the watchdog on the SPI heartbeat AFTER we've seen the first transfer. Before
    // that (GBA idle / console off / not yet in wireless) we feed unconditionally, so an idle
    // link doesn't reboot-loop the Pico — it just waits. A drop mid-session still recovers:
    // once armed, a frozen clock stops the heartbeat and the watchdog fires; the reboot
    // returns here disarmed, so it waits quietly again until the GBA next clocks.
    let mut wd_armed = false;

    loop {
        serial_usb::poll();
        // Software BOOTSEL: host sends 'B' over USB serial to reflash without the button.
        serial_usb::check_bootsel();
        // Pull the ESP32->Pico relay stream (peer presence / received slots from the Switch).
        relay::poll();
        let now = timer.get_counter().ticks();

        // Feed the watchdog until the first SPI activity; after that, only while the heartbeat
        // keeps advancing. A hung/frozen clock mid-session stops it and lets the watchdog fire.
        let hb = comms::Spi::tx_count();
        if hb != spi_hb {
            spi_hb = hb;
            spi_hb_time = now;
            wd_armed = true;
        }
        const FEED_WINDOW_US: u64 = 4_000_000; // tolerate up to ~4s of no clock (+~1.5s wd)
        if !wd_armed || now.wrapping_sub(spi_hb_time) < FEED_WINDOW_US {
            watchdog.feed();
        }
        // DIAGNOSTIC: dump the login handshake rx log over USB every ~300ms
        if now.wrapping_sub(last_dump) >= 300_000 {
            last_dump = now;
            let mut pkt = [0u32; 10];
            pkt[0] = 0x1061_1061; // login-log magic
            unsafe {
                pkt[1] = comms::login::LOGIN_N;
                for k in 0..8 {
                    pkt[2 + k] = comms::login::LOGIN_RX[k];
                }
            }
            serial_usb::send_only32(&pkt);
        }
        if now.wrapping_sub(last) >= 1_000_000 {
            last = now;
            let mut hb = HbBuf { buf: [0; 40], len: 0 };
            let _ = write!(hb, "GBA-FW hb #{}\r\n", n);
            uart_link::send(&hb.buf[..hb.len]);
            n = n.wrapping_add(1);
        }
    }
}
