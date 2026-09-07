//! UART link to the ESP32 (Switch-side). Mirrors `serial_usb`'s cross-core static
//! pattern so core1 (the Router) can forward RFU slots to the ESP32 alongside USB.
//! Writes are non-blocking / best-effort (write what fits the TX FIFO, drop the rest)
//! so forwarding never stalls the GBA link's ~800us inter-transfer deadline. Slots are
//! <=16 bytes at 60 Hz and the 32-byte FIFO drains at 115200, so nothing is lost in practice.
use core::cell::RefCell;
use critical_section::Mutex;
use once_cell::sync::OnceCell;
use rp_pico::hal::gpio::bank0::{Gpio0, Gpio1};
use rp_pico::hal::gpio::{FunctionUart, Pin, PullDown};
use rp_pico::hal::pac::UART0;
use rp_pico::hal::uart::{Enabled, UartPeripheral};
use safe_transmute::transmute_to_bytes;

pub type EspUart = UartPeripheral<
    Enabled,
    UART0,
    (
        Pin<Gpio0, FunctionUart, PullDown>,
        Pin<Gpio1, FunctionUart, PullDown>,
    ),
>;

static UART: OnceCell<Mutex<RefCell<EspUart>>> = OnceCell::new();

pub fn init(uart: EspUart) {
    let _ = UART.set(Mutex::new(RefCell::new(uart)));
}

/// Fire-and-forget best-effort write to the ESP32. Non-blocking.
pub fn send(data: &[u8]) {
    critical_section::with(|cs| {
        if let Some(u) = UART.get() {
            let _ = u.borrow_ref_mut(cs).write_raw(data);
        }
    });
}

/// Forward u32 LE words (an RFU slot as `req.raw()`) to the ESP32.
pub fn send32(data: &[u32]) {
    send(transmute_to_bytes(data));
}
