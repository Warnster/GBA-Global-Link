//! USB serial to the host, redesigned to be **core0-exclusive and critical-section-free**.
//!
//! The RP2040 `critical_section` is a global (spinlock + IRQ disable): if core0 held it for USB
//! work, core1 spun and missed the GBA's ~800us per-command SPI deadline (GBA link crash when a
//! host actively read the port). Fix: only core0 ever touches the USB device (no Mutex), and
//! core1 hands it bytes through a lock-free SPSC ring. Core0 therefore takes NO critical-section
//! for USB, so it can never stall core1's SPI — a host can read/write the port freely.
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicUsize, Ordering};
use rp_pico::hal::usb;
use safe_transmute::transmute_to_bytes;
use static_cell::StaticCell;
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::SerialPort;
use usbd_webusb::{url_scheme, WebUsb};

struct SerialUsb<'d> {
    wusb: WebUsb<usb::UsbBus>,
    serial: SerialPort<'d, usb::UsbBus>,
    usb_dev: UsbDevice<'d, usb::UsbBus>,
}

impl<'d> SerialUsb<'d> {
    fn new(usb_bus: &'static UsbBusAllocator<usb::UsbBus>) -> Self {
        let serial = SerialPort::new(usb_bus);
        let wusb = WebUsb::new(usb_bus, url_scheme::HTTP, "localhost:8000");
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1234, 0x2000))
            .product("GBA Passthrough")
            .serial_number("GBA")
            .device_class(2)
            .build();
        Self { serial, wusb, usb_dev }
    }
}

static USB_BUS: StaticCell<UsbBusAllocator<usb::UsbBus>> = StaticCell::new();

// core0-only USB device. Only core0 touches it after init() (which runs on core0 before core1
// is spawned), so no lock is needed. UnsafeCell + `unsafe impl Sync` to hold it in a static.
struct UsbCell(UnsafeCell<Option<SerialUsb<'static>>>);
unsafe impl Sync for UsbCell {}
static SERIAL_USB: UsbCell = UsbCell(UnsafeCell::new(None));

#[inline]
fn usb() -> Option<&'static mut SerialUsb<'static>> {
    // SAFETY: only ever called on core0 (poll/check_bootsel), sequentially; no aliasing.
    unsafe { (*SERIAL_USB.0.get()).as_mut() }
}

// Lock-free SPSC ring for core1 -> core0 USB TX (the RFU command mirror). core1 is the sole
// producer (routes.rs + spi.rs, both on core1); core0 is the sole consumer. Atomics only
// (M0+ has no atomic RMW, and SPSC needs none) => core1 never takes a critical-section for USB.
const TXR: usize = 2048;
struct TxRing {
    buf: UnsafeCell<[u8; TXR]>,
}
unsafe impl Sync for TxRing {}
static TXRING: TxRing = TxRing {
    buf: UnsafeCell::new([0; TXR]),
};
static TX_HEAD: AtomicUsize = AtomicUsize::new(0);
static TX_TAIL: AtomicUsize = AtomicUsize::new(0);

fn tx_push(b: u8) {
    let h = TX_HEAD.load(Ordering::Relaxed);
    let n = (h + 1) % TXR;
    if n == TX_TAIL.load(Ordering::Acquire) {
        return; // full -> drop (best-effort mirror; never blocks the GBA)
    }
    unsafe { (*TXRING.buf.get())[h] = b };
    TX_HEAD.store(n, Ordering::Release);
}

fn tx_pop() -> Option<u8> {
    let t = TX_TAIL.load(Ordering::Relaxed);
    if t == TX_HEAD.load(Ordering::Acquire) {
        return None;
    }
    let b = unsafe { (*TXRING.buf.get())[t] };
    TX_TAIL.store((t + 1) % TXR, Ordering::Release);
    Some(b)
}

pub fn init(usb_bus: UsbBusAllocator<usb::UsbBus>) {
    let usb_bus = USB_BUS.init(usb_bus);
    // core0, before core1 spawn -> exclusive.
    unsafe { *SERIAL_USB.0.get() = Some(SerialUsb::new(usb_bus)) };
}

/// core0: drain the core1->core0 TX ring into the USB TX buffer, then poll the device (flush +
/// service the host). No critical-section anywhere.
pub fn poll() {
    if let Some(u) = usb() {
        let mut chunk = [0u8; 64];
        loop {
            let mut m = 0;
            while m < chunk.len() {
                match tx_pop() {
                    Some(b) => {
                        chunk[m] = b;
                        m += 1;
                    }
                    None => break,
                }
            }
            if m == 0 {
                break;
            }
            let _ = u.serial.write(&chunk[..m]);
        }
        let _ = u.usb_dev.poll(&mut [&mut u.serial, &mut u.wusb]);
    }
}

/// core0: read host->Pico bytes, feed them into the relay ring (parsed on core1), and watch for
/// the software-BOOTSEL magic HERE on core0 so a reflash works even when the GBA is idle (core1's
/// Router blocks on the GBA clock, so a core1-parsed bootsel would never fire while idle). The
/// magic is the exact 5-byte T_BOOTSEL frame `AA 55 7F 00 7F` — distinctive enough not to appear
/// in normal relay traffic.
static BOOTSEL_MAGIC: [u8; 5] = [0xAA, 0x55, 0x7F, 0x00, 0x7F];
static mut BOOTSEL_MATCH: usize = 0;

pub fn check_bootsel() {
    if let Some(u) = usb() {
        let mut buf = [0u8; 64];
        let n = u.serial.read(&mut buf).unwrap_or_default();
        for &b in &buf[..n] {
            // core0 rolling match of the BOOTSEL magic (GBA-state independent).
            unsafe {
                if b == BOOTSEL_MAGIC[BOOTSEL_MATCH] {
                    BOOTSEL_MATCH += 1;
                    if BOOTSEL_MATCH == BOOTSEL_MAGIC.len() {
                        // Reboot to the USB bootloader; disable the watchdog first.
                        (&*rp_pico::hal::pac::WATCHDOG::ptr())
                            .ctrl
                            .modify(|_, w| w.enable().clear_bit());
                        rp_pico::hal::rom_data::reset_to_usb_boot(0, 0);
                    }
                } else {
                    BOOTSEL_MATCH = if b == BOOTSEL_MAGIC[0] { 1 } else { 0 };
                }
            }
            crate::relay::usb_push(b);
        }
    }
}

/// core1: queue u32 LE words to the host via the lock-free ring (NO critical-section). core0's
/// poll() flushes them. Best-effort; must never block the GBA link.
pub fn send_only32(data: &[u32]) {
    for &b in transmute_to_bytes(data) {
        tx_push(b);
    }
}
