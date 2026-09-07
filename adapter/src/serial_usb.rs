use core::cell::RefCell;
use critical_section::Mutex;
use once_cell::sync::OnceCell;
use rp_pico::hal::usb;
use safe_transmute::{transmute_to_bytes, transmute_to_bytes_mut};
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
            .device_class(2) // from: https://www.usb.org/defined-class-codes
            .build();

        Self {
            serial,
            wusb,
            usb_dev,
        }
    }

    fn poll(&mut self) -> bool {
        self.usb_dev.poll(&mut [&mut self.serial, &mut self.wusb])
    }

    /// Waits until data is received, unless a recv buf of length 0 is given.
    fn transfer(&mut self, send: &[u8], recv: &mut [u8]) -> usize {
        let _ = self.serial.write(send);

        // A timeout should probably be added at some point
        while !self.poll() {}

        if recv.len() == 0 {
            return 0;
        }

        loop {
            let count = self.serial.read(recv).unwrap_or_default();
            if count != 0 {
                return count;
            }
        }
    }

    /// Queue bytes to the USB TX buffer and return immediately - no poll wait, no
    /// reply read. Core 0's `poll()` loop flushes the buffer. Used for fire-and-forget
    /// forwards (the child's outgoing 0x25 slot) that must not block the GBA's ~800us
    /// inter-transfer deadline. Best-effort single write; at 60 Hz with <=16-byte slots
    /// the TX buffer (drained continuously by core 0) does not overflow in practice.
    fn send_only(&mut self, data: &[u8]) {
        let _ = self.serial.write(data);
    }
}

static USB_BUS: StaticCell<UsbBusAllocator<usb::UsbBus>> = StaticCell::new();
static SERIAL_USB: OnceCell<Mutex<RefCell<SerialUsb<'static>>>> = OnceCell::new();

pub fn init(usb_bus: UsbBusAllocator<usb::UsbBus>) {
    let usb_bus = USB_BUS.init(usb_bus);
    let _ = SERIAL_USB.set(Mutex::new(RefCell::new(SerialUsb::new(usb_bus))));
}

pub fn poll() {
    critical_section::with(|cs| {
        SERIAL_USB.get().unwrap().borrow_ref_mut(cs).poll();
    })
}

/// Waits until data is received, unless a recv buf of length 0 is given.
pub fn transfer(send: &[u8], recv: &mut [u8]) -> usize {
    critical_section::with(|cs| {
        SERIAL_USB
            .get()
            .unwrap()
            .borrow_ref_mut(cs)
            .transfer(send, recv)
    })
}

/// Waits until data is received, unless a recv buf of length 0 is given.
pub fn transfer32(send: &[u32], recv: &mut [u32]) -> usize {
    let send = transmute_to_bytes(send);
    let recv = transmute_to_bytes_mut(recv);
    let bytes_received = transfer(send, recv);
    bytes_received / 4
}

/// Fire-and-forget: queue `data` (u32 LE words) to the host without waiting for a reply.
/// The Pico does NOT read a response for these, so the host must NOT send one.
pub fn send_only32(data: &[u32]) {
    critical_section::with(|cs| {
        let bytes = transmute_to_bytes(data);
        SERIAL_USB
            .get()
            .unwrap()
            .borrow_ref_mut(cs)
            .send_only(bytes);
    });
}
