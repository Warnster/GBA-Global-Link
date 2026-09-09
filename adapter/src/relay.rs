//! ESP32 -> Pico relay: parse the framed stream the ESP32 (Switch side) sends over UART and
//! expose it to the Router so the GBA sees Switch-driven peers/data in the Union Room.
//!
//! Frame: `AA 55 | type | len | payload[len] | xor_csum`, xor_csum = XOR(type, len, payload...).
//!   type 0x00 NO_PEER   len 0            -> clear the peer (nobody here)
//!   type 0x01 PEER      len 28           -> peer_id(u32 LE) + 6-word FRLG beacon (u32 LE) = a
//!                                           discoverable partner; served in BroadcastReadPoll
//!   type 0x02 RECV_SLOT len 4..=56       -> a received RFU/trade slot (trade phase; buffered)
//!
//! core0 calls `poll()` each loop iteration (reads UART, feeds the parser, updates state).
//! core1's Router calls `get_peer()` / `take_slot()` to answer GBA commands from that state.
use critical_section::Mutex;
use core::cell::{RefCell, UnsafeCell};
use core::sync::atomic::{AtomicUsize, Ordering};

pub const T_NO_PEER: u8 = 0x00;
pub const T_PEER: u8 = 0x01;
pub const T_RECV_SLOT: u8 = 0x02;
pub const T_BOOTSEL: u8 = 0x7f; // reboot to USB bootloader (reflash without the button)

const PBUF: usize = 60;

// Lock-free SPSC ring: core0 (USB reader) is the producer, core1 (Router/relay::poll) is the
// consumer. Using the inter-core atomics (load/store only; M0+ has no atomic RMW, and we need
// none for SPSC) means core0 NEVER takes a critical-section for the relay, so it can't starve
// core1's tight per-GBA-command SPI deadline (the documented core0-CS hazard). USB bytes are
// buffered here and parsed on core1, exactly where the ESP-UART relay is already parsed safely.
const RING_SZ: usize = 512;
struct Ring {
    buf: UnsafeCell<[u8; RING_SZ]>,
}
unsafe impl Sync for Ring {}
static RING: Ring = Ring {
    buf: UnsafeCell::new([0; RING_SZ]),
};
static HEAD: AtomicUsize = AtomicUsize::new(0);
static TAIL: AtomicUsize = AtomicUsize::new(0);

/// core0: push a USB byte into the ring for core1 to parse. Lock-free, non-blocking (drops the
/// byte if the ring is full). Takes NO critical-section, so it never contends with core1's SPI.
pub fn usb_push(b: u8) {
    let head = HEAD.load(Ordering::Relaxed);
    let next = (head + 1) % RING_SZ;
    if next == TAIL.load(Ordering::Acquire) {
        return; // full -> drop (reliable layer / retransmit recovers)
    }
    unsafe { (*RING.buf.get())[head] = b };
    HEAD.store(next, Ordering::Release);
}

/// core1: pop a byte the producer left, or None. SPSC-safe with usb_push.
fn ring_pop() -> Option<u8> {
    let tail = TAIL.load(Ordering::Relaxed);
    if tail == HEAD.load(Ordering::Acquire) {
        return None;
    }
    let b = unsafe { (*RING.buf.get())[tail] };
    TAIL.store((tail + 1) % RING_SZ, Ordering::Release);
    Some(b)
}

struct Relay {
    // frame parser (core0-only, but kept here so one lock covers everything)
    st: u8,       // 0=AA 1=55 2=type 3=len 4=payload 5=csum
    ptype: u8,
    plen: u8,
    pidx: u8,
    pbuf: [u8; PBUF],
    csum: u8,
    // decoded state (core1 reads)
    peer_present: bool,
    peer_id: u32,
    beacon: [u32; 6],
    // last received slot (trade phase); simple single-slot latch for now
    slot: [u32; 14],
    slot_len: u8,
    slot_new: bool,
}

impl Relay {
    const fn new() -> Self {
        Self {
            st: 0,
            ptype: 0,
            plen: 0,
            pidx: 0,
            pbuf: [0; PBUF],
            csum: 0,
            peer_present: false,
            peer_id: 0,
            beacon: [0; 6],
            slot: [0; 14],
            slot_len: 0,
            slot_new: false,
        }
    }

    fn byte(&mut self, b: u8) {
        match self.st {
            0 => self.st = if b == 0xAA { 1 } else { 0 },
            1 => self.st = if b == 0x55 { 2 } else if b == 0xAA { 1 } else { 0 },
            2 => {
                self.ptype = b;
                self.csum = b;
                self.st = 3;
            }
            3 => {
                self.plen = if (b as usize) <= PBUF { b } else { 0 };
                self.csum ^= b;
                self.pidx = 0;
                self.st = if self.plen == 0 { 5 } else { 4 };
            }
            4 => {
                self.pbuf[self.pidx as usize] = b;
                self.csum ^= b;
                self.pidx += 1;
                if self.pidx >= self.plen {
                    self.st = 5;
                }
            }
            5 => {
                if self.csum == b {
                    self.commit();
                }
                self.st = 0;
            }
            _ => self.st = 0,
        }
    }

    fn commit(&mut self) {
        match self.ptype {
            T_NO_PEER => self.peer_present = false,
            T_PEER if self.plen == 28 => {
                let w = |i: usize| {
                    u32::from_le_bytes([
                        self.pbuf[i],
                        self.pbuf[i + 1],
                        self.pbuf[i + 2],
                        self.pbuf[i + 3],
                    ])
                };
                self.peer_id = w(0);
                for k in 0..6 {
                    self.beacon[k] = w(4 + k * 4);
                }
                self.peer_present = true;
            }
            T_RECV_SLOT => {
                let n = (self.plen as usize / 4).min(14);
                for k in 0..n {
                    self.slot[k] = u32::from_le_bytes([
                        self.pbuf[k * 4],
                        self.pbuf[k * 4 + 1],
                        self.pbuf[k * 4 + 2],
                        self.pbuf[k * 4 + 3],
                    ]);
                }
                self.slot_len = n as u8;
                self.slot_new = true;
            }
            T_BOOTSEL => {
                // Reflash without the physical button: reboot into the USB mass-storage
                // bootloader. Disable the watchdog first (its timer would reset us back out
                // mid-flash). reset_to_usb_boot never returns.
                unsafe { &*rp_pico::hal::pac::WATCHDOG::ptr() }
                    .ctrl
                    .modify(|_, w| w.enable().clear_bit());
                rp_pico::hal::rom_data::reset_to_usb_boot(0, 0);
            }
            _ => {}
        }
    }
}

static RELAY: Mutex<RefCell<Relay>> = Mutex::new(RefCell::new(Relay::new()));

/// core0: pull any bytes the ESP32 sent and run them through the frame parser.
pub fn poll() {
    // Sources: the ESP UART (uart_link, may be disconnected) and the USB ring (PC-in-the-middle,
    // filled lock-free by core0's usb_push). Both parse on THIS core (core1), between GBA
    // commands where there is slack — never on core0.
    let mut buf = [0u8; 64];
    let n = crate::uart_link::recv(&mut buf);
    let mut ring = [0u8; 128]; // bounded drain per poll so the CS is never held too long
    let mut m = 0;
    while m < ring.len() {
        match ring_pop() {
            Some(b) => {
                ring[m] = b;
                m += 1;
            }
            None => break,
        }
    }
    if n == 0 && m == 0 {
        return;
    }
    critical_section::with(|cs| {
        let mut r = RELAY.borrow_ref_mut(cs);
        for &b in &buf[..n] {
            r.byte(b);
        }
        for &b in &ring[..m] {
            r.byte(b);
        }
    });
}

/// core1 (Router): if a partner is present, write [peer_id, beacon(6)] into `out` and return
/// the word count (7); otherwise 0 (= no peers).
pub fn get_peer(out: &mut [u32]) -> usize {
    critical_section::with(|cs| {
        let r = RELAY.borrow_ref(cs);
        if r.peer_present && out.len() >= 7 {
            out[0] = r.peer_id;
            out[1..7].copy_from_slice(&r.beacon);
            7
        } else {
            0
        }
    })
}

/// core1 (Router): return the peer_id if a partner is present (for Connect/IsConnecting).
pub fn peer_id() -> Option<u32> {
    critical_section::with(|cs| {
        let r = RELAY.borrow_ref(cs);
        if r.peer_present {
            Some(r.peer_id)
        } else {
            None
        }
    })
}

/// core1 (Router): take the latest received trade slot if a new one arrived. Returns word count.
pub fn take_slot(out: &mut [u32]) -> usize {
    critical_section::with(|cs| {
        let mut r = RELAY.borrow_ref_mut(cs);
        if !r.slot_new {
            return 0;
        }
        r.slot_new = false;
        let n = (r.slot_len as usize).min(out.len());
        out[..n].copy_from_slice(&r.slot[..n]);
        n
    })
}
