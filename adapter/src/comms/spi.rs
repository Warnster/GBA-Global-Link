use embedded_hal::digital::{InputPin, OutputPin};
use rp2040_hal::gpio::{
    bank0 as peris, FunctionNull, FunctionSio, Pin, PullDown, SioInput, SioOutput,
};

type DefaultPin<P> = Pin<P, FunctionNull, PullDown>;
type Input<P> = Pin<P, FunctionSio<SioInput>, PullDown>;
type Output<P> = Pin<P, FunctionSio<SioOutput>, PullDown>;

/// Spin-loop iteration cap before we assume the GBA's clock has stopped (powered off /
/// link dropped). Without it a stalled clock hangs core 1 forever and needs a physical
/// power-cycle. Generous: well above the ~16 ms between normal per-VBlank transactions,
/// and the GBA drops the link itself after ~3 s. Iterations->time depends on the built
use core::sync::atomic::{AtomicU32, Ordering};

/// SPI liveness heartbeat: bumped once per completed 32-bit transfer (on core 1). Core 0
/// feeds the hardware watchdog only while this keeps advancing, so a `transfer_bit` that
/// spins forever on a frozen clock (link dropped / room comm error) stops the heartbeat and
/// lets the watchdog reboot + re-login the chip. This replaces the old per-bit CLK_TIMEOUT
/// counter, which broke login timing (it delayed the sample and shifted every read left one
/// bit: 0x494E -> 0x929C). The recovery now costs the bit-sampling loop ZERO instructions.
pub static SPI_TX_COUNT: AtomicU32 = AtomicU32::new(0);

pub struct Spi {
    p_clk: Input<peris::Gpio2>,
    p_tx: Output<peris::Gpio3>,
    p_rx: Input<peris::Gpio4>,
    p_reset: Input<peris::Gpio5>,
    reset_requested: bool,
}

impl Spi {
    pub fn new(
        p_clk: DefaultPin<peris::Gpio2>,
        p_tx: DefaultPin<peris::Gpio3>,
        p_rx: DefaultPin<peris::Gpio4>,
        p_reset: DefaultPin<peris::Gpio5>,
    ) -> Self {
        Self {
            p_clk: p_clk.into_function(),
            p_tx: p_tx.into_function(),
            p_rx: p_rx.into_function(),
            p_reset: p_reset.into_function(),
            reset_requested: false,
        }
    }

    pub fn reset(&mut self) {
        self.reset_requested = false;
    }

    /// DIAGNOSTIC: monitor the GBA link wires and report over USB. Each window it sends a
    /// 4-word packet [0xD1A6D1A6, clk_rising_edges, rx_high_samples, reset_high_samples].
    /// If the GBA is talking and SC->GP2 is wired, clk edges climb; rx = SO(GP4) activity;
    /// reset = SD(GP5). All zeros while the GBA clocks = clock not reaching GP2 (mis-wire).
    pub fn diagnose(&mut self) -> ! {
        loop {
            let mut edges: u32 = 0;
            let mut rx_high: u32 = 0;
            let mut rst_high: u32 = 0;
            let mut last = self.p_clk.is_high().unwrap_or_default();
            for _ in 0..1_500_000u32 {
                let c = self.p_clk.is_high().unwrap_or_default();
                if c && !last {
                    edges += 1;
                }
                last = c;
                if self.p_rx.is_high().unwrap_or_default() {
                    rx_high += 1;
                }
                if self.p_reset.is_high().unwrap_or_default() {
                    rst_high += 1;
                }
            }
            crate::serial_usb::send_only32(&[0xD1A6_D1A6, edges, rx_high, rst_high]);
        }
    }

    pub fn request_reset(&mut self) {
        self.reset_requested = true;
    }

    /// Current SPI liveness heartbeat (completed 32-bit transfers). Read by core 0's
    /// watchdog feeder to detect a hung/frozen clock.
    pub fn tx_count() -> u32 {
        SPI_TX_COUNT.load(Ordering::Relaxed)
    }

    #[inline(never)]
    pub fn reset_requested(&self) -> bool {
        self.reset_requested
    }

    #[inline]
    pub fn p_tx_set_high(&mut self) {
        let _ = self.p_tx.set_high();
    }

    #[inline]
    pub fn p_tx_set_low(&mut self) {
        let _ = self.p_tx.set_low();
    }

    #[inline]
    pub fn p_rx_is_low(&self) -> bool {
        self.p_rx.is_low().unwrap_or_default()
    }

    // BISECT STEP 1: byte-identical to proven-working upstream (pre-b06099e). Both edge
    // waits are fully TIGHT with no timeout bookkeeping, and transfer_u32 has no inter-bit
    // break. This is the exact code that captured detection on 2026-09-06. If login now
    // reads 0x494E, the b06099e timeout instructions were the cause (the earlier partial
    // revert only touched the second loop and left the first). If it STILL reads 0x929C,
    // the source is exonerated and the 1.76.0 toolchain pin is the cause.
    //
    // NOTE: this temporarily removes link-drop self-recovery (core1 will hang on a frozen
    // clock again). Re-add it AFTER timing is confirmed good — at the transaction level
    // (e.g. a coarse hardware-timer guard around transfer_u32), NOT per-bit in the hot path.
    fn transfer_bit(&mut self, bit: u8) -> u8 {
        while self.p_clk.is_high().unwrap_or_default() {
            if self.p_reset.is_high().unwrap_or_default() {
                self.reset_requested = true;
            }
        }
        if bit == 0 {
            let _ = self.p_tx.set_low();
        } else {
            let _ = self.p_tx.set_high();
        }
        while self.p_clk.is_low().unwrap_or_default() {}
        self.p_rx.is_high().unwrap_or_default() as u8
    }

    #[inline(never)]
    pub fn transfer_u32(&mut self, data: u32) -> u32 {
        let bits = [
            ((data >> 31) & 1) as u8,
            ((data >> 30) & 1) as u8,
            ((data >> 29) & 1) as u8,
            ((data >> 28) & 1) as u8,
            ((data >> 27) & 1) as u8,
            ((data >> 26) & 1) as u8,
            ((data >> 25) & 1) as u8,
            ((data >> 24) & 1) as u8,
            ((data >> 23) & 1) as u8,
            ((data >> 22) & 1) as u8,
            ((data >> 21) & 1) as u8,
            ((data >> 20) & 1) as u8,
            ((data >> 19) & 1) as u8,
            ((data >> 18) & 1) as u8,
            ((data >> 17) & 1) as u8,
            ((data >> 16) & 1) as u8,
            ((data >> 15) & 1) as u8,
            ((data >> 14) & 1) as u8,
            ((data >> 13) & 1) as u8,
            ((data >> 12) & 1) as u8,
            ((data >> 11) & 1) as u8,
            ((data >> 10) & 1) as u8,
            ((data >> 9) & 1) as u8,
            ((data >> 8) & 1) as u8,
            ((data >> 7) & 1) as u8,
            ((data >> 6) & 1) as u8,
            ((data >> 5) & 1) as u8,
            ((data >> 4) & 1) as u8,
            ((data >> 3) & 1) as u8,
            ((data >> 2) & 1) as u8,
            ((data >> 1) & 1) as u8,
            (data & 1) as u8,
        ];
        let mut rx = 0u32;

        for bit in bits {
            rx <<= 1;
            rx |= self.transfer_bit(bit) as u32;
        }

        // Liveness heartbeat for the watchdog feeder on core 0. One store per 32-bit word,
        // OUTSIDE the bit loop — does not touch the timing-critical sample path. Cortex-M0+
        // has no atomic RMW (fetch_add), but core 1 is the sole writer, so a plain
        // load/add/store is race-free; core 0 only reads it.
        SPI_TX_COUNT.store(
            SPI_TX_COUNT.load(Ordering::Relaxed).wrapping_add(1),
            Ordering::Relaxed,
        );

        rx
    }
}
