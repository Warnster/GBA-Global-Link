use super::spi::Spi;

const INITIAL_LOGIN_TX: u32 = 0x00;

// DIAGNOSTIC: record the rx values the GBA sends during login so core0 can dump them over
// USB. Lets us see whether the handshake advances (0x..494E -> B6B1 -> ... -> 8001) or the
// Pico is mis-reading the line (garbage = SPI timing/signal issue).
pub static mut LOGIN_RX: [u32; 16] = [0; 16];
pub static mut LOGIN_N: u32 = 0;

pub fn login(spi: &mut Spi) {
    let mut tx = INITIAL_LOGIN_TX;

    loop {
        // Clear any stale clock-timeout flag, attempt one exchange, and retry if it timed
        // out (GBA not clocking yet). This lets the adapter re-sync on its own after the
        // GBA is power-cycled, instead of needing the Pico power-cycled too.
        spi.reset();
        let rx = spi.transfer_u32(tx);
        if spi.reset_requested() {
            continue;
        }
        unsafe {
            let i = (LOGIN_N as usize) % 16;
            LOGIN_RX[i] = rx;
            LOGIN_N = LOGIN_N.wrapping_add(1);
        }
        tx = match rx {
            0x0000494E => 0x494EB6B1,
            0xFFFF494E => 0x494EB6B1,
            0x7FFF494E => 0x494EB6B1,
            0xB6B1494E => 0x544EB6B1,
            0xB6B1544E => 0x544EABB1,
            0xABB1544E => 0x4E45ABB1,
            0xABB14E45 => 0x4E45B1BA,
            0xB1BA4E45 => 0x4F44B1BA,
            0xB1BA4F44 => 0x4F44B0BB,
            0xB0BB4F44 => 0x8001B0BB,
            0xB0BB8001 => {
                break;
            }
            rx => 0x494e0000 | !(rx >> 16),
        };
    }
}
