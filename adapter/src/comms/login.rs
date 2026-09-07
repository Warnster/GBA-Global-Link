use super::spi::Spi;

const INITIAL_LOGIN_TX: u32 = 0x00;

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
