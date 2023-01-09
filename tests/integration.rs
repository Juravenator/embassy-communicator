#![no_std]
#![no_main]

#[defmt_test::tests]
mod tests {
    use defmt::assert;
    use embassy_stm32::{Peripherals};
    use embassy_stm32::gpio::{Level, Output, Speed, Pin};
    use embassy_time::{Duration, Timer, Delay};
    use {defmt_rtt as _, panic_probe as _};
    use embedded_hal::blocking::delay::DelayMs;

    #[init]
    fn init() -> Peripherals {
        embassy_stm32::init(Default::default())
    }

    #[test]
    fn it_works(board: &mut Peripherals) {
        let mut led = Output::new(&mut board.PB14, Level::Low, Speed::Low);
        assert!(led.is_set_low());
        led.set_high();
        Delay.delay_ms(1_000_u32);
        assert!(led.is_set_high());
    }

    #[test]
    fn it_works_fast(board: &mut Peripherals) {
        let mut led = Output::new(&mut board.PB14, Level::High, Speed::Low);
        assert!(led.is_set_high());
        led.set_low();
        assert!(led.is_set_low());
    }
}
