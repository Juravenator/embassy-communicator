#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod proto;
mod storage;

use crate::proto::error::Error as ProtoError;
use crate::proto::v1::Response;
use crate::proto::{apimessage, v1, APIMessage};
use crate::storage::AdafruitStorage;
use aarch::crc::{CrcDecoder, CrcEncoder};
use atomic_enum::atomic_enum;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU32, Ordering};
use defmt::{debug, error, info, trace, unwrap, Format};
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::pac::Interrupt;
use embassy_stm32::peripherals::{DMA1_CH0, DMA1_CH1, PB0, PB14, PE1, USART2, USB_OTG_HS};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UsartConfig, Uart};
use embassy_stm32::usb_otg::Driver;
use embassy_stm32::{bind_interrupts, i2c, interrupt, peripherals, usart, usb_otg, Config};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, UsbDevice};
use storage::PostError;
use twpb::traits::Writer;
use twpb::{DecodeError, MessageDecoder, MessageEncoder, WriterError};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    OTG_HS => usb_otg::InterruptHandler<peripherals::USB_OTG_HS>;
    I2C1_EV => i2c::InterruptHandler<peripherals::I2C1>;
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});

static BLINK_MS: AtomicU32 = AtomicU32::new(1000);
static LED_SELECTED: AtomicSelectedLED = AtomicSelectedLED::new(SelectedLED::RED);

static mut USB_OUT_BUF: [u8; 256] = [0; 256];
static mut USB_DESC_BUF: [u8; 256] = [0; 256];
static mut USB_CONF_BUF: [u8; 256] = [0; 256];
static mut USB_BOS_BUF: [u8; 256] = [0; 256];
static mut USB_CTL_BUF: [u8; 64] = [0; 64];
static mut USB_STATE: MaybeUninit<State> = MaybeUninit::uninit();
static mut USB_CLASS: MaybeUninit<CdcAcmClass<Driver<USB_OTG_HS>>> = MaybeUninit::uninit();
static mut USB_DEV: MaybeUninit<UsbDevice<Driver<USB_OTG_HS>>> = MaybeUninit::uninit();

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();

static mut CRC_ENCODER: MaybeUninit<CrcEncoder<1024>> = MaybeUninit::uninit();
static mut CRC_DECODER: MaybeUninit<CrcDecoder<1024>> = MaybeUninit::uninit();

static mut UART: MaybeUninit<Uart<'static, USART2, DMA1_CH0, DMA1_CH1>> = MaybeUninit::uninit();

#[atomic_enum]
#[derive(PartialEq)]
enum SelectedLED {
    RED = 0,
    YELLOW,
    GREEN,
}

#[interrupt]
unsafe fn USART1() {
    EXECUTOR_HIGH.on_interrupt()
}

#[interrupt]
unsafe fn UART4() {
    EXECUTOR_MED.on_interrupt()
}

#[embassy_executor::main]
async fn main(spawner_low: Spawner) {
    info!("Initializing...");

    unsafe {
        CRC_ENCODER
            .as_mut_ptr()
            .write(CrcEncoder::<1024>::default());
        CRC_DECODER
            .as_mut_ptr()
            .write(CrcDecoder::<1024>::default());
    }

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.hsi48 = true; // needed for USB
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: None,
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
    }
    let peripherals = embassy_stm32::init(config);

    info!("Setting up I/O...");
    let mut led_red = Output::new(peripherals.PB14, Level::Low, Speed::Low);
    let led_green = Output::new(peripherals.PB0, Level::Low, Speed::Low);
    let led_yellow = Output::new(peripherals.PE1, Level::Low, Speed::Low);

    unsafe {
        let usart_config = UsartConfig::default();
        UART.as_mut_ptr().write(
            Uart::new(
                peripherals.USART2,
                peripherals.PD6,
                peripherals.PD5,
                Irqs,
                peripherals.DMA1_CH0,
                peripherals.DMA1_CH1,
                usart_config,
            )
            .unwrap(),
        );
    }

    debug!("Setting up USB...");
    let mut config = embassy_stm32::usb_otg::Config::default();
    config.vbus_detection = true;
    let driver = unsafe {
        Driver::new_fs(
            peripherals.USB_OTG_HS,
            Irqs,
            peripherals.PA12,
            peripherals.PA11,
            &mut USB_OUT_BUF,
            config,
        )
    };

    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");
    // Required for Windows 7 compatiblity.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    let mut builder = unsafe {
        Builder::new(
            driver,
            config,
            &mut USB_DESC_BUF,
            &mut USB_CONF_BUF,
            &mut USB_BOS_BUF,
            &mut USB_CTL_BUF,
        )
    };

    unsafe {
        USB_STATE.as_mut_ptr().write(State::new());
        USB_CLASS.as_mut_ptr().write(CdcAcmClass::new(
            &mut builder,
            &mut *USB_STATE.as_mut_ptr(),
            64,
        ));
        USB_DEV.as_mut_ptr().write(builder.build());
    }

    let i2c = I2c::new(
        peripherals.I2C1,
        peripherals.PB8,
        peripherals.PB9,
        Irqs,
        peripherals.DMA1_CH4,
        peripherals.DMA1_CH5,
        Hertz(100_000),
        Default::default(),
    );

    let mut storage = AdafruitStorage::new(i2c);

    match storage.read_byte(4096).await {
        Ok(b) => info!("illegal read succeeded 4096={:#04X}", b),
        Err(e) => error!("illegal read failed {}", e),
    }
    // match storage.i2c.write(0x55, &[0x00, 0x64]) {
    //     Ok(()) => info!("illegal write success"),
    //     Err(e) => error!("illegal write fail {}", e),
    // };

    match storage.post().await {
        Ok(s) => s,
        Err(e) => {
            error!("Storage POST error! {}", e);
            loop {
                // 3 short blinks denotes a Storage error
                for _ in 1..4 {
                    led_red.set_high();
                    Timer::after(Duration::from_millis(150)).await;
                    led_red.set_low();
                    Timer::after(Duration::from_millis(150)).await;
                }

                // extra blink codes for each error type
                Timer::after(Duration::from_millis(500)).await;
                let numblinks = match e {
                    PostError::Nack => 1,
                    PostError::Timeout => 2,
                    PostError::Unknown => 3,
                    PostError::BrokenChip(n) => 4 + n,
                };
                for _ in 1..numblinks + 1 {
                    led_red.set_high();
                    Timer::after(Duration::from_millis(150)).await;
                    led_red.set_low();
                    Timer::after(Duration::from_millis(150)).await;
                }

                // wait and repeat
                Timer::after(Duration::from_secs(2)).await;
            }
        }
    };

    interrupt::UART4.set_priority(Priority::P7);
    let spawner_med = EXECUTOR_MED.start(Interrupt::UART4);

    interrupt::USART1.set_priority(Priority::P6);
    let spawner_high = EXECUTOR_HIGH.start(Interrupt::USART1);

    info!("setting up jobs");
    unwrap!(spawner_low.spawn(task_blink(led_red, led_green, led_yellow)));
    unwrap!(spawner_low.spawn(task_usb_run()));
    unwrap!(spawner_med.spawn(task_handle_usb()));
    unwrap!(spawner_med.spawn(task_uart_handle()));
    unwrap!(spawner_high.spawn(run_high()));
    info!("done setting up jobs");

    let button = Input::new(peripherals.PC13, Pull::None);
    let mut button = ExtiInput::new(button, peripherals.EXTI13);
    info!("starting loop");
    loop {
        for colour in &[SelectedLED::RED, SelectedLED::YELLOW, SelectedLED::GREEN] {
            LED_SELECTED.store(*colour, Ordering::Relaxed);
            for speed in &[500, 100, 50, 1000] {
                info!("waiting for button");
                button.wait_for_rising_edge().await;
                BLINK_MS.store(*speed, Ordering::Relaxed);
            }
        }
    }
}

#[embassy_executor::task]
async fn task_usb_run() -> ! {
    let usb = unsafe { &mut *USB_DEV.as_mut_ptr() };
    usb.run().await
}

#[embassy_executor::task]
async fn task_handle_usb() {
    let class = unsafe { &mut *USB_CLASS.as_mut_ptr() };
    let mut crc_encoder = unsafe { &mut *CRC_ENCODER.as_mut_ptr() };
    let mut crc_decoder = unsafe { &mut *CRC_DECODER.as_mut_ptr() };
    let mut buf = [0u8; 256];
    loop {
        class.wait_connection().await;
        info!("USB connected");
        loop {
            match class.read_packet(&mut buf).await {
                Ok(n) => {
                    debug!("usb packet read of size {}", n);
                    match crc_decoder.write_all(&buf[0..n]) {
                        Ok(_) => {
                            debug!("crc decoded");
                            if crc_decoder.messages_in_queue() > 0 {
                                handle_proto_message(&mut crc_decoder, &mut crc_encoder).await;
                                debug!("sending response bytes");
                                let mut l = 0;
                                for byte in &mut crc_encoder.into_iter() {
                                    buf[l] = byte;
                                    l += 1;
                                    if l == buf.len() {
                                        class.write_packet(&buf).await.unwrap();
                                        debug!("written {} bytes to USB output", l);
                                        l = 0;
                                    }
                                }
                                if l != 0 {
                                    class.write_packet(&buf[0..l]).await.unwrap();
                                    debug!("written {} bytes to USB output", l);
                                }
                            }
                        }
                        Err(WriterError::BufferOverflow) => {
                            error!("failed to write to protobuf input queue. resetting queue");
                            crc_decoder.drain();
                        }
                    }
                }
                Err(EndpointError::BufferOverflow) => {
                    error!("buffer overflow while reading from USB. resetting queue");
                    // let mut crc_in = M_CRC_DECODER.lock().await;
                    crc_decoder.drain();
                }
                Err(EndpointError::Disabled) => {
                    error!("USB endpoint disappeared. resetting queue");
                    // let mut crc_in = M_CRC_DECODER.lock().await;
                    crc_decoder.drain();
                    break;
                }
            }
        }
        info!("USB disconnected");
    }
}

#[embassy_executor::task]
async fn task_uart_handle() {
    let uart = unsafe { &mut *UART.as_mut_ptr() };
    let mut crc_encoder = unsafe { &mut *CRC_ENCODER.as_mut_ptr() };
    let mut crc_decoder = unsafe { &mut *CRC_DECODER.as_mut_ptr() };
    let mut buf = [0u8; 1];
    loop {
        match uart.read(&mut buf).await {
            Ok(_) => {}
            Err(e) => {
                error!("error while reading uart {:?}", e);
                crc_decoder.drain();
                continue;
            }
        }
        match crc_decoder.write(buf[0]) {
            Ok(_) => {}
            Err(e) => {
                error!("error while writing to uart decoder {:?}", e);
                crc_decoder.drain();
            }
        }
        if crc_decoder.messages_in_queue() > 0 {
            match handle_proto_message(&mut crc_decoder, &mut crc_encoder).await {
                Ok(_) => {}
                Err(e) => {
                    error!("error while handling proto message {:?}", e);
                    crc_decoder.drain();
                    crc_encoder.drain();
                    continue;
                }
            }
            debug!("sending response bytes");
            let mut l = 0;
            let mut buf = [0u8; 100];
            for byte in &mut crc_encoder.into_iter() {
                buf[l] = byte;
                l += 1;
                if l == buf.len() {
                    debug!("writing bytes to UART {:#04X}", buf);
                    uart.write(&buf).await.unwrap();
                    debug!("written {} bytes to UART output", l);
                    l = 0;
                }
            }
            if l != 0 {
                uart.write(&buf[0..l]).await.unwrap();
                debug!("written {} bytes to UART output", l);
            }
        }
    }
}

#[embassy_executor::task]
async fn task_blink(
    led_red: Output<'static, PB14>,
    led_green: Output<'static, PB0>,
    led_yellow: Output<'static, PE1>,
) {
    let mut led_red = led_red.degrade();
    let mut led_green = led_green.degrade();
    let mut led_yellow = led_yellow.degrade();

    loop {
        let led = match LED_SELECTED.load(Ordering::Relaxed) {
            SelectedLED::RED => &mut led_red,
            SelectedLED::GREEN => &mut led_green,
            SelectedLED::YELLOW => &mut led_yellow,
        };
        info!("blink high");
        // unwrap!(usb_try_write(b"high").await);
        led.set_high();
        Timer::after(Duration::from_millis(
            BLINK_MS.load(Ordering::Relaxed).into(),
        ))
        .await;

        info!("blink low");
        // unwrap!(usb_try_write(b"low").await);
        led.set_low();
        Timer::after(Duration::from_millis(
            BLINK_MS.load(Ordering::Relaxed).into(),
        ))
        .await;
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Format)]
enum HandleProtoError {
    Decode(DecodeError),
    Encode(WriterError),
}
impl From<DecodeError> for HandleProtoError {
    fn from(e: DecodeError) -> Self {
        HandleProtoError::Decode(e)
    }
}
impl From<WriterError> for HandleProtoError {
    fn from(e: WriterError) -> Self {
        HandleProtoError::Encode(e)
    }
}

async fn handle_proto_message<const N: usize>(
    crc_in: &mut CrcDecoder<N>,
    crc_out: &mut CrcEncoder<N>,
) -> Result<(), HandleProtoError> {
    while crc_in.messages_in_queue() > 0 {
        debug!(
            "input buffer contains {} message(s)",
            crc_in.messages_in_queue()
        );

        debug!("decoding message");
        let message = APIMessage::twpb_decode_iter(&mut crc_in.into_iter())?;
        debug!("message decoded; constructing response");
        let response = match message.content {
            Some(apimessage::Content::V1Request(message)) => match message.request {
                Some(v1::request::Request::GetInfo(_)) => APIMessage {
                    content: Some(proto::apimessage::Content::V1Response(Response {
                        nr: 0,
                        response: Some(v1::response::Response::Info(proto::v1::SysInfo {
                            serial: heapless::String::from("some serial"),
                            firmware_version: heapless::String::from("0.e.pi"),
                            vendor: heapless::String::from("RoboCow"),
                            product: heapless::String::from("Product"),
                        })),
                    })),
                },
                _ => APIMessage {
                    content: Some(proto::apimessage::Content::V1Response(Response {
                        nr: 0,
                        response: Some(v1::response::Response::UnknownV1Request(
                            proto::v1::Empty {},
                        )),
                    })),
                },
            },
            _ => APIMessage {
                content: Some(apimessage::Content::Error(ProtoError {
                    error: Some(proto::error::Errors::UnknownApiVersion(
                        proto::error::EmptyError {},
                    )),
                })),
            },
        };

        debug!("encoding response");
        response
            .twpb_encode(crc_out)
            .and_then(|_| crc_out.write_crc())?;
    }
    Ok(())
}

#[embassy_executor::task]
async fn run_high() {
    loop {
        info!("        [high] tick!");
        Timer::after(Duration::from_millis(250)).await;
    }
}
