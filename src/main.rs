#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod proto;
mod storage;

use atomic_enum::atomic_enum;
use embassy_stm32::pac::Interrupt;
use storage::PostError;
use core::fmt;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU32, Ordering};
use defmt::{trace, debug, error, info, unwrap};
use embassy_executor::{Spawner, InterruptExecutor};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{USB_OTG_HS, PB14, PB0, PE1};
use embassy_stm32::time::Hertz;
use embassy_stm32::i2c::I2c;
use embassy_stm32::usb_otg::{Driver, Instance};
use embassy_stm32::{bind_interrupts, peripherals, usb_otg, i2c, Config};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, UsbDevice};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};
use aarch::crc::{CrcDecoder, CrcEncoder};
use twpb::traits::Writer;
use crate::proto::error::Error as ProtoError;
use crate::proto::v1::Response;
use crate::proto::{APIMessage, apimessage, v1};
use crate::storage::AdafruitStorage;
use twpb::{MessageDecoder, MessageEncoder};

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
unsafe fn USART2() {
    EXECUTOR_MED.on_interrupt()
}

bind_interrupts!(struct Irqs {
    OTG_HS => usb_otg::InterruptHandler<peripherals::USB_OTG_HS>;
    I2C1_EV => i2c::InterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(spawner_low: Spawner) {
    info!("Initializing...");

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

    debug!("Setting up USB...");
    let mut config = embassy_stm32::usb_otg::Config::default();
    config.vbus_detection = true;
    let driver = unsafe {Driver::new_fs(
        peripherals.USB_OTG_HS,
        Irqs,
        peripherals.PA12,
        peripherals.PA11,
        &mut USB_OUT_BUF,
        config
    )};

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
        },
    };

    interrupt::USART1.set_priority(Priority::P6);
    let spawner_high = EXECUTOR_HIGH.start(Interrupt::USART1);

    interrupt::USART2.set_priority(Priority::P7);
    let spawner_med = EXECUTOR_MED.start(Interrupt::USART2);


    info!("setting up jobs");
    unwrap!(spawner_high.spawn(run_high()));
    unwrap!(spawner_med.spawn(task_usb_handle_connections()));
    unwrap!(spawner_low.spawn(task_usb_run()));
    unwrap!(spawner_low.spawn(task_blink(led_red, led_green, led_yellow)));
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
                let mut msg = String::<50>::new();
                if fmt::write(
                    &mut msg,
                    format_args!("Setting LED config to {:?}-{}ms\r\n", *colour, speed),
                )
                .is_err()
                {
                    error!("unable to write error message");
                }
                unwrap!(usb_try_write(&msg.into_bytes()).await);
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
async fn task_usb_handle_connections() {
    let class = unsafe { &mut *USB_CLASS.as_mut_ptr() };
    loop {
        class.wait_connection().await;
        info!("Connected");
        // let _ = echo(&mut class).await;
        loop {
            handle_proto_message(class).await;
        }
        // info!("Disconnected");
    }
}

#[embassy_executor::task]
async fn task_blink(
    led_red: Output<'static, PB14>,
    led_green: Output<'static, PB0>,
    led_yellow: Output<'static, PE1>
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

fn usb_is_ready() -> bool {
    let usb = unsafe { &mut *USB_CLASS.as_mut_ptr() };
    usb.dtr() && usb.rts()
}

async fn usb_try_write(data: &[u8]) -> Result<bool, EndpointError> {
    let usb = unsafe { &mut *USB_CLASS.as_mut_ptr() };
    match usb_is_ready() {
        false => Ok(false),
        true => usb.write_packet(data).await.and(Ok(true)),
    }
}

async fn handle_proto_message<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) {
    debug!("waiting for proto message");
    let mut crc_in = CrcDecoder::<1024>::default();
    let mut crc_out = CrcEncoder::<1024>::default();
    let mut buf = [0u8; 256];

    loop {
        let n = unwrap!(class.read_packet(&mut buf).await);
        debug!("usb packet read");
        unwrap!(crc_in.write_all(&buf[0..n]));
        debug!("crc decoded");
        if crc_in.messages_in_queue() == 0 {
            continue;
        } else {
            debug!("USB input buffer contains {} message(s)", crc_in.messages_in_queue());
        }

        let message = APIMessage::twpb_decode_iter(&mut crc_in.into_iter());
        debug!("message decoded");
        match message {
            Err(err) => {
                error!("error while parsing message: {}", err);
            }
            Ok(message) => {
                debug!("constructing response");
                let response = match message.content {
                    Some(apimessage::Content::V1Request(message)) => match message.request {
                        Some(v1::request::Request::GetInfo(_)) => {
                            APIMessage { content: Some(proto::apimessage::Content::V1Response(Response{
                                nr: 0,
                                response: Some(v1::response::Response::Info(proto::v1::SysInfo{
                                    serial:  heapless::String::from("some serial"),
                                    firmware_version:  heapless::String::from("0.e.pi"),
                                    vendor:  heapless::String::from("RoboCow"),
                                    product:  heapless::String::from("Product"),
                                })),
                            })) }
                        },
                        _ => APIMessage { content: Some(proto::apimessage::Content::V1Response(Response{
                            nr: 0,
                            response: Some(v1::response::Response::UnknownV1Request(proto::v1::Empty{})),
                        })) },
                    }
                    _ => APIMessage { content: Some(apimessage::Content::Error(ProtoError{error: Some(proto::error::Errors::UnknownApiVersion(proto::error::EmptyError {  }))})) },
                };

                debug!("encoding response");
                match response.twpb_encode(&mut crc_out).and_then(|_| crc_out.write_crc()) {
                    Ok(_) => {
                        debug!("sending response");
                        let mut l = 0;
                        for byte in &mut crc_out {
                            trace!("writing byte to USB {:#04X}", byte);
                            unwrap!(usb_try_write(&[byte]).await);
                            // Timer::after(Duration::from_millis(10)).await;
                            l += 1;
                        }
                        debug!("written {} bytes to USB output", l);
                    }
                    Err(err) => {
                        error!("failed to send response over USB. {}", err);
                    },
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn run_high() {
    loop {
        info!("        [high] tick!");
        Timer::after(Duration::from_millis(250)).await;
    }
}