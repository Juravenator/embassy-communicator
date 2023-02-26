#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod proto;

use atomic_enum::atomic_enum;
use embassy_stm32::interrupt::InterruptExt;
use core::fmt;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU32, Ordering};
use defmt::{trace, debug, error, info, panic, unwrap};
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed};
use embassy_stm32::peripherals::USB_OTG_HS;
use embassy_stm32::time::mhz;
use embassy_stm32::usb_otg::{Driver, Instance};
use embassy_stm32::executor::{Executor, InterruptExecutor};
use embassy_stm32::{interrupt, Config};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, UsbDevice};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};
use aarch::crc::{CrcDecoder, CrcEncoder};
use twpb::traits::{Writer};
use crate::proto::error::Error;
use crate::proto::v1::Response;    
use crate::proto::{APIMessage, apimessage, v1};
use twpb::{MessageDecoder, MessageEncoder};
use static_cell::StaticCell;

static BLINK_MS: AtomicU32 = AtomicU32::new(1000);
static LED_SELECTED: AtomicSelectedLED = AtomicSelectedLED::new(SelectedLED::RED);

static mut LED_RED: MaybeUninit<Output<'static, AnyPin>> = MaybeUninit::uninit();
static mut LED_GREEN: MaybeUninit<Output<'static, AnyPin>> = MaybeUninit::uninit();
static mut LED_YELLOW: MaybeUninit<Output<'static, AnyPin>> = MaybeUninit::uninit();

static mut USB_OUT_BUF: [u8; 256] = [0; 256];
static mut USB_DESC_BUF: [u8; 256] = [0; 256];
static mut USB_CONF_BUF: [u8; 256] = [0; 256];
static mut USB_BOS_BUF: [u8; 256] = [0; 256];
static mut USB_CTL_BUF: [u8; 64] = [0; 64];
static mut USB_STATE: MaybeUninit<State> = MaybeUninit::uninit();
static mut USB_CLASS: MaybeUninit<CdcAcmClass<Driver<USB_OTG_HS>>> = MaybeUninit::uninit();
static mut USB_DEV: MaybeUninit<UsbDevice<Driver<USB_OTG_HS>>> = MaybeUninit::uninit();
static EXECUTOR_HIGH: StaticCell<InterruptExecutor<interrupt::USART1>> = StaticCell::new();
static EXECUTOR_MED: StaticCell<InterruptExecutor<interrupt::USART2>> = StaticCell::new();

#[atomic_enum]
#[derive(PartialEq)]
enum SelectedLED {
    RED = 0,
    YELLOW,
    GREEN,
}

#[embassy_executor::main]
async fn main(spawner_low: Spawner) {
    info!("Initializing...");
    let mut config = Config::default();
    config.rcc.sys_ck = Some(mhz(400));
    config.rcc.hclk = Some(mhz(200));
    config.rcc.pll1.q_ck = Some(mhz(100));
    let board = embassy_stm32::init(config);

    info!("Setting up I/O...");
    unsafe {
        LED_RED
            .as_mut_ptr()
            .write(Output::new(board.PB14.degrade(), Level::Low, Speed::Low));
        LED_GREEN
            .as_mut_ptr()
            .write(Output::new(board.PB0.degrade(), Level::Low, Speed::Low));
        LED_YELLOW
            .as_mut_ptr()
            .write(Output::new(board.PE1.degrade(), Level::Low, Speed::Low));
    }

    debug!("Setting up USB...");
    let irq = interrupt::take!(OTG_HS);
    let driver = unsafe {
        Driver::new_fs(
            board.USB_OTG_HS,
            irq,
            board.PA12,
            board.PA11,
            &mut USB_OUT_BUF,
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
            None,
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

    let irq = interrupt::take!(USART1);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_HIGH.init(InterruptExecutor::new(irq));
    let spawner_high = executor.start();

    let irq = interrupt::take!(USART2);
    irq.set_priority(interrupt::Priority::P7);
    let executor = EXECUTOR_MED.init(InterruptExecutor::new(irq));
    let spawner_med = executor.start();

    unwrap!(spawner_high.spawn(run_high()));
    unwrap!(spawner_med.spawn(task_usb_handle_connections()));
    unwrap!(spawner_low.spawn(task_usb_run()));
    unwrap!(spawner_low.spawn(task_blink(42)));

    let button = Input::new(board.PC13, Pull::None);
    let mut button = ExtiInput::new(button, board.EXTI13);
    loop {
        for colour in &[SelectedLED::RED, SelectedLED::YELLOW, SelectedLED::GREEN] {
            LED_SELECTED.store(*colour, Ordering::Relaxed);
            for speed in &[500, 100, 50, 1000] {
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
async fn task_blink(_somenum: usize) {
    loop {
        let led = unsafe {
            match LED_SELECTED.load(Ordering::Relaxed) {
                SelectedLED::RED => &mut *LED_RED.as_mut_ptr(),
                SelectedLED::GREEN => &mut *LED_GREEN.as_mut_ptr(),
                SelectedLED::YELLOW => &mut *LED_YELLOW.as_mut_ptr(),
            }
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
                    _ => APIMessage { content: Some(apimessage::Content::Error(Error{error: Some(proto::error::Errors::UnknownApiVersion(proto::error::EmptyError {  }))})) },
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