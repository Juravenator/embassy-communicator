// pub mod equalizer;

#[derive(Debug, PartialEq, Default, ::twpb_derive::Message)]
pub struct Empty {}

#[derive(Debug, PartialEq, Default, ::twpb_derive::Message)]
pub struct Request {
    #[twpb(oneof,nr="1-3")]
    pub request: ::core::option::Option<request::Request>,
}

#[derive(Debug, PartialEq, Default, ::twpb_derive::Message)]
pub struct Response {
    #[twpb(uint32,nr=1)]
    pub nr: u32,

    #[twpb(oneof,nr="2-4")]
    pub response: ::core::option::Option<response::Response>,
}

pub mod request {
    use ::twpb::{MessageDecoder};

    #[derive(PartialEq, Debug, ::twpb_derive::Enum)]
    pub enum Request {
        #[twpb(message,nr=1)]
        GetInfo(super::Empty),
        #[twpb(message,nr=2)]
        GetDsp(super::Empty),
        #[twpb(message,nr=3)]
        ConfigureDsp(super::DspConfig),
    }
}

pub mod response {
    use ::twpb::{MessageDecoder};

    #[derive(PartialEq, Debug, ::twpb_derive::Enum)]
    pub enum Response {
        #[twpb(message,nr=2)]
        Info(super::SysInfo),
        // #[twpb(message,nr=3)]
        // dsp(super::DspState),
        #[twpb(message,nr=4)]
        UnknownV1Request(super::Empty),
    }
}

#[derive(Debug, PartialEq, Default, ::twpb_derive::Message)]
pub struct SysInfo{
    #[twpb(string,nr=1)]
    pub serial: heapless::String<30>,
    #[twpb(string,nr=2)]
    pub firmware_version: heapless::String<30>,
    #[twpb(string,nr=3)]
    pub vendor: heapless::String<30>,
    #[twpb(string,nr=4)]
    pub product: heapless::String<30>,
}

#[derive(Debug, PartialEq, Default, ::twpb_derive::Message)]
pub struct DspConfig{
    #[twpb(uint32,nr=1)]
    pub sample_rate: u32,
    #[twpb(uint32,nr=2)]
    pub block_size: u32,
}