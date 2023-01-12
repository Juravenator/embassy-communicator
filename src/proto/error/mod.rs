use twpb::{MessageDecoder};

#[derive(Debug, PartialEq, Default, ::twpb_derive::Message)]
pub struct EmptyError {}

#[derive(Debug, PartialEq, Default, ::twpb_derive::Message)]
pub struct Error {
    #[twpb(oneof,nr="1-2")]
    pub error: ::core::option::Option<Errors>,
}

#[derive(PartialEq, Debug, ::twpb_derive::Enum)]
pub enum Errors {
    #[twpb(message,nr=1)]
    CrcMismatch(EmptyError),
    #[twpb(message,nr=2)]
    UnknownApiVersion(EmptyError),
}