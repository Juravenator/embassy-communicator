pub mod error;
pub mod v1;

#[derive(Debug, PartialEq, Default, ::twpb_derive::Message)]
pub struct APIMessage {
    #[twpb(oneof,nr="1-2")]
    pub content: ::core::option::Option<apimessage::Content>,
}

pub mod apimessage {
    use ::twpb::{MessageDecoder};

    #[derive(PartialEq, Debug, ::twpb_derive::Enum)]
    pub enum Content {
        #[twpb(message,nr=1)]
        Error(super::error::Error),
        #[twpb(message,nr=2)]
        V1Request(super::v1::Request),
        #[twpb(message,nr=3)]
        V1Response(super::v1::Response),
    }
}