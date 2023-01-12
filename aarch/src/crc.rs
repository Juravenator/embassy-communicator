use crc32fast::Hasher;
use defmt::Format;
use twpb::traits::{Writer, WriterError};

use crate::cobs::{CobsEncoder, CobsDecoder, CobsError};


#[derive(Debug, Copy, Clone, Eq, PartialEq, Format)]
pub enum CrcError {
    CrcMismatch((u32, u32)),
    Cobs(CobsError),
}

#[derive(Debug)]
pub struct CrcEncoder<const N: usize> {
    cobs: CobsEncoder<N>,
    hasher: Option<Hasher>,
}

impl<const N: usize> Default for CrcEncoder<N> {
    fn default() -> Self {
        CrcEncoder {
            cobs: Default::default(),
            hasher: Some(Hasher::new()),
        }
    }
}

impl<const N: usize> Writer for CrcEncoder<N> {
    fn write(&mut self, byte: u8) -> Result<(), WriterError> {
        self.hasher.as_mut().map(|h| h.update(&[byte]));
        self.cobs.write(byte)
    }
}

// impl<const N: usize> embedded_io::Io for CrcEncoder<N> {
//     type Error = QueueError;
// }

// impl<const N: usize> Write for CrcEncoder<N> {
//     fn write(&mut self, bytes: &[u8]) -> Result<usize, QueueError> {
//         self.hasher.as_mut().map(|h| h.update(bytes));
//         self.cobs.write(bytes)
//     }

//     fn flush(&mut self) -> Result<(), QueueError> {
//         self.cobs.flush()
//     }
// }

impl<const N: usize> CrcEncoder<N> {
    /// Signal that a full message has been sent, and we can append
    /// a CRC checksum.
    pub fn write_crc(&mut self) -> Result<(), WriterError> {
        let digest = self.hasher.take().unwrap().finalize();
        self.write_all(&digest.to_be_bytes())?;

        self.hasher = Some(Hasher::new());
        Ok(())
    }

    pub fn drain(&mut self) -> usize {
        self.hasher = Some(Hasher::new());
        self.cobs.drain()
    }
}

impl<'a, const N: usize> Iterator for &'a mut CrcEncoder<N> {
    type Item = u8;
    fn next(&mut self) -> Option<Self::Item> {
        self.cobs.into_iter().next()
    }
}

#[derive(Debug)]
pub struct CrcDecoder<const N: usize> {
    cobs: CobsDecoder<N>,

    /// We need a tiny buffer to hold our CRC at the end.
    buffer: [u8; 4],
    tail: usize,
    buffer_empty: bool,

    /// While writing, we also compute a hash ourselves
    /// to verify the message.
    hasher: Option<Hasher>,

    // last_message_crc_match: bool,
    last_crc_expected: u32,
    last_crc_message: u32,
}


impl<const N: usize> Default for CrcDecoder<N> {
    fn default() -> Self {
        CrcDecoder {
            cobs: Default::default(),
            hasher: Some(Hasher::new()),
            buffer: [0;4],
            tail: 0,
            buffer_empty: true,
            last_crc_expected: 0,
            last_crc_message: 0,
        }
    }
}

impl<const N: usize> Writer for CrcDecoder<N> {
    fn write(&mut self, byte: u8) -> Result<(), WriterError> {
        self.cobs.write(byte)
    }
}

// impl<const N: usize> embedded_io::Io for CrcDecoder<N> {
//     type Error = QueueError;
// }

// impl<const N: usize> Write for CrcDecoder<N> {
//     fn write(&mut self, bytes: &[u8]) -> Result<usize, QueueError> {
//         self.cobs.write(bytes)
//     }

//     fn flush(&mut self) -> Result<(), QueueError> {
//         self.cobs.flush()
//     }
// }

impl<const N: usize> CrcDecoder<N> {
    pub fn drain(&mut self) -> usize {
        self.hasher = Some(Hasher::new());
        self.tail = 0;
        self.buffer_empty = true;
        self.cobs.drain()
    }

    /// Amount of messages waiting in buffer,
    /// based on amount of null bytes present.
    /// Incremented when a null byte is written.
    /// Decremented when a null byte is read
    /// (i.e when a message is fully read from the buffer).
    pub fn messages_in_queue(&self) -> usize {
        self.cobs.messages_in_queue()
    }

    /// If the decoder saw illegal stuff in the last message.
    /// This value will be set/unset after a message has been fully READ
    /// from the decoder, NOT when written to.
    /// This because a buffer can contain multiple messages. If we'd
    /// set this value at write time, you'd have no way of knowing _which_
    /// message is in error.
    pub fn last_message_error(&self) -> Option<CrcError> {
        match self.cobs.last_message_error() {
            Some(err) =>
                Some(CrcError::Cobs(err)),
            None => if self.last_crc_expected != self.last_crc_message {
                Some(CrcError::CrcMismatch((self.last_crc_expected, self.last_crc_message)))
            } else {
                None
            }
        }
    }

    /// Total messages ever decoded by this decoder.
    /// Incremented when a null byte is written.
    pub fn messages_seen(&self) -> usize {
        self.cobs.messages_seen()
    }
}

impl<'a, const N: usize> Iterator for &'a mut CrcDecoder<N> {
    type Item = u8;
    fn next(&mut self) -> Option<Self::Item> {
        // if the buffer is empty, we're going to have to fill it
        // before we can start returning bits
        if self.buffer_empty {
            for i in 0..4 {
                self.buffer[i] = self.cobs.into_iter().next().unwrap();
            }
            self.buffer_empty = false;
        }

        match self.cobs.into_iter().next() {
            // rotate buffer and send byte
            Some(byte) => {
                // Now we know that the last byte in the
                // buffer _isn't_ part of the CRC.
                // Release and overwrite with new potential
                // CRC byte.
                let data_byte = self.buffer[self.tail];
                self.buffer[self.tail] = byte;
                self.tail += 1;
                self.tail %= 4;

                // Feed the data byte into our own hasher to
                // verify later.
                self.hasher.as_mut().map(|h| h.update(&[data_byte]));

                Some(data_byte)
            },
            // end of message. check crc hash.
            None => {
                let crc = self.hasher.take().unwrap().finalize();

                self.buffer.rotate_left(self.tail);
                let message_crc = u32::from_be_bytes(self.buffer);

                self.last_crc_expected = crc;
                self.last_crc_message = message_crc;

                // reset iterator
                self.tail = 0;
                self.buffer_empty = true;
                self.hasher = Some(Hasher::new());

                None
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use twpb::traits::{Writer};
    use super::*;

    /// Verify from a known example that our crc library
    /// behaves like we expect it.
    #[test]
    fn test_crc_lib() {
        // https://crccalc.com/?crc=hello-world&method=crc32&datatype=ascii&outtype=0
        // 'hello-world' gives a crc of 0xB1D4025B
        let mut h = Hasher::new();
        // let mut d = CRC_IEEE.digest();
        for byte in b"hello-world" {
            h.update(&[*byte]);
        }
        let c = h.finalize();
        let expected: u32 = 0xB1D4025B;
        println!("{:#X} vs {:#X}", c, expected);
        assert_eq!(c, expected);
    }
    #[test]
    fn test_happy_path() {
        let input = b"hello-world";
        let mut e = CrcEncoder::<100>::default();
        e.write_all(input).unwrap();
        e.write_crc().unwrap();

        let mut d = CrcDecoder::<100>::default();
        for byte in e.into_iter() {
            d.write(byte).unwrap();
        }

        let decoded: Vec<u8> = d.into_iter().collect();
        let decoded = &decoded[..];
        assert_eq!(d.last_message_error(), None);
        assert_eq!(input, decoded);
    }

    #[test]
    fn test_composition() {
        let input = b"hello-world";
        let mut cobs_encoder = CobsEncoder::<100>::default();
        cobs_encoder.write_all(input).unwrap();

        let mut crc_encoder = CrcEncoder::<100>::default();
        crc_encoder.write_all(input).unwrap();
        crc_encoder.write_crc().unwrap();

        let cobs_encoded: Vec<u8> = cobs_encoder.into_iter().collect();
        let crc_encoded: Vec<u8> = crc_encoder.into_iter().collect();

        // verify that our data has only grew by one crc hash (u32)
        assert_eq!(cobs_encoded.len() + 4, crc_encoded.len());
        // verify that our cobs encoded data passed through, except:
        // the null byte, which has been moved
        // (in this case) the first overhead byte, which changed because of the moved null byte
        assert_eq!(
            cobs_encoded[1..cobs_encoded.len() - 1],
            crc_encoded [1..cobs_encoded.len() - 1]
        );

        // verify that we then have the correct CRC value
        // https://crccalc.com/?crc=hello-world&method=crc32&datatype=ascii&outtype=0
        // 'hello-world' gives a crc of 0xB1D4025B
        assert_eq!(crc_encoded[crc_encoded.len()-5..crc_encoded.len()-1], [0xB1, 0xD4, 0x2, 0x5B]);

        // verify the null byte at the end
        assert_eq!(crc_encoded[crc_encoded.len()-1], 0);
    }
}