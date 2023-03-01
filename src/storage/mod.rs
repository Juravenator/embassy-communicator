//! Control storage on the dev board.
//! 
//! This module assumes two Adafruit i2c memory chips connected in a daisy-chained fashion.
//! See [the datasheet](https://cdn-learn.adafruit.com/assets/assets/000/115/136/original/24AA32A-24LC32A-32-Kbit-I2C-Serial-EEPROM-20001713N.pdf?1663269248) for more info

use embassy_stm32::i2c::{self, TimeoutI2c, Instance};
use embassy_time::{Duration, Timer};
use defmt::Format;
use defmt::{trace, debug, error, info, panic, unwrap};

// The address bits to control these boards consist of the control bits (1010) +
// three hardwired address pins (to daisy chain up to 8 boards).
// Unless you did some soldering, these pins are logical 0.
// The address bits are bit-shifted left by one. The last bit is the read/write bit.
// Reverting the logic, we can derive the base address: (0b(1010)(000)(x) >> 1) = 0b01010000 = 0x50
pub const BASE_EEPROM_I2C_ADDR: u8 = 0x50;

// Page sizes in these chips.
// Needed for write operations which can only ever be in one page per write command.
pub const EEPROM_PAGE_SIZE: u16 = 32;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Format)]
pub enum PostError {
    Nack,
    Timeout,
    BrokenChip(u8),
    Unknown,
}

pub struct AdafruitStorage<'d, T: Instance, TXDMA, RXDMA> {
    i2c: TimeoutI2c<'d, T, TXDMA, RXDMA>,
}

impl<'d, T: Instance, TXDMA, RXDMA> AdafruitStorage<'d, T, TXDMA, RXDMA> {
    // Set up and perform Power-On Self-Test
    pub fn new(i2c: TimeoutI2c<'d, T, TXDMA, RXDMA>) -> Self {
        Self{i2c}
    }

    // Read a byte from the specified memory address.
    pub async fn read_byte(&mut self, addr: u16) -> Result<u8, i2c::Error> {
        let mut data = [0u8; 1];
        self.read(addr, &mut data).await.and_then(|_| Ok(data[0]))
    }

    // Start address of a page derived from any address inside the page.
    fn page_base_addr(addr: u16) -> u16{
        addr - (addr % EEPROM_PAGE_SIZE)
    }

    // Address converted to a byte array to send to the i2c device.
    // This includes the control byte, because it contains 3 bytes
    // of addressing (for daisy chaining).
    // See datasheet: 5.1 Contiguous Addressing Across Multiple Devices
    fn addr_bytes(addr: u16) -> [u8; 3] {
        let addr: [u8; 2] = addr.to_be_bytes();
        let overflow = (addr[0] & 0b01110000) >> 4;
        [BASE_EEPROM_I2C_ADDR | overflow, addr[0] & 0x0F, addr[1]]
    }

    pub async fn read(&mut self, addr: u16, mut bytes: &mut [u8]) -> Result<(), i2c::Error> {
        let addr_b = Self::addr_bytes(addr);
        match self.i2c.blocking_write_read(addr_b[0], &addr_b[1..], &mut bytes) {
            Ok(_) => {
                trace!("Reading storage {:#06X}={:#04X}", addr, bytes);
                Ok(())
            },
            Err(e) => {
                trace!("Reading error {:#06X}={}", addr, e);
                Err(e)
            },
        }
    }

    async fn write_operation(&mut self, bytes: &[u8]) -> Result<(), i2c::Error> {
        assert!(bytes.len() < 36); // cannot write more than one page size + 3 address bits
        trace!("Writing storage {:#04X}={:#04X}", bytes[..3], bytes[3..]);
        self.i2c.blocking_write(bytes[0], &bytes[1..])?;

        // Wait for the device to become available again.
        // See datasheet: 7.0 ACKNOWLEDGE POLLING
        for _ in 1..1000 {
            // No fancy stuff here like trying to blocking_write_read() to verify data right away.
            // This ack polling loop MUST be the control byte immediately followed by STOP, or
            // subsequent reads will start at address 0x0000 no matter what you do.
            match self.i2c.blocking_write(bytes[0], &[]) {
                Ok(()) => {
                    return Ok(())
                },
                Err(i2c::Error::Nack) => {
                    Timer::after(Duration::from_millis(1)).await;
                },
                Err(e) => {
                    trace!("Wait after write error {}", e);
                    return Err(e)
                }
            }
            
        }
        trace!("Wait after write timeout");
        Err(i2c::Error::Timeout)
    }

    pub async fn write(&mut self, addr: u16, mut bytes: &[u8]) -> Result<(), i2c::Error> {

        let mut scratchspace = [0u8; 35];

        // If first chunk is not aligned to 32-bit page boundary, we deal with that first.
        let mut page_addr = Self::page_base_addr(addr);
        if page_addr != addr {
            // Max amount of bits we can write in this page given the misaligned start address.
            let max_chunk_size = EEPROM_PAGE_SIZE - (addr - page_addr);
            // Determine how much we gonna write.
            let chunk_size = bytes.len().min(max_chunk_size.into());

            // Steal that first (misaligned) chunk from our byte array.
            let chunk = &bytes[..chunk_size];
            bytes = &bytes[chunk_size..];

            // Compose our write payload.
            scratchspace[0..3].clone_from_slice(&Self::addr_bytes(addr));
            scratchspace[3..chunk_size+3].clone_from_slice(chunk);
            self.write_operation(&scratchspace[..chunk_size+3]).await?;

            // Read same memory region to verify our data.
            self.read(addr, &mut scratchspace[..chunk_size]).await?;
            for n in 0..chunk_size {
                if scratchspace[n] != chunk[n] {
                    return Err(i2c::Error::Crc)
                }
            }

            // We now have a nicely aligned start address for the rest of our data.
            page_addr += EEPROM_PAGE_SIZE;
        }
        drop(addr);

        // Deal with our data one page at a time.
        for chunk in bytes.chunks(EEPROM_PAGE_SIZE.into()) {
            // Would be that we're not writing a full page.
            let chunk_size = chunk.len();

            // Compose our write payload.
            scratchspace[0..3].clone_from_slice(&Self::addr_bytes(page_addr));
            scratchspace[3..chunk_size+3].clone_from_slice(chunk);
            self.write_operation(&scratchspace[..chunk_size+3]).await?;

            // Read same memory region to verify our data.
            self.read(addr, &mut scratchspace[..chunk_size]).await?;
            for n in 0..chunk_size {
                if scratchspace[n] != chunk[n] {
                    return Err(i2c::Error::Crc)
                }
            }

            // Set page address to next page.
            page_addr += EEPROM_PAGE_SIZE;
        };

        Ok(())
    }

    // Perform a Power-On Self-Test.
    pub async fn post(&mut self) -> Result<(), PostError> {
        // validate basic i2c communications
        let mut data = [0u8; 1];
        match self.i2c.blocking_read(BASE_EEPROM_I2C_ADDR, &mut data) {
            Ok(()) => {
                debug!("Storage i2C communication OK");
                Ok(())
            },
            Err(i2c::Error::Nack) => Err(PostError::Nack),
            Err(i2c::Error::Timeout) => Err(PostError::Timeout),
            Err(e) => {
                error!("Unknown storage error: {:?}", e);
                Err(PostError::Unknown)
            }
        }?;
        drop(data);

        // Read and write the first byte in the first block.
        // Also doubles as a startup counter.
        let mut bytes = [0u8; 2];
        // This would reset the boot counter
        // self.write(0, &mut bytes).await.or(Err(PostError::BrokenChip(0)))?;
        self.read(0, &mut bytes).await.or(Err(PostError::BrokenChip(0)))?;
        let mut bootcount = if bytes[0] == 0xFF && bytes[1] == 0xFF {
            0
        } else {
            u16::from_be_bytes(bytes)
        };
        debug!("Previous boots count: {}", bootcount);
        bootcount += 1;
        self.write(0, &bootcount.to_be_bytes()).await.or(Err(PostError::BrokenChip(0)))?;
        debug!("Storage single-byte read/write OK");

        // doesn't work yet. The chip just wraps around and overwrites stuff.
        // self.read_byte(4095).await.or(Err(PostError::SecondBlockNotWriteable))?;
        // self.read_byte(4096).await.or(Err(PostError::SecondBlockNotWriteable))?;
        // self.read_byte(4097).await.or(Err(PostError::SecondBlockNotWriteable))?;
        // self.read_byte(0xFFFF).await.or(Err(PostError::SecondBlockNotWriteable))?;

        // self.write(4095, &[0x02]).await.or(Err(PostError::SecondBlockNotWriteable))?;
        // self.write(4096, &[0x03]).await.or(Err(PostError::SecondBlockNotWriteable))?;
        // self.write(4097, &[0x04]).await.or(Err(PostError::SecondBlockNotWriteable))?;
        // self.write(0xFFFF, &[0x05]).await.or(Err(PostError::SecondBlockNotWriteable))?;

        info!("Storage POST OK");
        Ok(())
    }

}