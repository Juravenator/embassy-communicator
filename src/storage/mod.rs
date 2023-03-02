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

    pub async fn read(&mut self, addr: u16, bytes: &mut [u8]) -> Result<(), i2c::Error> {
        // We could be reading over the address space of multiple chips.
        // In this case, we need to split the read command into multiple parts, one per chip.
        // Each chip can address 4096 bits, which makes their address space 0x0FFF. The rest 
        // (0x7000) act as chip selector bits.
        let chip_num_start = addr >> 12;
        let chip_num_end = (addr + bytes.len() as u16 - 1) >> 12;

        for chip_num in chip_num_start..chip_num_end+1 {

            // Figure out what EEPROM addres to start reading from, how much bytes to read, and what slice in the bytes array it goes into.
            let (start_addr, bytes_start_addr, read_len) = match chip_num {
                // We're only going to read from one chip. The easiest case.
                chip_num if chip_num == chip_num_start && chip_num == chip_num_end => {
                    (addr, 0 as usize, bytes.len() as usize)
                },
                // We're going to start reading from this chip, but ending in another.
                chip_num if chip_num == chip_num_start => {
                    let max_readable_bits = 0x0FFF - (addr & 0x0FFF) + 1;
                    (addr, 0, max_readable_bits as usize)
                },
                // We're going to read up until this chip, but we started reading in a previous chip.
                chip_num if chip_num == chip_num_end => {
                    let chip_start_addr = chip_num << 12;
                    let bytes_start_addr = chip_start_addr - addr;
                    let read_len = (addr + bytes.len() as u16) - chip_start_addr;
                    (chip_start_addr, bytes_start_addr as usize, read_len as usize)
                },
                // We are neither on the first nor the last chip to read from.
                // Read the whole thing.
                chip_num => {
                    let chip_start_addr = chip_num << 12;
                    let bytes_start_addr = chip_start_addr - addr;
                    (chip_start_addr, bytes_start_addr as usize, 4096)
                },
            };

            self.read_operation(&Self::addr_bytes(start_addr), &mut bytes[bytes_start_addr..bytes_start_addr+read_len]).await?;
        }
        Ok(())
    }

    async fn read_operation(&mut self, addr_b: &[u8; 3], mut bytes: &mut [u8]) -> Result<(), i2c::Error> {
        match self.i2c.blocking_write_read(addr_b[0], &addr_b[1..3], &mut bytes) {
            Ok(_) => {
                trace!("Reading storage {:#04X}={:#04X}", addr_b, bytes);
                Ok(())
            },
            Err(e) => {
                trace!("Reading error {:#04X}={}", addr_b, e);
                Err(e)
            },
        }
    }

    async fn write_operation(&mut self, bytes: &[u8]) -> Result<(), i2c::Error> {
        assert!(bytes.len() < 36); // cannot write more than one page size + 3 address bits
        trace!("Writing storage {:#04X}={:#04X}", bytes[..3], bytes[3..]);
        self.i2c.blocking_write(bytes[0], &bytes[1..])?;

        self.wait_for_chip(bytes[0]).await
    }

    // Wait for the device to become available again.
    // See datasheet: 7.0 ACKNOWLEDGE POLLING
    // Datasheet only mentions this must occur after a write operation,
    // but it appears that it also must occur after failed reads. Acts like a reset?
    async fn wait_for_chip(&mut self, ctl_byte: u8) -> Result<(), i2c::Error> {
        for _ in 1..1000 {
            // No fancy stuff here like trying to blocking_write_read() to verify data right away.
            // This ack polling loop MUST be the control byte immediately followed by STOP, or
            // subsequent reads will start at address 0x0000 no matter what you do.
            match self.i2c.blocking_write(ctl_byte, &[]) {
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

        // Deal with our data one page at a time.
        for chunk in bytes.chunks(EEPROM_PAGE_SIZE.into()) {
            // Would be that we're not writing a full page.
            let chunk_size = chunk.len();

            // Compose our write payload.
            scratchspace[0..3].clone_from_slice(&Self::addr_bytes(page_addr));
            scratchspace[3..chunk_size+3].clone_from_slice(chunk);
            self.write_operation(&scratchspace[..chunk_size+3]).await?;

            // Read same memory region to verify our data.
            self.read(page_addr, &mut scratchspace[..chunk_size]).await?;
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

        // Enable this, everything goes to hell
        // // We require at least two EEPROM chips
        // match self.detect_chip_count().await {
        //     n if n < 1 => return Err(PostError::BrokenChip(n)),
        //     _ => debug!("Storage capacity OK"),
        // };
        self.ensure_minimum_chip_count(2).await?;

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

        // Read a whole page and compare with byte-per-byte reads.
        let mut bytes = [0u8; EEPROM_PAGE_SIZE as usize];
        self.read(0, &mut bytes).await.or(Err(PostError::BrokenChip(0)))?;
        for i in 0..bytes.len() as usize {
            let b = self.read_byte(i as u16).await.or(Err(PostError::BrokenChip(0)))?;
            if b != bytes[i] {
                error!("Page vs byte read mismatch at {:#06X} (page read {:#04X}!={:#04X}", i, bytes[i], b);
                return Err(PostError::BrokenChip(0));
            }
        }
        debug!("Storage page read/write OK");

        self.read_byte(4094).await.or(Err(PostError::BrokenChip(0)))?;
        self.read_byte(4095).await.or(Err(PostError::BrokenChip(0)))?;
        self.read_byte(4096).await.or(Err(PostError::BrokenChip(1)))?;
        self.read_byte(4097).await.or(Err(PostError::BrokenChip(1)))?;


        // Multi-page write/read accross multiple chips
        // TODO move this to a dedicated functional-test program.
        let mut bytes = [0u8; EEPROM_PAGE_SIZE as usize * 2];
        for i in 0..bytes.len() {
            bytes[i] = i as u8;
        }
        self.write(0x1000 - EEPROM_PAGE_SIZE, &mut bytes).await.or(Err(PostError::BrokenChip(1)))?;

        let mut bytes = [0u8; EEPROM_PAGE_SIZE as usize * 2];
        self.read(0x1000 - EEPROM_PAGE_SIZE, &mut bytes).await.or(Err(PostError::BrokenChip(1)))?;
        debug!("Storage multi-chip read/write OK");

        info!("Storage POST OK");
        Ok(())
    }

    async fn detect_chip_count(&mut self) -> u8 {
        for i in 0..0b0111u8 { // check for up to 8 daisy chained chips
            let addr = ((i as u16) << 12) | 0x0FFF;
            match self.read_byte(addr).await {
                Ok(_) => {},
                Err(e) if i == 0 => {
                    error!("Failure to read last byte in first storage chip (address {:#06X}): {}.", addr, e);
                    return 0;
                }
                Err(_) => {
                    let addr_prev = (((i-1) as u16) << 12) | 0x0FFF;
                    info!("{} storage chip(s) detected ({} addressable bytes)", i, addr_prev + 1);
                    // for n in 0..i as u16 {
                    //     let b = Self::addr_bytes(n << 12);
                    //     match self.wait_for_chip(b[0]).await {
                    //         Ok(()) => {},
                    //         Err(e) => error!("Error waiting for chip {} to be ready: {}", n, e),
                    //     }
                    // }
                    // match self.write(addr, &mut [1, 2]).await {
                    //     Ok(()) => error!("wtf"),
                    //     Err(e) => trace!("totally expected error {}", e),
                    // };
                    return i;
                }
            }
        };
        8
    }

    // A variant of detect_chip_count() that prevents any errors on the i2c bus.
    async fn ensure_minimum_chip_count(&mut self, chip_count: u8)  -> Result<(), PostError> {
        for i in 0..chip_count {
            let addr = ((i as u16) << 12) | 0x0FFF;
            self.read_byte(addr).await.or(Err(PostError::BrokenChip(i)))?;
        }
        Ok(())
    }

}