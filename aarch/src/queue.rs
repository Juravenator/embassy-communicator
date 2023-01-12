use defmt::Format;
use twpb::traits::{Writer, WriterError};

// #[derive(Debug, Copy, Clone, Eq, PartialEq, Format)]
// pub enum QueueError {
//     BufferOverflow,
//     BufferUnderflow,
// }

#[derive(Debug, Copy, Clone, Eq, PartialEq, Format)]
pub enum ReadError {
    BufferUnderflow,
}

// pub trait Writer {
//     fn write(&mut self, byte: u8) -> Result<(), QueueError>;

//     fn write_all(&mut self, bytes: &[u8]) -> Result<usize, QueueError> {
//         for byte in bytes {
//             if let Err(err) = self.write(*byte) {
//                 return Err(err);
//             }
//         }
//         Ok(bytes.len())
//     }
// }

// impl embedded_io::Error for QueueError {
//     fn kind(&self) -> embedded_io::ErrorKind {
//         embedded_io::ErrorKind::Other
//     }
// }

// pub trait Writer: Write {
//     // fn write(&mut self, bytes: &[u8]) -> Result<usize, Self::Error>;

//     fn write_all(&mut self, mut bytes: &[u8]) -> Result<(), Self::Error> {
//         // for byte in bytes {
//         //     if let Err(err) = self.write(*byte) {
//         //         return Err(err);
//         //     }
//         // }
//         while !bytes.is_empty() {
//             match self.write(bytes) {
//                 Ok(0) => panic!("zero-length write."),
//                 Ok(n) => bytes = &bytes[n..],
//                 Err(e) => return Err(e),
//             }
//         }
//         Ok(())
//         // Ok(bytes.len())
//     }
// }

pub trait Reader {
    fn read(&mut self) -> Result<u8, ReadError>;

    fn drain(&mut self) -> usize;
}

pub trait QueueInfo<const N: usize> {
    fn used(&self) -> usize;
    fn available(&self) -> usize {
        // // 0................N
        // //      H<====>T
        // if self.head < self.tail {
        //     self.tail - self.head

        // // 0................N
        // // ====>T      H<====
        // } else {
        //     N - self.head + self.tail
        // }
        N - self.used()
    }

    fn capacity() -> usize {
        N
    }

    fn is_full(&self) -> bool;
    fn is_empty(&self) -> bool;
}

#[derive(Debug)]
pub struct Queue<const N: usize> {
    /// raw byte buffer handling
    buffer: [u8; N],
    /// the very first index in the buffer that has a value in use
    tail: usize,
    /// the first unused index in the buffer after the tail index
    head: usize,
    /// with just head and tail we can't see the difference between
    /// a completely empty, or a completely full buffer
    is_full: bool,
}

// impl<const N: usize> embedded_io::Io for Queue<N> {
//     type Error = QueueError;
// }

impl<const N: usize> Iterator for Queue<N> {
    type Item = u8;
    fn next(&mut self) -> Option<Self::Item> {
        self.read().ok()
        // match self.read() {
        //     Ok(byte) => Some(*byte),
        //     Err(_) => None,
        // }
    }
}

// impl<'a, T> Iterator for &'a Vec<T> {
//     type Item = &'a T;
//     type IntoIter = slice::Iter<'a, T>;

//     fn into_iter(self) -> slice::Iter<'a, T> { /* ... */ }
// }

impl<const N: usize> Default for Queue<N> {
    fn default() -> Self {
        Queue {
            buffer: [0; N],
            head: 0,
            tail: 0,
            is_full: false,
        }
    }
}

impl<const N: usize> Writer for Queue<N> {
    /// Writes a new byte in the buffer, if possible.
    /// 
    /// # Example
    /// ```
    /// # use aarch::queue::{Queue, Reader};
    /// # use twpb::traits::{Writer};
    ///
    /// let mut w = Queue::<10>::new();
    /// assert!(w.write(3 as u8).is_ok());
    /// assert!(w.write(5 as u8).is_ok());
    /// assert_eq!(w.read().unwrap(), 3);
    /// assert_eq!(w.read().unwrap(), 5);
    /// ```
    fn write(&mut self, byte: u8) -> Result<(), WriterError> {
        if self.available() == 0 {
            return Err(WriterError::BufferOverflow);
        }
        let mut next_head = self.head + 1;
        if next_head == N {
            next_head = 0;
        }
        self.buffer[self.head] = byte;
        self.head = next_head;
        self.is_full = self.head == self.tail;
        Ok(())
    }
}

impl<const N: usize> Reader for Queue<N> {
    fn read(&mut self) -> Result<u8, ReadError> {
        if self.is_empty() {
            return Err(ReadError::BufferUnderflow);
        }
        let mut next_tail = self.tail + 1;
        if next_tail == N {
            next_tail = 0;
        }
        let val = self.buffer[self.tail];
        self.tail = next_tail;
        self.is_full = false;
        Ok(val)
    }

    fn drain(&mut self) -> usize {
        let l = self.used();
        self.head = 0;
        self.tail = 0;
        self.is_full = false;
        l
    }
}

impl<const N: usize> QueueInfo<N> for Queue<N> {

    /// Returns the amount of bytes currently in use.
    /// 
    /// # Example
    /// ```rust
    /// # use aarch::queue::{Queue, QueueInfo};
    /// # use twpb::traits::{Writer};
    ///
    /// let mut w = Queue::<10>::new();
    /// assert!(w.write_all(&[3,5]).is_ok());
    /// assert_eq!(w.used(), 2);
    /// assert_eq!(w.available(), 8);
    /// ```
    fn used(&self) -> usize {
        if self.is_full {
            return N;
        }
        // 0................N
        // ====>H      T<====
        if self.head < self.tail {
            N + self.head - self.tail

        // 0................N
        //      T<====>H
        } else {
            self.head - self.tail
        }
        
    }

    fn is_empty(&self) -> bool {
        self.tail == self.head && !self.is_full
    }

    fn is_full(&self) -> bool {
        self.is_full
    }
}

impl<const N: usize> Queue<N> {
    pub fn new() -> Self {
        Queue::default()
    }

    /// The number of bytes to read until one would
    /// encounter a zero, if any.
    /// 
    /// # Example
    /// ```
    /// # use aarch::queue::{Queue, QueueInfo, Reader};
    /// # use twpb::traits::{Writer};
    /// let mut w = Queue::<10>::new();
    /// assert_eq!(w.next_zero(), None);
    /// w.write_all(&[1,2,3]).unwrap();
    /// assert_eq!(w.next_zero(), None);
    /// w.write(0).unwrap();
    /// assert_eq!(w.next_zero(), Some(4));
    /// assert_eq!(w.read().unwrap(), 1);
    /// assert_eq!(w.read().unwrap(), 2);
    /// assert_eq!(w.read().unwrap(), 3);
    /// assert_eq!(w.read().unwrap(), 0);
    /// ```
    pub fn next_zero(&self) -> Option<usize> {
        if self.is_empty() {
            return None;
        }
        let mut peek_tail = self.tail;
        let mut bytes_passed = 0;
        loop {
            // simulate a read(), and look for a 0
            bytes_passed += 1;
            if self.buffer[peek_tail] == 0 {
                return Some(bytes_passed);
            }

            // no match, increment virtual tail for next round
            peek_tail += 1;
            peek_tail %= N;
            if peek_tail == self.head {
                break;
            }
        }
        None
    }
}



#[cfg(test)]
mod tests {
    use twpb::traits::{Writer, WriterError};
    use super::*;

    #[test]
    fn test_queue_capacity() {
        let mut w = Queue::<10>::new();
        for i in 0..10 {
            println!("writing {}", i);
            assert_eq!(w.used(), i);
            assert_eq!(w.available(), 10-i);
            assert!(w.write(i as u8).is_ok());
            println!("write complete: head {}, tail {}", w.head, w.tail);
        }
        assert_eq!(w.used(), 10);
        assert_eq!(w.available(), 0);
    }

    #[test]
    fn test_queue_overflow() {
        let mut w = Queue::<10>::new();
        for i in 0..10 {
            println!("writing {}", i);
            assert!(w.write(i as u8).is_ok());
        }
        assert_eq!(w.write(1).unwrap_err(), WriterError::BufferOverflow);
    }

    #[test]
    fn test_queue_underrun() {
        let mut w = Queue::<10>::new();
        assert!(w.write(2).is_ok());
        assert_eq!(w.read().unwrap(), 2);
        assert_eq!(w.read().unwrap_err(), ReadError::BufferUnderflow);
    }

    #[test]
    fn test_read_write() {
        let mut w = Queue::<10>::new();
        for i1 in 0..10 {
            for i2 in 0..10 {
                let val = i1 * 10 + i2;
                println!("writing {}", val);
                assert!(w.write(val).is_ok());
            }
            for i2 in 0..10 {
                let expected_val = i1 * 10 + i2;
                println!("reading. expecting {}", expected_val);
                assert_eq!(w.read().unwrap(), expected_val);
            }
        }
        assert_eq!(w.available(), 10);
        assert_eq!(w.used(), 0);
    }

    #[test]
    fn test_next_zero_simple() {
        let mut w = Queue::<10>::new();
        assert_eq!(w.next_zero(), None);

        w.write(0).unwrap();
        assert_eq!(w.next_zero(), Some(1));

        w.read().unwrap();
        assert_eq!(w.next_zero(), None);

        for byte in 1..5 {
            println!("writing {}", byte);
            w.write(byte).unwrap();
        }
        assert_eq!(w.next_zero(), None);

        w.write(0).unwrap();
        assert_eq!(w.next_zero(), Some(5));
    }

    #[test]
    fn test_next_zero_accuracy() {
        for l in 0..9 {
            let mut w = Queue::<10>::new();
            assert_eq!(w.next_zero(), None);

            println!("writing {} bytes", l);
            for byte in 1..l+1 {
                println!("writing {}", byte);
                w.write(byte).unwrap();
            }
            w.write(0).unwrap();
            let next_zero = w.next_zero().unwrap();
            let expected = l as usize + 1;
            assert_eq!(w.used(), expected);
            assert_eq!(next_zero, expected);

            // call bullshit, actually count as many bytes as
            // next_zero() claims and check
            for byte in 1..l+1 {
                assert_eq!(w.read().unwrap(), byte);
            }
            assert_eq!(w.read().unwrap(), 0);
        }
    }

    #[test]
    fn test_next_zero_buffer_wrap_around() {
        let mut w = Queue::<10>::new();

        // write + read 7 values so our tail pointer is near the end
        for byte in 1..8 {
            w.write(byte).unwrap();
            assert_eq!(w.read().unwrap(), byte);
        }

        // write 7 values again, follewed by a 0
        // a lookahead will now have to wrap around the buffer correctly
        for byte in 1..8 {
            w.write(byte).unwrap();
        }
        w.write(0).unwrap();
        assert_eq!(w.used(), 8);
        assert_ne!(w.tail, 0);
        assert_eq!(w.next_zero(), Some(8));
    }

    #[test]
    fn test_next_zero_empty_buffer() {
        let w = Queue::<10>::new();
        assert_eq!(w.next_zero(), None);
    }

    #[test]
    fn test_drain() {
        let mut w = Queue::<10>::new();
        w.write_all(&[1,2,3,4,5,6,7,8]).unwrap();
        assert_eq!(w.used(), 8);
        assert_eq!(w.drain(), 8);
        assert_eq!(w.used(), 0);
    }
}

// #[derive(Debug, Copy, Clone, Eq, PartialEq)]
// pub enum CobsError {
//     WriteBufferOverflow,
//     Other,
// }
// impl embedded_io::Error for CobsError {
//     fn kind(&self) -> embedded_io::ErrorKind {
//         embedded_io::ErrorKind::Other
//     }
// }

// impl fmt::Display for CobsError {
//     fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
//         write!(f, "{:?}", self)
//     }
// }

// pub struct CobsStreamingDecoder<'a, const N: usize> {
//     source: &'a Consumer<'a, u8, N>,
// }

// impl<'a, const N: usize> CobsStreamingDecoder<'a, N> {
//     pub fn new(source: &'a Consumer<'a, u8, N>) -> Self {
//         CobsStreamingDecoder{source}
//     }
// }

// impl<'a, const N: usize> Iterator for CobsStreamingDecoder<'a, N> {
//     type Item = u8;
//     fn next(&mut self) -> Option<Self::Item> {
//         self.source.dequeue()
//     }
// }


// pub struct CobsStreamingEncoder<'a, const N: usize> {
//     source: &'a Consumer<'a, u8, N>,
// }

// impl<'a, const N: usize> CobsStreamingEncoder<'a, N> {
//     pub fn new(source: &'a Consumer<'a, u8, N>) -> Self {
//         CobsStreamingEncoder{source}
//     }
// }

// impl<'a, const N: usize> embedded_io::Io for CobsStreamingEncoder<'a, N> {
//     // errors can't happen because this buffer
//     // implementation doesn't actually do anything
//     type Error = CobsError;
// }

// impl<'a, const N: usize> Iterator for CobsStreamingEncoder<'a, N> {
//     type Item = u8;
//     fn next(&mut self) -> Option<Self::Item> {
//         self.source.dequeue()
//     }
// }


// impl<'a, const N: usize> Write for CobsStreamingEncoder<'a, N> {
//     fn write(&mut self, buf: &[u8]) -> Result<usize, CobsError> {
//         let mut count = 0;
//         for byte in buf {
//             match self.target.enqueue(*byte) {
//                 Ok(_) => count += 1,
//                 Err(_) => return Err(CobsError::WriteBufferOverflow),
//             }
//         };
//         Ok(count)
//     }

//     fn flush(&mut self) -> Result<(), CobsError> {
//         Ok(())
//     }
// }

// impl<'a, u8, const N: usize> Read for CobsStreamingDecoder<'a, u8, N> {
//     fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
//         match self.input.dequeue() {
//             Some(byte) => Ok(byte),
//             None => Err(CobsError::Other),
//         }
//     }
// }