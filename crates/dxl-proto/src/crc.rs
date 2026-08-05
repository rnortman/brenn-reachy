//! CRC-16 over a Protocol 2.0 frame.
//!
//! Polynomial `0x8005`, initial value 0, non-reflected input and output, no
//! final xor, computed over every byte from the first header byte through the
//! last parameter — i.e. the whole frame except the two CRC bytes themselves.
//!
//! The 256-entry table is built from the polynomial at compile time rather than
//! transcribed, so there is no literal that can drift away from the definition.

use crate::frame::CRC_LEN;

const POLYNOMIAL: u16 = 0x8005;

/// Byte-at-a-time table, entry `i` being the CRC contribution of a top byte `i`.
static TABLE: [u16; 256] = build_table();

const fn build_table() -> [u16; 256] {
    let mut table = [0u16; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut crc = (i as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ POLYNOMIAL
            } else {
                crc << 1
            };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

/// CRC of `data`, which must be the frame bytes preceding the CRC field.
#[must_use]
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        let index = ((crc >> 8) ^ u16::from(byte)) & 0xFF;
        crc = (crc << 8) ^ TABLE[index as usize];
    }
    crc
}

/// True when the trailing two bytes of `frame` are its own CRC. A frame shorter
/// than the CRC field is never valid.
#[must_use]
pub fn crc_matches(frame: &[u8]) -> bool {
    if frame.len() < CRC_LEN {
        return false;
    }
    let split = frame.len() - CRC_LEN;
    let expected = u16::from_le_bytes([frame[split], frame[split + 1]]);
    crc16(&frame[..split]) == expected
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Bit-serial reference: the polynomial definition, unaccelerated.
    fn crc16_bitwise(data: &[u8]) -> u16 {
        let mut crc: u16 = 0;
        for &byte in data {
            crc ^= u16::from(byte) << 8;
            for _ in 0..8 {
                crc = if crc & 0x8000 != 0 {
                    (crc << 1) ^ POLYNOMIAL
                } else {
                    crc << 1
                };
            }
        }
        crc
    }

    #[test]
    fn table_matches_the_polynomial() {
        for (i, entry) in TABLE.iter().enumerate() {
            assert_eq!(*entry, crc16_bitwise(&[i as u8]), "table entry {i}");
        }
    }

    #[test]
    fn table_and_bitwise_agree_on_payloads() {
        let mut data = Vec::new();
        for i in 0..=255u8 {
            data.push(i.wrapping_mul(31).wrapping_add(7));
            assert_eq!(crc16(&data), crc16_bitwise(&data));
        }
    }

    /// The published check value for CRC-16/UMTS (poly 0x8005, init 0,
    /// non-reflected, no xorout), which is the variant Protocol 2.0 uses.
    #[test]
    fn check_value() {
        assert_eq!(crc16(b"123456789"), 0xFEE8);
    }

    #[test]
    fn crc_matches_validates_a_real_frame() {
        // Ping instruction to ID 2 (Robotis Protocol 2.0 documented example).
        let frame = [0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x03, 0x00, 0x01, 0x19, 0x72];
        assert!(crc_matches(&frame));

        let mut broken = frame;
        broken[8] ^= 0x01;
        assert!(!crc_matches(&broken));
    }

    #[test]
    fn crc_matches_rejects_a_runt() {
        assert!(!crc_matches(&[0x00]));
    }
}
