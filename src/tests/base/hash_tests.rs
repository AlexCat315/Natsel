//! Ported from Tests/NatselTests/Base/HashTests.swift.

use crate::base::hash::{hash, hash64_non_zero, rapidhash};

#[test]
fn rapid_hash_repeatability() {
    // "rapidhash repeats across aligned and unaligned input boundaries"
    let bytes: Vec<u8> = (0..4098u32).map(|i| (i * 131 + 17) as u8).collect();
    let copy: Vec<u8> = bytes.iter().rev().rev().copied().collect();
    let mut unaligned: Vec<u8> = Vec::with_capacity(bytes.len() + 1);
    unaligned.push(0u8);
    unaligned.extend_from_slice(&bytes);

    let mut lengths: Vec<usize> = (0..=256).collect();
    lengths.extend_from_slice(&[257, 511, 512, 1023, 1024, 2047, 2048, 4095, 4096, 4097]);

    // `withUnsafeBufferPointer` is identity here: the slices own the same bytes the Swift
    // pointers addressed, so a sub-slice starting at `shifted + 1` reproduces the unaligned read.
    let shifted = &unaligned[1..];
    for length in lengths {
        let expected = rapidhash(&bytes[..length]);
        assert_eq!(rapidhash(&copy[..length]), expected);
        assert_eq!(rapidhash(&shifted[..length]), expected);
        let non_zero = hash64_non_zero(&bytes[..length]);
        let wanted = if length == 0 { 1 } else { expected };
        assert_eq!(non_zero, wanted);
    }
}

#[test]
fn determinism_hash() {
    // "folded determinism hash preserves word order, tails, and seed"
    let bytes: [u8; 17] = [
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
    ];
    for seed in [0u32, 5381, u32::MAX] {
        for count in 0..=bytes.len() {
            let mut expected = seed;
            let mut offset = 0usize;
            while offset + 8 <= count {
                for half in 0..2usize {
                    let j = offset + half * 4;
                    let word = (bytes[j] as u32)
                        | (bytes[j + 1] as u32) << 8
                        | (bytes[j + 2] as u32) << 16
                        | (bytes[j + 3] as u32) << 24;
                    expected = expected.wrapping_mul(33).wrapping_add(word);
                }
                offset += 8;
            }
            for byte in &bytes[offset..count] {
                expected = expected.wrapping_mul(33).wrapping_add(*byte as u32);
            }
            assert_eq!(hash(seed, &bytes[..count]), expected);
        }
    }
}
