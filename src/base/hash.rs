//! rapidhash V3 (compact, fast) - <https://github.com/Nicoshev/rapidhash>
//! Copyright (C) 2025 Nicolas De Carli, BSD 2-Clause License.

use crate::base::constants::HASH_INIT;

const RAPID_SECRET: [u64; 8] = [
    0x2d35_8dcc_aa6c_78a5,
    0x8bb8_4b93_962e_acc9,
    0x4b33_a62e_d433_d4a3,
    0x4d5a_2da5_1de1_aa47,
    0xa076_1d64_78bd_642f,
    0xe703_7ed1_a0b4_28db,
    0x90ed_1765_281c_388c,
    0xaaaa_aaaa_aaaa_aaaa,
];

/// 64x64 -> 128 bit multiply. `a` receives the low bits and `b` the high bits.
#[inline(always)]
fn rapid_mum(a: &mut u64, b: &mut u64) {
    let r = (*a as u128) * (*b as u128);
    *a = r as u64;
    *b = (r >> 64) as u64;
}

#[inline(always)]
fn rapid_mix(a: u64, b: u64) -> u64 {
    let mut a = a;
    let mut b = b;
    rapid_mum(&mut a, &mut b);
    a ^ b
}

#[inline(always)]
unsafe fn rapid_read64(p: *const u8) -> u64 {
    u64::from_le(unsafe { std::ptr::read_unaligned(p.cast::<u64>()) })
}

#[inline(always)]
unsafe fn rapid_read32(p: *const u8) -> u64 {
    u32::from_le(unsafe { std::ptr::read_unaligned(p.cast::<u32>()) }) as u64
}

/// Compute a 64-bit rapidhash of `key`.
pub fn rapidhash(key: &[u8]) -> u64 {
    let secret = RAPID_SECRET;
    let mut p = key.as_ptr();
    let mut seed: u64 = 0;
    seed ^= rapid_mix(seed ^ secret[2], secret[1]);
    let mut a: u64;
    let mut b: u64;
    let mut i = key.len();

    unsafe {
        if key.len() <= 16 {
            if key.len() >= 4 {
                seed ^= key.len() as u64;
                if key.len() >= 8 {
                    a = rapid_read64(p);
                    b = rapid_read64(p.add(key.len() - 8));
                } else {
                    a = rapid_read32(p);
                    b = rapid_read32(p.add(key.len() - 4));
                }
            } else if !key.is_empty() {
                a = ((*p as u64) << 45) | (*p.add(key.len() - 1) as u64);
                b = *p.add(key.len() >> 1) as u64;
            } else {
                a = 0;
                b = 0;
            }
        } else {
            if key.len() > 112 {
                let mut see1 = seed;
                let mut see2 = seed;
                let mut see3 = seed;
                let mut see4 = seed;
                let mut see5 = seed;
                let mut see6 = seed;
                loop {
                    seed = rapid_mix(rapid_read64(p) ^ secret[0], rapid_read64(p.add(8)) ^ seed);
                    see1 = rapid_mix(
                        rapid_read64(p.add(16)) ^ secret[1],
                        rapid_read64(p.add(24)) ^ see1,
                    );
                    see2 = rapid_mix(
                        rapid_read64(p.add(32)) ^ secret[2],
                        rapid_read64(p.add(40)) ^ see2,
                    );
                    see3 = rapid_mix(
                        rapid_read64(p.add(48)) ^ secret[3],
                        rapid_read64(p.add(56)) ^ see3,
                    );
                    see4 = rapid_mix(
                        rapid_read64(p.add(64)) ^ secret[4],
                        rapid_read64(p.add(72)) ^ see4,
                    );
                    see5 = rapid_mix(
                        rapid_read64(p.add(80)) ^ secret[5],
                        rapid_read64(p.add(88)) ^ see5,
                    );
                    see6 = rapid_mix(
                        rapid_read64(p.add(96)) ^ secret[6],
                        rapid_read64(p.add(104)) ^ see6,
                    );
                    p = p.add(112);
                    i -= 112;
                    if i <= 112 {
                        break;
                    }
                }
                seed ^= see1;
                see2 ^= see3;
                see4 ^= see5;
                seed ^= see6;
                see2 ^= see4;
                seed ^= see2;
            }

            if i > 16 {
                seed = rapid_mix(rapid_read64(p) ^ secret[2], rapid_read64(p.add(8)) ^ seed);
                if i > 32 {
                    seed = rapid_mix(
                        rapid_read64(p.add(16)) ^ secret[2],
                        rapid_read64(p.add(24)) ^ seed,
                    );
                    if i > 48 {
                        seed = rapid_mix(
                            rapid_read64(p.add(32)) ^ secret[1],
                            rapid_read64(p.add(40)) ^ seed,
                        );
                        if i > 64 {
                            seed = rapid_mix(
                                rapid_read64(p.add(48)) ^ secret[1],
                                rapid_read64(p.add(56)) ^ seed,
                            );
                            if i > 80 {
                                seed = rapid_mix(
                                    rapid_read64(p.add(64)) ^ secret[2],
                                    rapid_read64(p.add(72)) ^ seed,
                                );
                                if i > 96 {
                                    seed = rapid_mix(
                                        rapid_read64(p.add(80)) ^ secret[1],
                                        rapid_read64(p.add(88)) ^ seed,
                                    );
                                }
                            }
                        }
                    }
                }
            }
            // `p` has advanced past every 112-byte block, so the remainder `i` may be smaller
            // than 16 even though these two reads are still in bounds: Swift's `p + i - 16`
            // performs pointer arithmetic on a signed `Int`, which simply steps backwards. A
            // `usize` subtraction would underflow here and panic (or wrap and read out of
            // bounds), so the offset is computed in `isize` like Swift's.
            a = rapid_read64(p.offset(i as isize - 16)) ^ i as u64;
            b = rapid_read64(p.offset(i as isize - 8));
        }
    }

    a ^= secret[1];
    b ^= seed;
    rapid_mum(&mut a, &mut b);
    rapid_mix(a ^ secret[7], b ^ secret[1] ^ i as u64)
}

pub fn hash64_non_zero(bytes: &[u8]) -> u64 {
    if bytes.is_empty() {
        return 1;
    }
    let h = rapidhash(bytes);
    if h == 0 { 1 } else { h }
}

/// djb2 hash for determinism testing, folded 8 bytes per iteration to shorten the dependency
/// chain. Words are read little-endian so the value is identical across endianness.
pub fn hash(hash: u32, data: &[u8]) -> u32 {
    let mut result = hash;
    let count = data.len();
    let mut i = 0;

    while i + 8 <= count {
        let word = u64::from_le_bytes([
            data[i],
            data[i + 1],
            data[i + 2],
            data[i + 3],
            data[i + 4],
            data[i + 5],
            data[i + 6],
            data[i + 7],
        ]);
        result = (result << 5)
            .wrapping_add(result)
            .wrapping_add(word as u32);
        result = (result << 5)
            .wrapping_add(result)
            .wrapping_add((word >> 32) as u32);
        i += 8;
    }

    while i < count {
        result = (result << 5)
            .wrapping_add(result)
            .wrapping_add(data[i] as u32);
        i += 1;
    }

    result
}

/// The initial value for [`hash`].
pub const HASH_INIT_VALUE: u32 = HASH_INIT;

/// Hash an arbitrary plain-old-data value by its raw bytes.
pub fn hash_bytes<T: Copy>(hash: u32, value: &T) -> u32 {
    let bytes = unsafe {
        std::slice::from_raw_parts(value as *const T as *const u8, std::mem::size_of::<T>())
    };
    self::hash(hash, bytes)
}

/// Hash an arbitrary plain-old-data value.
pub fn hash_value<T: Copy>(value: &T) -> u32 {
    hash_bytes(HASH_INIT, value)
}

/// Hash a slice of plain-old-data values.
pub fn hash_slice<T: Copy>(hash: u32, values: &[T]) -> u32 {
    let bytes = unsafe {
        std::slice::from_raw_parts(values.as_ptr() as *const u8, std::mem::size_of_val(values))
    };
    self::hash(hash, bytes)
}
