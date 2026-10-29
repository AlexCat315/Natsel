//! Bit manipulation helpers.

#[inline(always)]
pub fn ctz32(block: u32) -> u32 {
    block.trailing_zeros()
}

#[inline(always)]
pub fn clz32(value: u32) -> u32 {
    value.leading_zeros()
}

#[inline(always)]
pub fn ctz64(block: u64) -> u32 {
    block.trailing_zeros()
}

#[inline(always)]
pub fn pop_count64(block: u64) -> i32 {
    block.count_ones() as i32
}

#[inline(always)]
pub fn is_power_of_2(x: i32) -> bool {
    (x & (x.wrapping_sub(1))) == 0
}

pub fn bounding_power_of_2(x: i32) -> i32 {
    if x <= 1 {
        return 1;
    }
    32 - clz32((x as u32).wrapping_sub(1)) as i32
}

pub fn round_up_power_of_2(x: i32) -> i32 {
    if x <= 1 {
        return 1;
    }
    1 << (32 - clz32((x as u32).wrapping_sub(1)) as i32)
}

pub fn lower_power_of_2_exponent(x: i32) -> i32 {
    crate::base::core::b3_assert(x > 0);
    let clz = clz32(x as u32) as i32;
    31 - clz
}

/// Swap two values.
#[inline(always)]
pub fn b3_swap<T>(x: &mut T, y: &mut T) {
    std::mem::swap(x, y);
}
