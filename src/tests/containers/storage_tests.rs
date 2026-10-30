//! Ported from Tests/NatselTests/Containers/StorageTests.swift.

use crate::base::constants::NULL_INDEX;
use crate::containers::bit_set::BitSet;
use crate::containers::dynamic_array::DynamicArray;
use crate::containers::hash_set::{key_hash, HashSet, SetItem};
use crate::containers::id_pool::IDPool;

/// `BitSet.bits` and `BitSet.blockCapacity` are private in the port, so the released-storage check
/// reads them through the public surface: `byte_count()` is `block_capacity * 8`, so zero there
/// means zero `block_capacity`, and zero live bits means no block survives `destroy()`.
#[test]
fn bit_set_lifecycle() {
    // "bit set growth, union, clearing, and reset preserve block boundaries"
    let mut a = BitSet::with_bit_capacity(64);
    let mut b = BitSet::with_bit_capacity(64);
    a.set_bit_count_and_clear(64);
    a.set_bit(63);
    a.set_bit_grow(128);
    assert_eq!(a.block_count(), 3);
    assert!(a.get_bit(63));
    assert!(a.get_bit(128));
    assert!(!a.get_bit(64));
    assert_eq!(a.count_set_bits(), 2);
    b.set_bit_count_and_clear(129);
    b.set_bit(64);
    b.set_bit(128);
    a.in_place_union(&b);
    assert_eq!(a.count_set_bits(), 3);
    a.clear_bit(128);
    assert_eq!(a.count_set_bits(), 2);
    a.set_bit_count_and_clear(1);
    assert_eq!(a.count_set_bits(), 0);
    a.set_bit_count_and_clear(129);
    assert_eq!(a.count_set_bits(), 0);
    a.destroy();
    // Swift also asserted `a.bits == nil`; the storage field is private, and the zero byte count
    // plus zero live bits below is the whole observable state it implied.
    assert_eq!(a.count_set_bits(), 0);
    assert_eq!(a.block_count(), 0);
    assert_eq!(a.byte_count(), 0);
    b.destroy();
}

#[test]
fn probe_chains() {
    // "hash set deletion repairs wraparound probe chains"
    // All keys start in the final bucket, forcing probing to wrap through zero.
    let keys: Vec<u64> = (1u64..=10_000)
        .filter(|key| key_hash(*key) & 15 == 15)
        .take(7)
        .collect();
    let mut set = HashSet::with_capacity(16);
    for key in &keys {
        let observed = !set.add_key(*key);
        assert!(observed);
    }
    for key in &keys {
        let observed = set.add_key(*key);
        assert!(observed);
    }
    assert_eq!(set.count(), 7);
    for key in &keys {
        {
            let observed = set.remove_key(*key);
            assert!(observed);
        }
        assert!(!set.contains_key(*key));
        {
            let observed = !set.remove_key(*key);
            assert!(observed);
        }
        for remaining in keys.iter().filter(|remaining| *remaining > key) {
            assert!(set.contains_hashed_key(*remaining, key_hash(*remaining)));
        }
    }
    assert_eq!(set.count(), 0);
    for key in 1u64..=1024 {
        let observed = !set.add_key(key);
        assert!(observed);
    }
    // Swift read `set.capacity` directly; the port exposes it only through `byte_count()`, which
    // is `capacity * size_of::<SetItem>()`.
    assert_eq!(
        set.byte_count() as usize / core::mem::size_of::<SetItem>(),
        2048
    );
    for key in 1u64..=1024 {
        assert!(set.contains_key(key));
    }
    set.clear();
    assert_eq!(set.count(), 0);
    for key in 1u64..=1024 {
        assert!(!set.contains_key(key));
    }
    set.destroy();
}

#[test]
fn id_recycling() {
    // "ID recycling preserves last-in-first-out order"
    let mut pool = IDPool::create();
    for id in 0..100i32 {
        let observed = pool.alloc() == id;
        assert!(observed);
    }
    for id in [17i32, 3, 99, 42] {
        pool.free(id);
    }
    assert_eq!(pool.id_count(), 96);
    assert_eq!(pool.id_capacity(), 100);
    for id in [42i32, 99, 3, 17, 100] {
        let observed = pool.alloc() == id;
        assert!(observed);
    }
    pool.destroy();
    let observed = pool.alloc() == 0;
    assert!(observed);
}

#[test]
fn dynamic_array() {
    // "dynamic array growth and swap removal preserve indices"
    let mut array: DynamicArray<i32> = DynamicArray::new();
    for value in 0..100i32 {
        array.push(value);
    }
    assert_eq!(array.count(), 100);
    // `capacity` has no accessor; `byte_count()` is `capacity * size_of::<i32>()`.
    assert_eq!(
        array.byte_count() as usize / core::mem::size_of::<i32>(),
        128
    );
    {
        let observed = array.remove_swap(3) == 99;
        assert!(observed);
    }
    assert_eq!(array[3usize], 99);
    {
        let observed = array.remove_swap(98) == NULL_INDEX;
        assert!(observed);
    }
    {
        let observed = array.pop() == 97;
        assert!(observed);
    }
    let tail: [i32; 3] = [500, 501, 502];
    array.append(&tail);
    {
        let observed = array.pop() == 502;
        assert!(observed);
    }
    {
        let observed = array.pop() == 501;
        assert!(observed);
    }
    {
        let observed = array.pop() == 500;
        assert!(observed);
    }
    array.clear();
    assert_eq!(array.count(), 0);
    assert_eq!(
        array.byte_count() as usize / core::mem::size_of::<i32>(),
        128
    );
    array.destroy();
}
