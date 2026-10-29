//! Deterministic quicksort operating on indices through closures.

const Q_THRESHOLD: usize = 16;

#[inline(always)]
fn qsort3<L: Fn(usize, usize) -> bool, S: FnMut(usize, usize)>(
    a1: usize,
    a2: usize,
    a3: usize,
    less: &L,
    swap: &mut S,
) {
    if less(a2, a1) {
        if less(a3, a2) {
            swap(a1, a3);
        } else {
            swap(a1, a2);
            if less(a3, a2) {
                swap(a2, a3);
            }
        }
    } else if less(a3, a2) {
        swap(a2, a3);
        if less(a2, a1) {
            swap(a1, a2);
        }
    }
}

/// Sort `0..n` using the supplied comparison and swap operations.
pub fn quick_sort<L, S>(n: i32, less: L, mut swap: S)
where
    L: Fn(usize, usize) -> bool,
    S: FnMut(usize, usize),
{
    if n <= 1 {
        return;
    }

    let mut l: usize = 0;
    let mut r = n as usize - 1;
    let mut stack_l: Vec<usize> = Vec::with_capacity(48);
    let mut stack_r: Vec<usize> = Vec::with_capacity(48);
    stack_l.resize(48, 0);
    stack_r.resize(48, 0);
    let mut sp = 0usize;

    loop {
        if r - l + 1 >= Q_THRESHOLD {
            let m = l + ((r - l) >> 1);
            qsort3(l + 1, m, r, &less, &mut swap);
            swap(l, m);
            let mut i = l + 1;
            let mut j = r;
            loop {
                loop {
                    i += 1;
                    if !less(i, l) {
                        break;
                    }
                }
                loop {
                    j -= 1;
                    if !less(l, j) {
                        break;
                    }
                }
                if i >= j {
                    break;
                }
                swap(i, j);
            }
            i = j + 1;
            swap(l, j);
            j -= 1;

            let (l1, r1, l2, r2) = if j - l >= r - i {
                (l, j, i, r)
            } else {
                (i, r, l, j)
            };
            if l2 == r2 {
                l = l1;
                r = r1;
            } else {
                stack_l[sp] = l1;
                stack_r[sp] = r1;
                sp += 1;
                l = l2;
                r = r2;
            }
        } else {
            let mut qi = l + 1;
            while qi <= r {
                let mut qj = qi;
                while qj > l && less(qj, qj - 1) {
                    swap(qj, qj - 1);
                    qj -= 1;
                }
                qi += 1;
            }

            if sp == 0 {
                break;
            }
            sp -= 1;
            l = stack_l[sp];
            r = stack_r[sp];
        }
    }
}

/// Sort a slice using the supplied comparison.
pub fn quick_sort_by<T>(data: &mut [T], less: impl Fn(&T, &T) -> bool) {
    let n = data.len() as i32;
    if n <= 1 {
        return;
    }
    // The closures borrow `data`; use raw pointers for the swap while the comparison reads.
    let ptr = data.as_mut_ptr();
    let len = data.len();
    let comparator = |i: usize, j: usize| -> bool {
        debug_assert!(i < len && j < len);
        let a = unsafe { &*ptr.add(i) };
        let b = unsafe { &*ptr.add(j) };
        less(a, b)
    };
    let swapper = |i: usize, j: usize| unsafe {
        if i != j {
            std::ptr::swap(ptr.add(i), ptr.add(j));
        }
    };
    quick_sort(n, comparator, swapper);
}
