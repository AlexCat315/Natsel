private let qThreshold: UInt = 16

@inline(__always)
private func qSort3(_ a1: UInt, _ a2: UInt, _ a3: UInt, _ less: (UInt, UInt) -> Bool, _ swap: (UInt, UInt) -> Void) {
    if less(a2, a1) {
        if less(a3, a2) {
            swap(a1, a3)
        } else {
            swap(a1, a2)
            if less(a3, a2) {
                swap(a2, a3)
            }
        }
    } else if less(a3, a2) {
        swap(a2, a3)
        if less(a2, a1) {
            swap(a1, a2)
        }
    }
}

@inline(__always)
func quickSort(_ n: Int32, less: (UInt, UInt) -> Bool, swap: (UInt, UInt) -> Void) {
    guard n > 1 else { return }

    var l: UInt = 0
    var r = UInt(n) - 1
    var sp = 0
    var stackL = InlineArray<48, UInt>(repeating: 0)
    var stackR = InlineArray<48, UInt>(repeating: 0)

    while true {
        if r - l + 1 >= qThreshold {
            // Partition [l,r] around a pivot. Afterwards [l,j] are the elements less than or
            // equal to the pivot, while [i,r] are the elements greater than or equal to it.
            let m = l + ((r - l) >> 1)
            qSort3(l + 1, m, r, less, swap)
            swap(l, m)
            var i = l + 1
            var j = r
            while true {
                repeat { i += 1 } while less(i, l)
                repeat { j -= 1 } while less(l, j)
                if i >= j { break }
                swap(i, j)
            }
            // Compensate for the i==j case.
            i = j + 1
            // Put the median to its final place.
            swap(l, j)
            // The median is not part of the left subfile.
            j -= 1

            // Now have two subfiles: [l,j] and [i,r]. Dealing with them depends on which one
            // is bigger. The first subfile is not smaller than the second.
            let (l1, r1, l2, r2) = j - l >= r - i ? (l, j, i, r) : (i, r, l, j)
            if l2 == r2 {
                l = l1
                r = r1
            } else {
                stackL[sp] = l1
                stackR[sp] = r1
                sp += 1
                l = l2
                r = r2
            }
        } else {
            // Insertion sort for small subfiles.
            var qi = l + 1
            while qi <= r {
                var qj = qi
                while qj > l && less(qj, qj - 1) {
                    swap(qj, qj - 1)
                    qj -= 1
                }
                qi += 1
            }

            // Pop subfiles from the stack, until it gets empty.
            if sp == 0 { break }
            sp -= 1
            l = stackL[sp]
            r = stackR[sp]
        }
    }
}
