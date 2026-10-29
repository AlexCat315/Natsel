//! Sweep-based overlapping bounds pairs over double precision world boxes.

use crate::math::world_aabb::WorldAABB;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BoundsPair {
    pub a: usize,
    pub b: usize,
}

#[derive(Clone, Debug, Default)]
pub struct BoundsPairResult {
    pub pairs: Vec<BoundsPair>,
    pub candidate_count: usize,
}

/// Sweep along the widest world axis, then reject on all three double precision intervals.
/// Output is in input-index order, independent of sort ties and world-coordinate magnitude.
pub fn overlapping_bounds_pairs(bounds: &[WorldAABB]) -> BoundsPairResult {
    if bounds.len() <= 1 {
        return BoundsPairResult::default();
    }

    let mut lower = bounds[0].lower_bound;
    let mut upper = bounds[0].upper_bound;
    for box_ in &bounds[1..] {
        lower = crate::math::types::Pos::new(
            lower.x.min(box_.lower_bound.x),
            lower.y.min(box_.lower_bound.y),
            lower.z.min(box_.lower_bound.z),
        );
        upper = crate::math::types::Pos::new(
            upper.x.max(box_.upper_bound.x),
            upper.y.max(box_.upper_bound.y),
            upper.z.max(box_.upper_bound.z),
        );
    }

    let span = [upper.x - lower.x, upper.y - lower.y, upper.z - lower.z];
    let axis = if span[1] > span[0] {
        if span[2] > span[1] { 2 } else { 1 }
    } else if span[2] > span[0] {
        2
    } else {
        0
    };

    let component = |p: crate::math::types::Pos| -> f64 {
        match axis {
            0 => p.x,
            1 => p.y,
            _ => p.z,
        }
    };

    let mut order: Vec<usize> = (0..bounds.len()).collect();
    order.sort_by(|&i, &j| {
        let a = component(bounds[i].lower_bound);
        let b = component(bounds[j].lower_bound);
        if a == b {
            i.cmp(&j)
        } else {
            a.partial_cmp(&b).unwrap_or(std::cmp::Ordering::Equal)
        }
    });

    let mut result = BoundsPairResult::default();
    for k in 0..order.len() {
        let a = order[k];
        let mut l = k + 1;
        while l < order.len()
            && component(bounds[order[l]].lower_bound) <= component(bounds[a].upper_bound)
        {
            let b = order[l];
            result.candidate_count += 1;
            if bounds[a].overlaps(&bounds[b]) {
                result.pairs.push(BoundsPair {
                    a: a.min(b),
                    b: a.max(b),
                });
            }
            l += 1;
        }
    }

    result.pairs.sort_by(|p, q| {
        if p.a == q.a {
            p.b.cmp(&q.b)
        } else {
            p.a.cmp(&q.a)
        }
    });
    result
}
