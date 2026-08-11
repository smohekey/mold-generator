use std::fmt;

const MAX_POSITIONS: usize = 1_024;

/// Evenly distributes positions between two fixed end setbacks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EndAnchoredDistribution {
    /// Distance from each end of the input range to its nearest position.
    pub end_setback: f64,
    /// Largest permitted distance between adjacent positions.
    pub maximum_spacing: f64,
    /// Smallest number of positions to return.
    pub minimum_positions: usize,
}

impl EndAnchoredDistribution {
    /// Returns end-anchored positions over `range`, including both setback points.
    pub fn positions(self, range: (f64, f64)) -> Result<Vec<f64>, EndAnchoredDistributionError> {
        if !range.0.is_finite()
            || !range.1.is_finite()
            || range.1 <= range.0
            || !self.end_setback.is_finite()
            || self.end_setback < 0.0
            || !self.maximum_spacing.is_finite()
            || self.maximum_spacing <= 0.0
            || self.minimum_positions == 0
        {
            return Err(EndAnchoredDistributionError::InvalidSettings);
        }

        let usable_start = range.0 + self.end_setback;
        let usable_end = range.1 - self.end_setback;
        if usable_end < usable_start || (usable_end == usable_start && self.minimum_positions > 1) {
            return Err(EndAnchoredDistributionError::InsufficientLength);
        }
        let usable_length = usable_end - usable_start;
        let spacing_intervals = (usable_length / self.maximum_spacing).ceil() as usize;
        let count = self
            .minimum_positions
            .max(spacing_intervals.saturating_add(1));
        if count > MAX_POSITIONS {
            return Err(EndAnchoredDistributionError::TooManyPositions);
        }

        Ok((0..count)
            .map(|index| {
                let fraction = if count == 1 {
                    0.5
                } else {
                    index as f64 / (count - 1) as f64
                };
                usable_start + usable_length * fraction
            })
            .collect())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EndAnchoredDistributionError {
    InvalidSettings,
    InsufficientLength,
    TooManyPositions,
}

impl fmt::Display for EndAnchoredDistributionError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSettings => {
                formatter.write_str("invalid end-anchored distribution settings")
            }
            Self::InsufficientLength => {
                formatter.write_str("range is too short for the requested end setbacks")
            }
            Self::TooManyPositions => {
                formatter.write_str("distribution produces too many positions")
            }
        }
    }
}

impl std::error::Error for EndAnchoredDistributionError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn anchors_both_ends_and_limits_interior_spacing() {
        let positions = EndAnchoredDistribution {
            end_setback: 12.0,
            maximum_spacing: 100.0,
            minimum_positions: 2,
        }
        .positions((0.0, 500.0))
        .unwrap();

        assert_eq!(positions.first(), Some(&12.0));
        assert_eq!(positions.last(), Some(&488.0));
        assert_eq!(positions.len(), 6);
        assert!(positions.windows(2).all(|pair| pair[1] - pair[0] <= 100.0));
    }

    #[test]
    fn rejects_a_range_that_cannot_hold_distinct_end_anchors() {
        let result = EndAnchoredDistribution {
            end_setback: 12.0,
            maximum_spacing: 100.0,
            minimum_positions: 2,
        }
        .positions((0.0, 24.0));

        assert_eq!(
            result,
            Err(EndAnchoredDistributionError::InsufficientLength)
        );
    }
}
