//! End-event double buffer.

/// End events can be queued by destruction between steps and published by the next step.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct EndEventBuffer<Event> {
    /// Events queued since the last publish.
    pub pending: Vec<Event>,
    /// Events the last publish made readable.
    published: Vec<Event>,
}

impl<Event> EndEventBuffer<Event> {
    /// `private(set) var published`
    pub fn published(&self) -> &[Event] {
        &self.published
    }

    /// Swap the queues: the events queued since the last step become the published ones, and the
    /// drained queue keeps its capacity for the next step.
    pub fn publish(&mut self) {
        std::mem::swap(&mut self.pending, &mut self.published);
        self.pending.clear();
    }
}
