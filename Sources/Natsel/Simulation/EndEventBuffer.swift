// End events can be queued by destruction between steps and published by the next step.
struct EndEventBuffer<Event> {
    var pending: [Event] = []
    private(set) var published: [Event] = []

    mutating func publish() {
        swap(&pending, &published)
        pending.removeAll(keepingCapacity: true)
    }
}
