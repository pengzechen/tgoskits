//! Repeated edge-interrupt delivery semantics.

/// Delivery state of an interrupt already represented by the guest backend.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum EdgeDeliveryState {
    Idle,
    Pending,
    Active,
    PendingAndActive,
}

/// Action required when another edge arrives for the same interrupt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum RepeatedEdgeAction {
    AlreadyPending,
    MarkPendingWhileActive,
}

/// Preserves a repeated edge while the guest is servicing the prior one.
pub(crate) const fn repeated_edge_action(state: EdgeDeliveryState) -> RepeatedEdgeAction {
    match state {
        EdgeDeliveryState::Active => RepeatedEdgeAction::MarkPendingWhileActive,
        EdgeDeliveryState::Idle
        | EdgeDeliveryState::Pending
        | EdgeDeliveryState::PendingAndActive => RepeatedEdgeAction::AlreadyPending,
    }
}

// Keep every state in the architecture-neutral compile boundary. This table
// also makes additions fail to compile until their repeated-edge action is set.
const _: [RepeatedEdgeAction; 4] = [
    repeated_edge_action(EdgeDeliveryState::Idle),
    repeated_edge_action(EdgeDeliveryState::Pending),
    repeated_edge_action(EdgeDeliveryState::Active),
    repeated_edge_action(EdgeDeliveryState::PendingAndActive),
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edge_arriving_while_active_remains_pending_after_eoi() {
        assert_eq!(
            repeated_edge_action(EdgeDeliveryState::Active),
            RepeatedEdgeAction::MarkPendingWhileActive,
        );
    }

    #[test]
    fn edge_does_not_duplicate_an_already_pending_interrupt() {
        assert_eq!(
            repeated_edge_action(EdgeDeliveryState::Pending),
            RepeatedEdgeAction::AlreadyPending,
        );
        assert_eq!(
            repeated_edge_action(EdgeDeliveryState::PendingAndActive),
            RepeatedEdgeAction::AlreadyPending,
        );
    }
}
