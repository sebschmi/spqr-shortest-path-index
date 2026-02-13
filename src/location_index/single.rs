use std::iter;

use bidirected_adjacency_array::index::{DirectedNodeIndex, GraphIndexInteger};

use crate::{
    dijkstra::GfaShortestPathSource,
    location::{GfaLocation, GfaNodeOffset},
    location_index::{GfaLocationIndex, GfaLocationIndexKind},
    path::{GfaPathLength, OptionalGfaPathLength},
};

pub struct SingleGfaLocationIndex<IndexType> {
    location: GfaLocation<IndexType>,
    costs: OptionalGfaPathLength<IndexType>,
    kind: GfaLocationIndexKind,
}

impl<IndexType: GraphIndexInteger> SingleGfaLocationIndex<IndexType> {
    pub fn new_source(source: GfaShortestPathSource<IndexType>) -> Self {
        Self {
            location: source.location(),
            costs: source.cost().into(),
            kind: GfaLocationIndexKind::Sources,
        }
    }

    pub fn new_target(target: GfaLocation<IndexType>) -> Self {
        Self {
            location: target,
            costs: OptionalGfaPathLength::new_none(),
            kind: GfaLocationIndexKind::Targets,
        }
    }
}

impl<IndexType: GraphIndexInteger> GfaLocationIndex<IndexType>
    for SingleGfaLocationIndex<IndexType>
{
    fn is_sources(&self) -> bool {
        matches!(self.kind, GfaLocationIndexKind::Sources)
    }

    fn is_targets(&self) -> bool {
        matches!(self.kind, GfaLocationIndexKind::Targets)
    }

    fn contains(&self, node: DirectedNodeIndex<IndexType>) -> bool {
        self.location.node() == node
    }

    fn offset(&self, node: DirectedNodeIndex<IndexType>) -> GfaNodeOffset<IndexType> {
        assert!(self.contains(node), "Node is not contained in the index");
        self.location.offset()
    }

    fn cost(&self, node: DirectedNodeIndex<IndexType>) -> GfaPathLength<IndexType> {
        assert!(self.contains(node), "Node is not a source");
        self.costs.into_option().expect("Locations are not sources")
    }

    fn iter_sources(&self) -> impl Iterator<Item = GfaShortestPathSource<IndexType>> {
        iter::once(GfaShortestPathSource::new(
            self.location,
            self.costs.expect("Locations are not sources"),
        ))
    }

    fn iter_targets(&self) -> impl Iterator<Item = GfaLocation<IndexType>> {
        iter::once(self.location)
    }

    fn len(&self) -> usize {
        1
    }
}
