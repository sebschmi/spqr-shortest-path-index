use bidirected_adjacency_array::index::DirectedNodeIndex;

use crate::{
    dijkstra::gfa::GfaShortestPathSource,
    location::{GfaLocation, GfaNodeOffset},
    path::GfaPathLength,
};

pub mod multi;
pub mod single;

enum GfaLocationIndexKind {
    Sources,
    Targets,
}

pub trait GfaLocationIndex<IndexType> {
    fn is_sources(&self) -> bool;

    fn is_targets(&self) -> bool;

    fn contains(&self, node: DirectedNodeIndex<IndexType>) -> bool;

    fn offset(&self, node: DirectedNodeIndex<IndexType>) -> GfaNodeOffset<IndexType>;

    /// Always panics if [`Self::is_sources`] returns `false`.
    fn cost(&self, node: DirectedNodeIndex<IndexType>) -> GfaPathLength<IndexType>;

    fn iter_sources(&self) -> impl Iterator<Item = GfaShortestPathSource<IndexType>>;

    fn iter_targets(&self) -> impl Iterator<Item = GfaLocation<IndexType>>;

    fn len(&self) -> usize;

    fn is_empty(&self) -> bool {
        self.len() == 0
    }
}
