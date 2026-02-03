use std::ops::Sub;

use bidirected_adjacency_array::index::{DirectedNodeIndex, GraphIndexInteger};
use optional_numeric_index::implement_generic_index;

use crate::location::GfaNodeOffset;

implement_generic_index!(pub GfaPathLength, pub OptionalGfaPathLength);

pub struct GfaPath<IndexType> {
    path: Vec<PathElement<IndexType>>,
    length: GfaPathLength<IndexType>,
}

#[derive(Debug, Clone, Copy)]
pub struct PathElement<IndexType> {
    node: DirectedNodeIndex<IndexType>,
    offset: GfaNodeOffset<IndexType>,
    limit: GfaNodeOffset<IndexType>,
}

impl<IndexType: GraphIndexInteger> GfaPath<IndexType> {
    pub fn new(path: Vec<PathElement<IndexType>>, length: GfaPathLength<IndexType>) -> Self {
        Self { path, length }
    }

    pub fn iter(&self) -> impl Iterator<Item = PathElement<IndexType>> {
        self.path.iter().copied()
    }

    pub fn length(&self) -> GfaPathLength<IndexType> {
        self.length
    }
}

impl<IndexType: GraphIndexInteger> PathElement<IndexType> {
    pub fn new(
        node: DirectedNodeIndex<IndexType>,
        offset: GfaNodeOffset<IndexType>,
        limit: GfaNodeOffset<IndexType>,
    ) -> Self {
        Self {
            node,
            offset,
            limit,
        }
    }

    pub fn node(&self) -> DirectedNodeIndex<IndexType> {
        self.node
    }

    pub fn offset(&self) -> GfaNodeOffset<IndexType> {
        self.offset
    }

    pub fn limit(&self) -> GfaNodeOffset<IndexType> {
        self.limit
    }

    pub fn length(&self) -> GfaPathLength<IndexType> {
        self.limit - self.offset
    }
}

impl<IndexType: GraphIndexInteger> Sub for GfaNodeOffset<IndexType> {
    type Output = GfaPathLength<IndexType>;

    fn sub(self, rhs: Self) -> Self::Output {
        GfaPathLength::from_raw(self.into_raw() - rhs.into_raw())
    }
}
