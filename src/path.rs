use std::{
    fmt::Debug,
    ops::{Add, AddAssign, Sub, SubAssign},
};

use bidirected_adjacency_array::{
    graph::BidirectedAdjacencyArray,
    index::{DirectedNodeIndex, GraphIndexInteger},
    io::gfa1::GfaNodeData,
};
use optional_numeric_index::implement_generic_index;

use crate::{gfa_graph_extensions::GfaNodeDataExt, location::GfaNodeOffset};

implement_generic_index!(pub GfaPathLength, pub OptionalGfaPathLength);

pub struct GfaPath<IndexType> {
    path: Vec<PathElement<IndexType>>,
    length: GfaPathLength<IndexType>,
}

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
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

impl<IndexType: GraphIndexInteger> GfaPathLength<IndexType> {
    pub fn into_offset(self) -> GfaNodeOffset<IndexType> {
        GfaNodeOffset::from_raw(self.into_raw())
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

    pub fn new_inverted<EdgeData>(
        node: DirectedNodeIndex<IndexType>,
        offset: GfaNodeOffset<IndexType>,
        limit: GfaNodeOffset<IndexType>,
        graph: &BidirectedAdjacencyArray<IndexType, impl GfaNodeData, EdgeData>,
    ) -> Self {
        Self {
            node: node.invert(),
            offset: graph.node_data(node.into_bidirected()).len() - limit,
            limit: graph.node_data(node.into_bidirected()).len() - offset,
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

    /// Push the path element to the left by the given decrement.
    ///
    /// The decrement is subtracted from the limit, and the offset is adjusted to not exceed the new limit.
    /// The amount of shift applied to the offset is returned.
    pub(crate) fn decrease_limit(
        &mut self,
        decrement: GfaNodeOffset<IndexType>,
    ) -> GfaNodeOffset<IndexType> {
        self.limit -= decrement;
        if self.offset > self.limit {
            let shift = self.offset - self.limit;
            self.offset = self.limit;
            shift.into_offset()
        } else {
            GfaNodeOffset::from_usize(0)
        }
    }
}

impl<IndexType: GraphIndexInteger> Sub for GfaNodeOffset<IndexType> {
    type Output = GfaPathLength<IndexType>;

    fn sub(self, rhs: Self) -> Self::Output {
        GfaPathLength::from_raw(self.into_raw() - rhs.into_raw())
    }
}

impl<IndexType: GraphIndexInteger> AddAssign for GfaNodeOffset<IndexType> {
    fn add_assign(&mut self, rhs: Self) {
        *self = GfaNodeOffset::from_raw(self.into_raw() + rhs.into_raw());
    }
}

impl<IndexType: GraphIndexInteger> SubAssign for GfaNodeOffset<IndexType> {
    fn sub_assign(&mut self, rhs: Self) {
        *self = GfaNodeOffset::from_raw(self.into_raw() - rhs.into_raw());
    }
}

impl<IndexType: GraphIndexInteger> Sub<GfaNodeOffset<IndexType>> for GfaPathLength<IndexType> {
    type Output = GfaNodeOffset<IndexType>;

    fn sub(self, rhs: GfaNodeOffset<IndexType>) -> Self::Output {
        GfaNodeOffset::from_raw(self.into_raw() - rhs.into_raw())
    }
}

impl<IndexType: GraphIndexInteger> Add for GfaPathLength<IndexType> {
    type Output = GfaPathLength<IndexType>;

    fn add(self, rhs: Self) -> Self::Output {
        GfaPathLength::from_raw(self.into_raw() + rhs.into_raw())
    }
}

impl<IndexType: GraphIndexInteger> Sub for GfaPathLength<IndexType> {
    type Output = GfaPathLength<IndexType>;

    fn sub(self, rhs: Self) -> Self::Output {
        GfaPathLength::from_raw(self.into_raw() - rhs.into_raw())
    }
}

impl<IndexType: GraphIndexInteger> Debug for GfaPath<IndexType> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "GfaPath(length: {}, path: [", self.length)?;

        for (i, element) in self.path.iter().enumerate() {
            if i > 0 {
                write!(f, ", ")?;
            }
            write!(
                f,
                "{}[{}..{}]",
                element.node(),
                element.offset(),
                element.limit()
            )?;
        }

        write!(f, "])")
    }
}
