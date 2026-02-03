use bidirected_adjacency_array::{
    graph::BidirectedAdjacencyArray,
    index::{DirectedNodeIndex, GraphIndexInteger},
    io::gfa1::GfaNodeData,
};
use optional_numeric_index::implement_generic_index;

use crate::gfa_graph_extensions::GfaNodeDataExt;

implement_generic_index!(pub GfaNodeOffset, pub OptionalGfaNodeOffset);

#[derive(Debug, Clone, Copy)]
pub struct GfaLocation<IndexType> {
    node: DirectedNodeIndex<IndexType>,
    offset: GfaNodeOffset<IndexType>,
}

impl<IndexType: GraphIndexInteger> GfaLocation<IndexType> {
    pub fn new(node: DirectedNodeIndex<IndexType>, offset: GfaNodeOffset<IndexType>) -> Self {
        Self { node, offset }
    }

    pub fn node(&self) -> DirectedNodeIndex<IndexType> {
        self.node
    }

    pub fn offset(&self) -> GfaNodeOffset<IndexType> {
        self.offset
    }

    pub fn invert<EdgeData>(
        self,
        graph: &BidirectedAdjacencyArray<IndexType, impl GfaNodeData, EdgeData>,
    ) -> Self {
        Self {
            node: self.node.invert(),
            offset: graph.node_data(self.node.into_bidirected()).len() - self.offset,
        }
    }
}

impl<IndexType: GraphIndexInteger> GfaNodeOffset<IndexType> {
    pub fn into_length(self) -> crate::path::GfaPathLength<IndexType> {
        crate::path::GfaPathLength::from_raw(self.into_raw())
    }
}
