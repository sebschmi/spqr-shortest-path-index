use bidirected_adjacency_array::{
    graph::BidirectedAdjacencyArray,
    index::{DirectedNodeIndex, GraphIndexInteger},
};

use crate::{path::OptionalGfaPathLength, spqr_decomposition_overlay::OverlayEdgeData};

pub fn overlay_shortest_path_length<IndexType: GraphIndexInteger, NodeData>(
    _graph: &BidirectedAdjacencyArray<IndexType, NodeData, OverlayEdgeData<IndexType>>,
    _source: DirectedNodeIndex<IndexType>,
    _target: DirectedNodeIndex<IndexType>,
) -> OptionalGfaPathLength<IndexType> {
    todo!()
}
