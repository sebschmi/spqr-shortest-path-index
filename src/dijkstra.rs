use std::collections::HashMap;

use bidirected_adjacency_array::{
    graph::BidirectedAdjacencyArray,
    index::GraphIndexInteger,
    io::gfa1::{GfaEdgeData, GfaNodeData},
};
use binary_heap_plus::BinaryHeap;

use crate::{location::GfaLocation, path::GfaPath};

pub fn shortest_path<IndexType: GraphIndexInteger, NodeData: GfaNodeData, EdgeData: GfaEdgeData>(
    graph: &BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>,
    source: GfaLocation<IndexType>,
    target: GfaLocation<IndexType>,
) -> GfaPath<IndexType> {
    let mut open_list = BinaryHeap::new_min();
    let mut closed_list = HashMap::new();

    todo!()
}
