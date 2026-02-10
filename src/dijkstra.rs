use std::collections::HashMap;

use bidirected_adjacency_array::{
    graph::BidirectedAdjacencyArray,
    index::{DirectedNodeIndex, GraphIndexInteger, OptionalDirectedNodeIndex},
    io::gfa1::{GfaEdgeData, GfaNodeData},
};
use binary_heap_plus::BinaryHeap;

use crate::{
    gfa_graph_extensions::GfaNodeDataExt,
    location::{GfaLocation, GfaNodeOffset},
    path::{GfaPath, GfaPathLength, OptionalGfaPathLength, PathElement},
    spqr_decomposition_overlay::OverlayEdgeData,
};

#[cfg(test)]
mod tests;

#[derive(Debug, Eq, PartialEq)]
struct OpenNode<IndexType: GraphIndexInteger> {
    node: DirectedNodeIndex<IndexType>,
    cost: GfaPathLength<IndexType>,
    predecessor: OptionalDirectedNodeIndex<IndexType>,
}

struct ClosedNode<IndexType> {
    cost: GfaPathLength<IndexType>,
    predecessor: OptionalDirectedNodeIndex<IndexType>,
}

/// Compute the shortest path between two sequence indices in two (possibly equal) GFA nodes.
///
/// If `target` is before `source` in the same node, then the path will be a cycle.
pub fn gfa_shortest_path<
    IndexType: GraphIndexInteger,
    NodeData: GfaNodeData,
    EdgeData: GfaEdgeData,
>(
    graph: &BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>,
    source: GfaLocation<IndexType>,
    target: GfaLocation<IndexType>,
) -> Option<GfaPath<IndexType>> {
    if source.node() == target.node() && source.offset() <= target.offset() {
        // If source is before or at target in the same node, then the shortest path is within the node (because we assume edges to be blunt-ended).
        return Some(GfaPath::new(
            vec![PathElement::new(
                source.node(),
                source.offset(),
                target.offset(),
            )],
            target.offset() - source.offset(),
        ));
    }

    // We search in reverse such that we don't need to invert the path after backtracking.
    let (source, target) = (target.invert(graph), source.invert(graph));

    let mut open_list = BinaryHeap::new_min();
    let mut closed_list = HashMap::<DirectedNodeIndex<IndexType>, ClosedNode<IndexType>>::new();
    let is_circular_special_case =
        source.node() == target.node() && source.offset() > target.offset();

    if is_circular_special_case {
        // If target is before source in the same node, then we have to expand the source manually, because Dijkstra's algorithm does not support circular paths.
        for outgoing_edge in graph.iter_outgoing_edges(source.node()) {
            debug_assert_eq!(
                graph
                    .directed_edge_data(outgoing_edge.index())
                    .data()
                    .overlap(),
                0,
                "Only GFA graphs with blunt-ended (i.e. zero overlap) edges are supported. Use for example https://github.com/vgteam/GetBlunted to bluntify your graph.",
            );

            let node = outgoing_edge.to();
            let cost = graph.node_data(source.node().into_bidirected()).len();
            let predecessor = OptionalDirectedNodeIndex::new_none();

            open_list.push(OpenNode {
                node,
                cost,
                predecessor,
            });
        }
    } else {
        open_list.push(OpenNode::new_root(source.node()));
    }

    while let Some(open_node) = open_list.pop() {
        // Close node.
        closed_list.insert(
            open_node.node,
            ClosedNode {
                cost: open_node.cost,
                predecessor: open_node.predecessor,
            },
        );

        if open_node.node == target.node() {
            // Target found, backtrack path and compute actual cost.
            let cost =
                open_node.cost + target.offset().into_length() - source.offset().into_length();

            // Initialise path with target node.
            let mut path = vec![PathElement::new_inverted(
                target.node(),
                GfaNodeOffset::from_usize(0),
                target.offset(),
                graph,
            )];

            // Collect nodes.
            let mut current_node = open_node.node;
            while let Some(predecessor) = closed_list
                .get(&current_node)
                .unwrap()
                .predecessor
                .into_option()
            {
                let offset = GfaNodeOffset::from_usize(0);
                let limit = graph
                    .node_data(current_node.into_bidirected())
                    .len()
                    .into_offset();
                path.push(PathElement::new_inverted(predecessor, offset, limit, graph));
                current_node = predecessor;
            }

            // Adjust the offset of the source node.
            if is_circular_special_case {
                // If target is before source in the same node, then we have to manually add the source node as we started the search from its successors.
                path.push(PathElement::new_inverted(
                    source.node(),
                    source.offset(),
                    graph
                        .node_data(source.node().into_bidirected())
                        .len()
                        .into_offset(),
                    graph,
                ));
            } else {
                path.last_mut().unwrap().decrease_limit(source.offset());
            }

            // Return path.
            return Some(GfaPath::new(path, cost));
        }

        // Expand node.
        for outgoing_edge in graph.iter_outgoing_edges(open_node.node) {
            debug_assert_eq!(
                graph
                    .directed_edge_data(outgoing_edge.index())
                    .data()
                    .overlap(),
                0,
                "Only GFA graphs with blunt-ended (i.e. zero overlap) edges are supported. Use for example https://github.com/vgteam/GetBlunted to bluntify your graph.",
            );

            let node = outgoing_edge.to();
            let cost = open_node.cost + graph.node_data(open_node.node.into_bidirected()).len();
            let predecessor = open_node.node.into();

            if let Some(closed_node) = closed_list.get(&node) {
                assert!(cost >= closed_node.cost);
            } else {
                open_list.push(OpenNode {
                    node,
                    cost,
                    predecessor,
                });
            }
        }
    }

    // Terminated without finding the target.
    None
}

pub fn overlay_shortest_path_length<IndexType: GraphIndexInteger, NodeData>(
    _graph: &BidirectedAdjacencyArray<IndexType, NodeData, OverlayEdgeData<IndexType>>,
    _source: DirectedNodeIndex<IndexType>,
    _target: DirectedNodeIndex<IndexType>,
) -> OptionalGfaPathLength<IndexType> {
    todo!()
}

impl<IndexType: GraphIndexInteger> OpenNode<IndexType> {
    fn new_root(node: DirectedNodeIndex<IndexType>) -> Self {
        Self {
            node,
            cost: GfaPathLength::from_usize(0),
            predecessor: OptionalDirectedNodeIndex::new_none(),
        }
    }
}

impl<IndexType: GraphIndexInteger> Ord for OpenNode<IndexType> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.cost
            .cmp(&other.cost)
            .then_with(|| self.node.cmp(&other.node))
            .then_with(|| self.predecessor.cmp(&other.predecessor))
    }
}

impl<IndexType: GraphIndexInteger> PartialOrd for OpenNode<IndexType> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
