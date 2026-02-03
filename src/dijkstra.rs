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
    path::{GfaPath, GfaPathLength, PathElement},
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

pub fn shortest_path<IndexType: GraphIndexInteger, NodeData: GfaNodeData, EdgeData: GfaEdgeData>(
    graph: &BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>,
    source: GfaLocation<IndexType>,
    target: GfaLocation<IndexType>,
) -> Option<GfaPath<IndexType>> {
    // We search in reverse such that we don't need to invert the path after backtracking.
    let (source, target) = (target.invert(graph), source.invert(graph));

    let mut open_list = BinaryHeap::new_min();
    let mut closed_list = HashMap::<DirectedNodeIndex<IndexType>, ClosedNode<IndexType>>::new();
    open_list.push(OpenNode::new_root(source.node()));

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
            // While backtracking, always assume that nodes start at offset 0.
            let mut current_node = open_node.node;
            while let Some(predecessor) = closed_list
                .get(&current_node)
                .unwrap()
                .predecessor
                .into_option()
            {
                let offset = GfaNodeOffset::from_usize(0);
                let limit = closed_list.get(&current_node).unwrap().cost
                    - closed_list.get(&predecessor).unwrap().cost;
                path.push(PathElement::new_inverted(
                    predecessor,
                    offset,
                    limit.into_offset(),
                    graph,
                ));
                current_node = predecessor;
            }

            // Adjust offset from source node.
            let mut remaining_offset = source.offset();
            for path_element in path.iter_mut().rev() {
                remaining_offset = path_element.decrease_limit(remaining_offset);
                if remaining_offset.into_raw().is_zero() {
                    break;
                }
            }
            assert!(
                remaining_offset.into_raw().is_zero(),
                "Found a path of negative length.",
            );

            // Return path.
            return Some(GfaPath::new(path, cost));
        }

        // Expand node.
        for outgoing_edge in graph.iter_outgoing_edges(open_node.node) {
            let node = outgoing_edge.to();
            let cost = open_node.cost + graph.node_data(open_node.node.into_bidirected()).len()
                - GfaPathLength::from_usize(
                    graph
                        .directed_edge_data(outgoing_edge.index())
                        .data()
                        .overlap()
                        .into(),
                );
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
