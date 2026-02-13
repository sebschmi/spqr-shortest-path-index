#[cfg(test)]
mod tests;

use bidirected_adjacency_array::{
    graph::BidirectedAdjacencyArray,
    index::{DirectedNodeIndex, GraphIndexInteger, OptionalDirectedNodeIndex},
    io::gfa1::{GfaEdgeData, GfaNodeData},
};
use binary_heap_plus::{BinaryHeap, MinComparator};
use epoch_reset_array::EpochResetArray;

use crate::{
    gfa_graph_extensions::GfaNodeDataExt,
    location::{GfaLocation, GfaNodeOffset},
    location_index::GfaLocationIndex,
    path::{GfaPath, GfaPathLength, OptionalGfaPathLength, PathElement},
};

pub struct GfaShortestPathSource<IndexType> {
    location: GfaLocation<IndexType>,
    cost: GfaPathLength<IndexType>,
}

pub struct GfaDijkstra<
    'graph,
    IndexType: GraphIndexInteger,
    NodeData: GfaNodeData,
    EdgeData: GfaEdgeData,
> {
    graph: &'graph BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>,
    open_list: BinaryHeap<OpenNode<IndexType>, MinComparator>,
    closed_list: EpochResetArray<DirectedNodeIndex<IndexType>, ClosedNode<IndexType>, u32>,
}

#[derive(Debug, Eq, PartialEq)]
struct OpenNode<IndexType: GraphIndexInteger> {
    node: DirectedNodeIndex<IndexType>,
    cost: GfaPathLength<IndexType>,
    predecessor: OptionalDirectedNodeIndex<IndexType>,
}

#[derive(Debug, Clone)]
struct ClosedNode<IndexType: GraphIndexInteger> {
    cost: OptionalGfaPathLength<IndexType>,
    predecessor: OptionalDirectedNodeIndex<IndexType>,
}

impl<'graph, IndexType: GraphIndexInteger, NodeData: GfaNodeData, EdgeData: GfaEdgeData>
    GfaDijkstra<'graph, IndexType, NodeData, EdgeData>
{
    pub fn new(graph: &'graph BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>) -> Self {
        let directed_node_count = graph.node_count() * 2;
        Self {
            graph,
            open_list: BinaryHeap::new_min(),
            closed_list: EpochResetArray::new(
                ClosedNode {
                    cost: OptionalGfaPathLength::new_none(),
                    predecessor: OptionalDirectedNodeIndex::new_none(),
                },
                directed_node_count.into(),
            ),
        }
    }

    /// Compute the shortest paths from the source to all given targets.
    ///
    /// If there is a target that is before the source in the same node, then this target's path may be a cycle.
    pub fn shortest_path(
        &mut self,
        source: GfaLocation<IndexType>,
        targets: &impl GfaLocationIndex<IndexType>,
    ) -> Vec<GfaPath<IndexType>> {
        assert!(targets.is_targets());

        self.open_list.clear();
        self.closed_list.reset();

        // By convention, we insert only the successors of the source node into the open list.
        // Therefore, each path will have one "missing" node at the start which we will handle separately when backtracking.
        // This is to enable circular paths in the case where the shortest path to a target starts from the same node but with a higher offset.
        for outgoing_edge in self.graph.iter_outgoing_edges(source.node()) {
            debug_assert_eq!(
                self.graph
                    .directed_edge_data(outgoing_edge.index())
                    .data()
                    .overlap(),
                0,
                "Only GFA graphs with blunt-ended (i.e. zero overlap) edges are supported. Use for example https://github.com/vgteam/GetBlunted to bluntify your graph.",
            );

            let node = outgoing_edge.to();
            let cost = self.graph.node_data(source.node().into_bidirected()).len()
                - source.offset().into_length();
            let predecessor = OptionalDirectedNodeIndex::new_none();

            self.open_list.push(OpenNode {
                node,
                cost,
                predecessor,
            });
        }

        let mut closed_target_counter = 0;

        while let Some(open_node) = self.open_list.pop()
            && closed_target_counter < targets.len()
        {
            // Close node.
            self.closed_list.set(
                open_node.node,
                ClosedNode {
                    cost: open_node.cost.into(),
                    predecessor: open_node.predecessor,
                },
            );

            if targets.contains(open_node.node) {
                closed_target_counter += 1;
            }

            // Expand node.
            for outgoing_edge in self.graph.iter_outgoing_edges(open_node.node) {
                debug_assert_eq!(
                    self.graph
                        .directed_edge_data(outgoing_edge.index())
                        .data()
                        .overlap(),
                    0,
                    "Only GFA graphs with blunt-ended (i.e. zero overlap) edges are supported. Use for example https://github.com/vgteam/GetBlunted to bluntify your graph.",
                );

                let node = outgoing_edge.to();
                let cost =
                    open_node.cost + self.graph.node_data(open_node.node.into_bidirected()).len();
                let predecessor = open_node.node.into();

                let closed_node = self.closed_list.get_mut(node);
                if let Some(closed_cost) = closed_node.cost.into_option() {
                    // Node already closed, ensure that we did not find a shorter path.
                    assert!(closed_cost <= cost);
                } else {
                    self.open_list.push(OpenNode {
                        node,
                        cost,
                        predecessor,
                    });
                }
            }
        }

        // All targets found, backtrack paths and compute actual cost.
        targets
            .iter_targets()
            .filter_map(|target| {
                let mut path = (target.node() == source.node()
                    && target.offset() >= source.offset())
                .then(|| {
                    GfaPath::new(
                        vec![PathElement::new(
                            source.node(),
                            source.offset(),
                            target.offset(),
                        )],
                        target.offset() - source.offset(),
                    )
                });

                if let Some(cost) = self.closed_list.get(target.node()).cost.into_option() {
                    let cost = cost + target.offset().into_length();
                    let outer_path = self.backtrack_path(source, target, cost);
                    if path
                        .as_ref()
                        .map(|p| outer_path.length() < p.length())
                        .unwrap_or(true)
                    {
                        path = Some(outer_path);
                    }
                }

                path
            })
            .collect()
    }

    fn backtrack_path(
        &self,
        source: GfaLocation<IndexType>,
        target: GfaLocation<IndexType>,
        cost: GfaPathLength<IndexType>,
    ) -> GfaPath<IndexType> {
        // Initialise path with target node.
        let mut path = vec![PathElement::new(
            target.node(),
            GfaNodeOffset::from_usize(0),
            target.offset(),
        )];

        // Collect nodes.
        let mut current_node = target.node();
        while let Some(predecessor) = self.closed_list.get(current_node).predecessor.into_option() {
            let offset = GfaNodeOffset::from_usize(0);
            let limit = self
                .graph
                .node_data(current_node.into_bidirected())
                .len()
                .into_offset();
            path.push(PathElement::new(predecessor, offset, limit));
            current_node = predecessor;
        }

        // Manually add the source node as we started the search from its successors.
        path.push(PathElement::new(
            source.node(),
            source.offset(),
            self.graph
                .node_data(source.node().into_bidirected())
                .len()
                .into_offset(),
        ));

        // Return path.
        path.reverse();
        GfaPath::new(path, cost)
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

impl<IndexType: GraphIndexInteger> GfaShortestPathSource<IndexType> {
    pub fn new(location: GfaLocation<IndexType>, cost: GfaPathLength<IndexType>) -> Self {
        if cost != GfaPathLength::from_usize(0) {
            assert_eq!(
                location.offset(),
                GfaNodeOffset::from_usize(0),
                "If the path starts in the middle of a node, "
            );
        }

        Self { location, cost }
    }

    pub fn location(&self) -> GfaLocation<IndexType> {
        self.location
    }

    pub fn node(&self) -> DirectedNodeIndex<IndexType> {
        self.location.node()
    }

    pub fn offset(&self) -> GfaNodeOffset<IndexType> {
        self.location.offset()
    }

    pub fn cost(&self) -> GfaPathLength<IndexType> {
        self.cost
    }
}
