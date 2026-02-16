use std::collections::HashSet;

use bidirected_adjacency_array::{
    index::{DirectedNodeIndex, GraphIndexInteger, OptionalDirectedNodeIndex},
    io::gfa1::{GfaEdgeData, GfaNodeData},
};
use binary_heap_plus::{BinaryHeap, MinComparator};
use epoch_reset_array::EpochResetArray;
use spqr_tree::decomposition::indices::{BlockIndex, SPQRNodeIndex};

use crate::{
    gfa_graph_extensions::GfaNodeDataExt,
    location::{GfaLocation, GfaNodeOffset},
    location_index::GfaLocationIndex,
    path::{GfaPath, GfaPathLength, OptionalGfaPathLength, PathElement},
    spqr_decomposition_overlay::{OverlayLevel, SPQRDecompositionOverlay},
};

pub struct OverlayDijkstra<
    'graph,
    'spqr,
    'overlay,
    IndexType: GraphIndexInteger,
    NodeData: GfaNodeData,
    EdgeData: GfaEdgeData,
> {
    overlay: &'overlay SPQRDecompositionOverlay<'graph, 'spqr, IndexType, NodeData, EdgeData>,

    /// Open list for Dijkstra's algorithm.
    open_list: BinaryHeap<OpenNode<IndexType>, MinComparator>,

    /// Closed list for Dijkstra's algorithm.
    closed_list: EpochResetArray<DirectedNodeIndex<IndexType>, ClosedNode<IndexType>, u32>,
}

#[derive(Debug, Eq, PartialEq)]
pub(super) struct OpenNode<IndexType: GraphIndexInteger> {
    node: DirectedNodeIndex<IndexType>,
    cost: GfaPathLength<IndexType>,
    predecessor: OptionalDirectedNodeIndex<IndexType>,
}

#[derive(Debug, Clone)]
pub(super) struct ClosedNode<IndexType: GraphIndexInteger> {
    cost: OptionalGfaPathLength<IndexType>,
    predecessor: OptionalDirectedNodeIndex<IndexType>,
}

impl<
    'graph,
    'spqr,
    'overlay,
    IndexType: GraphIndexInteger,
    NodeData: GfaNodeData,
    EdgeData: GfaEdgeData,
> OverlayDijkstra<'graph, 'spqr, 'overlay, IndexType, NodeData, EdgeData>
{
    pub fn new(
        overlay: &'overlay SPQRDecompositionOverlay<'graph, 'spqr, IndexType, NodeData, EdgeData>,
    ) -> Self {
        let directed_node_count = overlay.graph().node_count() * 2;
        Self {
            overlay,
            open_list: BinaryHeap::new_min(),
            closed_list: EpochResetArray::new(ClosedNode::new_none(), directed_node_count.into()),
        }
    }

    pub fn shortest_paths(
        &mut self,
        source: GfaLocation<IndexType>,
        targets: &impl GfaLocationIndex<IndexType>,
    ) -> Vec<GfaPath<IndexType>> {
        let (maximum_level, active_blocks, active_spqr_nodes) = {
            let component = self
                .overlay
                .spqr_decomposition()
                .node_component_index(source.node().into_bidirected());
            let source_blocks: HashSet<_> = self
                .overlay
                .spqr_decomposition()
                .node_block_indices(source.node().into_bidirected())
                .collect();
            let source_spqr_nodes: HashSet<_> = self
                .overlay
                .spqr_decomposition()
                .node_spqr_node_indices(source.node().into_bidirected())
                .collect();

            let mut maximum_level = OverlayLevel::SPQRNode;
            let mut active_blocks = source_blocks.clone();
            let mut active_spqr_nodes = source_spqr_nodes.clone();

            for target in targets.iter_targets() {
                if self
                    .overlay
                    .spqr_decomposition()
                    .node_component_index(target.node().into_bidirected())
                    != component
                {
                    // Targets in different components cannot be reached, so no need to raise the level for them.
                    continue;
                }

                active_blocks.extend(
                    self.overlay
                        .spqr_decomposition()
                        .node_block_indices(target.node().into_bidirected()),
                );
                active_spqr_nodes.extend(
                    self.overlay
                        .spqr_decomposition()
                        .node_spqr_node_indices(target.node().into_bidirected()),
                );

                if !self
                    .overlay
                    .spqr_decomposition()
                    .node_block_indices(target.node().into_bidirected())
                    .any(|block| source_blocks.contains(&block))
                {
                    maximum_level = maximum_level.max(OverlayLevel::BlockCutTree);
                }

                if !self
                    .overlay
                    .spqr_decomposition()
                    .node_spqr_node_indices(target.node().into_bidirected())
                    .any(|spqr_node| source_spqr_nodes.contains(&spqr_node))
                {
                    maximum_level = maximum_level.max(OverlayLevel::SPQRTree);
                }
            }

            (maximum_level, active_blocks, active_spqr_nodes)
        };

        self.open_list.clear();
        self.closed_list.reset();

        // By convention, we insert only the successors of the source node into the open list.
        // Therefore, each path will have one "missing" node at the start which we will handle separately when backtracking.
        // This is to enable circular paths in the case where the shortest path to a target starts from the same node but with a higher offset.
        self.expand_node(
            source.node(),
            GfaPathLength::from_usize(0),
            maximum_level,
            &active_blocks,
            &active_spqr_nodes,
        );
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
            self.expand_node(
                open_node.node,
                open_node.cost,
                maximum_level,
                &active_blocks,
                &active_spqr_nodes,
            );
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
                    let cost = cost
                        + self
                            .overlay
                            .graph()
                            .node_data(source.node().into_bidirected())
                            .len()
                        - source.offset().into_length()
                        + target.offset().into_length();
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

    fn expand_node(
        &mut self,
        from_node: DirectedNodeIndex<IndexType>,
        from_cost: GfaPathLength<IndexType>,
        maximum_level: OverlayLevel,
        active_blocks: &HashSet<BlockIndex<IndexType>>,
        active_spqr_nodes: &HashSet<SPQRNodeIndex<IndexType>>,
    ) {
        if self
            .overlay
            .spqr_decomposition()
            .is_cut_node(from_node.into_bidirected())
            && maximum_level >= OverlayLevel::BlockCutTree
        {
            // Expand node on BC-tree level.
            let overlay_node = self
                .overlay
                .directed_graph_node_to_overlay_node(from_node)
                .unwrap();

            for outgoing_edge in self
                .overlay
                .iter_outgoing_block_cut_tree_edges(overlay_node)
            {
                let to_overlay_node = outgoing_edge.to();
                let to_node = self
                    .overlay
                    .directed_overlay_node_to_graph_node(to_overlay_node);
                let cost = from_cost
                    + self
                        .overlay
                        .overlay()
                        .directed_edge_data(outgoing_edge.index())
                        .data()
                        .length();
                let predecessor = from_node.into();

                let closed_node = self.closed_list.get(to_node);
                if let Some(closed_cost) = closed_node.cost.into_option() {
                    // Node already closed, ensure that we did not find a shorter path.
                    assert!(closed_cost <= cost);
                } else {
                    self.open_list.push(OpenNode {
                        node: to_overlay_node,
                        cost,
                        predecessor,
                    });
                }
            }
        }

        if self
            .overlay
            .spqr_decomposition()
            .has_incident_virtual_edge(from_node.into_bidirected())
            && active_blocks.contains(
                &self
                    .overlay
                    .spqr_decomposition()
                    .node_block_indices(from_node.into_bidirected())
                    .next()
                    .unwrap(),
            )
            && maximum_level >= OverlayLevel::SPQRTree
        {
            // Expand node on SPQR-tree level.
            let overlay_node = self
                .overlay
                .directed_graph_node_to_overlay_node(from_node)
                .unwrap();

            for outgoing_edge in self.overlay.iter_outgoing_spqr_tree_edges(overlay_node) {
                let to_overlay_node = outgoing_edge.to();
                let to_node = self
                    .overlay
                    .directed_overlay_node_to_graph_node(to_overlay_node);

                if !active_blocks.contains(
                    &self
                        .overlay
                        .spqr_decomposition()
                        .node_block_indices(to_node.into_bidirected())
                        .next()
                        .unwrap(),
                ) {
                    // Skip edges that point into an inactive block.
                    continue;
                }

                let cost = from_cost
                    + self
                        .overlay
                        .overlay()
                        .directed_edge_data(outgoing_edge.index())
                        .data()
                        .length();
                let predecessor = from_node.into();

                let closed_node = self.closed_list.get(to_node);
                if let Some(closed_cost) = closed_node.cost.into_option() {
                    // Node already closed, ensure that we did not find a shorter path.
                    assert!(closed_cost <= cost);
                } else {
                    self.open_list.push(OpenNode {
                        node: to_overlay_node,
                        cost,
                        predecessor,
                    });
                }
            }
        }

        if active_spqr_nodes.contains(
            &self
                .overlay
                .spqr_decomposition()
                .node_spqr_node_indices(from_node.into_bidirected())
                .next()
                .unwrap(),
        ) && maximum_level >= OverlayLevel::SPQRNode
        {
            // Expand node on original graph level.
            for outgoing_edge in self.overlay.iter_outgoing_spqr_node_edges(from_node) {
                debug_assert_eq!(
                    self.overlay
                        .graph()
                        .directed_edge_data(outgoing_edge.index())
                        .data()
                        .overlap(),
                    0,
                    "Only GFA graphs with blunt-ended (i.e. zero overlap) edges are supported. Use for example https://github.com/vgteam/GetBlunted to bluntify your graph.",
                );

                let to_node = outgoing_edge.to();

                if !active_spqr_nodes.contains(
                    &self
                        .overlay
                        .spqr_decomposition()
                        .node_spqr_node_indices(to_node.into_bidirected())
                        .next()
                        .unwrap(),
                ) {
                    // Skip edges that point into an inactive SPQR-tree node.
                    continue;
                }

                let cost = from_cost
                    + self
                        .overlay
                        .graph()
                        .node_data(from_node.into_bidirected())
                        .len();
                let predecessor = from_node.into();

                let closed_node = self.closed_list.get(to_node);
                if let Some(closed_cost) = closed_node.cost.into_option() {
                    // Node already closed, ensure that we did not find a shorter path.
                    assert!(closed_cost <= cost);
                } else {
                    self.open_list.push(OpenNode {
                        node: to_node,
                        cost,
                        predecessor,
                    });
                }
            }
        }
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
                .overlay
                .graph()
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
            self.overlay
                .graph()
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

impl<IndexType: GraphIndexInteger> ClosedNode<IndexType> {
    pub fn new_none() -> Self {
        Self {
            cost: OptionalGfaPathLength::new_none(),
            predecessor: OptionalDirectedNodeIndex::new_none(),
        }
    }
}
