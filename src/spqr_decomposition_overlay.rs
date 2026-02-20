use std::{collections::HashSet, iter};

use bidirected_adjacency_array::{
    graph::{BidirectedAdjacencyArray, BidirectedEdge, DirectedEdge},
    index::{
        DirectedEdgeIndex, DirectedNodeIndex, GraphIndexInteger, NodeIndex,
        OptionalDirectedNodeIndex, OptionalNodeIndex,
    },
    io::gfa1::{GfaEdgeData, GfaNodeData},
};
use spqr_tree::decomposition::SPQRDecomposition;
use tagged_vec::TaggedVec;

use crate::{
    dijkstra::GfaDijkstra, gfa_graph_extensions::GfaNodeDataExt, location::GfaLocation,
    location_index::single::SingleGfaLocationIndex, path::GfaPathLength,
};

#[cfg(feature = "binary-io")]
mod binary_io;
pub mod dijkstra;

static DEBUG: bool = false;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SPQRDecompositionOverlay<
    'graph,
    'spqr,
    IndexType: GraphIndexInteger,
    NodeData: GfaNodeData,
    EdgeData: GfaEdgeData,
> {
    graph: &'graph BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>,
    spqr_decomposition:
        &'spqr SPQRDecomposition<'graph, BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>>,

    /// A two-level overlay of the graph where the lower level are the SPQR trees and the upper level is the block-cut tree of the graph.
    ///
    /// Contains copies of all cut nodes and nodes that are incident to virtual edges in the SPQR decomposition.
    ///
    /// The edges represent the shortest paths between the nodes as if the missing nodes were contracted (see e.g. contraction hierarchies).
    /// However, edges between cut nodes may skip over nodes that are not cut nodes, because they represent the upper level of the overlay.
    overlay:
        BidirectedAdjacencyArray<IndexType, OverlayNodeData<IndexType>, OverlayEdgeData<IndexType>>,

    /// The overlay graph stores edges for the BC tree and SPQR tree overlays together.
    /// First, it stores the SPQR tree edges, and afterwards the BC tree edges.
    /// This vector stores the offsets of the BC tree edges, so that they can be distinguished from the BC tree edges.
    block_cut_overlay_edge_offsets:
        TaggedVec<DirectedNodeIndex<IndexType>, DirectedEdgeIndex<IndexType>>,

    /// Maps node indices in the original graph to node indices in the overlay graph, if they exist.
    graph_to_overlay_node_map: TaggedVec<NodeIndex<IndexType>, OptionalNodeIndex<IndexType>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct OverlayNodeData<IndexType> {
    original_node: NodeIndex<IndexType>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct OverlayEdgeData<IndexType> {
    /// Length of the shortest path between the offsets zero on both nodes in the original graph.
    length: GfaPathLength<IndexType>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum OverlayLevel {
    SPQRNode,
    SPQRTree,
    BlockCutTree,
}

impl<'graph, 'spqr, IndexType: GraphIndexInteger, NodeData: GfaNodeData, EdgeData: GfaEdgeData>
    SPQRDecompositionOverlay<'graph, 'spqr, IndexType, NodeData, EdgeData>
{
    pub fn new(
        graph: &'graph BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>,
        spqr_decomposition: &'spqr SPQRDecomposition<
            'graph,
            BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>,
        >,
    ) -> Self {
        let mut nodes = TaggedVec::new();
        let mut edges = TaggedVec::new();
        let mut graph_to_overlay_node_map = TaggedVec::from_iter(iter::repeat_n(
            OptionalNodeIndex::new_none(),
            graph.node_count(),
        ));
        let mut dijkstra = GfaDijkstra::new(graph);

        // Create nodes.
        for node_index in graph.iter_nodes() {
            if spqr_decomposition.is_cut_node(node_index)
                || spqr_decomposition.has_incident_virtual_edge(node_index)
            {
                nodes.push(OverlayNodeData {
                    original_node: node_index,
                });
                graph_to_overlay_node_map[node_index] =
                    OptionalNodeIndex::from_usize(nodes.len() - 1);
            }
        }

        let mut block_cut_overlay_edge_offsets = TaggedVec::from_iter(iter::repeat_n(
            DirectedEdgeIndex::from_usize(0),
            graph.node_count() * 2,
        ));

        // Create edges for each SPQR node.
        for component_index in spqr_decomposition.iter_component_indices() {
            for (block_index, _block) in
                spqr_decomposition.iter_blocks_in_component(component_index)
            {
                let mut existing_edges = HashSet::new();
                for (_spqr_node_index, spqr_node) in
                    spqr_decomposition.iter_spqr_nodes_in_block(block_index)
                {
                    let nodes_with_incident_virtual_edges: HashSet<_> = spqr_node
                        .iter_incident_spqr_edges()
                        .flat_map(|spqr_edge_index| {
                            let spqr_edge = spqr_decomposition.spqr_edge(spqr_edge_index);
                            let (a, b) = spqr_edge.virtual_edge();
                            [a, b]
                        })
                        .collect();
                    let nodes_with_incident_virtual_edges = nodes_with_incident_virtual_edges
                        .into_iter()
                        .collect::<Vec<_>>();

                    for (offset, from_node_index) in nodes_with_incident_virtual_edges
                        .iter()
                        .copied()
                        .enumerate()
                    {
                        Self::create_overlay_self_loops(
                            from_node_index,
                            &mut dijkstra,
                            &graph_to_overlay_node_map,
                            Some(&mut block_cut_overlay_edge_offsets),
                            &mut edges,
                        );

                        for to_node_index in nodes_with_incident_virtual_edges
                            .iter()
                            .copied()
                            .skip(offset + 1)
                        {
                            let (from_node_index, to_node_index) =
                                if from_node_index < to_node_index {
                                    (from_node_index, to_node_index)
                                } else {
                                    (to_node_index, from_node_index)
                                };
                            if existing_edges.contains(&(from_node_index, to_node_index)) {
                                continue;
                            } else {
                                existing_edges.insert((from_node_index, to_node_index));
                            }

                            Self::create_overlay_edges_between(
                                from_node_index,
                                to_node_index,
                                &mut dijkstra,
                                &graph_to_overlay_node_map,
                                Some(&mut block_cut_overlay_edge_offsets),
                                &mut edges,
                            );
                        }
                    }
                }
            }
        }

        // Create edges for each block.
        for component_index in spqr_decomposition.iter_component_indices() {
            for (_block_index, block) in
                spqr_decomposition.iter_blocks_in_component(component_index)
            {
                for (offset, from_cut_node_index) in block.iter_cut_nodes().enumerate() {
                    let from_node_index =
                        spqr_decomposition.cut_node_index_to_node_index(from_cut_node_index);
                    Self::create_overlay_self_loops(
                        from_node_index,
                        &mut dijkstra,
                        &graph_to_overlay_node_map,
                        None,
                        &mut edges,
                    );

                    for to_cut_node_index in block.iter_cut_nodes().skip(offset + 1) {
                        let to_node_index =
                            spqr_decomposition.cut_node_index_to_node_index(to_cut_node_index);

                        Self::create_overlay_edges_between(
                            from_node_index,
                            to_node_index,
                            &mut dijkstra,
                            &graph_to_overlay_node_map,
                            None,
                            &mut edges,
                        );
                    }
                }
            }
        }

        let overlay = BidirectedAdjacencyArray::new(nodes, edges);

        Self {
            graph,
            spqr_decomposition,
            overlay,
            block_cut_overlay_edge_offsets,
            graph_to_overlay_node_map,
        }
    }

    /// Create the overlay edges between the plus and minus orientations of the given nodes.
    fn create_overlay_edges_between(
        from_node_index: NodeIndex<IndexType>,
        to_node_index: NodeIndex<IndexType>,
        dijkstra: &mut GfaDijkstra<IndexType, NodeData, EdgeData>,
        graph_to_overlay_node_map: &impl std::ops::Index<
            NodeIndex<IndexType>,
            Output = OptionalNodeIndex<IndexType>,
        >,
        mut block_cut_overlay_edge_offsets: Option<
            &mut TaggedVec<DirectedNodeIndex<IndexType>, DirectedEdgeIndex<IndexType>>,
        >,
        edges: &mut impl Extend<BidirectedEdge<IndexType, OverlayEdgeData<IndexType>>>,
    ) {
        let from_overlay_index = graph_to_overlay_node_map[from_node_index]
            .expect("From node must have an overlay node.");
        let to_overlay_index =
            graph_to_overlay_node_map[to_node_index].expect("To node must have an overlay node.");
        let from_node_length = dijkstra.graph().node_data(from_node_index).len();

        if let Some(shortest_path_length_plus_plus) = dijkstra
            .shortest_paths(
                GfaLocation::new(
                    from_node_index.into_directed_forward(),
                    from_node_length.into_offset(),
                ),
                &SingleGfaLocationIndex::new_target(GfaLocation::new(
                    to_node_index.into_directed_forward(),
                    0.into(),
                )),
            )
            .values()
            .map(|path| path.length())
            .next()
        {
            if DEBUG {
                println!(
                    "Inserting ++ overlay edge from {} to {} with cost {}",
                    from_node_index.into_directed_forward(),
                    to_node_index.into_directed_forward(),
                    shortest_path_length_plus_plus,
                );
            }

            edges.extend(std::iter::once(BidirectedEdge::new(
                from_overlay_index.into_directed_forward(),
                to_overlay_index.into_directed_forward(),
                OverlayEdgeData {
                    length: shortest_path_length_plus_plus,
                },
            )));

            if let Some(block_cut_overlay_edge_offsets) = block_cut_overlay_edge_offsets.as_mut() {
                block_cut_overlay_edge_offsets[from_overlay_index.into_directed_forward()] =
                    DirectedEdgeIndex::from_usize(
                        block_cut_overlay_edge_offsets[from_overlay_index.into_directed_forward()]
                            .into_usize()
                            + 1,
                    );
                block_cut_overlay_edge_offsets[to_overlay_index.into_directed_reverse()] =
                    DirectedEdgeIndex::from_usize(
                        block_cut_overlay_edge_offsets[to_overlay_index.into_directed_reverse()]
                            .into_usize()
                            + 1,
                    );
            }
        }

        if let Some(shortest_path_length_plus_minus) = dijkstra
            .shortest_paths(
                GfaLocation::new(
                    from_node_index.into_directed_forward(),
                    from_node_length.into_offset(),
                ),
                &SingleGfaLocationIndex::new_target(GfaLocation::new(
                    to_node_index.into_directed_reverse(),
                    0.into(),
                )),
            )
            .values()
            .map(|path| path.length())
            .next()
        {
            if DEBUG {
                println!(
                    "Inserting +- overlay edge from {} to {} with cost {}",
                    from_node_index.into_directed_forward(),
                    to_node_index.into_directed_forward(),
                    shortest_path_length_plus_minus,
                );
            }

            edges.extend(std::iter::once(BidirectedEdge::new(
                from_overlay_index.into_directed_forward(),
                to_overlay_index.into_directed_reverse(),
                OverlayEdgeData {
                    length: shortest_path_length_plus_minus,
                },
            )));

            if let Some(block_cut_overlay_edge_offsets) = block_cut_overlay_edge_offsets.as_mut() {
                block_cut_overlay_edge_offsets[from_overlay_index.into_directed_forward()] =
                    DirectedEdgeIndex::from_usize(
                        block_cut_overlay_edge_offsets[from_overlay_index.into_directed_forward()]
                            .into_usize()
                            + 1,
                    );
                block_cut_overlay_edge_offsets[to_overlay_index.into_directed_forward()] =
                    DirectedEdgeIndex::from_usize(
                        block_cut_overlay_edge_offsets[to_overlay_index.into_directed_forward()]
                            .into_usize()
                            + 1,
                    );
            }
        }

        if let Some(shortest_path_length_minus_plus) = dijkstra
            .shortest_paths(
                GfaLocation::new(
                    from_node_index.into_directed_reverse(),
                    from_node_length.into_offset(),
                ),
                &SingleGfaLocationIndex::new_target(GfaLocation::new(
                    to_node_index.into_directed_forward(),
                    0.into(),
                )),
            )
            .values()
            .map(|path| path.length())
            .next()
        {
            if DEBUG {
                println!(
                    "Inserting -+ overlay edge from {} to {} with cost {}",
                    from_node_index.into_directed_forward(),
                    to_node_index.into_directed_forward(),
                    shortest_path_length_minus_plus,
                );
            }

            edges.extend(std::iter::once(BidirectedEdge::new(
                from_overlay_index.into_directed_reverse(),
                to_overlay_index.into_directed_forward(),
                OverlayEdgeData {
                    length: shortest_path_length_minus_plus,
                },
            )));

            if let Some(block_cut_overlay_edge_offsets) = block_cut_overlay_edge_offsets.as_mut() {
                block_cut_overlay_edge_offsets[from_overlay_index.into_directed_reverse()] =
                    DirectedEdgeIndex::from_usize(
                        block_cut_overlay_edge_offsets[from_overlay_index.into_directed_reverse()]
                            .into_usize()
                            + 1,
                    );
                block_cut_overlay_edge_offsets[to_overlay_index.into_directed_reverse()] =
                    DirectedEdgeIndex::from_usize(
                        block_cut_overlay_edge_offsets[to_overlay_index.into_directed_reverse()]
                            .into_usize()
                            + 1,
                    );
            }
        }

        if let Some(shortest_path_length_minus_minus) = dijkstra
            .shortest_paths(
                GfaLocation::new(
                    from_node_index.into_directed_reverse(),
                    from_node_length.into_offset(),
                ),
                &SingleGfaLocationIndex::new_target(GfaLocation::new(
                    to_node_index.into_directed_reverse(),
                    0.into(),
                )),
            )
            .values()
            .map(|path| path.length())
            .next()
        {
            if DEBUG {
                println!(
                    "Inserting -- overlay edge from {} to {} with cost {}",
                    from_node_index.into_directed_forward(),
                    to_node_index.into_directed_forward(),
                    shortest_path_length_minus_minus,
                );
            }

            edges.extend(std::iter::once(BidirectedEdge::new(
                from_overlay_index.into_directed_reverse(),
                to_overlay_index.into_directed_reverse(),
                OverlayEdgeData {
                    length: shortest_path_length_minus_minus,
                },
            )));

            if let Some(block_cut_overlay_edge_offsets) = block_cut_overlay_edge_offsets.as_mut() {
                block_cut_overlay_edge_offsets[from_overlay_index.into_directed_reverse()] =
                    DirectedEdgeIndex::from_usize(
                        block_cut_overlay_edge_offsets[from_overlay_index.into_directed_reverse()]
                            .into_usize()
                            + 1,
                    );
                block_cut_overlay_edge_offsets[to_overlay_index.into_directed_forward()] =
                    DirectedEdgeIndex::from_usize(
                        block_cut_overlay_edge_offsets[to_overlay_index.into_directed_forward()]
                            .into_usize()
                            + 1,
                    );
            }
        }
    }

    /// Create the overlay edges between the plus and minus orientations of the given nodes.
    fn create_overlay_self_loops(
        node_index: NodeIndex<IndexType>,
        dijkstra: &mut GfaDijkstra<IndexType, NodeData, EdgeData>,
        graph_to_overlay_node_map: &impl std::ops::Index<
            NodeIndex<IndexType>,
            Output = OptionalNodeIndex<IndexType>,
        >,
        mut block_cut_overlay_edge_offsets: Option<
            &mut TaggedVec<DirectedNodeIndex<IndexType>, DirectedEdgeIndex<IndexType>>,
        >,
        edges: &mut impl Extend<BidirectedEdge<IndexType, OverlayEdgeData<IndexType>>>,
    ) {
        let overlay_index =
            graph_to_overlay_node_map[node_index].expect("Node must have an overlay node.");
        let node_length =
            <NodeData as GfaNodeDataExt<IndexType>>::len(dijkstra.graph().node_data(node_index));

        if node_length > 0.into() {
            // Only consider true self loops if the length is non-zero.
            // Otherwise, they will never be part of any shortest path.

            if let Some(shortest_path_length_plus_plus) = dijkstra
                .shortest_paths(
                    GfaLocation::new(
                        node_index.into_directed_forward(),
                        node_length.into_offset(),
                    ),
                    &SingleGfaLocationIndex::new_target(GfaLocation::new(
                        node_index.into_directed_forward(),
                        0.into(),
                    )),
                )
                .values()
                .map(|path| path.length())
                .next()
            {
                edges.extend(std::iter::once(BidirectedEdge::new(
                    overlay_index.into_directed_forward(),
                    overlay_index.into_directed_forward(),
                    OverlayEdgeData {
                        length: shortest_path_length_plus_plus,
                    },
                )));

                if let Some(block_cut_overlay_edge_offsets) =
                    block_cut_overlay_edge_offsets.as_mut()
                {
                    block_cut_overlay_edge_offsets[overlay_index.into_directed_forward()] =
                        DirectedEdgeIndex::from_usize(
                            block_cut_overlay_edge_offsets[overlay_index.into_directed_forward()]
                                .into_usize()
                                + 2,
                        );
                }
            }

            if let Some(shortest_path_length_minus_minus) = dijkstra
                .shortest_paths(
                    GfaLocation::new(
                        node_index.into_directed_reverse(),
                        node_length.into_offset(),
                    ),
                    &SingleGfaLocationIndex::new_target(GfaLocation::new(
                        node_index.into_directed_reverse(),
                        0.into(),
                    )),
                )
                .values()
                .map(|path| path.length())
                .next()
            {
                edges.extend(std::iter::once(BidirectedEdge::new(
                    overlay_index.into_directed_reverse(),
                    overlay_index.into_directed_reverse(),
                    OverlayEdgeData {
                        length: shortest_path_length_minus_minus,
                    },
                )));

                if let Some(block_cut_overlay_edge_offsets) =
                    block_cut_overlay_edge_offsets.as_mut()
                {
                    block_cut_overlay_edge_offsets[overlay_index.into_directed_reverse()] =
                        DirectedEdgeIndex::from_usize(
                            block_cut_overlay_edge_offsets[overlay_index.into_directed_reverse()]
                                .into_usize()
                                + 2,
                        );
                }
            }
        }

        if let Some(shortest_path_length_plus_minus) = dijkstra
            .shortest_paths(
                GfaLocation::new(
                    node_index.into_directed_forward(),
                    node_length.into_offset(),
                ),
                &SingleGfaLocationIndex::new_target(GfaLocation::new(
                    node_index.into_directed_reverse(),
                    0.into(),
                )),
            )
            .values()
            .map(|path| path.length())
            .next()
        {
            if DEBUG {
                println!(
                    "Inserting self-loop overlay edge from {} to {} with cost {}",
                    node_index.into_directed_forward(),
                    node_index.into_directed_reverse(),
                    shortest_path_length_plus_minus,
                );
            }

            edges.extend(std::iter::once(BidirectedEdge::new(
                overlay_index.into_directed_forward(),
                overlay_index.into_directed_reverse(),
                OverlayEdgeData {
                    length: shortest_path_length_plus_minus,
                },
            )));

            if let Some(block_cut_overlay_edge_offsets) = block_cut_overlay_edge_offsets.as_mut() {
                block_cut_overlay_edge_offsets[overlay_index.into_directed_forward()] =
                    DirectedEdgeIndex::from_usize(
                        block_cut_overlay_edge_offsets[overlay_index.into_directed_forward()]
                            .into_usize()
                            + 2,
                    );
            }
        }

        if let Some(shortest_path_length_minus_plus) = dijkstra
            .shortest_paths(
                GfaLocation::new(
                    node_index.into_directed_reverse(),
                    node_length.into_offset(),
                ),
                &SingleGfaLocationIndex::new_target(GfaLocation::new(
                    node_index.into_directed_forward(),
                    0.into(),
                )),
            )
            .values()
            .map(|path| path.length())
            .next()
        {
            if DEBUG {
                println!(
                    "Inserting self-loop overlay edge from {} to {} with cost {}",
                    node_index.into_directed_reverse(),
                    node_index.into_directed_forward(),
                    shortest_path_length_minus_plus,
                );
            }

            edges.extend(std::iter::once(BidirectedEdge::new(
                overlay_index.into_directed_reverse(),
                overlay_index.into_directed_forward(),
                OverlayEdgeData {
                    length: shortest_path_length_minus_plus,
                },
            )));

            if let Some(block_cut_overlay_edge_offsets) = block_cut_overlay_edge_offsets.as_mut() {
                block_cut_overlay_edge_offsets[overlay_index.into_directed_reverse()] =
                    DirectedEdgeIndex::from_usize(
                        block_cut_overlay_edge_offsets[overlay_index.into_directed_reverse()]
                            .into_usize()
                            + 2,
                    );
            }
        }
    }

    pub fn graph(&self) -> &'graph BidirectedAdjacencyArray<IndexType, NodeData, EdgeData> {
        self.graph
    }

    pub fn spqr_decomposition(
        &self,
    ) -> &'spqr SPQRDecomposition<'graph, BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>>
    {
        self.spqr_decomposition
    }

    pub fn overlay(
        &self,
    ) -> &BidirectedAdjacencyArray<IndexType, OverlayNodeData<IndexType>, OverlayEdgeData<IndexType>>
    {
        &self.overlay
    }

    pub fn graph_node_to_overlay_node(
        &self,
        node_index: NodeIndex<IndexType>,
    ) -> OptionalNodeIndex<IndexType> {
        self.graph_to_overlay_node_map[node_index]
    }

    pub fn overlay_node_to_graph_node(
        &self,
        overlay_node_index: NodeIndex<IndexType>,
    ) -> NodeIndex<IndexType> {
        self.overlay.node_data(overlay_node_index).original_node()
    }

    pub fn directed_graph_node_to_overlay_node(
        &self,
        node_index: DirectedNodeIndex<IndexType>,
    ) -> OptionalDirectedNodeIndex<IndexType> {
        self.graph_node_to_overlay_node(node_index.into_bidirected())
            .into_option()
            .map(|overlay_node_index| node_index.with_bidirected_node_index(overlay_node_index))
            .into()
    }

    pub fn directed_overlay_node_to_graph_node(
        &self,
        overlay_node_index: DirectedNodeIndex<IndexType>,
    ) -> DirectedNodeIndex<IndexType> {
        let graph_node_index =
            self.overlay_node_to_graph_node(overlay_node_index.into_bidirected());
        overlay_node_index.with_bidirected_node_index(graph_node_index)
    }

    pub fn iter_outgoing_edges<'this>(
        &'this self,
        node: DirectedNodeIndex<IndexType>,
        overlay_level: OverlayLevel,
    ) -> Box<dyn 'this + Iterator<Item = DirectedEdge<IndexType>>> {
        match overlay_level {
            OverlayLevel::BlockCutTree => Box::new(self.iter_outgoing_block_cut_tree_edges(node)),
            OverlayLevel::SPQRTree => Box::new(self.iter_outgoing_spqr_tree_edges(node)),
            OverlayLevel::SPQRNode => Box::new(self.iter_outgoing_spqr_node_edges(node)),
        }
    }

    pub fn iter_outgoing_block_cut_tree_edges(
        &self,
        node: DirectedNodeIndex<IndexType>,
    ) -> impl Iterator<Item = DirectedEdge<IndexType>> {
        self.overlay.iter_outgoing_edges(node).skip(
            self.block_cut_overlay_edge_offsets
                .get(node)
                .unwrap_or_else(|| {
                    panic!("Node index {node} is out of bounds for block_cut_overlay_edge_offsets with length {}.", self.block_cut_overlay_edge_offsets.len())
                })
                .into_usize(),
        )
    }

    pub fn iter_outgoing_spqr_tree_edges(
        &self,
        node: DirectedNodeIndex<IndexType>,
    ) -> impl Iterator<Item = DirectedEdge<IndexType>> {
        self.overlay
            .iter_outgoing_edges(node)
            .take(self.block_cut_overlay_edge_offsets[node].into_usize())
    }

    pub fn iter_outgoing_spqr_node_edges(
        &self,
        node: DirectedNodeIndex<IndexType>,
    ) -> impl Iterator<Item = DirectedEdge<IndexType>> {
        self.graph.iter_outgoing_edges(node)
    }
}

impl<IndexType: GraphIndexInteger> OverlayNodeData<IndexType> {
    pub fn original_node(&self) -> NodeIndex<IndexType> {
        self.original_node
    }
}

impl<IndexType: GraphIndexInteger> OverlayEdgeData<IndexType> {
    pub fn length(&self) -> GfaPathLength<IndexType> {
        self.length
    }
}

impl PartialOrd for OverlayLevel {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for OverlayLevel {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        match (self, other) {
            (OverlayLevel::BlockCutTree, OverlayLevel::BlockCutTree)
            | (OverlayLevel::SPQRTree, OverlayLevel::SPQRTree)
            | (OverlayLevel::SPQRNode, OverlayLevel::SPQRNode) => std::cmp::Ordering::Equal,
            (OverlayLevel::BlockCutTree, _) => std::cmp::Ordering::Greater,
            (OverlayLevel::SPQRNode, _) => std::cmp::Ordering::Less,
            (OverlayLevel::SPQRTree, OverlayLevel::BlockCutTree) => std::cmp::Ordering::Less,
            (OverlayLevel::SPQRTree, OverlayLevel::SPQRNode) => std::cmp::Ordering::Greater,
        }
    }
}
