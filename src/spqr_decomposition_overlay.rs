use std::{collections::HashSet, iter};

use bidirected_adjacency_array::{
    graph::{BidirectedAdjacencyArray, BidirectedEdge, DirectedEdge},
    index::{DirectedNodeIndex, GraphIndexInteger, NodeIndex, OptionalNodeIndex},
    io::gfa1::{GfaEdgeData, GfaNodeData},
};
use spqr_tree::decomposition::SPQRDecomposition;
use tagged_vec::TaggedVec;

use crate::{
    dijkstra::GfaDijkstra, location::GfaLocation, location_index::single::SingleGfaLocationIndex,
    path::GfaPathLength,
};

#[expect(dead_code)]
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

    /// Maps node indices in the original graph to node indices in the overlay graph, if they exist.
    graph_to_overlay_node_map: TaggedVec<NodeIndex<IndexType>, OptionalNodeIndex<IndexType>>,
}

pub struct OverlayNodeData<IndexType> {
    original_node: NodeIndex<IndexType>,
}

pub struct OverlayEdgeData<IndexType> {
    /// Length of the shortest path between the offsets zero on both nodes in the original graph.
    length: GfaPathLength<IndexType>,
}

pub enum OverlayLevel {
    BlockCutTree,
    SPQRTree,
    SPQRNode,
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

                    for to_cut_node_index in block.iter_cut_nodes().skip(offset + 1) {
                        let to_node_index =
                            spqr_decomposition.cut_node_index_to_node_index(to_cut_node_index);

                        Self::create_overlay_edges_between(
                            from_node_index,
                            to_node_index,
                            &mut dijkstra,
                            &graph_to_overlay_node_map,
                            &mut edges,
                        );
                    }
                }
            }
        }

        Self {
            graph,
            spqr_decomposition,
            overlay: BidirectedAdjacencyArray::new(nodes, edges),
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
        edges: &mut impl Extend<BidirectedEdge<IndexType, OverlayEdgeData<IndexType>>>,
    ) {
        let from_overlay_index = graph_to_overlay_node_map[from_node_index]
            .expect("Both nodes must have an overlay node.");
        let to_overlay_index = graph_to_overlay_node_map[to_node_index]
            .expect("Both nodes must have an overlay node.");

        if let Some(shortest_path_length_plus_plus) = dijkstra
            .shortest_path(
                GfaLocation::new(from_node_index.into_directed_forward(), 0.into()),
                &SingleGfaLocationIndex::new_target(GfaLocation::new(
                    to_node_index.into_directed_forward(),
                    0.into(),
                )),
            )
            .iter()
            .map(|path| path.length())
            .next()
        {
            edges.extend(std::iter::once(BidirectedEdge::new(
                from_overlay_index.into_directed_forward(),
                to_overlay_index.into_directed_forward(),
                OverlayEdgeData {
                    length: shortest_path_length_plus_plus,
                },
            )));
        }

        if let Some(shortest_path_length_plus_minus) = dijkstra
            .shortest_path(
                GfaLocation::new(from_node_index.into_directed_forward(), 0.into()),
                &SingleGfaLocationIndex::new_target(GfaLocation::new(
                    to_node_index.into_directed_reverse(),
                    0.into(),
                )),
            )
            .iter()
            .map(|path| path.length())
            .next()
        {
            edges.extend(std::iter::once(BidirectedEdge::new(
                from_overlay_index.into_directed_forward(),
                to_overlay_index.into_directed_reverse(),
                OverlayEdgeData {
                    length: shortest_path_length_plus_minus,
                },
            )));
        }

        if let Some(shortest_path_length_minus_plus) = dijkstra
            .shortest_path(
                GfaLocation::new(from_node_index.into_directed_reverse(), 0.into()),
                &SingleGfaLocationIndex::new_target(GfaLocation::new(
                    to_node_index.into_directed_forward(),
                    0.into(),
                )),
            )
            .iter()
            .map(|path| path.length())
            .next()
        {
            edges.extend(std::iter::once(BidirectedEdge::new(
                from_overlay_index.into_directed_reverse(),
                to_overlay_index.into_directed_forward(),
                OverlayEdgeData {
                    length: shortest_path_length_minus_plus,
                },
            )));
        }

        if let Some(shortest_path_length_minus_minus) = dijkstra
            .shortest_path(
                GfaLocation::new(from_node_index.into_directed_reverse(), 0.into()),
                &SingleGfaLocationIndex::new_target(GfaLocation::new(
                    to_node_index.into_directed_reverse(),
                    0.into(),
                )),
            )
            .iter()
            .map(|path| path.length())
            .next()
        {
            edges.extend(std::iter::once(BidirectedEdge::new(
                from_overlay_index.into_directed_reverse(),
                to_overlay_index.into_directed_reverse(),
                OverlayEdgeData {
                    length: shortest_path_length_minus_minus,
                },
            )));
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

    pub fn iter_outgoing_edges(
        &self,
        node: DirectedNodeIndex<IndexType>,
        overlay_level: OverlayLevel,
    ) -> impl Iterator<Item = DirectedEdge<IndexType>> {
        match overlay_level {
            OverlayLevel::BlockCutTree => todo!(),
            OverlayLevel::SPQRTree => todo!(),
            OverlayLevel::SPQRNode => self.graph.iter_outgoing_edges(node),
        }
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
