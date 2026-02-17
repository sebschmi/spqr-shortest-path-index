use std::io::Read;

use bidirected_adjacency_array::{
    graph::BidirectedAdjacencyArray,
    index::GraphIndexInteger,
    io::gfa1::{GfaEdgeData, GfaNodeData},
};
use spqr_tree::decomposition::SPQRDecomposition;
use tagged_vec::TaggedVec;

use crate::spqr_decomposition_overlay::SPQRDecompositionOverlay;

impl<'graph, 'spqr, IndexType: GraphIndexInteger, NodeData: GfaNodeData, EdgeData: GfaEdgeData>
    SPQRDecompositionOverlay<'graph, 'spqr, IndexType, NodeData, EdgeData>
{
    /// Reads an SPQR decomposition overlay from a platform-dependent binary format.
    pub fn read_binary(
        graph: &'graph BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>,
        spqr_decomposition: &'spqr SPQRDecomposition<
            'graph,
            BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>,
        >,
        mut reader: impl Read,
    ) -> std::io::Result<Self> {
        Ok(Self {
            graph,
            spqr_decomposition,
            overlay: BidirectedAdjacencyArray::read_binary(&mut reader)?,
            block_cut_overlay_edge_offsets: TaggedVec::read_binary(&mut reader)?,
            graph_to_overlay_node_map: TaggedVec::read_binary(&mut reader)?,
        })
    }

    /// Writes the SPQR decomposition overlay into a platform-dependent binary format.
    pub fn write_binary(&self, mut writer: impl std::io::Write) -> std::io::Result<()> {
        self.overlay.write_binary(&mut writer)?;
        self.block_cut_overlay_edge_offsets
            .write_binary(&mut writer)?;
        self.graph_to_overlay_node_map.write_binary(&mut writer)?;
        Ok(())
    }
}
