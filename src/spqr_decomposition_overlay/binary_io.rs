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

#[cfg(test)]
mod tests {
    use std::{
        fs::{self, File},
        io::BufReader,
    };

    use bidirected_adjacency_array::{
        graph::BidirectedAdjacencyArray,
        io::gfa1::{PlainGfaEdgeData, PlainGfaNodeData},
    };
    use spqr_tree::decomposition::SPQRDecomposition;

    use crate::spqr_decomposition_overlay::SPQRDecompositionOverlay;

    #[test]
    fn test_binary_io() {
        let graph = BidirectedAdjacencyArray::<u8, PlainGfaNodeData, PlainGfaEdgeData>::read_gfa1(
            BufReader::new(File::open("test_files/tiny1.gfa").unwrap()),
        )
        .unwrap();
        let spqr_decomposition_file = fs::read_to_string("test_files/tiny1.spqr").unwrap();
        let spqr_decomposition =
            SPQRDecomposition::read_plain_spqr(&graph, &mut spqr_decomposition_file.as_bytes())
                .unwrap();
        let overlay = SPQRDecompositionOverlay::new(&graph, &spqr_decomposition);

        let mut buffer = Vec::new();
        overlay.write_binary(&mut buffer).unwrap();
        let read_overlay =
            SPQRDecompositionOverlay::read_binary(&graph, &spqr_decomposition, &buffer[..])
                .unwrap();

        assert_eq!(overlay, read_overlay);
    }
}
