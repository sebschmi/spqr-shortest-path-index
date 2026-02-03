use bidirected_adjacency_array::{index::GraphIndexInteger, io::gfa1::GfaNodeData};

use crate::path::GfaPathLength;

pub trait GfaNodeDataExt<IndexType: GraphIndexInteger> {
    fn len(&self) -> GfaPathLength<IndexType>;

    fn is_empty(&self) -> bool {
        self.len().into_usize() == 0
    }
}

impl<T: GfaNodeData, IndexType: GraphIndexInteger> GfaNodeDataExt<IndexType> for T {
    fn len(&self) -> GfaPathLength<IndexType> {
        GfaPathLength::from_usize(self.sequence().len())
    }
}
