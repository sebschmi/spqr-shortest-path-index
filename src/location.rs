use bidirected_adjacency_array::index::{DirectedNodeIndex, GraphIndexInteger};
use optional_numeric_index::implement_generic_index;

implement_generic_index!(pub GfaNodeOffset, pub OptionalGfaNodeOffset);

pub struct GfaLocation<IndexType> {
    node: DirectedNodeIndex<IndexType>,
    offset: GfaNodeOffset<IndexType>,
}

impl<IndexType: GraphIndexInteger> GfaLocation<IndexType> {
    pub fn new(node: DirectedNodeIndex<IndexType>, offset: GfaNodeOffset<IndexType>) -> Self {
        Self { node, offset }
    }

    pub fn node(&self) -> DirectedNodeIndex<IndexType> {
        self.node
    }

    pub fn offset(&self) -> GfaNodeOffset<IndexType> {
        self.offset
    }
}
