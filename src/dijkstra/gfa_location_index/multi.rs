use std::{collections::HashMap, iter};

use bidirected_adjacency_array::{
    graph::BidirectedAdjacencyArray,
    index::{DirectedNodeIndex, GraphIndexInteger},
};
use bitvec::{bitvec, vec::BitVec};

use crate::{
    dijkstra::{
        gfa::GfaShortestPathSource,
        gfa_location_index::{GfaLocationIndex, GfaLocationIndexKind},
    },
    location::{GfaLocation, GfaNodeOffset},
    path::GfaPathLength,
};

pub struct MultiGfaLocationIndex<IndexType> {
    is_contained_index: BitVec,
    offsets: HashMap<DirectedNodeIndex<IndexType>, GfaNodeOffset<IndexType>>,
    costs: HashMap<DirectedNodeIndex<IndexType>, GfaPathLength<IndexType>>,
    kind: GfaLocationIndexKind,
}

impl<IndexType: GraphIndexInteger> MultiGfaLocationIndex<IndexType> {
    pub fn new_sources<NodeData, EdgeData>(
        graph: &BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>,
        sources: impl IntoIterator<Item = GfaShortestPathSource<IndexType>>,
    ) -> Self {
        let directed_node_count = graph.node_count() * 2;
        let mut is_contained_index = bitvec![0; directed_node_count];
        let mut offsets = HashMap::new();
        let mut costs = HashMap::new();

        for source in sources {
            assert!(
                !is_contained_index[source.node().into_usize()],
                "Duplicate source node",
            );
            is_contained_index.set(source.node().into_usize(), true);
            offsets.insert(source.node(), source.offset());
            costs.insert(source.node(), source.cost());
        }

        Self {
            is_contained_index,
            offsets,
            costs,
            kind: GfaLocationIndexKind::Sources,
        }
    }

    pub fn new_target<NodeData, EdgeData>(
        graph: &BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>,
        target: GfaLocation<IndexType>,
    ) -> Self {
        Self::new_targets(graph, iter::once(target))
    }

    pub fn new_targets<NodeData, EdgeData>(
        graph: &BidirectedAdjacencyArray<IndexType, NodeData, EdgeData>,
        targets: impl IntoIterator<Item = GfaLocation<IndexType>>,
    ) -> Self {
        let directed_node_count = graph.node_count() * 2;
        let mut is_contained_index = bitvec![0; directed_node_count];
        let mut offsets = HashMap::new();

        for target in targets {
            assert!(
                !is_contained_index[target.node().into_usize()],
                "Duplicate target node",
            );
            is_contained_index.set(target.node().into_usize(), true);
            offsets.insert(target.node(), target.offset());
        }

        Self {
            is_contained_index,
            offsets,
            costs: HashMap::new(),
            kind: GfaLocationIndexKind::Targets,
        }
    }
}

impl<IndexType: GraphIndexInteger> GfaLocationIndex<IndexType>
    for MultiGfaLocationIndex<IndexType>
{
    fn is_sources(&self) -> bool {
        matches!(self.kind, GfaLocationIndexKind::Sources)
    }

    fn is_targets(&self) -> bool {
        matches!(self.kind, GfaLocationIndexKind::Targets)
    }

    fn contains(&self, node: DirectedNodeIndex<IndexType>) -> bool {
        *self
            .is_contained_index
            .get(node.into_usize())
            .expect("Node index out of bounds")
    }

    fn offset(&self, node: DirectedNodeIndex<IndexType>) -> GfaNodeOffset<IndexType> {
        self.offsets
            .get(&node)
            .copied()
            .expect("Node is not contained in the index")
    }

    fn cost(&self, node: DirectedNodeIndex<IndexType>) -> GfaPathLength<IndexType> {
        self.costs
            .get(&node)
            .copied()
            .expect("Node is not a source")
    }

    fn iter_sources(&self) -> impl Iterator<Item = GfaShortestPathSource<IndexType>> {
        self.offsets.iter().map(|(&node, &offset)| {
            GfaShortestPathSource::new(GfaLocation::new(node, offset), self.cost(node))
        })
    }

    fn iter_targets(&self) -> impl Iterator<Item = GfaLocation<IndexType>> {
        self.offsets
            .iter()
            .map(|(&node, &offset)| GfaLocation::new(node, offset))
    }

    fn len(&self) -> usize {
        self.offsets.len()
    }
}
