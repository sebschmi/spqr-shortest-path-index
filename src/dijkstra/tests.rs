use bidirected_adjacency_array::{
    graph::{BidirectedAdjacencyArray, BidirectedEdge},
    index::DirectedNodeIndex,
    io::gfa1::PlainGfaNodeData,
};

use crate::{
    dijkstra::gfa::gfa_shortest_path,
    location::GfaLocation,
    path::{GfaPath, PathElement},
};

#[test]
fn test_simple_tight_source() {
    let nodes = vec![
        PlainGfaNodeData::new("A", "AAA"),
        PlainGfaNodeData::new("B", "BBB"),
        PlainGfaNodeData::new("C", "CCC"),
        PlainGfaNodeData::new("D", "DDDDDDDDD"),
        PlainGfaNodeData::new("E", "EEE"),
    ];
    let edges =
        [(0, 2, 0), (2, 4, 0), (4, 8, 0), (0, 6, 0), (6, 8, 0)].map(|(from, to, overlap)| {
            BidirectedEdge::new_gfa(
                DirectedNodeIndex::new(from),
                DirectedNodeIndex::new(to),
                overlap,
            )
        });
    let graph =
        BidirectedAdjacencyArray::<u8, _, _>::new(nodes.into(), FromIterator::from_iter(edges));

    let path = gfa_shortest_path(
        &graph,
        GfaLocation::new(0.into(), 3.into()),
        GfaLocation::new(8.into(), 2.into()),
    )
    .unwrap();

    let expected_path = vec![
        PathElement::new(0.into(), 3.into(), 3.into()),
        PathElement::new(2.into(), 0.into(), 3.into()),
        PathElement::new(4.into(), 0.into(), 3.into()),
        PathElement::new(8.into(), 0.into(), 2.into()),
    ];
    let expected_path = GfaPath::new(expected_path, 8.into());

    assert_eq!(path.length(), expected_path.length());
    assert_eq!(
        path.iter().collect::<Vec<_>>(),
        expected_path.iter().collect::<Vec<_>>(),
        "Paths differ:\nExpected: {expected_path:?}\nActual:   {path:?}",
    );
}

#[test]
fn test_simple_tight_target() {
    let nodes = vec![
        PlainGfaNodeData::new("A", "AAA"),
        PlainGfaNodeData::new("B", "BBB"),
        PlainGfaNodeData::new("C", "CCC"),
        PlainGfaNodeData::new("D", "DDDDDDDDD"),
        PlainGfaNodeData::new("E", "EEE"),
    ];
    let edges =
        [(0, 2, 0), (2, 4, 0), (4, 8, 0), (0, 6, 0), (6, 8, 0)].map(|(from, to, overlap)| {
            BidirectedEdge::new_gfa(
                DirectedNodeIndex::new(from),
                DirectedNodeIndex::new(to),
                overlap,
            )
        });
    let graph =
        BidirectedAdjacencyArray::<u8, _, _>::new(nodes.into(), FromIterator::from_iter(edges));

    let path = gfa_shortest_path(
        &graph,
        GfaLocation::new(0.into(), 2.into()),
        GfaLocation::new(8.into(), 0.into()),
    )
    .unwrap();

    let expected_path = vec![
        PathElement::new(0.into(), 2.into(), 3.into()),
        PathElement::new(2.into(), 0.into(), 3.into()),
        PathElement::new(4.into(), 0.into(), 3.into()),
        PathElement::new(8.into(), 0.into(), 0.into()),
    ];
    let expected_path = GfaPath::new(expected_path, 7.into());

    assert_eq!(path.length(), expected_path.length());
    assert_eq!(
        path.iter().collect::<Vec<_>>(),
        expected_path.iter().collect::<Vec<_>>(),
        "Paths differ:\nExpected: {expected_path:?}\nActual:   {path:?}",
    );
}

#[test]
fn test_simple_tight_source_and_target() {
    let nodes = vec![
        PlainGfaNodeData::new("A", "AAA"),
        PlainGfaNodeData::new("B", "BBB"),
        PlainGfaNodeData::new("C", "CCC"),
        PlainGfaNodeData::new("D", "DDDDDDDDD"),
        PlainGfaNodeData::new("E", "EEE"),
    ];
    let edges =
        [(0, 2, 0), (2, 4, 0), (4, 8, 0), (0, 6, 0), (6, 8, 0)].map(|(from, to, overlap)| {
            BidirectedEdge::new_gfa(
                DirectedNodeIndex::new(from),
                DirectedNodeIndex::new(to),
                overlap,
            )
        });
    let graph =
        BidirectedAdjacencyArray::<u8, _, _>::new(nodes.into(), FromIterator::from_iter(edges));

    let path = gfa_shortest_path(
        &graph,
        GfaLocation::new(0.into(), 3.into()),
        GfaLocation::new(8.into(), 0.into()),
    )
    .unwrap();

    let expected_path = vec![
        PathElement::new(0.into(), 3.into(), 3.into()),
        PathElement::new(2.into(), 0.into(), 3.into()),
        PathElement::new(4.into(), 0.into(), 3.into()),
        PathElement::new(8.into(), 0.into(), 0.into()),
    ];
    let expected_path = GfaPath::new(expected_path, 6.into());

    assert_eq!(path.length(), expected_path.length());
    assert_eq!(
        path.iter().collect::<Vec<_>>(),
        expected_path.iter().collect::<Vec<_>>(),
        "Paths differ:\nExpected: {expected_path:?}\nActual:   {path:?}",
    );
}

#[test]
fn test_cycle() {
    let nodes = vec![
        PlainGfaNodeData::new("A", "AAA"),
        PlainGfaNodeData::new("B", "BBB"),
        PlainGfaNodeData::new("C", "CCC"),
    ];
    let edges = [(0, 2, 0), (2, 4, 0), (4, 0, 0)].map(|(from, to, overlap)| {
        BidirectedEdge::new_gfa(
            DirectedNodeIndex::new(from),
            DirectedNodeIndex::new(to),
            overlap,
        )
    });
    let graph =
        BidirectedAdjacencyArray::<u8, _, _>::new(nodes.into(), FromIterator::from_iter(edges));

    let path = gfa_shortest_path(
        &graph,
        GfaLocation::new(0.into(), 3.into()),
        GfaLocation::new(0.into(), 0.into()),
    )
    .unwrap();

    let expected_path = vec![
        PathElement::new(0.into(), 3.into(), 3.into()),
        PathElement::new(2.into(), 0.into(), 3.into()),
        PathElement::new(4.into(), 0.into(), 3.into()),
        PathElement::new(0.into(), 0.into(), 0.into()),
    ];
    let expected_path = GfaPath::new(expected_path, 6.into());

    assert_eq!(path.length(), expected_path.length());
    assert_eq!(
        path.iter().collect::<Vec<_>>(),
        expected_path.iter().collect::<Vec<_>>(),
        "Paths differ:\nExpected: {expected_path:?}\nActual:   {path:?}",
    );
}

#[test]
fn test_internal_path() {
    let nodes = vec![PlainGfaNodeData::new("A", "AAAAA")];
    let edges = [(0, 0, 0)].map(|(from, to, overlap)| {
        BidirectedEdge::new_gfa(
            DirectedNodeIndex::new(from),
            DirectedNodeIndex::new(to),
            overlap,
        )
    });
    let graph =
        BidirectedAdjacencyArray::<u8, _, _>::new(nodes.into(), FromIterator::from_iter(edges));

    let path = gfa_shortest_path(
        &graph,
        GfaLocation::new(0.into(), 1.into()),
        GfaLocation::new(0.into(), 3.into()),
    )
    .unwrap();

    let expected_path = vec![PathElement::new(0.into(), 1.into(), 3.into())];
    let expected_path = GfaPath::new(expected_path, 2.into());

    assert_eq!(path.length(), expected_path.length());
    assert_eq!(
        path.iter().collect::<Vec<_>>(),
        expected_path.iter().collect::<Vec<_>>(),
        "Paths differ:\nExpected: {expected_path:?}\nActual:   {path:?}",
    );
}

#[test]
fn test_self_loop_external() {
    let nodes = vec![PlainGfaNodeData::new("A", "AAAAA")];
    let edges = [(0, 0, 0)].map(|(from, to, overlap)| {
        BidirectedEdge::new_gfa(
            DirectedNodeIndex::new(from),
            DirectedNodeIndex::new(to),
            overlap,
        )
    });
    let graph =
        BidirectedAdjacencyArray::<u8, _, _>::new(nodes.into(), FromIterator::from_iter(edges));

    let path = gfa_shortest_path(
        &graph,
        GfaLocation::new(0.into(), 4.into()),
        GfaLocation::new(0.into(), 2.into()),
    )
    .unwrap();

    let expected_path = vec![
        PathElement::new(0.into(), 4.into(), 5.into()),
        PathElement::new(0.into(), 0.into(), 2.into()),
    ];
    let expected_path = GfaPath::new(expected_path, 3.into());

    assert_eq!(path.length(), expected_path.length());
    assert_eq!(
        path.iter().collect::<Vec<_>>(),
        expected_path.iter().collect::<Vec<_>>(),
        "Paths differ:\nExpected: {expected_path:?}\nActual:   {path:?}",
    );
}
