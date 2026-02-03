use bidirected_adjacency_array::{
    graph::{BidirectedAdjacencyArray, BidirectedEdge},
    index::DirectedNodeIndex,
    io::gfa1::PlainGfaNodeData,
};

use crate::{
    dijkstra::shortest_path,
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
        [(0, 2, 1), (2, 4, 1), (4, 8, 1), (0, 6, 1), (6, 8, 1)].map(|(from, to, overlap)| {
            BidirectedEdge::new_gfa(
                DirectedNodeIndex::new(from),
                DirectedNodeIndex::new(to),
                overlap,
            )
        });
    let graph =
        BidirectedAdjacencyArray::<u8, _, _>::new(nodes.into(), FromIterator::from_iter(edges));

    let path = shortest_path(
        &graph,
        GfaLocation::new(0.into(), 3.into()),
        GfaLocation::new(8.into(), 2.into()),
    )
    .unwrap();

    let expected_path = vec![
        PathElement::new(0.into(), 3.into(), 3.into()),
        PathElement::new(2.into(), 1.into(), 3.into()),
        PathElement::new(4.into(), 1.into(), 3.into()),
        PathElement::new(8.into(), 1.into(), 2.into()),
    ];
    let expected_path = GfaPath::new(expected_path, 5.into());

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
        [(0, 2, 1), (2, 4, 1), (4, 8, 1), (0, 6, 1), (6, 8, 1)].map(|(from, to, overlap)| {
            BidirectedEdge::new_gfa(
                DirectedNodeIndex::new(from),
                DirectedNodeIndex::new(to),
                overlap,
            )
        });
    let graph =
        BidirectedAdjacencyArray::<u8, _, _>::new(nodes.into(), FromIterator::from_iter(edges));

    let path = shortest_path(
        &graph,
        GfaLocation::new(0.into(), 2.into()),
        GfaLocation::new(8.into(), 0.into()),
    )
    .unwrap();

    let expected_path = vec![
        PathElement::new(0.into(), 2.into(), 3.into()),
        PathElement::new(2.into(), 1.into(), 3.into()),
        PathElement::new(4.into(), 1.into(), 2.into()),
        PathElement::new(8.into(), 0.into(), 0.into()),
    ];
    let expected_path = GfaPath::new(expected_path, 4.into());

    assert_eq!(path.length(), expected_path.length());
    assert_eq!(
        path.iter().collect::<Vec<_>>(),
        expected_path.iter().collect::<Vec<_>>(),
        "Paths differ:\nExpected: {expected_path:?}\nActual:   {path:?}",
    );
}

#[test]
fn test_simple_tight_target_iteratively() {
    let nodes = vec![
        PlainGfaNodeData::new("A", "AAA"),
        PlainGfaNodeData::new("B", "BBB"),
        PlainGfaNodeData::new("C", "CCC"),
        PlainGfaNodeData::new("D", "DDDDDDDDD"),
        PlainGfaNodeData::new("E", "EEE"),
    ];
    let edges =
        [(0, 2, 1), (2, 4, 2), (4, 8, 3), (0, 6, 1), (6, 8, 1)].map(|(from, to, overlap)| {
            BidirectedEdge::new_gfa(
                DirectedNodeIndex::new(from),
                DirectedNodeIndex::new(to),
                overlap,
            )
        });
    let graph =
        BidirectedAdjacencyArray::<u8, _, _>::new(nodes.into(), FromIterator::from_iter(edges));

    let path = shortest_path(
        &graph,
        GfaLocation::new(0.into(), 3.into()),
        GfaLocation::new(8.into(), 0.into()),
    )
    .unwrap();

    let expected_path = vec![
        PathElement::new(0.into(), 3.into(), 3.into()),
        PathElement::new(2.into(), 1.into(), 1.into()),
        PathElement::new(4.into(), 0.into(), 0.into()),
        PathElement::new(8.into(), 0.into(), 0.into()),
    ];
    let expected_path = GfaPath::new(expected_path, 0.into());

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
    let edges = [(0, 2, 1), (2, 4, 1), (4, 0, 1)].map(|(from, to, overlap)| {
        BidirectedEdge::new_gfa(
            DirectedNodeIndex::new(from),
            DirectedNodeIndex::new(to),
            overlap,
        )
    });
    let graph =
        BidirectedAdjacencyArray::<u8, _, _>::new(nodes.into(), FromIterator::from_iter(edges));

    let path = shortest_path(
        &graph,
        GfaLocation::new(0.into(), 3.into()),
        GfaLocation::new(0.into(), 0.into()),
    )
    .unwrap();

    let expected_path = vec![
        PathElement::new(0.into(), 3.into(), 3.into()),
        PathElement::new(2.into(), 1.into(), 3.into()),
        PathElement::new(4.into(), 1.into(), 2.into()),
        PathElement::new(0.into(), 0.into(), 0.into()),
    ];
    let expected_path = GfaPath::new(expected_path, 3.into());

    assert_eq!(path.length(), expected_path.length());
    assert_eq!(
        path.iter().collect::<Vec<_>>(),
        expected_path.iter().collect::<Vec<_>>(),
        "Paths differ:\nExpected: {expected_path:?}\nActual:   {path:?}",
    );
}

#[test]
fn test_negative_cycle() {
    let nodes = vec![
        PlainGfaNodeData::new("A", "AAA"),
        PlainGfaNodeData::new("B", "BBB"),
        PlainGfaNodeData::new("C", "CCC"),
    ];
    let edges = [(0, 2, 2), (2, 4, 3), (4, 0, 2)].map(|(from, to, overlap)| {
        BidirectedEdge::new_gfa(
            DirectedNodeIndex::new(from),
            DirectedNodeIndex::new(to),
            overlap,
        )
    });
    let graph =
        BidirectedAdjacencyArray::<u8, _, _>::new(nodes.into(), FromIterator::from_iter(edges));

    let path = shortest_path(
        &graph,
        GfaLocation::new(0.into(), 3.into()),
        GfaLocation::new(0.into(), 0.into()),
    )
    .unwrap();

    let expected_path = vec![
        PathElement::new(0.into(), 3.into(), 3.into()),
        PathElement::new(2.into(), 2.into(), 2.into()),
        PathElement::new(4.into(), 2.into(), 2.into()),
        PathElement::new(0.into(), 1.into(), 1.into()),
        PathElement::new(2.into(), 0.into(), 0.into()),
        PathElement::new(4.into(), 0.into(), 1.into()),
        PathElement::new(0.into(), 0.into(), 0.into()),
    ];
    let expected_path = GfaPath::new(expected_path, 1.into());

    assert_eq!(path.length(), expected_path.length());
    assert_eq!(
        path.iter().collect::<Vec<_>>(),
        expected_path.iter().collect::<Vec<_>>(),
        "Paths differ:\nExpected: {expected_path:?}\nActual:   {path:?}",
    );
}

#[test]
fn test_self_loop_internal() {
    let nodes = vec![PlainGfaNodeData::new("A", "AAAAA")];
    let edges = [(0, 0, 3)].map(|(from, to, overlap)| {
        BidirectedEdge::new_gfa(
            DirectedNodeIndex::new(from),
            DirectedNodeIndex::new(to),
            overlap,
        )
    });
    let graph =
        BidirectedAdjacencyArray::<u8, _, _>::new(nodes.into(), FromIterator::from_iter(edges));

    let path = shortest_path(
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
fn test_self_loop_external_one() {
    let nodes = vec![PlainGfaNodeData::new("A", "AAAAA")];
    let edges = [(0, 0, 3)].map(|(from, to, overlap)| {
        BidirectedEdge::new_gfa(
            DirectedNodeIndex::new(from),
            DirectedNodeIndex::new(to),
            overlap,
        )
    });
    let graph =
        BidirectedAdjacencyArray::<u8, _, _>::new(nodes.into(), FromIterator::from_iter(edges));

    let path = shortest_path(
        &graph,
        GfaLocation::new(0.into(), 4.into()),
        GfaLocation::new(0.into(), 2.into()),
    )
    .unwrap();

    let expected_path = vec![
        PathElement::new(0.into(), 4.into(), 4.into()),
        PathElement::new(0.into(), 2.into(), 2.into()),
    ];
    let expected_path = GfaPath::new(expected_path, 0.into());

    assert_eq!(path.length(), expected_path.length());
    assert_eq!(
        path.iter().collect::<Vec<_>>(),
        expected_path.iter().collect::<Vec<_>>(),
        "Paths differ:\nExpected: {expected_path:?}\nActual:   {path:?}",
    );
}

#[test]
fn test_self_loop_external_two() {
    let nodes = vec![PlainGfaNodeData::new("A", "AAAAA")];
    let edges = [(0, 0, 3)].map(|(from, to, overlap)| {
        BidirectedEdge::new_gfa(
            DirectedNodeIndex::new(from),
            DirectedNodeIndex::new(to),
            overlap,
        )
    });
    let graph =
        BidirectedAdjacencyArray::<u8, _, _>::new(nodes.into(), FromIterator::from_iter(edges));

    let path = shortest_path(
        &graph,
        GfaLocation::new(0.into(), 4.into()),
        GfaLocation::new(0.into(), 1.into()),
    )
    .unwrap();

    let expected_path = vec![
        PathElement::new(0.into(), 4.into(), 4.into()),
        PathElement::new(0.into(), 2.into(), 2.into()),
        PathElement::new(0.into(), 1.into(), 2.into()),
    ];
    let expected_path = GfaPath::new(expected_path, 1.into());

    assert_eq!(path.length(), expected_path.length());
    assert_eq!(
        path.iter().collect::<Vec<_>>(),
        expected_path.iter().collect::<Vec<_>>(),
        "Paths differ:\nExpected: {expected_path:?}\nActual:   {path:?}",
    );
}
