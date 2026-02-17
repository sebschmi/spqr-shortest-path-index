use std::{fs::File, io::BufReader};

use bidirected_adjacency_array::{
    graph::BidirectedAdjacencyArray,
    index::DirectedNodeIndex,
    io::gfa1::{PlainGfaEdgeData, PlainGfaNodeData},
};
use spqr_tree::{decomposition::SPQRDecomposition, graph::StaticGraph};

use crate::{
    dijkstra::GfaDijkstra,
    location::GfaLocation,
    location_index::single::SingleGfaLocationIndex,
    path::OptionalGfaPathLength,
    spqr_decomposition_overlay::{SPQRDecompositionOverlay, dijkstra::OverlayDijkstra},
};

#[test]
fn test_tiny1_pairwise() {
    let graph = BidirectedAdjacencyArray::<u8, PlainGfaNodeData, PlainGfaEdgeData>::read_gfa1(
        BufReader::new(File::open("test_files/tiny1.gfa").unwrap()),
    )
    .unwrap();
    let spqr_decomposition = SPQRDecomposition::read_plain_spqr(
        &graph,
        BufReader::new(File::open("test_files/tiny1.spqr").unwrap()),
    )
    .unwrap();
    let overlay = SPQRDecompositionOverlay::new(&graph, &spqr_decomposition);

    let mut dijkstra = GfaDijkstra::new(&graph);
    let mut overlay_dijkstra = OverlayDijkstra::new(&overlay);

    for source in graph
        .node_indices()
        .flat_map(|node| [node.into_directed(true), node.into_directed(false)])
        .flat_map(|node| {
            [
                GfaLocation::new(node, 0.into()),
                GfaLocation::new(node, 1.into()),
            ]
        })
    {
        for target in graph
            .node_indices()
            .flat_map(|node| [node.into_directed(true), node.into_directed(false)])
            .flat_map(|node| {
                [
                    GfaLocation::new(node, 0.into()),
                    GfaLocation::new(node, 1.into()),
                ]
            })
        {
            let distance = OptionalGfaPathLength::from(
                dijkstra
                    .shortest_paths(source, &SingleGfaLocationIndex::new_target(target))
                    .values()
                    .map(|path| path.length())
                    .next(),
            );
            let overlay_distance = OptionalGfaPathLength::from(
                overlay_dijkstra
                    .shortest_paths(source, &SingleGfaLocationIndex::new_target(target))
                    .values()
                    .map(|path| path.length())
                    .next(),
            );
            assert_eq!(
                distance, overlay_distance,
                "Distances differ for source {source} and target {target}",
            );
        }
    }
}

#[test]
fn test_tiny1_0p0_0m0() {
    let graph = BidirectedAdjacencyArray::<u8, PlainGfaNodeData, PlainGfaEdgeData>::read_gfa1(
        BufReader::new(File::open("test_files/tiny1.gfa").unwrap()),
    )
    .unwrap();
    let spqr_decomposition = SPQRDecomposition::read_plain_spqr(
        &graph,
        BufReader::new(File::open("test_files/tiny1.spqr").unwrap()),
    )
    .unwrap();
    let overlay = SPQRDecompositionOverlay::new(&graph, &spqr_decomposition);

    let mut dijkstra = GfaDijkstra::new(&graph);
    let mut overlay_dijkstra = OverlayDijkstra::new(&overlay);

    let source = GfaLocation::new(DirectedNodeIndex::from_bidirected(0.into(), true), 0.into());
    let target = GfaLocation::new(
        DirectedNodeIndex::from_bidirected(0.into(), false),
        0.into(),
    );

    let paths = dijkstra.shortest_paths(source, &SingleGfaLocationIndex::new_target(target));
    let path = paths.values().next();
    let distance = OptionalGfaPathLength::from(path.map(|p| p.length()));
    let overlay_distance = OptionalGfaPathLength::from(
        overlay_dijkstra
            .shortest_paths(source, &SingleGfaLocationIndex::new_target(target))
            .values()
            .map(|path| path.length())
            .next(),
    );
    assert_eq!(
        distance,
        overlay_distance,
        "Distances differ for source {source} and target {target}\nPath: {}",
        {
            if let Some(path) = path {
                format!("{path:?}")
            } else {
                "None".to_string()
            }
        }
    );
}
