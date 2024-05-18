package graph

import (
	"strings"
)

// The goal of this class is to handle the way IDs of edges.
// In OpenStreetMap, one street can be composed of multiple ways.
// We would like to consider all of these ways as the same street, and so give it the same way ID.

type WayBuilder struct {
	EdgeToName map[int]string
}

func NewWayBuilder() *WayBuilder {
	return &WayBuilder{EdgeToName: make(map[int]string)}
}

func (w *WayBuilder) AddStreetName(edgeId int, name string) {
	w.EdgeToName[edgeId] = name
}

func (w *WayBuilder) ReassignWayIDs(graph *Graph) {
	// Start at an edge, and assign the same way ID to all connected edges (with the same name)

	visited := make(map[int]bool) // Which edges have we visited?

	for _, edgePointers := range graph.Edges {
		if visited[edgePointers.Edge1.Id] || w.EdgeToName[edgePointers.Edge1.Id] == "" {
			continue
		}

		visited[edgePointers.Edge1.Id] = true

		queue := []int{edgePointers.Edge1.Id}

		consistentWayId := edgePointers.Edge1.Way

		for len(queue) > 0 {
			edgeId := queue[0]
			queue = queue[1:]

			// Find all connected edges with the same name
			connectedEdges := w.findConnectedEdgesWithSameName(graph, edgeId, visited)

			// Assign the same way ID to all connected edges
			for _, connectedEdge := range connectedEdges {
				// Assign the same way ID
				connectedEdge.Edge1.Way = consistentWayId
				connectedEdge.Edge2.Way = consistentWayId

				// Add the connected edge to the queue
				queue = append(queue, connectedEdge.Edge2.Id)
			}
		}
	}
}

func (w *WayBuilder) findConnectedEdgesWithSameName(g *Graph, edgeId int, visited map[int]bool) []PointerToEdges {
	// Find all connected edges with the same name

	connectedEdges := make([]PointerToEdges, 0)

	edgePointers := g.Edges[edgeId]
	name := w.EdgeToName[edgeId]

	// Find all connected edges
	for _, edge := range g.AdjacencyList[edgePointers.Edge1.To] {
		if visited[edge.Id] {
			continue
		}

		// Check if the edge has the same name
		if w.EdgeToName[edge.Id] == name {
			visited[edge.Id] = true

			connectedEdges = append(connectedEdges, g.Edges[edge.Id])
		}
	}

	for _, edge := range g.AdjacencyList[edgePointers.Edge2.To] {
		if visited[edge.Id] {
			continue
		}

		// Check if the edge has the same name
		if w.EdgeToName[edge.Id] == name {
			visited[edge.Id] = true

			connectedEdges = append(connectedEdges, g.Edges[edge.Id])
		}
	}

	return connectedEdges
}

func (w *WayBuilder) TryAddingNames(graph *Graph) {

	associationPrefix := "0Fs9d8 " // Prefix to denote that this is a sidewalk or a path associated with a street
	// Pick some random prefix that is unlikely to be a real street name

	// Attempt to add names to edges that do not have names
	// Do this by checking if the majority of edges in a way overlap with a certain named edge (by checking overlapsWith)

	// Map from unnamed way ID to the list of edge IDs
	unnamedWays := make(map[int][]int)

	// Find all unnamed ways
	for _, edgePointers := range graph.Edges {
		if w.EdgeToName[edgePointers.Edge1.Id] == "" {
			if _, ok := unnamedWays[edgePointers.Edge1.Way]; !ok {
				unnamedWays[edgePointers.Edge1.Way] = []int{edgePointers.Edge1.Id}
			} else {
				unnamedWays[edgePointers.Edge1.Way] = append(unnamedWays[edgePointers.Edge1.Way], edgePointers.Edge1.Id)
			}
		}
	}

	for _, edgeIds := range unnamedWays {
		// Find the most common name among the edges
		nameCounts := make(map[string]int)

		for _, edgeId := range edgeIds {
			// Don't need to check Edge1 and Edge2 separately since they are the same edge
			for _, overlap := range graph.Edges[edgeId].Edge1.OverlapsWith {
				if w.EdgeToName[overlap] != "" && !strings.HasPrefix(w.EdgeToName[overlap], associationPrefix) {
					nameCounts[w.EdgeToName[overlap]]++
				}
			}
		}

		// Find the most common name
		maxName := ""
		maxCount := 0
		for name, count := range nameCounts {
			if count > maxCount {
				maxName = name
				maxCount = count
			}
		}

		// Is it the majority?
		if maxCount > len(edgeIds)/2 {
			// Assign the name to all edges in the way
			for _, edgeId := range edgeIds {
				w.EdgeToName[edgeId] = associationPrefix + maxName
			}
		}
	}
}
