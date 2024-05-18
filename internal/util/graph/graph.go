package graph

import (
	"fmt"
	"math"
	"math/rand"

	"github.com/ColinToft/JogRoute/internal/util/heap"
	"github.com/ColinToft/JogRoute/internal/util/mapdata"
)

// A Graph struct is a collection of nodes and edges represented as an adjacency list.
// Each edge has a weight (float64).

type Edge struct {
	To           int
	Id           int
	OverlapsWith []int
	Way          int
	Distance     float64
	Heuristic    float64
}

// An expansion edge is similar to an edge, but it contains multiple inner nodes that it replaces
type ExpansionEdge struct {
	To         int
	InnerNodes []int
}

type Node struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
}

type PointerToEdges struct {
	Edge1 *Edge
	Edge2 *Edge
}

type Graph struct {
	AdjacencyList      [][]Edge
	Edges              []PointerToEdges
	Nodes              []Node
	expansionEdgeBegin int
	expansionEdges     [][]ExpansionEdge
	StartNode          int
}

func haversine(a, b Node) float64 {
	// Convert to radians
	aLat := a.Lat * math.Pi / 180
	aLon := a.Lon * math.Pi / 180
	bLat := b.Lat * math.Pi / 180
	bLon := b.Lon * math.Pi / 180

	// CalcuLate the distance
	dLat := bLat - aLat
	dLon := bLon - aLon

	EARTH_RADIUS := 6372797.560856

	haversine_a := math.Sin(dLat/2)*math.Sin(dLat/2) + math.Cos(aLat)*math.Cos(bLat)*math.Sin(dLon/2)*math.Sin(dLon/2)
	haversine_c := 2 * math.Atan2(math.Sqrt(haversine_a), math.Sqrt(1-haversine_a))
	return EARTH_RADIUS * haversine_c
}

func isElementValid(element mapdata.MapDataElement) bool {
	return element.Type == "way" &&
		element.Tags.Access != "private" &&
		element.Tags.Building != "yes" &&
		element.Tags.Highway != "service"
}

func isElementWalkable(element mapdata.MapDataElement) bool {
	return isElementValid(element) &&
		element.Tags.Highway != "primary" &&
		element.Tags.Highway != "secondary" &&
		element.Tags.Highway != "tertiary" &&
		element.Tags.Highway != "cycleway"
}

// Convert MapData to a Graph
func NewGraph(data *mapdata.MapData, startLat, startLon float64, heuristics map[string]float64) *Graph {
	g := &Graph{AdjacencyList: make([][]Edge, len(data.Nodes)), Nodes: make([]Node, 0)}

	// Map from real IDs to assigned IDs
	assignedIds := make(map[int]int)

	id := 0
	for _, element := range data.Elements {
		if element.Type == "node" {
			g.Nodes = append(g.Nodes, Node{element.Lat, element.Lon})
			assignedIds[element.ID] = id
			id++
		}
	}

	g.AdjacencyList = make([][]Edge, id)

	// Add edges
	// We use "zones" to divide up the graph into smaller parts, so we can check for edges that are close and parallel.
	// This is because we want to identify pairs of sidewalks and roads that are parallel.
	// These are then added as "overlapsWith" to the edges.
	zoneWidth := 0.001
	zoneMap := NewZoneMap(zoneWidth)

	wayBuilder := NewWayBuilder()

	unwalkableEdgeIds := make(map[int]bool)

	edgeId := 0
	for _, element := range data.Elements {
		// Make sure the element is a road or sidewalk, and not a private road, building, etc.
		if isElementValid(element) {
			for i := 0; i < len(element.Nodes)-1; i++ {
				assignedId1 := assignedIds[element.Nodes[i]]
				assignedId2 := assignedIds[element.Nodes[i+1]]

				distance := haversine(g.Nodes[assignedId1], g.Nodes[assignedId2])

				// Calculate heuristic, ex. for crossings, etc.
				heuristic := 0.0
				if heuristics != nil {
					if element.Tags.Footway == "crossing" {
						if element.Tags.Crossing == "traffic_signals" {
							heuristic = float64(heuristics["traffic_signals"])
						} else {
							heuristic = float64(heuristics["crossing"])
						}
					} else if element.Tags.Footway == "sidewalk" {
						heuristic = float64(heuristics["sidewalk"]) * distance
					} else if element.Tags.Highway == "path" {
						heuristic = float64(heuristics["path"]) * distance
					} else if element.Tags.Highway == "service" {
						heuristic = float64(heuristics["service"]) * distance
					} else if element.Tags.Highway == "footway" {
						heuristic = float64(heuristics["footway"]) * distance
					}
				}

				// Check if the edge is close to any other edges
				lineSegment := LineSegment{g.Nodes[assignedId1], g.Nodes[assignedId2], assignedId1, assignedId2, edgeId}
				overlapping := zoneMap.AddSegmentAndGetOverlaps(&lineSegment) // A list of edge IDs that this edge overlaps with

				for _, otherSegment := range overlapping {
					doPrint := false

					// Add this edge to the other edge's overlapsWith list
					// Each edge will have 2 Edge objects in the adjacency list, one for each direction
					// So we need this loop as well as the second loop
					for eIndex := range g.AdjacencyList[otherSegment.FromID] {
						e := &g.AdjacencyList[otherSegment.FromID][eIndex] // Use a pointer so we can modify the object
						if e.Id == otherSegment.Id {
							e.OverlapsWith = append(e.OverlapsWith, edgeId)
							if doPrint {
								fmt.Printf("Added edge %d to edge %d's overlapsWith list\n", edgeId, otherSegment.Id)

							}
							// Print out all four points for debugging purposes
							// fmt.Printf("[%f,%f],\n", segment.From.Lat, segment.From.Lon)
							// fmt.Printf("[%f,%f],\n", segment.To.Lat, segment.To.Lon)
							// fmt.Printf("[%f,%f],\n", otherSegment.From.Lat, otherSegment.From.Lon)
							// fmt.Printf("[%f,%f],\n", otherSegment.To.Lat, otherSegment.To.Lon)
						}
					}

					// Add this edge to the other edge's overlapsWith list (other direction)
					for eIndex := range g.AdjacencyList[otherSegment.ToID] {
						e := &g.AdjacencyList[otherSegment.ToID][eIndex] // Use a pointer so we can modify the object
						if e.Id == otherSegment.Id {
							e.OverlapsWith = append(e.OverlapsWith, edgeId)
						}
					}
				}

				// overlaps with is just the Id field of the overlapping edges
				overlapsWith := make([]int, len(overlapping))
				for i, overlap := range overlapping {
					overlapsWith[i] = overlap.Id
				}

				if element.Tags.Name != "" {
					wayBuilder.AddStreetName(edgeId, element.Tags.Name)
				}

				// Mark if the edge is unwalkable (this includes things like major roads)
				// We need to keep the unwalkable edges in during the initial processing,
				// for example associating sidewalks with their respective roads.
				// They are removed after this processing is complete.
				if !isElementWalkable(element) {
					unwalkableEdgeIds[edgeId] = true
				}

				wayId := element.ID
				g.AddEdge(edgeId, assignedId1, assignedId2, wayId, distance, heuristic, overlapsWith)
				edgeId++
			}
		}
	}

	g.expansionEdgeBegin = edgeId

	// Build the edge map (needed for wayBuilder)
	g.BuildEdgeMap()

	// Add names to unnamed edges
	wayBuilder.TryAddingNames(g)

	// Reassign way IDs to coallesce edges with the same street name
	wayBuilder.ReassignWayIDs(g)

	// Remove unwalkable edges
	for i, edges := range g.AdjacencyList {
		outputIndex := 0
		for _, edge := range edges {
			if edge.Way == 1025389579 {
				fmt.Printf("Edge %d shouldn't be here\n", edge.Id)
			}
			if !unwalkableEdgeIds[edge.Id] {
				g.AdjacencyList[i][outputIndex] = edge
				outputIndex++
			}
		}
		g.AdjacencyList[i] = g.AdjacencyList[i][:outputIndex]
	}

	// Find the start node
	start := -1
	minDist := math.MaxFloat64
	idealStartNode := Node{startLat, startLon}
	for node := range g.Nodes {
		dist := haversine(idealStartNode, g.Nodes[node])
		if dist < minDist && len(g.AdjacencyList[node]) >= 1 { // Make sure the node has at least one edge
			minDist = dist
			start = node
		}
	}

	fmt.Printf("Start node is %d, distance %f\n", start, minDist)
	g.StartNode = start

	// Simplify graph by removing nodes with degree 1 (deadends)
	// and nodes with degree 2 (combine two edges into one edge)
	g.RemoveDegree1()
	g.RemoveDegree2()

	return g
}

// Checks whether a graph is valid
func (g *Graph) IsValid() bool {
	return len(g.AdjacencyList) > 0 && g.StartNode != -1
}

// AddEdge adds an edge to the graph.
func (g *Graph) AddEdge(id, from, to, way int, distance, heuristic float64, overlapsWith []int) {
	if g.AdjacencyList[from] == nil {
		g.AdjacencyList[from] = make([]Edge, 0)
	}
	if g.AdjacencyList[to] == nil {
		g.AdjacencyList[to] = make([]Edge, 0)
	}

	g.AdjacencyList[from] = append(g.AdjacencyList[from], Edge{to, id, overlapsWith, way, distance, heuristic})
	g.AdjacencyList[to] = append(g.AdjacencyList[to], Edge{from, id, overlapsWith, way, distance, heuristic})
}

func (g *Graph) BuildEdgeMap() {
	g.Edges = make([]PointerToEdges, g.expansionEdgeBegin)
	for from, edges := range g.AdjacencyList {
		for i, edge := range edges {
			if from <= edge.To { // Only add the edge once
				edge1 := &g.AdjacencyList[from][i]

				// Find the edge in the other direction
				var edge2 *Edge
				for j, otherEdge := range g.AdjacencyList[edge.To] {
					if otherEdge.To == from {
						edge2 = &g.AdjacencyList[edge.To][j]
						break
					}
				}

				if edge2 == nil {
					fmt.Printf("Error: could not find edge in the other direction when building edge map\n")
				}

				g.Edges[edge.Id] = PointerToEdges{edge1, edge2}
			}
		}
	}
}

// RemoveEdge removes an edge from the graph.
func (g *Graph) RemoveEdge(from, to int) {
	edgeId := -1

	// Remove the edge from the from node
	for i, edge := range g.AdjacencyList[from] {
		if edge.To == to {
			g.AdjacencyList[from] = append(g.AdjacencyList[from][:i], g.AdjacencyList[from][i+1:]...)
			edgeId = edge.Id
			break
		}
	}

	// Remove the edge from the to node
	for i, edge := range g.AdjacencyList[to] {
		if edge.To == from {
			g.AdjacencyList[to] = append(g.AdjacencyList[to][:i], g.AdjacencyList[to][i+1:]...)
			break
		}
	}

	// Remove the edge from the edges list
	// Expansion edges are not stored in the edges list (IDs larger than g.Edges)
	if edgeId == -1 || edgeId >= len(g.Edges) {
		return
	}

	g.Edges[edgeId] = PointerToEdges{nil, nil}
}

func Contains(a []int, x int) bool {
	for _, element := range a {
		if element == x {
			return true
		}
	}
	return false
}

func Union(a, b []int) []int {
	for _, element := range b {
		if !Contains(a, element) {
			a = append(a, element)
		}
	}
	return a
}

func (g *Graph) RemoveDegree2() {
	g.expansionEdges = make([][]ExpansionEdge, len(g.AdjacencyList))

	// A map from old edge IDs to the expansion edge ID that replaces it
	edgeReplacements := make(map[int]int)

	fmt.Println("Removing nodes with degree 2...")

	fmt.Println("Graph has", len(g.AdjacencyList), "nodes")

	edgeCount := 0
	for _, edges := range g.AdjacencyList {
		edgeCount += len(edges)
	}
	fmt.Println("Graph has", edgeCount/2, "edges")

	edgeId := g.expansionEdgeBegin

	// Next, remove nodes that have degree 2 by merging all adjacent edges joined by a node with degree 2 into one edge
node:
	for node := range g.AdjacencyList {
		if len(g.AdjacencyList[node]) == 2 && node != g.StartNode {
			// Get the two neighbours, make sure they have the same way id
			edge1 := g.AdjacencyList[node][0]
			edge2 := g.AdjacencyList[node][1]

			start := edge1.To
			end := edge2.To

			innerNodes := make([]int, 0)
			innerNodes = append(innerNodes, node)

			if edge1.Way != edge2.Way {
				continue node
			}

			way := edge1.Way
			distance := edge1.Distance + edge2.Distance
			heuristic := edge1.Heuristic + edge2.Heuristic
			overlapsWith := Union(edge1.OverlapsWith, edge2.OverlapsWith)

			// See how far we can extend the beginning of the edge
			for len(g.AdjacencyList[start]) == 2 {
				secondNode := start
				if g.AdjacencyList[secondNode][0].Id == edge1.Id {
					edge1 = g.AdjacencyList[secondNode][1]
				} else {
					edge1 = g.AdjacencyList[secondNode][0]
				}
				if edge1.Way != way || secondNode == g.StartNode || Contains(innerNodes, edge1.To) {
					break
				}
				start = edge1.To
				distance += edge1.Distance
				heuristic += edge1.Heuristic
				overlapsWith = Union(overlapsWith, edge1.OverlapsWith)

				// Add secondNode to the start of the innerNodes list
				innerNodes = append([]int{secondNode}, innerNodes...)
			}

			// See how far we can extend the end of the edge
			for len(g.AdjacencyList[end]) == 2 {
				secondLastNode := end
				if g.AdjacencyList[secondLastNode][0].Id == edge2.Id {
					edge2 = g.AdjacencyList[secondLastNode][1]
				} else {
					edge2 = g.AdjacencyList[secondLastNode][0]
				}
				if edge2.Way != way || secondLastNode == g.StartNode || Contains(innerNodes, edge2.To) {
					break
				}
				end = edge2.To
				distance += edge2.Distance
				heuristic += edge2.Heuristic
				overlapsWith = Union(overlapsWith, edge2.OverlapsWith)

				// Add secondLastNode to the end of the innerNodes list
				innerNodes = append(innerNodes, secondLastNode)
			}

			// Check if the edge already exists
			for _, edge := range g.AdjacencyList[start] {
				if edge.To == end {
					// Edge already exists, don't add it
					continue node
				}
			}

			g.AddEdge(edgeId, start, end, way, distance, heuristic, overlapsWith)

			// Remove the old edges
			for _, innerNode := range innerNodes {
				for _, edge := range g.AdjacencyList[innerNode] {
					g.RemoveEdge(innerNode, edge.To)
					edgeReplacements[edge.Id] = edgeId
				}
			}

			// store the expansion edge
			g.expansionEdges[start] = append(g.expansionEdges[start], ExpansionEdge{end, innerNodes})
			// When adding end -> start, we need to reverse the innerNodes list
			reversedInnerNodes := make([]int, len(innerNodes))
			for i, node := range innerNodes {
				reversedInnerNodes[len(innerNodes)-i-1] = node
			}

			g.expansionEdges[end] = append(g.expansionEdges[end], ExpansionEdge{start, reversedInnerNodes})

			edgeId++
		}
	}

	fmt.Println("Graph has", len(g.AdjacencyList), "nodes")

	edgeCount = 0
	for _, edges := range g.AdjacencyList {
		edgeCount += len(edges)
	}
	fmt.Println("Graph has", edgeCount/2, "edges")

	// Clean up overlaps using the edge replacements
	for node := range g.AdjacencyList {
		for i, edge := range g.AdjacencyList[node] {
			outputIndex := 0

			for _, overlap := range edge.OverlapsWith {
				if newId, ok := edgeReplacements[overlap]; ok {
					if !Contains(g.AdjacencyList[node][i].OverlapsWith[:outputIndex], newId) {
						// Replace the old ID with the new ID
						g.AdjacencyList[node][i].OverlapsWith[outputIndex] = newId
						outputIndex++
					}
				} else if !Contains(g.AdjacencyList[node][i].OverlapsWith[:outputIndex], overlap) {
					g.AdjacencyList[node][i].OverlapsWith[outputIndex] = overlap
					outputIndex++
				}

			}
			// Slice off the rest of the overlaps
			g.AdjacencyList[node][i].OverlapsWith = g.AdjacencyList[node][i].OverlapsWith[:outputIndex]
		}
	}
}

func (g *Graph) RemoveDegree1() {
	// Remove nodes with degree 1
	fmt.Println("Removing nodes with degree 1...")
	changed := true

	// Whether or not an edge has been removed
	removedEdges := make(map[int]bool)

	for changed {
		changed = false
		for node := range g.AdjacencyList {
			if len(g.AdjacencyList[node]) == 1 && node != g.StartNode && g.AdjacencyList[node][0].To != g.StartNode {
				// Remove the edge
				changed = true
				removedEdges[g.AdjacencyList[node][0].Id] = true
				g.RemoveEdge(node, g.AdjacencyList[node][0].To)
			}
		}
	}

	fmt.Println("Graph has", len(g.AdjacencyList), "nodes")

	edgeCount := 0
	for _, edges := range g.AdjacencyList {
		edgeCount += len(edges)
	}
	fmt.Println("Graph has", edgeCount/2, "edges")

	// Clean up the overlaps
	for node := range g.AdjacencyList {
		for i, edge := range g.AdjacencyList[node] {
			outputIndex := 0

			for _, overlap := range edge.OverlapsWith {
				if !removedEdges[overlap] {
					g.AdjacencyList[node][i].OverlapsWith[outputIndex] = overlap
					outputIndex++
				}
			}

			// Slice off the rest of the overlaps
			g.AdjacencyList[node][i].OverlapsWith = g.AdjacencyList[node][i].OverlapsWith[:outputIndex]
		}
	}
}

// Print prints the graph.
func (g *Graph) Print() {
	for from, edges := range g.AdjacencyList {
		for _, edge := range edges {
			fmt.Printf("#%d: %d -> %d (%f) %v\n", edge.Id, from, edge.To, edge.Distance, edge.OverlapsWith)
		}
	}
}

// Outputs a list where each element is a [edgeId, centerCoordinateX, centerCoordinateY] array
func (g *Graph) PrintEdgesDebug() {
	for from, edges := range g.AdjacencyList {
		for _, edge := range edges {
			fmt.Printf("[%d, %f, %f],\n", edge.Id, (g.Nodes[from].Lat+g.Nodes[edge.To].Lat)/2, (g.Nodes[from].Lon+g.Nodes[edge.To].Lon)/2)
		}
	}
}

func (g *Graph) ShuffleOrder() {
	for i := range g.AdjacencyList {
		// Randomly shuffle the order of the edges
		for j := range g.AdjacencyList[i] {
			k := rand.Intn(j + 1)
			g.AdjacencyList[i][j], g.AdjacencyList[i][k] = g.AdjacencyList[i][k], g.AdjacencyList[i][j]
		}
	}
}

func (g *Graph) GetDistanceToStart() []float64 {
	// Use Dijkstra's algorithm to calcuLate the shortest distance to the start node for each node,
	// keeping in mind edges are weighted by distance

	// Initialize the distance to the start node for each node to infinity
	distanceToStart := make([]float64, len(g.AdjacencyList))
	for node := range g.AdjacencyList {
		distanceToStart[node] = math.MaxFloat64
	}

	// Initialize the distance to the start node for the start node to 0
	distanceToStart[g.StartNode] = 0

	// Unvisited heap
	unvisited := heap.NewImplicitHeapMin(false)

	// Set to true all nodes that are not the start node
	for node := range g.AdjacencyList {
		if node != g.StartNode {
			unvisited.Push(math.MaxFloat64, node)
		}
	}

	unvisited.Push(0, g.StartNode)

	// While there are unvisited nodes
	for unvisited.Len() > 0 {
		// Find the unvisited node with the smallest distance to the start node, where the distance to the start node is not infinity
		smallestNode, _ := unvisited.Pop()

		if distanceToStart[smallestNode] == math.MaxFloat64 {
			break
		}

		// For each of the smallest node's neighbors
		for _, neighborEdge := range g.AdjacencyList[smallestNode] {
			// If the neighbor is unvisited
			if unvisited.ContainsValue(neighborEdge.To) {
				// CalcuLate the distance to the start node through the smallest node
				throughSmallestNode := distanceToStart[smallestNode] + neighborEdge.Distance

				// If the distance to the start node through the smallest node is smaller than the current distance to the start node
				if throughSmallestNode < distanceToStart[neighborEdge.To] {
					// Update the distance to the start node for the neighbor
					distanceToStart[neighborEdge.To] = throughSmallestNode
					unvisited.DecreaseKey(neighborEdge.To, throughSmallestNode)
				}
			}
		}
	}

	return distanceToStart
}

func (g *Graph) ExpandRoute(nodes []int) []int {
	for i := 0; i < len(nodes)-1; i++ {
		// Find the edge
		for _, edge := range g.expansionEdges[nodes[i]] {
			if edge.To == nodes[i+1] {
				// Insert the expansion node's InnerNodes between the two nodes
				nodes = append(nodes[:i+1], append(edge.InnerNodes, nodes[i+1:]...)...)
				i += len(edge.InnerNodes)
				break
			}
		}
	}

	return nodes
}
