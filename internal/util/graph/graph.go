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

type LineSegment struct {
	From   Node
	To     Node
	FromID int
	ToID   int
	Id     int
}

func (l LineSegment) DistanceTo(node Node, doPrint bool) (float64, float64) {
	lineLengthSquared := (l.To.Lat-l.From.Lat)*(l.To.Lat-l.From.Lat) + (l.To.Lon-l.From.Lon)*(l.To.Lon-l.From.Lon)
	if lineLengthSquared == 0 {
		return (node.Lat-l.From.Lat)*(node.Lat-l.From.Lat) + (node.Lon-l.From.Lon)*(node.Lon-l.From.Lon), 0
	}

	nodeMinusFrom := Node{Lat: node.Lat - l.From.Lat, Lon: node.Lon - l.From.Lon}
	ToMinusFrom := Node{Lat: l.To.Lat - l.From.Lat, Lon: l.To.Lon - l.From.Lon}
	dotproduct := nodeMinusFrom.Lat*ToMinusFrom.Lat + nodeMinusFrom.Lon*ToMinusFrom.Lon
	t := dotproduct / lineLengthSquared

	projection := Node{Lat: l.From.Lat + t*(l.To.Lat-l.From.Lat), Lon: l.From.Lon + t*(l.To.Lon-l.From.Lon)}
	if doPrint {
		fmt.Printf("[%f, %f]\n", projection.Lat, projection.Lon)
	}

	return math.Sqrt((node.Lat-projection.Lat)*(node.Lat-projection.Lat) + (node.Lon-projection.Lon)*(node.Lon-projection.Lon)), t
}

func (l LineSegment) IsCloseAndParallel(other LineSegment, doPrint bool) bool {
	if l.Id == other.Id {
		return false
	}
	if l.From == other.From || l.From == other.To || l.To == other.From || l.To == other.To {
		return false
	}

	// Find the angle between the two lines arc cos of the dot product of the two lines divided by the product of the lengths
	dotproduct := (l.To.Lat-l.From.Lat)*(other.To.Lat-other.From.Lat) + (l.To.Lon-l.From.Lon)*(other.To.Lon-other.From.Lon)
	lengthOfThis := math.Sqrt((l.To.Lat-l.From.Lat)*(l.To.Lat-l.From.Lat) + (l.To.Lon-l.From.Lon)*(l.To.Lon-l.From.Lon))
	lengthOfOther := math.Sqrt((other.To.Lat-other.From.Lat)*(other.To.Lat-other.From.Lat) + (other.To.Lon-other.From.Lon)*(other.To.Lon-other.From.Lon))
	angle := math.Acos(dotproduct / (lengthOfThis * lengthOfOther))

	distanceToFrom, tFrom := l.DistanceTo(other.From, doPrint)
	distanceToTo, tTo := l.DistanceTo(other.To, doPrint)

	notBothOnSameSide := !(tFrom < 0 && tTo < 0) && !(tFrom > 1 && tTo > 1)

	// If the angle is close to 0 or pi, then the lines are parallel
	if (angle < 0.1 || angle > math.Pi-0.1) && (distanceToFrom < 0.0002 || distanceToTo < 0.0002) && notBothOnSameSide {
		if doPrint {
			fmt.Printf("Angle: %f, Distance to from: %f, to: %f, tFrom: %f, tTo: %f\n", angle, distanceToFrom, distanceToTo, tFrom, tTo)
		}
		return true
	}

	return false
}

// An expansion edge is similar to an edge, but it contains multiple nodes
type ExpansionEdge struct {
	To         int
	InnerNodes []int
}

type Node struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
}

type Graph struct {
	AdjacencyList      [][]Edge
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

	g.expansionEdgeBegin = id

	// Add edges
	// We use "zones" to divide up the graph into smaller parts, so we can check for edges that are close and parallel.
	// This is because we want to identify pairs of sidewalks and roads that are parallel.
	// These are then added as "overlapsWith" to the edges.
	// The zones are 0.001 degrees wide and 0.001 degrees high.
	zones := make(map[int][]LineSegment)

	edgeId := 0
	for _, element := range data.Elements {
		// If it is a way that is not of type "primary", "secondary" or "tertiary", add it to the graph
		if element.Type == "way" && element.Tags.Highway != "primary" && element.Tags.Highway != "secondary" && element.Tags.Highway != "tertiary" && element.Tags.Access != "private" && element.Tags.Building != "yes" && element.Tags.Highway != "service" {
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
				// If it is, add it to the "overlapsWith" list
				zone := int(math.Floor(g.Nodes[assignedId1].Lat*1000)) + int(math.Floor(g.Nodes[assignedId1].Lon*1000))*1000
				lineSegment := LineSegment{g.Nodes[assignedId1], g.Nodes[assignedId2], assignedId1, assignedId2, edgeId}
				overlapsWith := make([]int, 0)
				// doPrint := element.ID == 1126376445 || element.ID == 36826577
				doPrint := edgeId == 21575 || edgeId == 8217 || edgeId == 8218
				if _, ok := zones[zone]; !ok {
					// If the zone does not exist, create it
					zones[zone] = make([]LineSegment, 0)

				} else {
					// Check if the edge is close and parallel to any other edges
					zonesToCheck := make([]int, 0)
					for i := -1; i < 2; i++ {
						for j := -1; j < 2; j++ {
							zonesToCheck = append(zonesToCheck, zone+i+1000*j)
						}
					}

					for _, checkingZone := range zonesToCheck {
						for _, otherSegment := range zones[checkingZone] {
							if doPrint {
								fmt.Printf("[[%f, %f], [%f, %f]] %d [[%f, %f], [%f, %f]]\n", lineSegment.From.Lat, lineSegment.From.Lon, lineSegment.To.Lat, lineSegment.To.Lon, otherSegment.Id, otherSegment.From.Lat, otherSegment.From.Lon, otherSegment.To.Lat, otherSegment.To.Lon)
							}
							if lineSegment.IsCloseAndParallel(otherSegment, doPrint) {
								if doPrint {
									fmt.Printf("Edge %d is close and parallel to edge %d\n", edgeId, otherSegment.Id)
								}
								overlapsWith = append(overlapsWith, otherSegment.Id)

								// Add this edge to the other edge's overlapsWith list
								for _, e := range g.AdjacencyList[otherSegment.FromID] {
									if e.Id == otherSegment.Id {
										e.OverlapsWith = append(e.OverlapsWith, edgeId)
										if doPrint {
											fmt.Printf("Added edge %d to edge %d's overlapsWith list\n", edgeId, otherSegment.Id)
										}
									}
								}
							}

						}
					}
				}

				// Add the edge to the zone
				zones[zone] = append(zones[zone], lineSegment)
				if doPrint {
					fmt.Printf("Added edge %d, originally %d in zone %d\n", edgeId, element.ID, zone)
				}

				g.AddEdge(edgeId, assignedId1, assignedId2, element.ID, distance, heuristic, overlapsWith)
				edgeId++
			}
		}

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

	g.RemoveDegree1()
	g.RemoveDegree2()

	return g
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

// RemoveEdge removes an edge from the graph.
func (g *Graph) RemoveEdge(from, to int) {
	// Remove the edge from the from node
	for i, edge := range g.AdjacencyList[from] {
		if edge.To == to {
			g.AdjacencyList[from] = append(g.AdjacencyList[from][:i], g.AdjacencyList[from][i+1:]...)
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
				edgeReplacements[edge1.Id] = edgeId

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
				edgeReplacements[edge2.Id] = edgeId

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
			edgeId++

			// Remove the old edges
			for _, innerNode := range innerNodes {
				for _, edge := range g.AdjacencyList[innerNode] {
					g.RemoveEdge(innerNode, edge.To)
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
			for j, overlap := range edge.OverlapsWith {
				if newId, ok := edgeReplacements[overlap]; ok {
					g.AdjacencyList[node][i].OverlapsWith = append(g.AdjacencyList[node][i].OverlapsWith[:j], g.AdjacencyList[node][i].OverlapsWith[j+1:]...)
					if !Contains(g.AdjacencyList[node][i].OverlapsWith, newId) {
						g.AdjacencyList[node][i].OverlapsWith = append(g.AdjacencyList[node][i].OverlapsWith, newId)
					}
				}
			}
		}
	}
}

func (g *Graph) RemoveDegree1() {
	// Remove nodes with degree 1
	fmt.Println("Removing nodes with degree 1...")
	changed := true

	for changed {
		changed = false
		for node := range g.AdjacencyList {
			if len(g.AdjacencyList[node]) == 1 && node != g.StartNode && g.AdjacencyList[node][0].To != g.StartNode {
				// Remove the edge
				g.RemoveEdge(node, g.AdjacencyList[node][0].To)
				changed = true
			}
		}
	}

	fmt.Println("Graph has", len(g.AdjacencyList), "nodes")

	edgeCount := 0
	for _, edges := range g.AdjacencyList {
		edgeCount += len(edges)
	}
	fmt.Println("Graph has", edgeCount/2, "edges")
}

// Print prints the graph.
func (g *Graph) Print() {
	for from, edges := range g.AdjacencyList {
		for to, edge := range edges {
			fmt.Printf("%d -> %d (%f)", from, to, edge.Distance)
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
