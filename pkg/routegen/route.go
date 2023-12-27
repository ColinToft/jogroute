package routegen

import (
	"fmt"
	"sync"

	"github.com/ColinToft/JogRoute/internal/util/graph"
)

// A route is a lightweight struct describing a route
// It is stored as a sequence of nodes (ints, which are the node ids)
// and a distance (float64)
// It also counts the total distance that is repeated (going over the same
// edge twice)
// To do this, it stores a set of edges that have been traversed before
/* type Route struct {
	Nodes            []int
	Distance         float64
	Repeated         []IntPair // two ints corresponding to the nodes, ordered by (smallest, largest)
	RepeatedDistance float64
	Ways             int
	LastWay          int
} */

// The size of a route tree is the maximum number of routes that can be stored
// in the tree. The array is allocated at the start, so we bound it to this size
const RouteTreeSize int = 67108864

type RouteNode struct {
	Prev             int32
	LastNode         int32
	LastEdge         int32
	Distance         float32
	RepeatedDistance float32
	Ways             int16
	LastWay          int64
	EdgeHeuristics   float32
}

type RouteTree struct {
	Routes [RouteTreeSize]RouteNode

	size     int
	nextFree int
	cap      int

	// Mutex for the route tree
	sync.Mutex
}

type Route struct {
	// A final route that can be shown to the user
	// Nodes is a 2d array of floats, where each element is a pair of coordinates
	Nodes    [][]float64 `json:"route"`
	Distance float64     `json:"distance"`
}

func NewRouteTree(start int, cap int) *RouteTree {
	r := &RouteTree{Routes: [RouteTreeSize]RouteNode{}}
	r.Routes[0] = NewRoute(start)
	r.size = 1
	r.cap = cap
	r.nextFree = -1
	return r
}

func (r *RouteTree) Free(index int) {
	r.Routes[index].Prev = r.nextFree
	r.nextFree = index
}

func (r *RouteTree) Allocate() int {
	if r.nextFree == -1 {
		if r.size == r.cap {
			// newRoutes := make([]RouteNode, r.cap*4)
			// copy(newRoutes, r.Routes)
			// r.Routes = newRoutes
			r.cap *= 4
			fmt.Printf("Resized to %v\n", r.cap)
		}
		r.size++
		return r.size - 1
	} else {
		index := r.nextFree
		r.nextFree = r.Routes[index].Prev
		return index
	}
}

type IntPair struct {
	A int
	B int
}

// NewRoute creates a new route
func NewRoute(start int) RouteNode {
	r := RouteNode{Prev: -1, LastNode: start, LastEdge: -1, Distance: 0, RepeatedDistance: 0, Ways: 0, LastWay: -1, EdgeHeuristics: 0}
	return r
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func max(a, b int) int {
	if a > b {
		return a
	}
	return b
}

// WithAddedNode adds a node to the route
func (r *RouteTree) AddNode(previousRouteIndex int, edge graph.Edge, minCycleLength float64) int {
	newRouteIndex := r.Allocate()

	prev := &r.Routes[previousRouteIndex]
	r.Routes[newRouteIndex] = RouteNode{Prev: previousRouteIndex, LastNode: edge.To, LastEdge: edge.Id, Distance: prev.Distance + edge.Distance, RepeatedDistance: prev.RepeatedDistance, Ways: prev.Ways, LastWay: edge.Way, EdgeHeuristics: prev.EdgeHeuristics + edge.Heuristic}
	newRoute := &r.Routes[newRouteIndex]

	// Check if the edge has been traversed before
	current := prev
	for {
		if current.LastEdge == edge.Id {
			newRoute.RepeatedDistance += edge.Distance
			break
		}

		for _, e := range edge.OverlapsWith {
			if e == current.LastEdge {
				newRoute.RepeatedDistance += edge.Distance
				break
			}
		}

		if current.Prev == -1 || newRoute.Distance-current.Distance > minCycleLength {
			break
		}
		current = &r.Routes[current.Prev]
	}

	if edge.Way != prev.LastWay {
		newRoute.Ways++
	}

	return newRouteIndex
}

func (r *RouteNode) Heuristic() int {
	return r.Ways*500 - int(r.Distance) - int(r.EdgeHeuristics)
}

func (r *RouteTree) routeFromIndex(g *graph.Graph, endNode int) Route {
	// Get the route from the list of loops at the given index
	nodeIDs := []int{}
	current := &r.Routes[endNode]
	distance := current.Distance
	edgeIDs := []int{}
	for {
		nodeIDs = append(nodeIDs, current.LastNode)
		edgeIDs = append(edgeIDs, current.LastEdge)
		if current.Prev == -1 {
			break
		}
		current = &r.Routes[current.Prev]
	}

	fmt.Printf("Before expansion the route is %v\n", nodeIDs)
	fmt.Printf("The edges are %v\n", edgeIDs)
	nodeIDs = g.ExpandRoute(nodeIDs)
	fmt.Printf("After expansion the route is %v\n", nodeIDs)
	nodes := make([][]float64, len(nodeIDs))

	// Convert the route to a list of nodes
	for i, node := range nodeIDs {
		nodes[i] = []float64{g.Nodes[node].Lat, g.Nodes[node].Lon}
	}

	return Route{Nodes: nodes, Distance: distance}
}

// func (r RouteNode) String() string {
// 	return fmt.Sprintf("%v (%f, %f)", r.GetNodes(), r.Distance, r.RepeatedDistance)
// }
