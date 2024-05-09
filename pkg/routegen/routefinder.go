package routegen

import (
	"fmt"
	"sync"

	"github.com/ColinToft/JogRoute/internal/util/graph"
	"github.com/ColinToft/JogRoute/internal/util/heap"
)

type RouteFinder struct {
	tree  *RouteTree
	graph *graph.Graph

	// Variables for in progress calculation by the goroutines

	routes          []Route
	routeCount      int
	queue           *heap.HierarchicalHeap
	size            int
	distanceToStart []float64
	numWorking      int // Number of workers that are currently working

	// Mutexes for the route finder (currently parallelism is not being used)
	routesMutex sync.Mutex
	queueMutex  sync.Mutex
}

// NewRouteFinder creates a new route finder.
func NewRouteFinder(graph *graph.Graph) *RouteFinder {
	return &RouteFinder{graph: graph}
}

func (rf *RouteFinder) findRoute(quit chan bool, minDistance, maxDistance, maxRepeated, minCycleLength float64) int {
	for rf.size > 0 {
		select {
		case <-quit:
			fmt.Println("Quitting")
			return -1
		default:

			// Pop the last element from the stack
			routeIndex := rf.queue.Dequeue()
			rf.size--

			route := &rf.tree.Routes[routeIndex]

			// Get the last node from the route
			lastNode := route.LastNode

			// If the route is too long or has too much repeated distance,
			// don't bother exploring it further
			if route.Distance+rf.distanceToStart[route.LastNode] > maxDistance || route.RepeatedDistance > maxRepeated {
				rf.tree.Free(routeIndex)
				continue
			}

			// If the last node is the start node and the route is long enough, we have found a loop
			if lastNode == rf.graph.StartNode && route.Distance >= minDistance && route.Distance <= maxDistance {
				fmt.Printf("The added route has %d ways\n", route.Ways)
				fmt.Printf("The edge heuristics are %f\n", route.EdgeHeuristics)

				return routeIndex
			}

			// Add all the neighbours of the last node to the stack
			for _, neighbourEdge := range rf.graph.AdjacencyList[lastNode] {
				// newRoute will be a copy of the current route, but with the new node added

				// Does this edge go to the node that we were at before lastNode?
				// If so, don't add it to the route
				if route.Prev == -1 || neighbourEdge.To != rf.tree.Routes[route.Prev].LastNode {
					newRoute := rf.tree.AddNode(routeIndex, neighbourEdge, minCycleLength)

					rf.queue.Enqueue(newRoute, rf.tree.Routes[newRoute].Heuristic(minDistance))
					rf.size++
				}
			}
		}
	}

	return -1
}

func (rf *RouteFinder) Initialize() {
	rf.tree = NewRouteTree(rf.graph.StartNode, 67108864)

	rf.routeCount = 0
	rf.routes = make([]Route, 0)

	// queue := []Route{*NewRoute(start)}

	rf.queue, _ = heap.NewHierarchicalHeap(1000, -100000, 100000)

	rf.queue.Enqueue(0, 0) // Enqueue route 0 (initial route) with priority 0
	rf.size = 1

	rf.distanceToStart = rf.graph.GetDistanceToStart()
	rf.numWorking = 0
}

func (rf *RouteFinder) FindAllRoutes(quit chan bool, minDistance, maxDistance, maxRepeated, minCycleLength float64, routesCap int) []Route {

out:
	for rf.size > 0 && rf.routeCount < routesCap {
		select {
		case <-quit:
			break out
		default:
			rf.FindNextRoute(quit, minDistance, maxDistance, maxRepeated, minCycleLength)
		}
	}

	return rf.routes
}

func (rf *RouteFinder) FindNextRoute(quit chan bool, minDistance, maxDistance, maxRepeated, minCycleLength float64) Route {
out:
	for rf.size > 0 {
		routeIndex := rf.findRoute(quit, minDistance, maxDistance, maxRepeated, minCycleLength)

		if routeIndex == -1 {
			return Route{} // No route found
		}

		route := rf.tree.routeFromIndex(rf.graph, routeIndex)

		// Check our new route is not the reverse of another route
		for _, r := range rf.routes {
			if r.IsIdentical(route) {
				continue out
			}
		}

		rf.routeCount++
		// Add the route to the list of routes
		rf.routes = append(rf.routes, route)

		return route
	}

	return Route{} // No route found
}
