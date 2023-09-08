package routegen

import (
	"fmt"
	"sync"

	"github.com/ColinToft/JogRoute/internal/util/graph"
	"github.com/bgadrian/data-structures/priorityqueue"
)

type RouteFinder struct {
	tree  *RouteTree
	graph *graph.Graph

	// Variables for in progress calculation by the goroutines
	loops           []int
	loopCount       int
	queue           *priorityqueue.HierarchicalHeap
	size            int
	distanceToStart []float64
	numWorking      int // Number of workers that are currently working

	// Mutexes for the route finder
	loopsMutex sync.Mutex
	queueMutex sync.Mutex
}

// NewRouteFinder creates a new route finder.
func NewRouteFinder(graph *graph.Graph) *RouteFinder {
	return &RouteFinder{graph: graph}
}

func (rf *RouteFinder) findLoop(quit chan bool, minDistance, maxDistance, maxRepeated, minCycleLength float64) int {
	for rf.size > 0 {
		select {
		case <-quit:
			fmt.Println("Quitting")
			return -1
		default:

			// Pop the last element from the stack
			temp, _ := rf.queue.Dequeue()
			rf.size--

			routeIndex := temp.(int)
			route := &rf.tree.Routes[routeIndex]

			// Get the last node from the route
			lastNode := route.LastNode

			// If the route is too long or has too much repeated distance,
			// don't bother exploring it further
			if route.Distance+rf.distanceToStart[route.LastNode] > maxDistance || route.RepeatedDistance > maxRepeated {
				rf.tree.Free(routeIndex)
				continue
			}

			// If the last node is the start node and the route is long enough, add it to the list of rf.loops
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

					rf.queue.Enqueue(newRoute, rf.tree.Routes[newRoute].Heuristic())
					rf.size++
				} /* else {
					if route.Prev != -1 {
						fmt.Printf("The edge from %d to %d is a repeat of the edge from %d to %d\n", lastNode, neighbourEdge.To, rf.tree.Routes[route.Prev].LastNode, lastNode)
					}
				} */
			}
		}
	}

	return -1
}

func (rf *RouteFinder) Initialize() {
	// Find all loops that start and end at node start.
	// A loop is a closed walk through the graph.
	rf.tree = NewRouteTree(rf.graph.StartNode, 67108864)

	rf.loops = []int{} // List of indices of loops in the routes list
	rf.loopCount = 0

	// queue := []Route{*NewRoute(start)}

	rf.queue, _ = priorityqueue.NewHierarchicalHeap(100, 0, 100000, false)

	rf.queue.Enqueue(0, 0) // Enqueue route 0 (initial route) with priority 0
	rf.size = 1

	rf.distanceToStart = rf.graph.GetDistanceToStart()
	rf.numWorking = 0
}

func (rf *RouteFinder) FindAllRoutes(quit chan bool, minDistance, maxDistance, maxRepeated, minCycleLength float64, routesCap int) []Route {
out:
	for rf.size > 0 && rf.loopCount < routesCap {
		select {
		case <-quit:
			break out
		default:
			loopIndex := rf.findLoop(quit, minDistance, maxDistance, maxRepeated, minCycleLength)

			rf.loops = append(rf.loops, loopIndex)
			rf.loopCount++
			// TODO check it is not reverse of another route
		}
	}

	routes := make([]Route, len(rf.loops))
	for i, routeIndex := range rf.loops {
		routes[i] = rf.tree.routeFromIndex(rf.graph, routeIndex)
	}
	return routes
}

func (rf *RouteFinder) FindNextRoute(quit chan bool, minDistance, maxDistance, maxRepeated, minCycleLength float64) Route {
	loopIndex := rf.findLoop(quit, minDistance, maxDistance, maxRepeated, minCycleLength)

	if loopIndex == -1 {
		return Route{}
	}

	return rf.tree.routeFromIndex(rf.graph, loopIndex)
}
