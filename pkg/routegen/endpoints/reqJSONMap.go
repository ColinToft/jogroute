package endpoints

import (
	"github.com/ColinToft/JogRoute/pkg/routegen"
)

// A request to generate routes
type GenerationRequest struct {
	// The number of routes to generate
	Count int `json:"count"`

	// The distance range of the routes
	MinDistance float64 `json:"min_distance"`
	MaxDistance float64 `json:"max_distance"`

	// Cycle length range
	MinCycleLength int `json:"min_cycle_length"`

	// Heuristics for edges (ex. crossing = 20, road = 10)
	Heuristics map[string]int `json:"heuristics"`

	Lat    float64 `json:"lat"`
	Lon    float64 `json:"lon"`
	Radius float64 `json:"radius"`
}

// Result of a route generation
type GenerationResponse struct {
	Routes []routegen.Route `json:"routes"`

	Err string `json:"err,omitempty"`
}
