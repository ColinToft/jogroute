package routegen

// Status is the status of the route generator service
type Status struct {
	// The number of routes that have been generated
	GeneratedRoutes int `json:"generated_routes"`

	// Whether or not a generation is in progress
	GenerationInProgress bool `json:"generation_in_progress"`

	// Has a map been loaded
	MapLoaded bool `json:"map_loaded"`
}
