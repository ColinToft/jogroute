package routegen

type Service interface {
	GenRoutes(quit chan bool, count int, minDistance, maxDistance float64,
		minCycleLength float64, heuristics map[string]float64, lat, lon, radius float64, eventChan chan string) error
}
