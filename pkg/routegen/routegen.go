package routegen

// Route generation service implementation

import (
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"os"
	"runtime/pprof"

	"github.com/ColinToft/JogRoute/internal/util/graph"
	"github.com/ColinToft/JogRoute/internal/util/mapdata"
	"github.com/go-kit/log"
)

type routeGenService struct{}

func NewService() Service {
	return &routeGenService{}
}

func (s *routeGenService) GenRoutes(quit chan bool, count int, minDistance, maxDistance float64,
	minCycleLength float64, heuristics map[string]float64, lat, lon, radius float64, eventChan chan string) error {

	distanceBound := 22000.0
	if maxDistance > distanceBound {
		fmt.Println("Max distance is too large")
		eventChan <- "error"
		return nil
	}

	// Get the URL of the map data microservice, if it does not exist use localhost
	mapDataURL := os.Getenv("MAP_DATA_URL")
	if mapDataURL == "" {
		mapDataURL = "http://localhost:8081"
	}

	eventChan <- "Loading map data"

	// Query the map data microservice for the map data
	client := &http.Client{}
	request, _ := http.NewRequest(http.MethodGet, mapDataURL+"/api/mapdata?lat="+fmt.Sprintf("%f", lat)+"&lon="+fmt.Sprintf("%f", lon)+"&radius="+fmt.Sprintf("%f", radius), nil)

	// Send the request
	response, err := client.Do(request)
	if err != nil {
		fmt.Println(err)
		return err
	}

	// Read the response
	if response.StatusCode != 200 {
		fmt.Println("Error: " + response.Status)
		return err
	}

	fmt.Println("Got map JSON")

	defer response.Body.Close()

	eventChan <- "Processing map data"

	// Read the response body
	body, _ := io.ReadAll(response.Body)

	// Unmarshal the JSON into a MapData struct
	var rawMapData mapdata.MapData
	json.Unmarshal(body, &rawMapData)

	// TODO mark crossings near traffic lights as crossings
	// This can be done by looking for nodes that are part of a crossing

	fmt.Println("Converting to graph")

	routeFinder := NewRouteFinder(graph.NewGraph(&rawMapData, lat, lon, heuristics))

	eventChan <- "Generating routes"

	fmt.Println("Finding routes")

	// Enable runtime profiling
	profile := false
	if profile {
		f, err := os.Create("cpu.prof")
		if err != nil {
			fmt.Println(err)
			return err
		}
		defer f.Close()
		pprof.StartCPUProfile(f)
		defer pprof.StopCPUProfile()
	}

	if !routeFinder.graph.IsValid() {
		fmt.Println("Graph is invalid")
		eventChan <- "error"
		return err
	}

	routeFinder.Initialize()

	for i := 0; i < count; i++ {
		maxRepeated := 0.0 // Could make this a paramater eventually
		route := routeFinder.FindNextRoute(quit, minDistance, maxDistance, maxRepeated, minCycleLength)

		// If route is nil or empty, we are done
		if route.Nodes == nil || len(route.Nodes) == 0 {
			fmt.Println("No more routes can be found")
			break
		}

		routeJSON, _ := json.Marshal(route)
		eventChan <- string(routeJSON)
	}

	fmt.Println("Done")
	eventChan <- "done"

	return nil
}

var logger log.Logger

func init() {
	logger = log.NewLogfmtLogger(log.NewSyncWriter(os.Stderr))
	logger = log.With(logger, "ts", log.DefaultTimestampUTC)
}
