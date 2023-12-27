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

	// Get the URL of the map data microservice, if it does not exist use localhost
	mapDataURL := os.Getenv("MAP_DATA_URL")
	if mapDataURL == "" {
		mapDataURL = "http://localhost:8081"
	}

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

	// Read the response body
	body, _ := io.ReadAll(response.Body)

	// Unmarshal the JSON into a MapData struct
	var rawMapData mapdata.MapData
	json.Unmarshal(body, &rawMapData)

	// TODO mark crossings near traffic lights as crossings
	// This can be done by looking for nodes that are part of a crossing

	fmt.Println("Converting to graph")

	routeFinder := NewRouteFinder(graph.NewGraph(&rawMapData, lat, lon, heuristics))

	fmt.Println("Finding routes")

	// Enable runtime profiling
	f, err := os.Create("cpu.prof")
	if err != nil {
		fmt.Println(err)
		return err
	}
	defer f.Close()
	pprof.StartCPUProfile(f)
	defer pprof.StopCPUProfile()

	routeFinder.Initialize()

	for i := 0; i < count; i++ {
		route := routeFinder.FindNextRoute(quit, minDistance, maxDistance, 0, minCycleLength)

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
