package endpoints

import (
	"fmt"
	"math"
	"net/http"
	"os"
	"strconv"
	"time"

	"github.com/ColinToft/JogRoute/pkg/routegen"
	"github.com/go-kit/kit/log"
)

type Set struct {
	GenerateEndpoint http.HandlerFunc
}

func NewEndpointSet(svc routegen.Service) Set {
	return Set{
		GenerateEndpoint: MakeGenerateEndpoint(svc),
	}
}

func WriteServerSentEvent(w http.ResponseWriter, message string) {
	_, err := w.Write([]byte(fmt.Sprintf("data: %s\n\n", message)))
	if err != nil {
		logger.Log("Error writing SSE event data", err)
		return
	}
	w.(http.Flusher).Flush()
}

// Test server-sent event functionality by making a test server-sent event endpoint that returns the generated routes.j
func MakeGenerateEndpoint(svc routegen.Service) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/event-stream")
		w.Header().Set("Cache-Control", "no-cache")
		w.Header().Set("Connection", "keep-alive")
		w.Header().Set("Access-Control-Allow-Origin", "*")

		// Flush the headers to establish SSE connection
		w.WriteHeader(http.StatusOK)
		w.(http.Flusher).Flush()

		// Send a random string once per second
		eventChan := make(chan string)

		// Temp: create default heuristics for the request
		// Heuristics (higher is better)
		heuristics := make(map[string]float64)
		heuristics["sidewalk"] = 0.5
		heuristics["footway"] = 1
		heuristics["crossing"] = -100
		heuristics["traffic_signals"] = -100
		heuristics["path"] = 5
		heuristics["service"] = -5

		// Read the request parameters from the URL
		distance, err := strconv.ParseFloat(r.URL.Query().Get("distance"), 64)
		if err != nil {
			logger.Log("Error parsing distance parameter", err)
			return
		}

		lat, err := strconv.ParseFloat(r.URL.Query().Get("lat"), 64)
		if err != nil {
			logger.Log("Error parsing lat parameter", err)
			return
		}

		lon, err := strconv.ParseFloat(r.URL.Query().Get("lon"), 64)
		if err != nil {
			logger.Log("Error parsing lon parameter", err)
			return
		}

		count := 10
		distanceRange := 50.0
		minCycleLength := math.Min(13000, distance-500)
		minDistance := distance - distanceRange
		maxDistance := distance + distanceRange

		radius := minDistance / 2

		quitChannel := make(chan bool)
		go svc.GenRoutes(quitChannel, count, minDistance, maxDistance, minCycleLength, heuristics, lat, lon, radius, eventChan)

		timeout := time.After(90 * time.Second)

	out:
		for {
			select {
			case event, ok := <-eventChan:
				if !ok {
					break out
				}
				if event == "done" {
					break out
				}
				if event == "error" {
					WriteServerSentEvent(w, "Error")
					break out
				}

				WriteServerSentEvent(w, event)
			case <-r.Context().Done():
				break out
			case <-timeout:
				// Quit the goroutine
				quitChannel <- true

				WriteServerSentEvent(w, "Timeout")
				break out
			}
		}

		fmt.Println("Connection closed")
		// Send a message to indicate that the connection has been closed
		WriteServerSentEvent(w, "Connection closed")
		w.(http.Flusher).Flush()
	}
}

var logger log.Logger

func init() {
	logger = log.NewLogfmtLogger(log.NewSyncWriter(os.Stderr))
	logger = log.With(logger, "ts", log.DefaultTimestampUTC)
}
