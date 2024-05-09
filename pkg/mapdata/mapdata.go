package mapdata

// Map data service implementation

import (
	"context"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"net/url"
	"os"
	"strings"

	"github.com/ColinToft/JogRoute/internal/util/mapdata"
	"github.com/go-kit/log"
)

type mapDataService struct{}

func NewService() Service {
	return &mapDataService{}
}

func (s *mapDataService) GetMapData(ctx context.Context, lat, lon, radius float64) (mapdata.MapData, error) {
	// Load the map data from OpenStreetMap

	apiUrl := "http://overpass-api.de/api/interpreter"
	data := url.Values{}
	query := "[out:json];(way(around:" + fmt.Sprintf("%f", radius) + "," + fmt.Sprintf("%f", lat) + "," + fmt.Sprintf("%f", lon) + ")[highway];>;);out body;"
	data.Set("data", query)

	fmt.Println("Querying map data")

	client := &http.Client{}
	r, _ := http.NewRequest(http.MethodPost, apiUrl, strings.NewReader(data.Encode())) // URL-encoded payload
	// r.Header.Add("Authorization", "auth_token=\"XXXXXXX\"")
	r.Header.Add("Content-Type", "application/x-www-form-urlencoded")

	resp, _ := client.Do(r)

	if resp.StatusCode != 200 {
		fmt.Println("Error getting map JSON")
	}

	defer resp.Body.Close()

	// Read the response body
	body, _ := io.ReadAll(resp.Body)

	// fmt.Println(string(body))

	// Unmarshal the JSON into a MapData struct
	var rawMapData mapdata.MapData
	json.Unmarshal(body, &rawMapData)

	return rawMapData, nil
}

var logger log.Logger

func init() {
	logger = log.NewLogfmtLogger(log.NewSyncWriter(os.Stderr))
	logger = log.With(logger, "ts", log.DefaultTimestampUTC)
}
