package main

import (
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"net/url"
	"strings"

	"github.com/ColinToft/JogRoute/internal/util/mapdata"
)

func getMapJSON(lat, lon float64) mapdata.MapData {
	apiUrl := "http://overpass-api.de/api/interpreter"
	data := url.Values{}
	query := "[out:json];(way(around:7000," + fmt.Sprintf("%f", lat) + "," + fmt.Sprintf("%f", lon) + ")[highway];>;);out body;"
	data.Set("data", query)

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

	// Unmarshal the JSON into a MapData struct
	var rawMapData mapdata.MapData
	json.Unmarshal(body, &rawMapData)

	// TODO mark crossings near traffic lights as crossings
	// This can be done by looking for nodes that are part of a crossing

	return rawMapData
}
