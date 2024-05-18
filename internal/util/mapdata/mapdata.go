package mapdata

import (
	"time"
)

type MapData struct {
	Version   float64 `json:"version"`
	Generator string  `json:"generator"`
	Osm3S     struct {
		TimestampOsmBase time.Time `json:"timestamp_osm_base"`
		Copyright        string    `json:"copyright"`
	} `json:"osm3s"`
	Nodes    map[int]OSMNode  // Used to store all nodes in the map
	TrueIDs  []int            // Used to store the true IDs of the nodes in the map
	Elements []MapDataElement `json:"elements"`
}

type MapDataElement struct {
	Type  string  `json:"type"`
	ID    int     `json:"id"`
	Lat   float64 `json:"lat,omitempty"`
	Lon   float64 `json:"lon,omitempty"`
	Nodes []int   `json:"nodes,omitempty"`
	Tags  struct {
		Access         string `json:"access,omitempty"`
		Bicycle        string `json:"bicycle,omitempty"`
		Building       string `json:"building,omitempty"`
		Crossing       string `json:"crossing,omitempty"`
		CrossingIsland string `json:"crossing:island,omitempty"`
		Cycleway       string `json:"cycleway,omitempty"`
		Direction      string `json:"direction,omitempty"`
		Footway        string `json:"footway,omitempty"`
		Highway        string `json:"highway,omitempty"`
		Lanes          string `json:"lanes,omitempty"`
		Maxspeed       string `json:"maxspeed,omitempty"`
		Name           string `json:"name,omitempty"`
		Ref            string `json:"ref,omitempty"`
		Service        string `json:"service,omitempty"`
		Sidewalk       string `json:"sidewalk,omitempty"`
		Source         string `json:"source,omitempty"`
		Surface        string `json:"surface,omitempty"`
	} `json:"tags,omitempty"`
}

type OSMNode struct {
	assignedId int
	lat        float64
	lon        float64
}
