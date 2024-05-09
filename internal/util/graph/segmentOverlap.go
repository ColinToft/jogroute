package graph

import (
	"fmt"
	"math"
)

// The goal of this class is to handle the relation between roads and nearby sidewalks.
// The route generator should create nice looking routes that are easy to follow, with
// minimal repeated roads.
// We would like to consider a road and its sidewalks as "the same" and not have routes
// that go on the road, and then back the same way on the sidewalk, for example.
// A road segment and its corresponding sidewalk segment are considered to overlap if they
// are within a certain distance of each other and are parallel (similar angle).

type LineSegment struct {
	From   Node
	To     Node
	FromID int // Assigned ID of the from node
	ToID   int // Assigned ID of the to node
	Id     int // Assigned ID of the line segment
}

type ZoneMap struct {
	ZoneWidth   float64
	ZonesPerRow int
	Zones       map[int][]LineSegment
}

// Gets the distance from this line segment to a node.
// This does not use haversine distance, but just uses the difference of degrees (as a quick approximation).
func (l *LineSegment) DistanceTo(node Node, doPrint bool) (float64, float64) {
	lineLengthSquared := (l.To.Lat-l.From.Lat)*(l.To.Lat-l.From.Lat) + (l.To.Lon-l.From.Lon)*(l.To.Lon-l.From.Lon)
	if lineLengthSquared == 0 {
		return (node.Lat-l.From.Lat)*(node.Lat-l.From.Lat) + (node.Lon-l.From.Lon)*(node.Lon-l.From.Lon), 0
	}

	nodeMinusFrom := Node{Lat: node.Lat - l.From.Lat, Lon: node.Lon - l.From.Lon}
	ToMinusFrom := Node{Lat: l.To.Lat - l.From.Lat, Lon: l.To.Lon - l.From.Lon}
	dotproduct := nodeMinusFrom.Lat*ToMinusFrom.Lat + nodeMinusFrom.Lon*ToMinusFrom.Lon
	t := dotproduct / lineLengthSquared

	projection := Node{Lat: l.From.Lat + t*(l.To.Lat-l.From.Lat), Lon: l.From.Lon + t*(l.To.Lon-l.From.Lon)}
	// if doPrint {
	// 	fmt.Printf("[%f, %f]\n", projection.Lat, projection.Lon)
	// }

	return math.Sqrt((node.Lat-projection.Lat)*(node.Lat-projection.Lat) + (node.Lon-projection.Lon)*(node.Lon-projection.Lon)), t
}

// Checks if two lines are close and parallel.
// The lines are considered close and parallel if:
// 1. The angle between the lines is close to 0 or pi.
// 2. The distance from one of the lines to the other is close to 0
func (l *LineSegment) IsCloseAndParallel(other LineSegment, doPrint bool) bool {
	if l.Id == other.Id {
		return false
	}
	if l.From == other.From || l.From == other.To || l.To == other.From || l.To == other.To {
		return false
	}

	// Find the angle between the two lines (arc cos of the dot product of the two lines divided by the product of the lengths)
	dotproduct := (l.To.Lat-l.From.Lat)*(other.To.Lat-other.From.Lat) + (l.To.Lon-l.From.Lon)*(other.To.Lon-other.From.Lon)
	lengthOfThis := math.Sqrt((l.To.Lat-l.From.Lat)*(l.To.Lat-l.From.Lat) + (l.To.Lon-l.From.Lon)*(l.To.Lon-l.From.Lon))
	lengthOfOther := math.Sqrt((other.To.Lat-other.From.Lat)*(other.To.Lat-other.From.Lat) + (other.To.Lon-other.From.Lon)*(other.To.Lon-other.From.Lon))
	angle := math.Acos(dotproduct / (lengthOfThis * lengthOfOther))

	distanceToFrom, tFrom := l.DistanceTo(other.From, doPrint)
	distanceToTo, tTo := l.DistanceTo(other.To, doPrint)

	overlap := 0.1 // There needs to be at least a little overlap between the line segments
	notBothOnSameSide := !(tFrom < overlap && tTo < overlap) && !(tFrom > (1-overlap) && tTo > (1-overlap))

	angleBound := 0.1        // If we are within this angle of 0 or pi, we consider the lines parallel
	distanceBound := 0.00034 // If we are within this distance of the line, we consider the lines close

	// If the angle is close to 0 or pi, then the lines are parallel
	if (angle < angleBound || angle > math.Pi-angleBound) && (distanceToFrom < distanceBound || distanceToTo < distanceBound) && notBothOnSameSide {
		if doPrint {
			fmt.Printf("Angle: %f, Distance to from: %f, to: %f, tFrom: %f, tTo: %f\n", angle, distanceToFrom, distanceToTo, tFrom, tTo)
		}
		return true
	}

	return false
}

// Use grid traversal to find all zones that a segment intersects
// Based on https://gamedev.stackexchange.com/a/182143/63053
func (zoneMap *ZoneMap) GetZones(segment *LineSegment) []int {
	zones := []int{}

	x := int(math.Floor(segment.From.Lat / zoneMap.ZoneWidth))
	y := int(math.Floor(segment.From.Lon / zoneMap.ZoneWidth))

	diffX := segment.To.Lat/zoneMap.ZoneWidth - segment.From.Lat/zoneMap.ZoneWidth
	diffY := segment.To.Lon/zoneMap.ZoneWidth - segment.From.Lon/zoneMap.ZoneWidth

	stepX := int(math.Copysign(1, diffX))
	stepY := int(math.Copysign(1, diffY))

	xOffset := 0.0
	if segment.To.Lat > segment.From.Lat {
		xOffset = math.Ceil(segment.From.Lat/zoneMap.ZoneWidth) - segment.From.Lat/zoneMap.ZoneWidth
	} else {
		xOffset = segment.From.Lat/zoneMap.ZoneWidth - math.Floor(segment.From.Lat/zoneMap.ZoneWidth)
	}

	yOffset := 0.0
	if segment.To.Lon > segment.From.Lon {
		yOffset = math.Ceil(segment.From.Lon/zoneMap.ZoneWidth) - segment.From.Lon/zoneMap.ZoneWidth
	} else {
		yOffset = segment.From.Lon/zoneMap.ZoneWidth - math.Floor(segment.From.Lon/zoneMap.ZoneWidth)
	}

	distance := math.Sqrt(diffX*diffX + diffY*diffY)
	tDeltaX := distance / diffX
	tDeltaY := distance / diffY

	tMaxX := tDeltaX * xOffset
	tMaxY := tDeltaY * yOffset

	manhattanDistance := math.Abs(math.Floor(segment.To.Lat/zoneMap.ZoneWidth)-math.Floor(segment.From.Lat/zoneMap.ZoneWidth)) + math.Abs(math.Floor(segment.To.Lon/zoneMap.ZoneWidth)-math.Floor(segment.From.Lon/zoneMap.ZoneWidth))
	for i := 0; i <= int(manhattanDistance); i++ {
		zone := x + y*zoneMap.ZonesPerRow
		zones = append(zones, zone)

		if math.Abs(tMaxX) < math.Abs(tMaxY) {
			tMaxX += tDeltaX
			x += stepX
		} else {
			tMaxY += tDeltaY
			y += stepY
		}
	}

	return zones
}

func NewZoneMap(zoneWidth float64) ZoneMap {
	return ZoneMap{
		ZoneWidth:   zoneWidth,
		ZonesPerRow: int(math.Ceil(1 / zoneWidth)),
		Zones:       make(map[int][]LineSegment),
	}
}

func (zoneMap *ZoneMap) AddSegmentAndGetOverlaps(segment *LineSegment) []LineSegment {
	doPrint := false
	intersectingZones := zoneMap.GetZones(segment) // Get the zoneMap.Zones that the line segment intersects with

	// Create all zoneMap.Zones if they do not yet exist in the map
	for _, zone := range intersectingZones {
		if zoneMap.Zones[zone] == nil {
			zoneMap.Zones[zone] = make([]LineSegment, 0)
		}
	}

	// Check if the edge is close and parallel to any other edges
	// We check all zoneMap.Zones that the edge intersects with, as well as the zoneMap.Zones that are adjacent to those zoneMap.Zones
	zonesToCheck := make([]int, 0)
	for i := -1; i < 2; i++ {
		for j := -1; j < 2; j++ {
			for _, zone := range intersectingZones {
				neighbouringZone := zone + i + zoneMap.ZonesPerRow*j
				if zoneMap.Zones[neighbouringZone] != nil && !Contains(zonesToCheck, neighbouringZone) {
					zonesToCheck = append(zonesToCheck, zone+i+zoneMap.ZonesPerRow*j)
				}
			}
		}
	}

	overlappingSegments := make([]LineSegment, 0)

	for _, checkingZone := range zonesToCheck {
		for _, otherSegment := range zoneMap.Zones[checkingZone] {
			if doPrint {
				fmt.Printf("[[%f, %f], [%f, %f]] %d [[%f, %f], [%f, %f]]\n", segment.From.Lat, segment.From.Lon, segment.To.Lat, segment.To.Lon, otherSegment.Id, otherSegment.From.Lat, otherSegment.From.Lon, otherSegment.To.Lat, otherSegment.To.Lon)
			}
			if segment.IsCloseAndParallel(otherSegment, doPrint) {
				if doPrint {
					fmt.Printf("Edge %d is close and parallel to edge %d\n", segment.Id, otherSegment.Id)
				}
				overlappingSegments = append(overlappingSegments, otherSegment)

			}
		}
	}

	// Add the edge to the zoneMap.Zones
	for _, zone := range intersectingZones {
		zoneMap.Zones[zone] = append(zoneMap.Zones[zone], *segment)
		if doPrint {
			fmt.Printf("Added edge %d with coordinates [(%f, %f), (%f, %f)] to zone %d\n", segment.Id, segment.From.Lat, segment.From.Lon, segment.To.Lat, segment.To.Lon, zone)
		}
	}

	return overlappingSegments
}
