package mapdata

import (
	"context"

	"github.com/ColinToft/JogRoute/internal/util/mapdata"
)

type Service interface {
	GetMapData(ctx context.Context, lat, lon, radius float64) (mapdata.MapData, error)
}
