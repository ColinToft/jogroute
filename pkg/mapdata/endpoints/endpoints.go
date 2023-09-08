package endpoints

import (
	"context"
	"net/http"
	"os"

	"github.com/ColinToft/JogRoute/pkg/mapdata"
	"github.com/go-kit/kit/endpoint"
	"github.com/go-kit/kit/log"
)

type Set struct {
	GenerateEndpoint endpoint.Endpoint
	SSEEndpoint      http.HandlerFunc
}

func NewEndpointSet(svc mapdata.Service) Set {
	return Set{
		GenerateEndpoint: MakeGenerateEndpoint(svc),
	}
}

func MakeGenerateEndpoint(svc mapdata.Service) endpoint.Endpoint {
	return func(ctx context.Context, request interface{}) (interface{}, error) {
		req := request.(GenerationRequest)
		data, err := svc.GetMapData(ctx, req.Lat, req.Lon, req.Radius)
		if err != nil {
			return data, err
		}
		return data, nil
	}
}

var logger log.Logger

func init() {
	logger = log.NewLogfmtLogger(log.NewSyncWriter(os.Stderr))
	logger = log.With(logger, "ts", log.DefaultTimestampUTC)
}
