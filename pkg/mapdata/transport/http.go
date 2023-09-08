package transport

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"strconv"

	"net/http"

	"github.com/ColinToft/JogRoute/internal/util/errors"
	"github.com/ColinToft/JogRoute/pkg/mapdata/endpoints"

	httptransport "github.com/go-kit/kit/transport/http"
	"github.com/go-kit/log"
)

func NewHTTPHandler(ep endpoints.Set) http.Handler {
	m := http.NewServeMux()

	m.Handle("/api/mapdata", httptransport.NewServer(
		ep.GenerateEndpoint,
		decodeGenerateRequest,
		encodeGenerateResponse,
	))

	return m
}

func decodeGenerateRequest(_ context.Context, r *http.Request) (interface{}, error) {
	// Decode the query parameters into a struct
	var req endpoints.GenerationRequest
	var err error
	req.Lat, err = strconv.ParseFloat(r.URL.Query().Get("lat"), 64)
	if err != nil {
		return nil, err
	}
	req.Lon, err = strconv.ParseFloat(r.URL.Query().Get("lon"), 64)
	if err != nil {
		return nil, err
	}
	req.Radius, err = strconv.ParseFloat(r.URL.Query().Get("radius"), 64)
	if err != nil {
		return nil, err
	}

	return req, nil
}

func encodeGenerateResponse(ctx context.Context, w http.ResponseWriter, response interface{}) error {
	if e, ok := response.(error); ok && e != nil {
		encodeError(ctx, e, w)
		fmt.Println("Error encoding response")
		return nil
	}
	return json.NewEncoder(w).Encode(response)
}

func encodeError(_ context.Context, err error, w http.ResponseWriter) {
	w.Header().Set("Content-Type", "application/json; charset=utf-8")
	switch err {
	case errors.ErrUnknown:
		w.WriteHeader(http.StatusNotFound)
	case errors.ErrInvalidArgument:
		w.WriteHeader(http.StatusBadRequest)
	default:
		w.WriteHeader(http.StatusInternalServerError)
	}
	json.NewEncoder(w).Encode(map[string]interface{}{
		"error": err.Error(),
	})
}

var logger log.Logger

func init() {
	logger = log.NewLogfmtLogger(log.NewSyncWriter(os.Stderr))
	logger = log.With(logger, "ts", log.DefaultTimestampUTC)
}
