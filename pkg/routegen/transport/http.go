package transport

import (
	"context"
	"encoding/json"
	"os"

	"net/http"

	"github.com/ColinToft/JogRoute/internal/util/errors"
	"github.com/ColinToft/JogRoute/pkg/routegen/endpoints"

	"github.com/go-kit/log"
)

func NewHTTPHandler(ep endpoints.Set) http.Handler {
	m := http.NewServeMux()

	m.HandleFunc("/api/generate", ep.GenerateEndpoint)

	return m
}

func decodeGenerateRequest(_ context.Context, r *http.Request) (interface{}, error) {
	var req endpoints.GenerationRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		return nil, err
	}
	return req, nil
}

func encodeGenerateResponse(ctx context.Context, w http.ResponseWriter, response interface{}) error {
	if e, ok := response.(error); ok && e != nil {
		encodeError(ctx, e, w)
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
