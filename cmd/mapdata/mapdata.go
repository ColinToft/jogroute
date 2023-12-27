package main

// The code to start and stop the HTTP server.

import (
	"context"
	"net"
	"net/http"
	"os"
	"os/signal"
	"syscall"

	"github.com/ColinToft/JogRoute/pkg/mapdata"
	"github.com/ColinToft/JogRoute/pkg/mapdata/endpoints"
	"github.com/ColinToft/JogRoute/pkg/mapdata/transport"

	"github.com/go-kit/kit/log"
)

const defaultPort = "8081"

func main() {
	var (
		logger log.Logger
		// 0.0.0.0 for Docker, 127.0.0.1 for local
		httpAddr = net.JoinHostPort("127.0.0.1", envString("PORT", defaultPort))
	)

	logger = log.NewLogfmtLogger(log.NewSyncWriter(os.Stderr))
	logger = log.With(logger, "ts", log.DefaultTimestampUTC)

	var (
		service     = mapdata.NewService()
		endpoints   = endpoints.NewEndpointSet(service)
		httpHandler = transport.NewHTTPHandler(endpoints)
	)

	httpListener, err := net.Listen("tcp", httpAddr)
	if err != nil {
		logger.Log("transport", "HTTP", "during", "Listen", "err", err)
		os.Exit(1)
	}

	httpServer := &http.Server{
		Handler: httpHandler,
	}

	go func() {
		logger.Log("transport", "HTTP", "addr", httpAddr)
		err := httpServer.Serve(httpListener)
		if err != nil {
			logger.Log("transport", "HTTP", "during", "Serve", "err", err)
		}
	}()

	// Wait for an interrupt signal to stop the server.
	// This will block the main goroutine.
	// The server will be stopped when the main goroutine exits.

	c := make(chan os.Signal, 1)
	signal.Notify(c, syscall.SIGINT, syscall.SIGTERM)
	sig := <-c
	logger.Log("signal", sig)

	// Stop the server gracefully.
	// This will block the main goroutine until the server is stopped.

	err = httpServer.Shutdown(context.Background())
	if err != nil {
		logger.Log("transport", "HTTP", "during", "Shutdown", "err", err)
	}
	httpListener.Close()

	logger.Log("transport", "HTTP", "status", "stopped")
}

func envString(key, defaultValue string) string {
	if value, ok := os.LookupEnv(key); ok {
		return value
	}
	return defaultValue
}
