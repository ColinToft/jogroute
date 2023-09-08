package errors

import "errors"

var (
	ErrUnknown = errors.New("unknown argument passed")

	ErrInvalidArgument = errors.New("invalid argument passed")
)
