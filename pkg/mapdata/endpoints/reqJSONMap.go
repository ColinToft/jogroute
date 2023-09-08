package endpoints

// A request to generate routes
type GenerationRequest struct {
	Lat    float64 `json:"lat"`
	Lon    float64 `json:"lon"`
	Radius float64 `json:"radius"`
}
