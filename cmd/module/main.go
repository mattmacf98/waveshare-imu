package main

import (
	"waveshareimu"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	movementsensor "go.viam.com/rdk/components/movementsensor"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(resource.APIModel{ movementsensor.API, waveshareimu.Waveshare10DofImuV2})
}
