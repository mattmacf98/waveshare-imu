package main

import (
	"context"
	"waveshareimu"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	movementsensor "go.viam.com/rdk/components/movementsensor"
)

func main() {
	err := realMain()
	if err != nil {
		panic(err)
	}
}

func realMain() error {
	ctx := context.Background()
	logger := logging.NewLogger("cli")

	deps := resource.Dependencies{}
	// can load these from a remote machine if you need

	cfg := waveshareimu.Config{}

	thing, err := waveshareimu.NewWaveshare10DofImuV2(ctx, deps, movementsensor.Named("foo"), &cfg, logger)
	if err != nil {
		return err
	}
	defer thing.Close(ctx)

	return nil
}
