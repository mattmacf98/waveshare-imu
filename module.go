package waveshareimu

import (
	"context"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"
	movementsensor "go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

const degToRad = math.Pi / 180

var Waveshare10DofImuV2 = resource.NewModel("mattmacf", "waveshare-imu", "waveshare-10-dof-imu-v2")

func init() {
	resource.RegisterComponent(movementsensor.API, Waveshare10DofImuV2,
		resource.Registration[movementsensor.MovementSensor, *Config]{
			Constructor: newWaveshareImuWaveshare10DofImuV2,
		},
	)
}

type Config struct {
	I2CBus     string `json:"i2c_bus,omitempty"`
	I2CAddress int    `json:"i2c_address,omitempty"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.I2CAddress != 0 && (cfg.I2CAddress < 0x03 || cfg.I2CAddress > 0x77) {
		return nil, nil, fmt.Errorf("%s: i2c_address must be a valid 7-bit I2C address (0x03-0x77), or omit it to auto-detect 0x68/0x69", path)
	}
	return nil, nil, nil
}

type waveshareImuWaveshare10DofImuV2 struct {
	resource.AlwaysRebuild
	resource.Named

	name resource.Name

	logger logging.Logger
	cfg    *Config

	imu *mpu9250
}

func newWaveshareImuWaveshare10DofImuV2(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (movementsensor.MovementSensor, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewWaveshare10DofImuV2(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewWaveshare10DofImuV2(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (movementsensor.MovementSensor, error) {
	imu, err := newMPU9250(conf.I2CBus, conf.I2CAddress)
	if err != nil {
		return nil, fmt.Errorf("failed to initialize MPU9250: %w", err)
	}

	addr := conf.I2CAddress
	if addr == 0 {
		addr = int(imu.dev.Addr)
	}
	chip := "MPU6050"
	if imu.magnetometerSupported() {
		chip = "MPU9250"
	}
	logger.Infof("%s started on I2C bus %q at address 0x%02X", chip, conf.I2CBus, addr)

	return &waveshareImuWaveshare10DofImuV2{
		name:   name,
		logger: logger,
		cfg:    conf,
		imu:    imu,
	}, nil
}

func (s *waveshareImuWaveshare10DofImuV2) Name() resource.Name {
	return s.name
}

func (s *waveshareImuWaveshare10DofImuV2) Position(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
	return nil, 0, movementsensor.ErrMethodUnimplementedPosition
}

func (s *waveshareImuWaveshare10DofImuV2) LinearVelocity(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
	return r3.Vector{}, movementsensor.ErrMethodUnimplementedLinearVelocity
}

func (s *waveshareImuWaveshare10DofImuV2) AngularVelocity(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
	readings, err := s.imu.read()
	if err != nil {
		return spatialmath.AngularVelocity{}, err
	}
	return spatialmath.AngularVelocity{
		X: readings.GyroDPS[0],
		Y: readings.GyroDPS[1],
		Z: readings.GyroDPS[2],
	}, nil
}

func (s *waveshareImuWaveshare10DofImuV2) LinearAcceleration(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
	readings, err := s.imu.read()
	if err != nil {
		return r3.Vector{}, err
	}
	return r3.Vector{
		X: readings.AccelG[0] * gravityMS2,
		Y: readings.AccelG[1] * gravityMS2,
		Z: readings.AccelG[2] * gravityMS2,
	}, nil
}

func (s *waveshareImuWaveshare10DofImuV2) CompassHeading(ctx context.Context, extra map[string]interface{}) (float64, error) {
	if !s.imu.magnetometerSupported() {
		return 0, movementsensor.ErrMethodUnimplementedCompassHeading
	}
	readings, err := s.imu.read()
	if err != nil {
		return 0, err
	}
	return readings.HeadingDeg, nil
}

func (s *waveshareImuWaveshare10DofImuV2) Orientation(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
	readings, err := s.imu.read()
	if err != nil {
		return nil, err
	}
	yaw := 0.0
	if readings.HasMag {
		yaw = readings.HeadingDeg * degToRad
	}
	return &spatialmath.EulerAngles{
		Roll:  readings.RollDeg * degToRad,
		Pitch: readings.PitchDeg * degToRad,
		Yaw:   yaw,
	}, nil
}

func (s *waveshareImuWaveshare10DofImuV2) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	readings, err := movementsensor.DefaultAPIReadings(ctx, s, extra)
	if err != nil {
		return nil, err
	}

	imu, err := s.imu.read()
	if err != nil {
		return nil, err
	}

	readings["temperature_c"] = imu.TempC
	readings["roll_deg"] = imu.RollDeg
	readings["pitch_deg"] = imu.PitchDeg
	readings["accel_g"] = map[string]float64{
		"x": imu.AccelG[0],
		"y": imu.AccelG[1],
		"z": imu.AccelG[2],
	}
	readings["gyro_dps"] = map[string]float64{
		"x": imu.GyroDPS[0],
		"y": imu.GyroDPS[1],
		"z": imu.GyroDPS[2],
	}
	if imu.HasMag {
		readings["mag_ut"] = map[string]float64{
			"x": imu.MagUT[0],
			"y": imu.MagUT[1],
			"z": imu.MagUT[2],
		}
		readings["heading_deg"] = imu.HeadingDeg
	}

	return readings, nil
}

func (s *waveshareImuWaveshare10DofImuV2) Accuracy(ctx context.Context, extra map[string]interface{}) (*movementsensor.Accuracy, error) {
	return movementsensor.UnimplementedOptionalAccuracies(), nil
}

func (s *waveshareImuWaveshare10DofImuV2) Properties(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
	return &movementsensor.Properties{
		AngularVelocitySupported:    true,
		LinearAccelerationSupported: true,
		OrientationSupported:        true,
		CompassHeadingSupported:     s.imu.magnetometerSupported(),
	}, nil
}

func (s *waveshareImuWaveshare10DofImuV2) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, fmt.Errorf("not implemented")
}

func (s *waveshareImuWaveshare10DofImuV2) Status(ctx context.Context) (map[string]interface{}, error) {
	addr := s.cfg.I2CAddress
	if addr == 0 && s.imu != nil && s.imu.dev != nil {
		addr = int(s.imu.dev.Addr)
	}
	return map[string]interface{}{
		"i2c_bus":     s.cfg.I2CBus,
		"i2c_address": addr,
	}, nil
}

func (s *waveshareImuWaveshare10DofImuV2) Close(context.Context) error {
	if s.imu != nil {
		return s.imu.close()
	}
	return nil
}
