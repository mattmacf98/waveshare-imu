package waveshareimu

import (
	"fmt"
	"math"
	"strings"
	"sync"
	"time"

	"periph.io/x/conn/v3/i2c"
	"periph.io/x/conn/v3/i2c/i2creg"
	"periph.io/x/host/v3"
)

const (
	defaultMPUAddr = 0x68

	mpuWhoAmI     = 0x75
	mpuWhoAmI9250 = 0x71
	mpuWhoAmI6050 = 0x68

	mpuPwrMgmt1   = 0x6B
	mpuAccelXoutH = 0x3B
	mpuTempOutH   = 0x41
	mpuGyroXoutH  = 0x43

	accelScale = 16384.0 // ±2g
	gyroScale  = 131.0   // ±250 dps
	gravityMS2 = 9.80665
)

var mpuCandidateAddrs = []int{0x68, 0x69}

type mpu9250 struct {
	dev *i2c.Dev
	bus i2c.BusCloser
	mu  sync.Mutex
}

type imuReadings struct {
	AccelG   [3]float64
	GyroDPS  [3]float64
	TempC    float64
	RollDeg  float64
	PitchDeg float64
}

func newMPU9250(busName string, addr int) (*mpu9250, error) {
	if _, err := host.Init(); err != nil {
		return nil, fmt.Errorf("periph host init: %w", err)
	}

	bus, err := i2creg.Open(busName)
	if err != nil {
		return nil, fmt.Errorf("open I2C bus %q: %w (available buses: %s)", busName, err, listI2CBuses())
	}

	addrsToTry := candidateAddrs(addr)
	var lastErr error
	for _, candidate := range addrsToTry {
		dev := &i2c.Dev{
			Bus:  bus,
			Addr: uint16(candidate),
		}

		if err := wakeMPU(dev); err != nil {
			lastErr = fmt.Errorf("address 0x%02X: %w", candidate, err)
			continue
		}

		whoAmI, err := readByte(dev, mpuWhoAmI)
		if err != nil {
			lastErr = fmt.Errorf("address 0x%02X: read WHO_AM_I: %w", candidate, err)
			continue
		}
		if whoAmI != mpuWhoAmI9250 && whoAmI != mpuWhoAmI6050 {
			lastErr = fmt.Errorf("address 0x%02X: unexpected WHO_AM_I value 0x%02X (expected 0x71 for MPU9250 or 0x68 for MPU6050)", candidate, whoAmI)
			continue
		}

		return &mpu9250{dev: dev, bus: bus}, nil
	}

	_ = bus.Close()
	return nil, fmt.Errorf(
		"no MPU9250/MPU6050 found on I2C bus %q (tried addresses %v): %w; verify wiring and run `i2cdetect -y 1`, try setting i2c_bus to \"1\"",
		displayBusName(busName),
		addrsToTry,
		lastErr,
	)
}

func candidateAddrs(addr int) []int {
	if addr != 0 {
		return []int{addr}
	}
	return mpuCandidateAddrs
}

func displayBusName(busName string) string {
	if busName == "" {
		return `"" (first available bus)`
	}
	return busName
}

func listI2CBuses() string {
	refs := i2creg.All()
	if len(refs) == 0 {
		return "none detected"
	}
	names := make([]string, 0, len(refs))
	for _, ref := range refs {
		names = append(names, ref.Name)
	}
	return strings.Join(names, ", ")
}

func wakeMPU(dev *i2c.Dev) error {
	if err := dev.Tx([]byte{mpuPwrMgmt1, 0x00}, nil); err != nil {
		return err
	}
	time.Sleep(100 * time.Millisecond)
	return nil
}

func readByte(dev *i2c.Dev, reg byte) (byte, error) {
	buf := make([]byte, 1)
	if err := dev.Tx([]byte{reg}, buf); err != nil {
		return 0, err
	}
	return buf[0], nil
}

func (m *mpu9250) close() error {
	if m.bus == nil {
		return nil
	}
	return m.bus.Close()
}

func (m *mpu9250) readWord(reg byte) (int16, error) {
	buf := make([]byte, 2)
	if err := m.dev.Tx([]byte{reg}, buf); err != nil {
		return 0, err
	}
	return int16(uint16(buf[0])<<8 | uint16(buf[1])), nil
}

func (m *mpu9250) read() (*imuReadings, error) {
	m.mu.Lock()
	defer m.mu.Unlock()

	axRaw, err := m.readWord(mpuAccelXoutH)
	if err != nil {
		return nil, err
	}
	ayRaw, err := m.readWord(mpuAccelXoutH + 2)
	if err != nil {
		return nil, err
	}
	azRaw, err := m.readWord(mpuAccelXoutH + 4)
	if err != nil {
		return nil, err
	}
	tempRaw, err := m.readWord(mpuTempOutH)
	if err != nil {
		return nil, err
	}
	gxRaw, err := m.readWord(mpuGyroXoutH)
	if err != nil {
		return nil, err
	}
	gyRaw, err := m.readWord(mpuGyroXoutH + 2)
	if err != nil {
		return nil, err
	}
	gzRaw, err := m.readWord(mpuGyroXoutH + 4)
	if err != nil {
		return nil, err
	}

	ax := float64(axRaw) / accelScale
	ay := float64(ayRaw) / accelScale
	az := float64(azRaw) / accelScale

	gx := float64(gxRaw) / gyroScale
	gy := float64(gyRaw) / gyroScale
	gz := float64(gzRaw) / gyroScale

	temp := float64(tempRaw)/333.87 + 21.0

	roll := math.Atan2(ay, az) * 180 / math.Pi
	pitch := math.Atan2(-ax, math.Sqrt(ay*ay+az*az)) * 180 / math.Pi

	return &imuReadings{
		AccelG:   [3]float64{ax, ay, az},
		GyroDPS:  [3]float64{gx, gy, gz},
		TempC:    temp,
		RollDeg:  roll,
		PitchDeg: pitch,
	}, nil
}
