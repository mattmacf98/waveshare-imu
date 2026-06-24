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

	mpuIntPinCfg  = 0x37
	mpuUserCtrl   = 0x6A
	mpuPwrMgmt1   = 0x6B
	mpuAccelXoutH = 0x3B
	mpuTempOutH   = 0x41
	mpuGyroXoutH  = 0x43

	ak8963Addr           = 0x0C
	ak8963WhoAmI         = 0x00
	ak8963WhoAmIExpected = 0x48
	ak8963St1            = 0x02
	ak8963XoutL          = 0x03
	ak8963St2            = 0x09
	ak8963Cntl           = 0x0A
	ak8963Asax           = 0x10

	accelScale = 16384.0 // ±2g
	gyroScale  = 131.0   // ±250 dps
	magScale   = 4912.0 / 32768.0 // 16-bit mode, µT per LSB
	gravityMS2 = 9.80665
)

var mpuCandidateAddrs = []int{0x68, 0x69}

type mpu9250 struct {
	dev     *i2c.Dev
	magDev  *i2c.Dev
	bus     i2c.BusCloser
	mu      sync.Mutex
	hasMag  bool
	magSens [3]float64
}

type imuReadings struct {
	AccelG     [3]float64
	GyroDPS    [3]float64
	MagUT      [3]float64
	TempC      float64
	RollDeg    float64
	PitchDeg   float64
	HeadingDeg float64
	HasMag     bool
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

		imu := &mpu9250{dev: dev, bus: bus}
		if whoAmI == mpuWhoAmI9250 {
			if err := enableI2CBypass(dev); err != nil {
				lastErr = fmt.Errorf("address 0x%02X: %w", candidate, err)
				continue
			}
			if err := imu.initAK8963(); err != nil {
				lastErr = fmt.Errorf("address 0x%02X: init magnetometer: %w", candidate, err)
				continue
			}
		}

		return imu, nil
	}

	_ = bus.Close()
	return nil, fmt.Errorf(
		"no MPU9250/MPU6050 found on I2C bus %q (tried addresses %v): %w; verify wiring and run `i2cdetect -y 1`, try setting i2c_bus to \"1\"",
		displayBusName(busName),
		addrsToTry,
		lastErr,
	)
}

func (m *mpu9250) magnetometerSupported() bool {
	return m.hasMag
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
	if err := writeByte(dev, mpuPwrMgmt1, 0x00); err != nil {
		return err
	}
	time.Sleep(100 * time.Millisecond)
	return nil
}

func writeByte(dev *i2c.Dev, reg, val byte) error {
	return dev.Tx([]byte{reg, val}, nil)
}

func readByte(dev *i2c.Dev, reg byte) (byte, error) {
	buf := make([]byte, 1)
	if err := dev.Tx([]byte{reg}, buf); err != nil {
		return 0, err
	}
	return buf[0], nil
}

func enableI2CBypass(dev *i2c.Dev) error {
	if err := writeByte(dev, mpuUserCtrl, 0x00); err != nil {
		return fmt.Errorf("disable I2C master: %w", err)
	}
	intPinCfg, err := readByte(dev, mpuIntPinCfg)
	if err != nil {
		return fmt.Errorf("read INT_PIN_CFG: %w", err)
	}
	if err := writeByte(dev, mpuIntPinCfg, intPinCfg|0x02); err != nil {
		return fmt.Errorf("enable I2C bypass: %w", err)
	}
	time.Sleep(10 * time.Millisecond)
	return nil
}

func (m *mpu9250) initAK8963() error {
	mag := &i2c.Dev{Bus: m.bus, Addr: ak8963Addr}

	whoAmI, err := readByte(mag, ak8963WhoAmI)
	if err != nil {
		return fmt.Errorf("read WHO_AM_I: %w", err)
	}
	if whoAmI != ak8963WhoAmIExpected {
		return fmt.Errorf("unexpected WHO_AM_I 0x%02X (expected 0x%02X)", whoAmI, ak8963WhoAmIExpected)
	}

	if err := writeByte(mag, ak8963Cntl, 0x00); err != nil {
		return fmt.Errorf("power down: %w", err)
	}
	time.Sleep(10 * time.Millisecond)

	if err := writeByte(mag, ak8963Cntl, 0x0F); err != nil {
		return fmt.Errorf("enter fuse ROM: %w", err)
	}
	time.Sleep(10 * time.Millisecond)

	asax, err := readByte(mag, ak8963Asax)
	if err != nil {
		return fmt.Errorf("read ASAX: %w", err)
	}
	asay, err := readByte(mag, ak8963Asax+1)
	if err != nil {
		return fmt.Errorf("read ASAY: %w", err)
	}
	asaz, err := readByte(mag, ak8963Asax+2)
	if err != nil {
		return fmt.Errorf("read ASAZ: %w", err)
	}

	if err := writeByte(mag, ak8963Cntl, 0x00); err != nil {
		return fmt.Errorf("power down after fuse ROM: %w", err)
	}
	time.Sleep(10 * time.Millisecond)

	// 16-bit resolution, 100 Hz continuous sampling.
	if err := writeByte(mag, ak8963Cntl, 0x16); err != nil {
		return fmt.Errorf("start continuous mode: %w", err)
	}
	time.Sleep(10 * time.Millisecond)

	m.magDev = mag
	m.magSens = [3]float64{
		float64(asax-128)/256.0 + 1.0,
		float64(asay-128)/256.0 + 1.0,
		float64(asaz-128)/256.0 + 1.0,
	}
	m.hasMag = true
	return nil
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

func (m *mpu9250) readMag() ([3]float64, error) {
	for attempt := 0; attempt < 5; attempt++ {
		st1, err := readByte(m.magDev, ak8963St1)
		if err != nil {
			return [3]float64{}, err
		}
		if st1&0x01 != 0 {
			break
		}
		if attempt == 4 {
			return [3]float64{}, fmt.Errorf("magnetometer data not ready")
		}
		time.Sleep(2 * time.Millisecond)
	}

	buf := make([]byte, 6)
	if err := m.magDev.Tx([]byte{ak8963XoutL}, buf); err != nil {
		return [3]float64{}, err
	}

	st2, err := readByte(m.magDev, ak8963St2)
	if err != nil {
		return [3]float64{}, err
	}
	if st2&0x08 != 0 {
		return [3]float64{}, fmt.Errorf("magnetometer overflow")
	}

	rawX := int16(uint16(buf[1])<<8 | uint16(buf[0]))
	rawY := int16(uint16(buf[3])<<8 | uint16(buf[2]))
	rawZ := int16(uint16(buf[5])<<8 | uint16(buf[4]))

	return [3]float64{
		float64(rawX) * magScale * m.magSens[0],
		float64(rawY) * magScale * m.magSens[1],
		float64(rawZ) * magScale * m.magSens[2],
	}, nil
}

func tiltCompensatedHeading(mx, my, mz, rollDeg, pitchDeg float64) float64 {
	rollRad := rollDeg * math.Pi / 180
	pitchRad := pitchDeg * math.Pi / 180

	cosRoll := math.Cos(rollRad)
	sinRoll := math.Sin(rollRad)
	cosPitch := math.Cos(pitchRad)
	sinPitch := math.Sin(pitchRad)

	xh := mx*cosPitch + mz*sinPitch
	yh := mx*sinRoll*sinPitch + my*cosRoll - mz*sinRoll*cosPitch

	heading := math.Atan2(yh, xh) * 180 / math.Pi
	if heading < 0 {
		heading += 360
	}
	return heading
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

	readings := &imuReadings{
		AccelG:   [3]float64{ax, ay, az},
		GyroDPS:  [3]float64{gx, gy, gz},
		TempC:    temp,
		RollDeg:  roll,
		PitchDeg: pitch,
		HasMag:   m.hasMag,
	}

	if m.hasMag {
		mag, err := m.readMag()
		if err != nil {
			return nil, err
		}
		readings.MagUT = mag
		readings.HeadingDeg = tiltCompensatedHeading(mag[0], mag[1], mag[2], roll, pitch)
	}

	return readings, nil
}
