package main

import (
	"bytes"
	"device/nrf"
	"encoding/binary"
	"machine"
	"math"
	"strconv"
	"time"

	"tinygo.org/x/bluetooth"
)

const (
	ScanDuration = 500 * time.Millisecond
	// So that it fits with 3 digits.
	ParamScale = 99

	DEBUG = false
)

// Animation IDs
const (
	AnimStatic  = 1
	AnimBlink   = 2
	AnimBreathe = 3

	SetScanInterval = 9
)

type AnimationState struct {
	id    int
	param int

	stepCount  int32        // Current position in the fade cycle
	blinkState bool         // Current state for blink (true = ON, false = OFF)
	ticker     *time.Ticker // Ticker for the animation
}

var (
	DeviceIdString string
	MyDevId        uint16 = func() uint16 {
		id, _ := strconv.Atoi(DeviceIdString)
		return uint16(id)
	}()

	adapter = bluetooth.DefaultAdapter
	led     = machine.LED
	onOff   = machine.D111
	touch   = machine.D010
	pwm     = machine.PWM0
	pwmTop  = pwm.Top()

	anim = AnimationState{
		id:     AnimStatic,
		param:  0,
		ticker: time.NewTicker(30 * time.Second),
	}

	touchState = false
	animPin    = func() machine.Pin {
		if DEBUG {
			return led
		}
		return onOff
	}()
	scanInterval = func() time.Duration {
		if DEBUG {
			return 5 * time.Second
		}
		return 30 * time.Second
	}()
	pwmCh uint8
)

func blink(n int) {
	for i := 0; i < n; i++ {
		led.High()
		time.Sleep(50 * time.Millisecond)
		led.Low()
		time.Sleep(50 * time.Millisecond)
	}
}

type beaconT struct {
	Major uint16
	Minor uint16
}

func scanPlease(cc chan<- beaconT) {
	// println("scanning for 63d88905a2d64a7e8b2ed0de379c232a")
	t := time.Now()
	adapter.Scan(func(adapter *bluetooth.Adapter, device bluetooth.ScanResult) {
		if time.Since(t) > ScanDuration {
			adapter.StopScan()
			return
		}
		for _, e := range device.AdvertisementPayload.ManufacturerData() {
			if e.CompanyID == 76 && bytes.Compare(e.Data[2:2+16], []byte{0x63, 0xd8, 0x89, 0x05, 0xa2, 0xd6, 0x4a, 0x7e, 0x8b, 0x2e, 0xd0, 0xde, 0x37, 0x9c, 0x23, 0x2a}) == 0 {
				adapter.StopScan()
				r := bytes.NewReader(e.Data[2+16:])
				var b beaconT
				must("binary.Read", binary.Read(r, binary.BigEndian, &b))
				if !(b.Major == 0 || b.Major == MyDevId) {
					return
				}
				cc <- b
				return
			}
		}
	})
}

func main() {
	onOff.Configure(machine.PinConfig{Mode: machine.PinOutput})
	onOff.Low()
	led.Configure(machine.PinConfig{Mode: machine.PinOutput})
	led.Low()

	must("adapter.Enable", adapter.Enable())
	blink(2)
	disablePeripherals()

	touch.Configure(machine.PinConfig{Mode: machine.PinInputPulldown})
	must("SetInterrupt", touch.SetInterrupt(machine.PinToggle, func(p machine.Pin) {
		var t bool = p.Get()
		if touchState != t {
			touchState = t
			var brightness int = 0
			if touchState {
				brightness = ParamScale // Maximum.
			}
			anim.setAnimation(AnimStatic, brightness)
		}
	}))

	scanT := time.NewTicker(scanInterval)
	bChan := make(chan beaconT, 1)

	for {
		select {
		case beacon, ok := <-bChan:
			// println("got beacon", beacon.Major, beacon.Minor, ok)
			if ok {
				nId, nParam := beacon.Minor/100, beacon.Minor%100
				if nId == SetScanInterval {
					if nParam >= 1 {
						scanT.Reset(time.Second * time.Duration(nParam))
					}
				} else {
					anim.setAnimation(int(nId), int(nParam))
				}
			}
		case <-scanT.C:
			scanPlease(bChan)
		case <-anim.ticker.C:
			anim.tick()
		}
	}
}

const easeInSteps = 768
const easeInStepValue float64 = math.Pi * 2 / easeInSteps

const breatheMaxDelayMs = 20.0
const breatheMinDelayMs = 1.0

const blinkMaxPeriodMs = 6000.0
const blinkMinPeriodMs = 100.0

func (anim *AnimationState) setAnimation(id int, intParam int) {
	if anim.id == id && anim.param == intParam {
		return
	}
	shouldPwm := false
	floatParam := float64(intParam) / ParamScale
	var newDelay time.Duration
	switch id {
	case AnimStatic:
		if floatParam == 0.0 || floatParam == 1.0 {
			shouldPwm = false
		} else {
			shouldPwm = true
		}
		anim.ticker.Stop()
	case AnimBlink:
		periodMs := (1.0-floatParam)*(blinkMaxPeriodMs-blinkMinPeriodMs) + blinkMinPeriodMs
		delayMs := periodMs / 2.0
		newDelay = time.Millisecond * time.Duration(delayMs)
		shouldPwm = false
	case AnimBreathe:
		delayMs := (1.0-floatParam)*(breatheMaxDelayMs-breatheMinDelayMs) + breatheMinDelayMs
		newDelay = time.Millisecond * time.Duration(delayMs)
		shouldPwm = true
	default:
		return
	}
	if anim.id != id {
		anim.stepCount = 0
		anim.blinkState = false
	}
	anim.id = id
	anim.param = intParam
	// println("setAnimation", anim.id, anim.param, intParam)
	if shouldPwm {
		setupPWM()
	} else {
		pwm.PWM.ENABLE.Set(nrf.PWM_ENABLE_ENABLE_Disabled << nrf.PWM_ENABLE_ENABLE_Pos)
	}
	switch anim.id {
	case AnimStatic:
		anim.runStatic()
	case AnimBlink:
		anim.ticker.Reset(newDelay)
	case AnimBreathe:
		anim.ticker.Reset(newDelay)
	}
}

// runStatic applies a fixed brightness based on the parameter.
func (anim *AnimationState) runStatic() {
	floatParam := float64(anim.param) / ParamScale
	if floatParam == 0.0 {
		animPin.Low()
	} else if floatParam == 1.0 {
		animPin.High()
	} else {
		brightness := uint32(floatParam * float64(pwmTop))
		pwm.Set(pwmCh, brightness)
	}
}

// runBreatheStep sets the brightness based on a sinusoidal function.
func (anim *AnimationState) runBreatheStep() {
	i := anim.stepCount % easeInSteps
	anim.stepCount++
	zeroOne := (math.Cos(float64(i)*easeInStepValue) + 1) / 2
	const NotSureWhyIHaveToScaleLikeThisToGetMaxBrightness float64 = 2.0
	brightness := uint32(zeroOne * float64(pwmTop) * NotSureWhyIHaveToScaleLikeThisToGetMaxBrightness)
	pwm.Set(pwmCh, brightness)
}

// runBlinkStep toggles the LED.
func (anim *AnimationState) runBlinkStep() {
	anim.blinkState = !anim.blinkState
	if anim.blinkState {
		animPin.Low()
	} else {
		animPin.High()
	}
}

func (state *AnimationState) tick() {
	switch anim.id {
	case AnimBlink:
		anim.runBlinkStep()
	case AnimBreathe:
		anim.runBreatheStep()
	}
}

func setupPWM() {
	pwm.Configure(machine.PWMConfig{Period: 1e9 / 1000}) // 1kHz period
	var err error
	pwmCh, err = pwm.Channel(animPin)
	must("configure PWM", err)
	pwm.Set(pwmCh, 0)
}

func must(action string, err error) {
	if err != nil {
		for {
			println("failed to " + action + ": " + err.Error())
		}
	}
}

// RTC0, RTC1 already used
// var rtc = nrf.RTC2
// const irq = nrf.IRQ_RTC2
// var rtc_wakeup volatile.Register8
// func initRTC() {
// 	rtc.TASKS_START.Set(1)
// 	intr := interrupt.New(irq, func(intr interrupt.Interrupt) {
// 		if rtc.EVENTS_COMPARE[0].Get() != 0 {
// 			rtc.EVENTS_COMPARE[0].Set(0)
// 			rtc.INTENCLR.Set(nrf.RTC_INTENSET_COMPARE0)
// 			rtc.EVENTS_COMPARE[0].Set(0)
// 			rtc_wakeup.Set(1)
// 		}
// 	})
// 	rtc.INTENSET.Set(nrf.RTC_INTENSET_OVRFLW)
// 	intr.SetPriority(0xc0) // low priority
// 	intr.Enable()
// }
// func nanosecondsToTicks(ns int64) int64 {
// 	return int64(ns * 64 / 1953125)
// }
// // enterDeepSleep puts the device into System OFF mode with RTC wakeup
// func enterDeepSleep(t time.Duration) {
// 	disablePeripherals()
// 	initRTC()
// 	d := nanosecondsToTicks(t.Nanoseconds())
// 	tt := uint32(d) & 0x7fffff // 23 bits (to be on the safe side)
// 	rtcSleep(tt)
// 	// Enable deep sleep in ARM core (no effect lol)
// 	// enableDeepSleep()
// }
// func rtcSleep(ticks uint32) {
// 	rtc.INTENSET.Set(nrf.RTC_INTENSET_COMPARE0)
// 	rtc_wakeup.Set(0)
// 	rtc.CC[0].Set((rtc.COUNTER.Get() + ticks) & 0x00ffffff)
// 	for rtc_wakeup.Get() == 0 {
// 		arm.Asm("wfe")
// 	}
// }
// func enableDeepSleep() {
// 	// Set SLEEPDEEP bit in ARM System Control Register
// 	const SCB_SCR = 0xE000ED10
// 	scr := (*volatile.Register32)(unsafe.Pointer(uintptr(SCB_SCR)))
// 	scr.SetBits(1 << 2) // SLEEPDEEP bit
// }

func disablePeripherals() {
	if !DEBUG {
		nrf.UART0.ENABLE.Set(0)
	}
	// Disable SPI
	nrf.SPI0.ENABLE.Set(0)
	nrf.SPI1.ENABLE.Set(0)
	// Disable I2C
	nrf.TWI0.ENABLE.Set(0)
	nrf.TWI1.ENABLE.Set(0)
	// Disable QSPI (external flash on some boards)
	nrf.QSPI.ENABLE.Set(0)
	// Turn off ADC
	nrf.SAADC.ENABLE.Set(0)
	// (crashes)
	// nrf.TIMER0.TASKS_STOP.Set(1)
	nrf.TIMER1.TASKS_STOP.Set(1)
	nrf.TIMER2.TASKS_STOP.Set(1)
	// Disable RNG (crashes)
	// nrf.RNG.TASKS_STOP.Set(1)
	nrf.SPIM0.ENABLE.Set(0)
	nrf.SPIM1.ENABLE.Set(0)
	nrf.SPIM2.ENABLE.Set(0)
	// (crashes)
	// nrf.CLOCK.TASKS_HFCLKSTOP.Set(1)
	if !DEBUG {
		nrf.USBD.ENABLE.Set(0) // - 850µA!!
	}
	for i := uint8(0); i < 32; i++ {
		pin := machine.Pin(i)
		if pin != led && pin != onOff && pin != touch {
			pin.Configure(machine.PinConfig{Mode: machine.PinInput})
		}
	}
}
