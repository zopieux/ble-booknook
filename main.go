package main

import (
	"bytes"
	"device/nrf"
	"encoding/binary"
	"errors"
	"machine"
	"math"
	"time"

	"tinygo.org/x/bluetooth"
)

const (
	ScanInterval = 30 * time.Second
	// ScanInterval = 10 * time.Second
	ScanDuration = 500 * time.Millisecond

	MyDevId uint16 = 1

	DEBUG = false
)

var (
	adapter = bluetooth.DefaultAdapter
	led     = machine.LED
	onOff   = machine.D111
	touch   = machine.D010
	pwm     = machine.PWM0
	pwmTop  = pwm.Top()

	touchState = false
	animPin    = func() machine.Pin {
		if DEBUG {
			return led
		}
		return onOff
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

func scanPlease() (*beaconT, error) {
	// println("scanning for 63d88905a2d64a7e8b2ed0de379c232a")
	cc := make(chan beaconT, 1)
	t := time.Now()
	adapter.Scan(func(adapter *bluetooth.Adapter, device bluetooth.ScanResult) {
		if time.Since(t) > ScanDuration {
			adapter.StopScan()
			close(cc)
			return
		}
		// println("packet")
		for _, e := range device.AdvertisementPayload.ManufacturerData() {
			if e.CompanyID == 76 && bytes.Compare(e.Data[2:2+16], []byte{0x63, 0xd8, 0x89, 0x05, 0xa2, 0xd6, 0x4a, 0x7e, 0x8b, 0x2e, 0xd0, 0xde, 0x37, 0x9c, 0x23, 0x2a}) == 0 {
				adapter.StopScan()
				r := bytes.NewReader(e.Data[2+16:])
				var b beaconT
				must("binary.Read", binary.Read(r, binary.BigEndian, &b))
				if !(b.Major == 0 || b.Major == MyDevId) {
					continue
				}
				cc <- b
				return
			}
		}
	})
	b, ok := <-cc
	if !ok {
		return nil, errors.New("not found")
	}
	return &b, nil
}

func main() {
	onOff.Configure(machine.PinConfig{Mode: machine.PinOutput})
	onOff.Low()
	led.Configure(machine.PinConfig{Mode: machine.PinOutput})
	led.Low()

	must("adapter.Enable", adapter.Enable())
	blink(2)
	disablePeripherals()

	var animId = 0
	var animParam uint8 = 127

	touch.Configure(machine.PinConfig{Mode: machine.PinInputPulldown})
	must("SetInterrupt", touch.SetInterrupt(machine.PinToggle, func(p machine.Pin) {
		var t bool = p.Get()
		if touchState != t {
			touchState = t
			var brightness uint8 = 0
			if touchState {
				brightness = 255
			}
			setAnimation(AnimStatic, brightness)
			// Immediate feedback.
			runStatic(float64(brightness) / 255.0)
		}
	}))

	go animate()

	for {
		beacon, err := scanPlease()
		if err == nil {
			nAnimId, nAnimParam := beacon.Minor/1000, beacon.Minor%1000
			if int(nAnimId) != animId || uint8(nAnimParam) != animParam {
				setAnimation(int(nAnimId), uint8(nAnimParam))
			}
		}
		// Doesn't change anything. Most of the lower conso is disabling USBD.
		// enterDeepSleep(4 * time.Second)
		time.Sleep(ScanInterval)
	}
}

// Animation IDs
const (
	AnimStatic    = 1
	AnimBlink     = 2
	AnimEaseInOut = 3
)

type AnimationState struct {
	id    int     // Current animation ID
	param float64 // Normalized parameter (0.0 to 1.0)

	stepCount  int32 // Current position in the fade cycle
	blinkState bool  // Current state for blink (true = ON, false = OFF)
}

var anim AnimationState = AnimationState{
	id:    AnimStatic,
	param: 0.0,
}

const easeInSteps = 768
const easeInStepValue float64 = math.Pi * 2 / easeInSteps

const easeInOutMaxDelayMs = 20.0
const easeInOutMinDelayMs = 1.0

const blinkMaxPeriodMs = 5000.0
const blinkMinPeriodMs = 100.0

// The param255 (0-255) is scaled to a float (0.0-1.0).
func setAnimation(id int, param255 uint8) {
	// println("setAnimation", id, param255)
	normalizedParam := float64(param255) / 255.0
	if anim.id != id {
		anim.stepCount = 0
		anim.blinkState = false
	}
	anim.id = id
	anim.param = normalizedParam
	shouldPwm := false
	switch id {
	case AnimStatic:
		if normalizedParam == 0.0 || normalizedParam == 1.0 {
			shouldPwm = false
		} else {
			shouldPwm = true
		}
	case AnimBlink:
		shouldPwm = false
	case AnimEaseInOut:
		shouldPwm = true
	}
	if shouldPwm {
		setupPWM()
	} else {
		pwm.PWM.ENABLE.Set(nrf.PWM_ENABLE_ENABLE_Disabled << nrf.PWM_ENABLE_ENABLE_Pos)
	}
}

// runStatic applies a fixed brightness based on the parameter.
// Parameter 0.0 is off, 1.0 is full brightness.
func runStatic(param float64) {
	if param == 0.0 {
		animPin.Low()
	} else if param == 1.0 {
		animPin.High()
	} else {
		brightness := uint32(param * float64(pwmTop))
		pwm.Set(pwmCh, brightness)
	}
}

// runEaseInOutStep calculates the next step's brightness and returns the required delay.
func runEaseInOutStep(state *AnimationState) time.Duration {
	delayMs := (1.0-state.param)*(easeInOutMaxDelayMs-easeInOutMinDelayMs) + easeInOutMinDelayMs
	i := state.stepCount % easeInSteps
	zeroOne := (math.Cos(float64(i)*easeInStepValue) + 1) / 2
	brightness := uint32(zeroOne * float64(pwmTop))
	pwm.Set(pwmCh, brightness)
	state.stepCount++
	return time.Millisecond * time.Duration(delayMs)
}

// runBlinkStep toggles the LED ON/OFF and returns the required delay.
func runBlinkStep(state *AnimationState) time.Duration {
	periodMs := (1.0-state.param)*(blinkMaxPeriodMs-blinkMinPeriodMs) + blinkMinPeriodMs
	delayMs := periodMs / 2.0
	state.blinkState = !state.blinkState
	if state.blinkState {
		animPin.Low()
	} else {
		animPin.High()
	}
	return time.Millisecond * time.Duration(delayMs)
}

func animate() {
	var delay time.Duration
	for {
		switch anim.id {
		case AnimStatic:
			runStatic(anim.param)
			delay = time.Second * 10
		case AnimBlink:
			delay = runBlinkStep(&anim)
		case AnimEaseInOut:
			delay = runEaseInOutStep(&anim)
		default:
			animPin.Low()
			delay = time.Second * 10
		}
		time.Sleep(delay)
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
