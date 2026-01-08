package main

import (
	"encoding/binary"
	"fmt"
	"image/color"
	"machine"
	"os"
	"time"

	"ble-booknook/pkg/font"

	"tinygo.org/x/bluetooth"
	"tinygo.org/x/drivers/ssd1306"

	"tinygo.org/x/tinyfs/littlefs"
)

const (
	turnDebounceDuration  = 80 * time.Millisecond
	pressDebounceDuration = 30 * time.Millisecond
	MaxDeviceId           = 20
	StepSize              = 5
	// Font constants
	FontWidth  = 7
	FontHeight = 10
	// Menu Layout
	LineHeight  = 12
	MenuLeftPad = 16
)

var (
	blockDevice = machine.Flash
	filesystem  = littlefs.New(blockDevice)
)

func setupFS() {
	filesystem.Configure(&littlefs.Config{
		CacheSize:     512,
		LookaheadSize: 512,
		BlockCycles:   100,
	})
	if err := filesystem.Mount(); err != nil {
		if err := filesystem.Format(); err != nil {
			println("failed to format fs:", err.Error())
			return
		}
		if err := filesystem.Mount(); err != nil {
			println("failed to mount fs:", err.Error())
		}
	}
}

func saveSettings(s Settings) {
	f, err := filesystem.OpenFile("settings.dat", os.O_CREATE|os.O_WRONLY|os.O_TRUNC)
	if err != nil {
		println("open save failed:", err.Error())
		return
	}
	defer f.Close()

	data := []byte{
		byte(s.Brightness),
		byte(s.Blink),
		byte(s.Fade),
		byte(s.ScanFreq),
		byte(s.Device),
	}
	f.Write(data)
}

func loadSettings() Settings {
	// Default
	s := Settings{Brightness: 50, Blink: 50, Fade: 50, ScanFreq: 30, Device: 0}

	f, err := filesystem.OpenFile("settings.dat", os.O_RDONLY)
	if err != nil {
		return s // Return default if file not found
	}
	defer f.Close()

	buf := make([]byte, 5)
	n, err := f.Read(buf)
	if err != nil || n != 5 {
		return s
	}

	s.Brightness = int(buf[0])
	s.Blink = int(buf[1])
	s.Fade = int(buf[2])
	s.ScanFreq = int(buf[3])
	s.Device = int(buf[4])
	return s
}

type eventType int

const (
	evtNone eventType = iota
	evtTurnCW
	evtTurnCCW
	evtPress
)

type rotaryEncoder struct {
	pinS1   machine.Pin
	pinS2   machine.Pin
	pinKey  machine.Pin
	eventCh chan eventType

	state    int8
	lastTime time.Time
}

func newRotaryEncoder(pinS1, pinS2, pinKey machine.Pin, eventCh chan eventType) *rotaryEncoder {
	return &rotaryEncoder{
		pinS1:   pinS1,
		pinS2:   pinS2,
		pinKey:  pinKey,
		eventCh: eventCh,
	}
}

func (enc *rotaryEncoder) configure() {
	enc.pinS1.Configure(machine.PinConfig{Mode: machine.PinInput})
	enc.pinS1.SetInterrupt(machine.PinToggle, enc.interruptTurn)

	enc.pinS2.Configure(machine.PinConfig{Mode: machine.PinInput})
	enc.pinS2.SetInterrupt(machine.PinToggle, enc.interruptTurn)

	enc.pinKey.Configure(machine.PinConfig{Mode: machine.PinInput})
	enc.pinKey.SetInterrupt(machine.PinToggle, enc.interruptPress)
}

func (enc *rotaryEncoder) interruptTurn(pin machine.Pin) {
	s1, s2 := enc.pinS1.Get(), enc.pinS2.Get()
	val := int8(0)
	if s1 {
		val |= 2
	}
	if s2 {
		val |= 1
	}

	// Reset if the pattern is not respected (timeout)
	if time.Since(enc.lastTime) > turnDebounceDuration {
		enc.state = 0
	}
	enc.lastTime = time.Now()

	switch enc.state {
	case 0: // Idle
		if val == 1 { // 01
			enc.state = 1 // Start Reverse
		} else if val == 2 { // 10
			enc.state = -1 // Start Forward
		}
	case 1: // R1: Saw 01
		if val == 0 { // 00
			enc.state = 2
		} else if val == 3 { // 11
			enc.state = 0
		}
	case 2: // R2: Saw 00 (Reverse mid)
		if val == 2 { // 10
			enc.state = 3
		} else if val == 1 { // 01
			enc.state = 1
		} else if val == 3 { // 11
			enc.state = 0
		}
	case 3: // R3: Saw 10
		if val == 3 { // 11
			enc.state = 0
			// Turn CCW
			select {
			case enc.eventCh <- evtTurnCCW:
			default:
			}
		} else if val == 0 { // 00
			enc.state = 2
		}
	case -1: // F1: Saw 10
		if val == 0 { // 00
			enc.state = -2
		} else if val == 3 { // 11
			enc.state = 0
		}
	case -2: // F2: Saw 00 (Forward mid)
		if val == 1 { // 01
			enc.state = -3
		} else if val == 2 { // 10
			enc.state = -1
		} else if val == 3 { // 11
			enc.state = 0
		}
	case -3: // F3: Saw 01
		if val == 3 { // 11
			enc.state = 0
			// Turn CW
			select {
			case enc.eventCh <- evtTurnCW:
			default:
			}
		} else if val == 0 { // 00
			enc.state = -2
		}
	}
}

func (enc *rotaryEncoder) interruptPress(pin machine.Pin) {
	select {
	case enc.eventCh <- evtPress:
	default:
	}
}

type Settings struct {
	Brightness int
	Blink      int
	Fade       int
	ScanFreq   int
	Device     int // 0 = all
}

var adapter = bluetooth.DefaultAdapter

func main() {
	led := machine.LED
	led.Configure(machine.PinConfig{Mode: machine.PinOutput})
	led.Low()

	setupFS()

	must("adapter.Enable", adapter.Enable())

	machine.I2C0.Configure(machine.I2CConfig{
		Frequency: 1 * machine.MHz,
		SDA:       machine.D017,
		SCL:       machine.D020,
	})
	dev := ssd1306.NewI2C(machine.I2C0)
	dev.Configure(ssd1306.Config{
		Address: ssd1306.Address_128_32,
		Width:   128,
		Height:  32,
	})
	dev.ClearDisplay()
	display := font.NewDisplay(128, 32)

	// Boot Animation
	animStart := time.Now()

	const (
		ScreenW      = 128
		ScreenH      = 32
		TitleText    = "NOOKCTL"
		SubtitleText = "by zopieux"
		TitleW       = len(TitleText) * 16   // FONT_16x26 width
		TitleH       = 26                    // FONT_16x26 height
		SubtitleW    = len(SubtitleText) * 6 // FONT_6x8 width
		SubtitleH    = 8                     // FONT_6x8 height
		Gap          = 1
		BlockH       = TitleH + Gap + SubtitleH

		// Positions
		TitleX       = (ScreenW - TitleW) / 2
		SubtitleX    = (ScreenW - SubtitleW) / 2
		CenterBlockY = (ScreenH - BlockH) / 2
		StartBlockY  = -BlockH
		EndBlockY    = ScreenH
	)

	for {
		elapsed := time.Since(animStart)
		if elapsed > 3*time.Second {
			break
		}

		var blockY int16
		if elapsed < 1*time.Second {
			// Descending from top to center
			progress := float64(elapsed) / float64(1*time.Second)
			blockY = int16(float64(StartBlockY) + progress*float64(CenterBlockY-StartBlockY))
		} else if elapsed < 2*time.Second {
			// Hold at center
			blockY = int16(CenterBlockY)
		} else {
			// Descending from center to bottom
			progress := float64(elapsed-2*time.Second) / float64(1*time.Second)
			blockY = int16(float64(CenterBlockY) + progress*float64(EndBlockY-CenterBlockY))
		}

		dev.ClearBuffer()
		display.Clear()

		// Title - Font 16x26
		display.SetFont(font.FONT_16x26)
		display.XPos = int16(TitleX)
		display.YPos = blockY
		display.Print(TitleText)

		// Subtitle - Font 6x8
		display.SetFont(font.FONT_6x8)
		display.XPos = int16(SubtitleX)
		display.YPos = blockY + int16(TitleH+Gap)
		display.Print(SubtitleText)

		display.Apply(dev)
		dev.Display()
		time.Sleep(20 * time.Millisecond)
	}

	// Main Logic Setup
	display.SetFont(font.FONT_7x10)
	eventCh := make(chan eventType, 10)
	enc := newRotaryEncoder(machine.D022, machine.D024, machine.D100, eventCh)
	enc.configure()

	var (
		debounceCh       <-chan time.Time
		pressed          bool
		longPressHandled bool
		longPressTimer   *time.Timer

		activeMode int = 0 // The menu index that is currently active/transmitting. 0 = Stop.
		settings       = loadSettings()

		isEditing bool
		tempValue int // For editing

		menuIdx    int = 0 // 0..7
		menuItems      = []string{"== stop ==", "Full ON", "Full OFF", "Brightness", "Blink", "Fade", "Scan freq", "Devices"}
		animFrame  int
		animTicker = time.NewTicker(200 * time.Millisecond)
	)

	// Ensure timer is initialized (stopped state)
	longPressTimer = time.NewTimer(1 * time.Second)
	longPressTimer.Stop()

	// Initial Render
	render := func() {
		dev.ClearBuffer()
		display.Clear()

		// Scrolling logic: Always 3 items.
		// i=-1 (top), i=0 (center/selected), i=1 (bottom)
		// Y positions: Center is 10. Top -2, Bottom 22.
		const CenterY = 10

		for i := -1; i <= 1; i++ {
			// Calculate wrapped index
			idx := (menuIdx + i)
			// Handle negative wrap
			if idx < 0 {
				idx = len(menuItems) + idx
			}
			// Handle positive wrap
			if idx >= len(menuItems) {
				idx = idx - len(menuItems)
			}

			y := int16(CenterY + i*LineHeight)

			// Determine Label and Value
			label := menuItems[idx]
			valStr := ""

			// Get value to display (settings or tempValue if editing this item)
			val := 0
			hasValue := false

			if idx == 7 { // Devices
				hasValue = true
				if isEditing && idx == menuIdx {
					val = tempValue
				} else {
					val = settings.Device
				}
				if val == 0 {
					label = "Devices"
					valStr = "all"
				} else {
					label = "Device"
					valStr = fmt.Sprintf("%d", val)
				}
				// Pad to 3 chars for constant width: "all", "  1", " 20"
				valStr = fmt.Sprintf("%3s", valStr)

			} else if idx == 6 { // Scan Freq
				hasValue = true
				if isEditing && idx == menuIdx {
					val = tempValue
				} else {
					val = settings.ScanFreq
				}
				valStr = fmt.Sprintf("%ds", val)
				// Pad to 3 chars
				valStr = fmt.Sprintf("%3s", valStr)

			} else if idx == 3 { // Brightness
				hasValue = true
				if isEditing && idx == menuIdx {
					val = tempValue
				} else {
					val = settings.Brightness
				}
				valStr = fmt.Sprintf("%d%%", val)
				// Pad to 4 chars: "100%", " 50%", "  5%"
				valStr = fmt.Sprintf("%4s", valStr)

			} else if idx == 4 { // Blink
				hasValue = true
				if isEditing && idx == menuIdx {
					val = tempValue
				} else {
					val = settings.Blink
				}
				valStr = fmt.Sprintf("%d%%", val)
				valStr = fmt.Sprintf("%4s", valStr)

			} else if idx == 5 { // Fade
				hasValue = true
				if isEditing && idx == menuIdx {
					val = tempValue
				} else {
					val = settings.Fade
				}
				valStr = fmt.Sprintf("%d%%", val)
				valStr = fmt.Sprintf("%4s", valStr)
			}

			selected := (idx == menuIdx)

			// Draw Label
			display.XPos = int16(MenuLeftPad)
			display.YPos = y
			if selected {
				display.PrintInverted(label, 1)
			} else {
				display.Print(label)
			}

			// Draw Value (Right Aligned)
			if hasValue && valStr != "" {
				w := int16(len(valStr) * FontWidth)
				display.XPos = 128 - w - 1
				display.YPos = y
				// Inverted value only when editing AND selected
				if selected && isEditing {
					display.PrintInverted(valStr, 1)
				} else {
					display.Print(valStr)
				}
			}

			// Animation: only if this mode is active, and it is NOT "Stop" (0) or "Devices" (7).
			if idx == activeMode && idx != 0 && idx != 7 {
				drawWaveAnim(&display, 2, y, animFrame)
			}
		}

		display.Apply(dev)
		dev.Display()
	}

	// Update Advertising
	updateAds := func() {
		// Calculate current parameters for the ACTIVE mode
		adapter.DefaultAdvertisement().Stop()
		if activeMode == 0 {
			return
		}

		var modeID uint16
		var val int

		switch activeMode {
		case 1: // Full ON
			modeID = 1
			val = 100
		case 2: // Full OFF
			modeID = 1
			val = 0
		case 3: // Brightness
			modeID = 1
			val = settings.Brightness
		case 4: // Blink
			modeID = 2
			val = settings.Blink
		case 5: // Fade
			modeID = 3
			val = settings.Fade
		case 6: // Scan Freq
			modeID = 9
			val = settings.ScanFreq
		default:
			return
		}

		// Important: since the protocol only allows 0-99 values, we cheat by sending 99 for 100%, etc.
		minor := modeID*100 + uint16(min(99, max(0, int(val)-1)))

		// Major is Device ID
		devID := uint16(settings.Device)

		// Construct Packet
		// 0x02 (iBeacon), 0x15 (Len 21), UUID (16), Major (2), Minor (2), Tx (1)
		payload := make([]byte, 2+16+2+2+1)
		payload[0] = 0x02
		payload[1] = 0x15
		copy(payload[2:], SecretUuid)
		binary.BigEndian.PutUint16(payload[18:], devID)
		binary.BigEndian.PutUint16(payload[20:], minor)
		payload[22] = 0xC5 // Tx Power

		// Configure & Start
		must("config adv", adapter.DefaultAdvertisement().Configure(bluetooth.AdvertisementOptions{
			ManufacturerData: []bluetooth.ManufacturerDataElement{
				{
					CompanyID: 0x004C,
					Data:      payload,
				},
			},
		}))
		must("start adv", adapter.DefaultAdvertisement().Start())
	}

	render()

	for {
		select {
		case evt := <-eventCh:
			switch evt {
			case evtTurnCW:
				if isEditing {
					// Edit Value
					if menuIdx == 7 { // Devices
						tempValue++
						if tempValue > MaxDeviceId {
							tempValue = 0
						}
					} else if menuIdx == 6 { // Scan Freq (Step=1)
						tempValue++
						if tempValue > 99 {
							tempValue = 99
						}
					} else { // Others (Step=5)
						tempValue += StepSize
						if tempValue > 100 {
							tempValue = 100
						}
					}
				} else {
					// Navigate Menu
					menuIdx++
					if menuIdx >= len(menuItems) {
						menuIdx = 0
					}
				}
				render()

			case evtTurnCCW:
				if isEditing {
					// Edit Value
					if menuIdx == 7 { // Devices
						tempValue--
						if tempValue < 0 {
							tempValue = MaxDeviceId
						}
					} else if menuIdx == 6 { // Scan Freq (Step=1)
						tempValue--
						if tempValue < 1 {
							tempValue = 1
						}
					} else { // Others (Step=5)
						tempValue -= StepSize
						if tempValue < 0 {
							tempValue = 0
						}
					}
				} else {
					// Navigate Menu
					menuIdx--
					if menuIdx < 0 {
						menuIdx = len(menuItems) - 1
					}
				}
				render()

			case evtPress:
				if debounceCh == nil {
					debounceCh = time.NewTimer(pressDebounceDuration).C
				}
			}

		case <-debounceCh:
			debounceCh = nil
			isPressed := !enc.pinKey.Get() // Active Low check
			if isPressed != pressed {
				pressed = isPressed
				if pressed {
					// Button Down
					longPressHandled = false
					longPressTimer.Reset(1 * time.Second)
				} else {
					// Button Up
					longPressTimer.Stop()
					if !longPressHandled {
						// Short Click Action
						if isEditing {
							// Save & Exit Edit Mode
							switch menuIdx {
							case 3:
								settings.Brightness = tempValue
							case 4:
								settings.Blink = tempValue
							case 5:
								settings.Fade = tempValue
							case 6:
								settings.ScanFreq = tempValue
							case 7:
								settings.Device = tempValue
							}
							isEditing = false
							// Update ads (confirmed change)
							updateAds()
						} else {
							// Enter Edit Mode if applicable
							if menuIdx >= 3 && menuIdx <= 7 { // Brightness, Blink, Fade, ScanFreq, Devices
								isEditing = true
								// Load current value
								switch menuIdx {
								case 3:
									tempValue = settings.Brightness
								case 4:
									tempValue = settings.Blink
								case 5:
									tempValue = settings.Fade
								case 6:
									tempValue = settings.ScanFreq
								case 7:
									tempValue = settings.Device
								}
							}
							// "== stop ==", "Full ON" and "Full OFF", "Save" have no edit mode
						}
						render()
					}
				}
			}

		case <-longPressTimer.C:
			// Timer fired, meaning button has been held for 1s
			if pressed && !longPressHandled {
				longPressHandled = true
				// Long Click Action
				if isEditing {
					// Cancel Edit (Exit without saving)
					isEditing = false
				} else {
					// Set Active Mode (Stop or Transmit)
					if menuIdx != 7 { // Device is not a mode
						activeMode = menuIdx
						updateAds()
					}
				}
				render()
			}

		case <-animTicker.C:
			// Animate if active mode is not Stop (0) and not Devices (7)
			isTransmitting := activeMode != 0 && activeMode != 7
			if isTransmitting {
				animFrame++
				if animFrame > 3 {
					animFrame = 0
				}
				render()
			} else {
				if animFrame != 0 {
					animFrame = 0
					render()
				}
			}
		}
	}
}

func drawWaveAnim(d *font.Display, x, y int16, frame int) {
	white := color.RGBA{255, 255, 255, 255}
	// Center y relative to text (height 10, padding 1 => height 12).
	// Originally +6, user requested 3px higher => +3.
	cy := y + 3

	// Dot
	d.SetPixel(x, cy, white)

	if frame >= 1 {
		d.SetPixel(x+2, cy-1, white)
		d.SetPixel(x+3, cy, white)
		d.SetPixel(x+2, cy+1, white)
	}
	if frame >= 2 {
		d.SetPixel(x+4, cy-2, white)
		d.SetPixel(x+5, cy-1, white)
		d.SetPixel(x+5, cy+1, white)
		d.SetPixel(x+4, cy+2, white)
	}
	if frame >= 3 {
		d.SetPixel(x+6, cy-3, white)
		d.SetPixel(x+7, cy-2, white)
		d.SetPixel(x+7, cy+2, white)
		d.SetPixel(x+6, cy+3, white)
	}
}

func must(action string, err error) {
	if err != nil {
		for {
			println("failed to " + action + ": " + err.Error())
		}
	}
}
