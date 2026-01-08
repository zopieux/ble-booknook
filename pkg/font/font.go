package font

import (
	"image/color"

	"tinygo.org/x/drivers/ssd1306"
)

// Font represents the font type
type Font int

const (
	FONT_6x8   Font = 0
	FONT_7x10  Font = 1
	FONT_11x18 Font = 2
	FONT_16x26 Font = 3
)

// Display manages the text buffer and font settings
type Display struct {
	buffer []byte
	width  int16
	height int16

	font       Font
	fontWidth  int16
	fontHeight int16

	XPos int16
	YPos int16

	minX, minY, maxX, maxY int16
	dirty                  bool
}

// NewDisplay creates a new font display buffer
func NewDisplay(width, height int16) Display {
	d := Display{
		buffer: make([]byte, int(width)*int(height)/8),
		width:  width,
		height: height,
		minX:   width,
		minY:   height,
		maxX:   0,
		maxY:   0,
	}
	d.SetFont(FONT_6x8) // Default font
	return d
}

// SetFont sets the font and updates dimensions
func (d *Display) SetFont(font Font) {
	d.font = font
	switch font {
	case FONT_6x8:
		d.fontWidth = 6
		d.fontHeight = 8
	case FONT_7x10:
		d.fontWidth = 7
		d.fontHeight = 10
	case FONT_11x18:
		d.fontWidth = 11
		d.fontHeight = 18
	case FONT_16x26:
		d.fontWidth = 16
		d.fontHeight = 26
	}
}

// Clear resets the buffer
func (d *Display) Clear() {
	for i := range d.buffer {
		d.buffer[i] = 0
	}
	d.minX, d.minY = d.width, d.height
	d.maxX, d.maxY = 0, 0
	d.dirty = false
	d.XPos, d.YPos = 0, 0
}

func (d *Display) updateBounds(x, y int16) {
	if x < 0 || x >= d.width || y < 0 || y >= d.height {
		return
	}
	if x < d.minX {
		d.minX = x
	}
	if x > d.maxX {
		d.maxX = x
	}
	if y < d.minY {
		d.minY = y
	}
	if y > d.maxY {
		d.maxY = y
	}
	d.dirty = true
}

// SetPixel sets a pixel in the internal buffer
func (d *Display) SetPixel(x, y int16, c color.RGBA) {
	if x < 0 || x >= d.width || y < 0 || y >= d.height {
		return
	}
	d.updateBounds(x, y)

	byteIndex := int(x) + int(y/8)*int(d.width)
	if c.R != 0 || c.G != 0 || c.B != 0 {
		d.buffer[byteIndex] |= 1 << uint8(y%8)
	} else {
		d.buffer[byteIndex] &^= 1 << uint8(y%8)
	}
}

// Print prints text
func (d *Display) Print(str string) {
	for _, char := range str {
		d.printChar(byte(char), false)
		d.XPos += d.fontWidth
	}
}

// PrintInverted prints black text on white background with padding
func (d *Display) PrintInverted(str string, padding int16) {
	// Draw background box
	textWidth := int16(len(str)) * d.fontWidth
	bgX := d.XPos - padding
	bgY := d.YPos - padding
	bgW := textWidth + 2*padding
	bgH := d.fontHeight + 2*padding

	white := color.RGBA{255, 255, 255, 255}
	// Fill background
	for x := bgX; x < bgX+bgW; x++ {
		for y := bgY; y < bgY+bgH; y++ {
			d.SetPixel(x, y, white)
		}
	}

	for _, char := range str {
		d.printChar(byte(char), true)
		d.XPos += d.fontWidth
	}
}

func (d *Display) printChar(char byte, inverted bool) {
	fontOffset := int16(char-32) * int16(d.fontHeight)

	// Pre-calculate colors
	onColor := color.RGBA{255, 255, 255, 255}
	offColor := color.RGBA{0, 0, 0, 0}
	if inverted {
		onColor, offColor = offColor, onColor
	}

	for i := int16(0); i < int16(d.fontHeight); i++ {
		for j := int16(0); j < int16(d.fontWidth); j++ {
			var pixelVal uint16 = 0
			switch d.font {
			case FONT_6x8:
				pixelVal = Font6x8[i+fontOffset]
			case FONT_7x10:
				pixelVal = Font7x10[i+fontOffset]
			case FONT_11x18:
				pixelVal = Font11x18[i+fontOffset]
			case FONT_16x26:
				pixelVal = Font16x26[i+fontOffset]
			}

			if (pixelVal & (0x8000 >> j)) != 0 {
				d.SetPixel(j+d.XPos, i+d.YPos, onColor)
			} else {
				d.SetPixel(j+d.XPos, i+d.YPos, offColor)
			}
		}
	}
}

// Apply applies the internal buffer to the device buffer within the modified bounds
func (d *Display) Apply(dev *ssd1306.Device) {
	if !d.dirty {
		return
	}

	pageStart := d.minY / 8
	pageEnd := d.maxY / 8

	// Safety clamps
	if pageStart < 0 {
		pageStart = 0
	}
	if pageEnd >= d.height/8 {
		pageEnd = d.height/8 - 1
	}

	devBuf := dev.GetBuffer()
	w := int(d.width)

	for p := pageStart; p <= pageEnd; p++ {
		for x := d.minX; x <= d.maxX; x++ {
			if x >= 0 && x < d.width {
				idx := int(x) + int(p)*w
				if idx < len(devBuf) {
					devBuf[idx] = d.buffer[idx]
				}
			}
		}
	}
}
