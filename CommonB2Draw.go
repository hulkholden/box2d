package box2d

// Color for debug drawing. Each value has the range [0,1].
type B2Color struct {
	R, G, B, A float32
}

func MakeB2Color() B2Color {
	return MakeB2ColorRGB(0, 0, 0)
}

func MakeB2ColorRGB(r, g, b float32) B2Color {
	return MakeB2ColorRGBA(r, g, b, 1)
}

func MakeB2ColorRGBA(r, g, b, a float32) B2Color {
	return B2Color{r, g, b, a}
}

func (c *B2Color) SetRGB(r, g, b float32) {
	c.SetRGBA(r, g, b, 1)
}

func (c *B2Color) SetRGBA(r, g, b, a float32) {
	c.R = r
	c.G = g
	c.B = b
	c.A = a
}

var B2Draw_Flags = struct {
	E_shapeBit        uint32 //< draw shapes
	E_jointBit        uint32 //< draw joint connections
	E_aabbBit         uint32 //< draw axis aligned bounding boxes
	E_pairBit         uint32 //< draw broad-phase pairs
	E_centerOfMassBit uint32 //< draw center of mass frame
}{
	E_shapeBit:        0x0001,
	E_jointBit:        0x0002,
	E_aabbBit:         0x0004,
	E_pairBit:         0x0008,
	E_centerOfMassBit: 0x0010,
}

// Implement and register this class with a b2World to provide debug drawing of physics
// entities in your game.
type B2Draw interface {
	// Get the drawing flags.
	GetFlags() uint32

	// Draw a closed polygon provided in CCW order.
	DrawPolygon(vertices []B2Vec2, color B2Color)

	// Draw a solid closed polygon provided in CCW order.
	DrawSolidPolygon(vertices []B2Vec2, color B2Color)

	// Draw a circle.
	DrawCircle(center B2Vec2, radius float64, color B2Color)

	// Draw a solid circle.
	DrawSolidCircle(center B2Vec2, radius float64, axis B2Vec2, color B2Color)

	// Draw a line segment.
	DrawSegment(p1 B2Vec2, p2 B2Vec2, color B2Color)

	// Draw a transform. Choose your own length scale.
	// @param xf a transform.
	DrawTransform(xf B2Transform)

	// Draw a point.
	DrawPoint(p B2Vec2, size float64, color B2Color)
}
