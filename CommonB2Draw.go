package box2d

// Color for debug drawing. Each value has the range [0,1].
type Color struct {
	R, G, B, A float32
}

func MakeColor() Color {
	return MakeColorRGB(0, 0, 0)
}

func MakeColorRGB(r, g, b float32) Color {
	return MakeColorRGBA(r, g, b, 1)
}

func MakeColorRGBA(r, g, b, a float32) Color {
	return Color{r, g, b, a}
}

func (c *Color) SetRGB(r, g, b float32) {
	c.SetRGBA(r, g, b, 1)
}

func (c *Color) SetRGBA(r, g, b, a float32) {
	c.R = r
	c.G = g
	c.B = b
	c.A = a
}

var DrawFlags = struct {
	Shape        uint32 //< draw shapes
	Joint        uint32 //< draw joint connections
	AABB         uint32 //< draw axis aligned bounding boxes
	Pair         uint32 //< draw broad-phase pairs
	CenterOfMass uint32 //< draw center of mass frame
}{
	Shape:        0x0001,
	Joint:        0x0002,
	AABB:         0x0004,
	Pair:         0x0008,
	CenterOfMass: 0x0010,
}

// Implement and register this class with a b2World to provide debug drawing of physics
// entities in your game.
type B2Draw interface {
	// Get the drawing flags.
	GetFlags() uint32

	// Draw a closed polygon provided in CCW order.
	DrawPolygon(vertices []Vec2, color Color)

	// Draw a solid closed polygon provided in CCW order.
	DrawSolidPolygon(vertices []Vec2, color Color)

	// Draw a circle.
	DrawCircle(center Vec2, radius float64, color Color)

	// Draw a solid circle.
	DrawSolidCircle(center Vec2, radius float64, axis Vec2, color Color)

	// Draw a line segment.
	DrawSegment(p1 Vec2, p2 Vec2, color Color)

	// Draw a transform. Choose your own length scale.
	// @param xf a transform.
	DrawTransform(xf Transform)

	// Draw a point.
	DrawPoint(p Vec2, size float64, color Color)
}
