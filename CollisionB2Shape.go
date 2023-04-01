package box2d

// This holds the mass data computed for a shape.
type MassData struct {
	/// The mass of the shape, usually in kilograms.
	Mass float64

	/// The position of the shape's centroid relative to the shape's origin.
	Center Vec2

	/// The rotational inertia of the shape about the local origin.
	I float64
}

func MakeMassData() MassData {
	return MassData{
		Mass:   0.0,
		Center: MakeVec2(0, 0),
		I:      0.0,
	}
}

func NewMassData() *MassData {
	res := MakeMassData()
	return &res
}

/// A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in b2World are created automatically when a b2Fixture
/// is created. Shapes may encapsulate a one or more child shapes.

var ShapeType = struct {
	Circle  uint8
	Edge    uint8
	Polygon uint8
	Chain   uint8
	Count   uint8
}{
	Circle:  0,
	Edge:    1,
	Polygon: 2,
	Chain:   3,
	Count:   4,
}

type ShapeInterface interface {
	Destroy()

	/// Clone the concrete shape using the provided allocator.
	Clone() ShapeInterface

	/// Get the type of this shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	GetType() uint8

	/// Get the type of this shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	GetRadius() float64

	/// Get the number of child primitives.
	GetChildCount() int

	/// Test a point for containment in this shape. This only works for convex shapes.
	/// @param xf the shape world transform.
	/// @param p a point in world coordinates.
	TestPoint(xf Transform, p Vec2) bool

	/// Cast a ray against a child shape.
	/// @param output the ray-cast results.
	/// @param input the ray-cast input parameters.
	/// @param transform the transform to be applied to the shape.
	/// @param childIndex the child shape index
	RayCast(output *B2RayCastOutput, input B2RayCastInput, transform Transform, childIndex int) bool

	/// Given a transform, compute the associated axis aligned bounding box for a child shape.
	/// @param xf the world transform of the shape.
	/// @param childIndex the child shape
	/// @return the axis aligned box.
	ComputeAABB(xf Transform, childIndex int) AABB

	/// Compute the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin.
	/// @param density the density in kilograms per meter squared.
	/// @return the mass data for this shape.
	ComputeMass(density float64) MassData
}

type Shape struct {
	M_type uint8

	/// Radius of a shape. For polygonal shapes this must be b2_polygonRadius. There is no support for
	/// making rounded polygons.
	M_radius float64
}

func (shape Shape) GetType() uint8 {
	return shape.M_type
}

func (shape Shape) GetRadius() float64 {
	return shape.M_radius
}

// @addedgo
func (shape *Shape) SetRadius(r float64) {
	shape.M_radius = r
}
