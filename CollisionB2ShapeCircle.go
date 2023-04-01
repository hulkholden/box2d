package box2d

import (
	"math"
)

// A circle shape.
type CircleShape struct {
	Shape
	/// Position
	M_p Vec2
}

func MakeCircleShape() CircleShape {
	return CircleShape{
		Shape: Shape{
			M_type:   ShapeType.Circle,
			M_radius: 0.0,
		},
		M_p: MakeVec2(0, 0),
	}
}

func NewCircleShape() *CircleShape {
	res := MakeCircleShape()
	return &res
}

///////////////////////////////////////////////////////////////////////////////

func (shape CircleShape) Clone() ShapeInterface {
	clone := NewCircleShape()
	clone.M_radius = shape.M_radius
	clone.M_p = shape.M_p
	return clone
}

func (shape CircleShape) GetChildCount() int {
	return 1
}

func (shape CircleShape) TestPoint(transform Transform, p Vec2) bool {
	center := Vec2Add(transform.P, RotVec2Mul(transform.Q, shape.M_p))
	d := Vec2Sub(p, center)
	return Vec2Dot(d, d) <= shape.M_radius*shape.M_radius
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
func (shape CircleShape) RayCast(output *B2RayCastOutput, input RayCastInput, transform Transform, childIndex int) bool {
	// B2_NOT_USED(childIndex);

	position := Vec2Add(transform.P, RotVec2Mul(transform.Q, shape.M_p))
	s := Vec2Sub(input.P1, position)
	b := Vec2Dot(s, s) - shape.M_radius*shape.M_radius

	// Solve quadratic equation.
	r := Vec2Sub(input.P2, input.P1)
	c := Vec2Dot(s, r)
	rr := Vec2Dot(r, r)
	sigma := c*c - rr*b

	// Check for negative discriminant and short segment.
	if sigma < 0.0 || rr < epsilon {
		return false
	}

	// Find the point of intersection of the line with the circle.
	a := -(c + math.Sqrt(sigma))

	// Is the intersection point on the segment?
	if 0.0 <= a && a <= input.MaxFraction*rr {
		a /= rr
		output.Fraction = a
		output.Normal = Vec2Add(s, Vec2MulScalar(a, r))
		output.Normal.Normalize()
		return true
	}

	return false
}

func (shape CircleShape) ComputeAABB(transform Transform, childIndex int) AABB {
	// B2_NOT_USED(childIndex);

	p := Vec2Add(transform.P, RotVec2Mul(transform.Q, shape.M_p))
	lowerBound := MakeVec2(p.X-shape.M_radius, p.Y-shape.M_radius)
	upperBound := MakeVec2(p.X+shape.M_radius, p.Y+shape.M_radius)
	return MakeAABB(lowerBound, upperBound)
}

func (shape CircleShape) ComputeMass(density float64) MassData {
	massData := MakeMassData()
	massData.Mass = density * Pi * shape.M_radius * shape.M_radius
	massData.Center = shape.M_p

	// inertia about the local origin
	massData.I = massData.Mass * (0.5*shape.M_radius*shape.M_radius + Vec2Dot(shape.M_p, shape.M_p))
	return massData
}

func (shape CircleShape) Destroy() {}
