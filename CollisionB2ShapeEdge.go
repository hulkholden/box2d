package box2d

// A line segment (edge) shape. These can be connected in chains or loops
// to other edge shapes. The connectivity information is used to ensure
// correct contact normals.
type EdgeShape struct {
	Shape
	/// These are the edge vertices
	M_vertex1, M_vertex2 Vec2

	/// Optional adjacent vertices. These are used for smooth collision.
	M_vertex0, M_vertex3       Vec2
	M_hasVertex0, M_hasVertex3 bool
}

func MakeEdgeShape() EdgeShape {
	return EdgeShape{
		Shape: Shape{
			M_type:   ShapeType.Edge,
			M_radius: polygonRadius,
		},
		M_vertex0:    MakeVec2(0, 0),
		M_vertex3:    MakeVec2(0, 0),
		M_hasVertex0: false,
		M_hasVertex3: false,
	}
}

func NewEdgeShape() *EdgeShape {
	res := MakeEdgeShape()
	return &res
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// EdgeShape.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func (edge *EdgeShape) Set(v1 Vec2, v2 Vec2) {
	edge.M_vertex1 = v1
	edge.M_vertex2 = v2
	edge.M_hasVertex0 = false
	edge.M_hasVertex3 = false
}

func (edge EdgeShape) Clone() ShapeInterface {
	clone := NewEdgeShape()
	clone.M_vertex0 = edge.M_vertex0
	clone.M_vertex1 = edge.M_vertex1
	clone.M_vertex2 = edge.M_vertex2
	clone.M_vertex3 = edge.M_vertex3
	clone.M_hasVertex0 = edge.M_hasVertex0
	clone.M_hasVertex3 = edge.M_hasVertex3

	return clone
}

func (edge *EdgeShape) Destroy() {}

func (edge EdgeShape) GetChildCount() int {
	return 1
}

func (edge EdgeShape) TestPoint(xf Transform, p Vec2) bool {
	return false
}

// p = p1 + t * d
// v = v1 + s * e
// p1 + t * d = v1 + s * e
// s * e - t * d = p1 - v1
func (edge EdgeShape) RayCast(output *RayCastOutput, input RayCastInput, xf Transform, childIndex int) bool {
	// Put the ray into the edge's frame of reference.
	p1 := RotVec2MulT(xf.Q, Vec2Sub(input.P1, xf.P))
	p2 := RotVec2MulT(xf.Q, Vec2Sub(input.P2, xf.P))
	d := Vec2Sub(p2, p1)

	v1 := edge.M_vertex1
	v2 := edge.M_vertex2
	e := Vec2Sub(v2, v1)
	normal := MakeVec2(e.Y, -e.X)
	normal.Normalize()

	// q = p1 + t * d
	// dot(normal, q - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	numerator := Vec2Dot(normal, Vec2Sub(v1, p1))
	denominator := Vec2Dot(normal, d)

	if denominator == 0.0 {
		return false
	}

	t := numerator / denominator
	if t < 0.0 || input.MaxFraction < t {
		return false
	}

	q := Vec2Add(p1, Vec2MulScalar(t, d))

	// q = v1 + s * r
	// s = dot(q - v1, r) / dot(r, r)
	r := Vec2Sub(v2, v1)
	rr := Vec2Dot(r, r)
	if rr == 0.0 {
		return false
	}

	s := Vec2Dot(Vec2Sub(q, v1), r) / rr
	if s < 0.0 || 1.0 < s {
		return false
	}

	output.Fraction = t
	if numerator > 0.0 {
		output.Normal = RotVec2Mul(xf.Q, normal).OperatorNegate()
	} else {
		output.Normal = RotVec2Mul(xf.Q, normal)
	}

	return true
}

func (edge EdgeShape) ComputeAABB(xf Transform, childIndex int) AABB {
	v1 := TransformVec2Mul(xf, edge.M_vertex1)
	v2 := TransformVec2Mul(xf, edge.M_vertex2)

	lower := Vec2Min(v1, v2)
	upper := Vec2Max(v1, v2)

	r := MakeVec2(edge.M_radius, edge.M_radius)
	lowerBound := Vec2Sub(lower, r)
	upperBound := Vec2Sub(upper, r)
	return MakeAABB(lowerBound, upperBound)
}

func (edge EdgeShape) ComputeMass(density float64) MassData {
	massData := MakeMassData()
	massData.Mass = 0.0
	massData.Center = Vec2MulScalar(0.5, Vec2Add(edge.M_vertex1, edge.M_vertex2))
	massData.I = 0.0
	return massData
}
