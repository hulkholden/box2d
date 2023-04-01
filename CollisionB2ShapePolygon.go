package box2d

/// A convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.

type B2PolygonShape struct {
	B2Shape

	M_centroid Vec2
	M_vertices [maxPolygonVertices]Vec2
	M_normals  [maxPolygonVertices]Vec2
	M_count    int
}

func MakeB2PolygonShape() B2PolygonShape {
	return B2PolygonShape{
		B2Shape: B2Shape{
			M_type:   B2Shape_Type.E_polygon,
			M_radius: polygonRadius,
		},
		M_count:    0,
		M_centroid: MakeVec2(0, 0),
	}
}

func NewB2PolygonShape() *B2PolygonShape {
	res := MakeB2PolygonShape()
	return &res
}

func (poly *B2PolygonShape) GetVertex(index int) *Vec2 {
	assert(0 <= index && index < poly.M_count)
	return &poly.M_vertices[index]
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2PolygonShape.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func (poly B2PolygonShape) Clone() B2ShapeInterface {
	clone := NewB2PolygonShape()
	clone.M_centroid = poly.M_centroid
	clone.M_count = poly.M_count

	// These are arrays, not slices so we can safely copy via assigment.
	clone.M_vertices = poly.M_vertices
	clone.M_normals = poly.M_normals

	return clone
}

func (edge *B2PolygonShape) Destroy() {}

func (poly *B2PolygonShape) SetAsBox(hx float64, hy float64) {
	poly.M_count = 4
	poly.M_vertices[0].Set(-hx, -hy)
	poly.M_vertices[1].Set(hx, -hy)
	poly.M_vertices[2].Set(hx, hy)
	poly.M_vertices[3].Set(-hx, hy)
	poly.M_normals[0].Set(0.0, -1.0)
	poly.M_normals[1].Set(1.0, 0.0)
	poly.M_normals[2].Set(0.0, 1.0)
	poly.M_normals[3].Set(-1.0, 0.0)
	poly.M_centroid.SetZero()
}

func (poly *B2PolygonShape) SetAsBoxFromCenterAndAngle(hx float64, hy float64, center Vec2, angle float64) {
	poly.M_count = 4
	poly.M_vertices[0].Set(-hx, -hy)
	poly.M_vertices[1].Set(hx, -hy)
	poly.M_vertices[2].Set(hx, hy)
	poly.M_vertices[3].Set(-hx, hy)
	poly.M_normals[0].Set(0.0, -1.0)
	poly.M_normals[1].Set(1.0, 0.0)
	poly.M_normals[2].Set(0.0, 1.0)
	poly.M_normals[3].Set(-1.0, 0.0)
	poly.M_centroid = center

	xf := MakeB2Transform()
	xf.P = center
	xf.Q.Set(angle)

	// Transform vertices and normals.
	for i := 0; i < poly.M_count; i++ {
		poly.M_vertices[i] = B2TransformVec2Mul(xf, poly.M_vertices[i])
		poly.M_normals[i] = B2RotVec2Mul(xf.Q, poly.M_normals[i])
	}
}

func (poly B2PolygonShape) GetChildCount() int {
	return 1
}

func ComputeCentroid(vs []Vec2, count int) Vec2 {
	assert(count >= 3)

	c := MakeVec2(0, 0)
	area := 0.0

	// pRef is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	pRef := MakeVec2(0.0, 0.0)

	// This code would put the reference point inside the polygon.
	for i := 0; i < count; i++ {
		pRef.OperatorPlusInplace(vs[i])
	}
	pRef.OperatorScalarMulInplace(1.0 / float64(count))

	inv3 := 1.0 / 3.0

	for i := 0; i < count; i++ {
		// Triangle vertices.
		p1 := pRef
		p2 := vs[i]
		var p3 Vec2
		if i+1 < count {
			p3 = vs[i+1]
		} else {
			p3 = vs[0]
		}

		e1 := Vec2Sub(p2, p1)
		e2 := Vec2Sub(p3, p1)

		D := Vec2Cross(e1, e2)

		triangleArea := 0.5 * D
		area += triangleArea

		// Area weighted centroid
		c.OperatorPlusInplace(Vec2MulScalar(triangleArea*inv3, Vec2Add(Vec2Add(p1, p2), p3)))
	}

	// Centroid
	assert(area > epsilon)
	c.OperatorScalarMulInplace(1.0 / area)
	return c
}

func (poly *B2PolygonShape) Set(vertices []Vec2, count int) {
	assert(3 <= count && count <= maxPolygonVertices)
	if count < 3 {
		poly.SetAsBox(1.0, 1.0)
		return
	}

	n := MinInt(count, maxPolygonVertices)

	// Perform welding and copy vertices into local buffer.
	ps := make([]Vec2, maxPolygonVertices)
	tempCount := 0

	for i := 0; i < n; i++ {
		v := vertices[i]

		unique := true
		for j := 0; j < tempCount; j++ {
			if Vec2DistanceSquared(v, ps[j]) < ((0.5 * linearSlop) * (0.5 * linearSlop)) {
				unique = false
				break
			}
		}

		if unique {
			ps[tempCount] = v
			tempCount++
		}
	}

	n = tempCount
	if n < 3 {
		// Polygon is degenerate.
		assert(false)
		poly.SetAsBox(1.0, 1.0)
		return
	}

	// Create the convex hull using the Gift wrapping algorithm
	// http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

	// Find the right most point on the hull
	i0 := 0
	x0 := ps[0].X
	for i := 1; i < n; i++ {
		x := ps[i].X
		if x > x0 || (x == x0 && ps[i].Y < ps[i0].Y) {
			i0 = i
			x0 = x
		}
	}

	hull := make([]int, maxPolygonVertices)
	m := 0
	ih := i0

	for {
		assert(m < maxPolygonVertices)
		hull[m] = ih

		ie := 0
		for j := 1; j < n; j++ {
			if ie == ih {
				ie = j
				continue
			}

			r := Vec2Sub(ps[ie], ps[hull[m]])
			v := Vec2Sub(ps[j], ps[hull[m]])
			c := Vec2Cross(r, v)
			if c < 0.0 {
				ie = j
			}

			// Collinearity check
			if c == 0.0 && v.LengthSquared() > r.LengthSquared() {
				ie = j
			}
		}

		m++
		ih = ie

		if ie == i0 {
			break
		}
	}

	if m < 3 {
		// Polygon is degenerate.
		assert(false)
		poly.SetAsBox(1.0, 1.0)
		return
	}

	poly.M_count = m

	// Copy vertices.
	for i := 0; i < m; i++ {
		poly.M_vertices[i] = ps[hull[i]]
	}

	// Compute normals. Ensure the edges have non-zero length.
	for i := 0; i < m; i++ {
		i1 := i
		i2 := 0
		if i+1 < m {
			i2 = i + 1
		}

		edge := Vec2Sub(poly.M_vertices[i2], poly.M_vertices[i1])
		assert(edge.LengthSquared() > epsilon*epsilon)
		poly.M_normals[i] = Vec2CrossVectorScalar(edge, 1.0)
		poly.M_normals[i].Normalize()
	}

	// Compute the polygon centroid.
	poly.M_centroid = ComputeCentroid(poly.M_vertices[:], m)
}

func (poly B2PolygonShape) TestPoint(xf B2Transform, p Vec2) bool {
	pLocal := B2RotVec2MulT(xf.Q, Vec2Sub(p, xf.P))

	for i := 0; i < poly.M_count; i++ {
		dot := Vec2Dot(poly.M_normals[i], Vec2Sub(pLocal, poly.M_vertices[i]))
		if dot > 0.0 {
			return false
		}
	}

	return true
}

func (poly B2PolygonShape) RayCast(output *B2RayCastOutput, input B2RayCastInput, xf B2Transform, childIndex int) bool {
	// Put the ray into the polygon's frame of reference.
	p1 := B2RotVec2MulT(xf.Q, Vec2Sub(input.P1, xf.P))
	p2 := B2RotVec2MulT(xf.Q, Vec2Sub(input.P2, xf.P))
	d := Vec2Sub(p2, p1)

	lower := 0.0
	upper := input.MaxFraction

	index := -1

	for i := 0; i < poly.M_count; i++ {
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		numerator := Vec2Dot(poly.M_normals[i], Vec2Sub(poly.M_vertices[i], p1))
		denominator := Vec2Dot(poly.M_normals[i], d)

		if denominator == 0.0 {
			if numerator < 0.0 {
				return false
			}
		} else {
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.
			if denominator < 0.0 && numerator < lower*denominator {
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator
				index = i
			} else if denominator > 0.0 && numerator < upper*denominator {
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator
			}
		}

		// The use of epsilon here causes the assert on lower to trip
		// in some cases. Apparently the use of epsilon was to make edge
		// shapes work, but now those are handled separately.
		// if (upper < lower - b2_epsilon)
		if upper < lower {
			return false
		}
	}

	assert(0.0 <= lower && lower <= input.MaxFraction)

	if index >= 0 {
		output.Fraction = lower
		output.Normal = B2RotVec2Mul(xf.Q, poly.M_normals[index])
		return true
	}

	return false
}

func (poly B2PolygonShape) ComputeAABB(xf B2Transform, childIndex int) B2AABB {
	lower := B2TransformVec2Mul(xf, poly.M_vertices[0])
	upper := lower

	for i := 1; i < poly.M_count; i++ {
		v := B2TransformVec2Mul(xf, poly.M_vertices[i])
		lower = Vec2Min(lower, v)
		upper = Vec2Max(upper, v)
	}

	r := MakeVec2(poly.M_radius, poly.M_radius)
	lowerBound := Vec2Sub(lower, r)
	upperBound := Vec2Sub(upper, r)
	return MakeB2AABB(lowerBound, upperBound)
}

func (poly B2PolygonShape) ComputeMass(density float64) B2MassData {
	// Polygon mass, centroid, and inertia.
	// Let rho be the polygon density in mass per unit area.
	// Then:
	// mass = rho * int(dA)
	// centroid.x = (1/mass) * rho * int(x * dA)
	// centroid.y = (1/mass) * rho * int(y * dA)
	// I = rho * int((x*x + y*y) * dA)
	//
	// We can compute these integrals by summing all the integrals
	// for each triangle of the polygon. To evaluate the integral
	// for a single triangle, we make a change of variables to
	// the (u,v) coordinates of the triangle:
	// x = x0 + e1x * u + e2x * v
	// y = y0 + e1y * u + e2y * v
	// where 0 <= u && 0 <= v && u + v <= 1.
	//
	// We integrate u from [0,1-v] and then v from [0,1].
	// We also need to use the Jacobian of the transformation:
	// D = cross(e1, e2)
	//
	// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
	//
	// The rest of the derivation is handled by computer algebra.

	assert(poly.M_count >= 3)

	center := MakeVec2(0, 0)

	area := 0.0
	I := 0.0

	// s is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	s := MakeVec2(0.0, 0.0)

	// This code would put the reference point inside the polygon.
	for i := 0; i < poly.M_count; i++ {
		s.OperatorPlusInplace(poly.M_vertices[i])
	}

	s.OperatorScalarMulInplace(1.0 / float64(poly.M_count))

	k_inv3 := 1.0 / 3.0

	for i := 0; i < poly.M_count; i++ {
		// Triangle vertices.
		e1 := Vec2Sub(poly.M_vertices[i], s)
		e2 := MakeVec2(0, 0)

		if i+1 < poly.M_count {
			e2 = Vec2Sub(poly.M_vertices[i+1], s)
		} else {
			e2 = Vec2Sub(poly.M_vertices[0], s)
		}

		D := Vec2Cross(e1, e2)

		triangleArea := 0.5 * D
		area += triangleArea

		// Area weighted centroid
		center.OperatorPlusInplace(Vec2MulScalar(triangleArea*k_inv3, Vec2Add(e1, e2)))

		ex1 := e1.X
		ey1 := e1.Y
		ex2 := e2.X
		ey2 := e2.Y

		intx2 := ex1*ex1 + ex2*ex1 + ex2*ex2
		inty2 := ey1*ey1 + ey2*ey1 + ey2*ey2

		I += (0.25 * k_inv3 * D) * (intx2 + inty2)
	}

	massData := MakeMassData()

	// Total mass
	massData.Mass = density * area

	// Center of mass
	assert(area > epsilon)
	center.OperatorScalarMulInplace(1.0 / area)
	massData.Center = Vec2Add(center, s)

	// Inertia tensor relative to the local origin (point s).
	massData.I = density * I

	// Shift to center of mass then to original body origin.
	massData.I += massData.Mass * (Vec2Dot(massData.Center, massData.Center) - Vec2Dot(center, center))
	return massData
}

func (poly B2PolygonShape) Validate() bool {
	for i := 0; i < poly.M_count; i++ {
		i1 := i
		i2 := 0

		if i < poly.M_count-1 {
			i2 = i1 + 1
		}

		p := poly.M_vertices[i1]
		e := Vec2Sub(poly.M_vertices[i2], p)

		for j := 0; j < poly.M_count; j++ {
			if j == i1 || j == i2 {
				continue
			}

			v := Vec2Sub(poly.M_vertices[j], p)
			c := Vec2Cross(e, v)
			if c < 0.0 {
				return false
			}
		}
	}

	return true
}
