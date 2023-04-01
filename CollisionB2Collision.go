package box2d

import (
	"math"
)

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2Collision.h
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

const B2_nullFeature uint8 = math.MaxUint8

var ContactFeatureType = struct {
	Vertex uint8
	Face   uint8
}{
	Vertex: 0,
	Face:   1,
}

// The features that intersect to form the contact point
// This must be 4 bytes or less.
type B2ContactFeature struct {
	IndexA uint8 ///< Feature index on shapeA
	IndexB uint8 ///< Feature index on shapeB
	TypeA  uint8 ///< The feature type on shapeA
	TypeB  uint8 ///< The feature type on shapeB
}

func MakeB2ContactFeature() B2ContactFeature { return B2ContactFeature{} }

type B2ContactID B2ContactFeature

// Contact ids to facilitate warm starting.
// < Used to quickly compare contact ids.
func (v B2ContactID) Key() uint32 {
	var key uint32 = 0
	key |= uint32(v.IndexA)
	key |= uint32(v.IndexB) << 8
	key |= uint32(v.TypeA) << 16
	key |= uint32(v.TypeB) << 24
	return key
}

func (v *B2ContactID) SetKey(key uint32) {
	(*v).IndexA = uint8(key & 0xFF)
	(*v).IndexB = byte(key >> 8 & 0xFF)
	(*v).TypeA = byte(key >> 16 & 0xFF)
	(*v).TypeB = byte(key >> 24 & 0xFF)
}

// A manifold point is a contact point belonging to a contact
// manifold. It holds details related to the geometry and dynamics
// of the contact points.
// The local point usage depends on the manifold type:
// -e_circles: the local center of circleB
// -e_faceA: the local center of cirlceB or the clip point of polygonB
// -e_faceB: the clip point of polygonA
// This structure is stored across time steps, so we keep it small.
// Note: the impulses are used for internal caching and may not
// provide reliable contact forces, especially for high speed collisions.
type B2ManifoldPoint struct {
	LocalPoint     Vec2        ///< usage depends on manifold type
	NormalImpulse  float64     ///< the non-penetration impulse
	TangentImpulse float64     ///< the friction impulse
	Id             B2ContactID ///< uniquely identifies a contact point between two shapes
}

/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.

var ManifoldType = struct {
	Circles uint8
	FaceA   uint8
	FaceB   uint8
}{
	Circles: 0,
	FaceA:   1,
	FaceB:   2,
}

type Manifold struct {
	Points      [maxManifoldPoints]B2ManifoldPoint ///< the points of contact
	LocalNormal Vec2                               ///< not use for Type::e_points
	LocalPoint  Vec2                               ///< usage depends on manifold type
	Type        uint8                              // ManifoldType
	PointCount  int                                ///< the number of manifold points
}

func NewManifold() *Manifold { return &Manifold{} }

// This is used to compute the current state of a contact manifold.
type WorldManifold struct {
	Normal      Vec2                       ///< world vector pointing from A to B
	Points      [maxManifoldPoints]Vec2    ///< world contact point (point of intersection)
	Separations [maxManifoldPoints]float64 ///< a negative value indicates overlap, in meters
}

func MakeWorldManifold() WorldManifold { return WorldManifold{} }

var PointState = struct {
	Null    uint8 ///< point does not exist
	Add     uint8 ///< point was added in the update
	Persist uint8 ///< point persisted across the update
	Remove  uint8 ///< point was removed in the update
}{
	Null:    0,
	Add:     1,
	Persist: 2,
	Remove:  3,
}

// Used for computing contact manifolds.
type ClipVertex struct {
	V  Vec2
	Id B2ContactID
}

// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
type B2RayCastInput struct {
	P1, P2      Vec2
	MaxFraction float64
}

func MakeB2RayCastInput() B2RayCastInput { return B2RayCastInput{} }

// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
// come from b2RayCastInput.
type B2RayCastOutput struct {
	Normal   Vec2
	Fraction float64
}

func MakeB2RayCastOutput() B2RayCastOutput { return B2RayCastOutput{} }

// An axis aligned bounding box.
type AABB struct {
	LowerBound Vec2 ///< the lower vertex
	UpperBound Vec2 ///< the upper vertex
}

func MakeAABB(lower, upper Vec2) AABB {
	return AABB{
		LowerBound: lower,
		UpperBound: upper,
	}
}
func NewAABB() *AABB { return &AABB{} }

// Get the center of the AABB.
func (bb AABB) GetCenter() Vec2 {
	return Vec2MulScalar(
		0.5,
		Vec2Add(bb.LowerBound, bb.UpperBound),
	)
}

// Get the extents of the AABB (half-widths).
func (bb AABB) GetExtents() Vec2 {
	return Vec2MulScalar(
		0.5,
		Vec2Sub(bb.UpperBound, bb.LowerBound),
	)
}

// Get the perimeter length
func (bb AABB) GetPerimeter() float64 {
	wx := bb.UpperBound.X - bb.LowerBound.X
	wy := bb.UpperBound.Y - bb.LowerBound.Y
	return 2.0 * (wx + wy)
}

// Combine an AABB into this one.
func (bb *AABB) CombineInPlace(aabb AABB) {
	bb.LowerBound = Vec2Min(bb.LowerBound, aabb.LowerBound)
	bb.UpperBound = Vec2Max(bb.UpperBound, aabb.UpperBound)
}

// Combine two AABBs into this one.
func (bb *AABB) CombineTwoInPlace(aabb1, aabb2 AABB) {
	bb.LowerBound = Vec2Min(aabb1.LowerBound, aabb2.LowerBound)
	bb.UpperBound = Vec2Max(aabb1.UpperBound, aabb2.UpperBound)
}

// Does this aabb contain the provided AABB.
func (bb AABB) Contains(aabb AABB) bool {
	return (bb.LowerBound.X <= aabb.LowerBound.X &&
		bb.LowerBound.Y <= aabb.LowerBound.Y &&
		aabb.UpperBound.X <= bb.UpperBound.X &&
		aabb.UpperBound.Y <= bb.UpperBound.Y)
}

func (bb AABB) IsValid() bool {
	d := Vec2Sub(bb.UpperBound, bb.LowerBound)
	valid := d.X >= 0.0 && d.Y >= 0.0
	valid = valid && bb.LowerBound.IsValid() && bb.UpperBound.IsValid()
	return valid
}

func B2TestOverlapBoundingBoxes(a, b AABB) bool {
	d1 := Vec2Sub(b.LowerBound, a.UpperBound)
	d2 := Vec2Sub(a.LowerBound, b.UpperBound)

	if d1.X > 0.0 || d1.Y > 0.0 {
		return false
	}

	if d2.X > 0.0 || d2.Y > 0.0 {
		return false
	}

	return true
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2Collision.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func (wm *WorldManifold) Initialize(manifold *Manifold, xfA Transform, radiusA float64, xfB Transform, radiusB float64) {
	if manifold.PointCount == 0 {
		return
	}

	switch manifold.Type {
	case ManifoldType.Circles:
		{
			wm.Normal.Set(1.0, 0.0)
			pointA := TransformVec2Mul(xfA, manifold.LocalPoint)
			pointB := TransformVec2Mul(xfB, manifold.Points[0].LocalPoint)
			if Vec2DistanceSquared(pointA, pointB) > epsilon*epsilon {
				wm.Normal = Vec2Sub(pointB, pointA)
				wm.Normal.Normalize()
			}

			cA := Vec2Add(pointA, Vec2MulScalar(radiusA, wm.Normal))
			cB := Vec2Sub(pointB, Vec2MulScalar(radiusB, wm.Normal))

			wm.Points[0] = Vec2MulScalar(0.5, Vec2Add(cA, cB))
			wm.Separations[0] = Vec2Dot(Vec2Sub(cB, cA), wm.Normal)
		}

	case ManifoldType.FaceA:
		{
			wm.Normal = RotVec2Mul(xfA.Q, manifold.LocalNormal)
			planePoint := TransformVec2Mul(xfA, manifold.LocalPoint)

			for i := 0; i < manifold.PointCount; i++ {
				clipPoint := TransformVec2Mul(xfB, manifold.Points[i].LocalPoint)
				cA := Vec2Add(
					clipPoint,
					Vec2MulScalar(
						radiusA-Vec2Dot(
							Vec2Sub(clipPoint, planePoint),
							wm.Normal,
						),
						wm.Normal,
					),
				)
				cB := Vec2Sub(clipPoint, Vec2MulScalar(radiusB, wm.Normal))
				wm.Points[i] = Vec2MulScalar(0.5, Vec2Add(cA, cB))
				wm.Separations[i] = Vec2Dot(
					Vec2Sub(cB, cA),
					wm.Normal,
				)
			}
		}

	case ManifoldType.FaceB:
		{
			wm.Normal = RotVec2Mul(xfB.Q, manifold.LocalNormal)
			planePoint := TransformVec2Mul(xfB, manifold.LocalPoint)

			for i := 0; i < manifold.PointCount; i++ {
				clipPoint := TransformVec2Mul(xfA, manifold.Points[i].LocalPoint)
				cB := Vec2Add(clipPoint, Vec2MulScalar(
					radiusB-Vec2Dot(
						Vec2Sub(clipPoint, planePoint),
						wm.Normal,
					), wm.Normal,
				))
				cA := Vec2Sub(clipPoint, Vec2MulScalar(radiusA, wm.Normal))
				wm.Points[i] = Vec2MulScalar(0.5, Vec2Add(cA, cB))
				wm.Separations[i] = Vec2Dot(
					Vec2Sub(cA, cB),
					wm.Normal,
				)
			}

			// Ensure normal points from A to B.
			wm.Normal = wm.Normal.OperatorNegate()
		}
	}
}

func B2GetPointStates(state1 *[maxManifoldPoints]uint8, state2 *[maxManifoldPoints]uint8, manifold1 Manifold, manifold2 Manifold) {
	for i := 0; i < maxManifoldPoints; i++ {
		state1[i] = PointState.Null
		state2[i] = PointState.Null
	}

	// Detect persists and removes.
	for i := 0; i < manifold1.PointCount; i++ {
		id := manifold1.Points[i].Id

		state1[i] = PointState.Remove

		for j := 0; j < manifold2.PointCount; j++ {
			if manifold2.Points[j].Id.Key() == id.Key() {
				state1[i] = PointState.Persist
				break
			}
		}
	}

	// Detect persists and adds.
	for i := 0; i < manifold2.PointCount; i++ {
		id := manifold2.Points[i].Id

		state2[i] = PointState.Add

		for j := 0; j < manifold1.PointCount; j++ {
			if manifold1.Points[j].Id.Key() == id.Key() {
				state2[i] = PointState.Persist
				break
			}
		}
	}
}

// From Real-time Collision Detection, p179.
func (bb AABB) RayCast(output *B2RayCastOutput, input B2RayCastInput) bool {
	tmin := -maxFloat
	tmax := maxFloat

	p := input.P1
	d := Vec2Sub(input.P2, input.P1)
	absD := Vec2Abs(d)

	normal := MakeVec2(0, 0)

	for i := 0; i < 2; i++ {
		if absD.OperatorIndexGet(i) < epsilon {
			// Parallel.
			if p.OperatorIndexGet(i) < bb.LowerBound.OperatorIndexGet(i) || bb.UpperBound.OperatorIndexGet(i) < p.OperatorIndexGet(i) {
				return false
			}
		} else {
			inv_d := 1.0 / d.OperatorIndexGet(i)
			t1 := (bb.LowerBound.OperatorIndexGet(i) - p.OperatorIndexGet(i)) * inv_d
			t2 := (bb.UpperBound.OperatorIndexGet(i) - p.OperatorIndexGet(i)) * inv_d

			// Sign of the normal vector.
			s := -1.0

			if t1 > t2 {
				t1, t2 = t2, t1
				s = 1.0
			}

			// Push the min up
			if t1 > tmin {
				normal.SetZero()
				normal.OperatorIndexSet(i, s)
				tmin = t1
			}

			// Pull the max down
			tmax = math.Min(tmax, t2)

			if tmin > tmax {
				return false
			}
		}
	}

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if tmin < 0.0 || input.MaxFraction < tmin {
		return false
	}

	// Intersection.
	output.Fraction = tmin
	output.Normal = normal
	return true
}

// Sutherland-Hodgman clipping.
func ClipSegmentToLine(vOut []ClipVertex, vIn []ClipVertex, normal Vec2, offset float64, vertexIndexA int) int {
	// Start with no output points
	numOut := 0

	// Calculate the distance of end points to the line
	distance0 := Vec2Dot(normal, vIn[0].V) - offset
	distance1 := Vec2Dot(normal, vIn[1].V) - offset

	// If the points are behind the plane
	if distance0 <= 0.0 {
		vOut[numOut] = vIn[0]
		numOut++
	}

	if distance1 <= 0.0 {
		vOut[numOut] = vIn[1]
		numOut++
	}

	// If the points are on different sides of the plane
	if distance0*distance1 < 0.0 {
		// Find intersection point of edge and plane
		interp := distance0 / (distance0 - distance1)
		vOut[numOut].V = Vec2Add(
			vIn[0].V,
			Vec2MulScalar(interp, Vec2Sub(vIn[1].V, vIn[0].V)),
		)

		// VertexA is hitting edgeB.
		vOut[numOut].Id.IndexA = uint8(vertexIndexA)
		vOut[numOut].Id.IndexB = vIn[0].Id.IndexB
		vOut[numOut].Id.TypeA = ContactFeatureType.Vertex
		vOut[numOut].Id.TypeB = ContactFeatureType.Face
		numOut++
	}

	return numOut
}

func B2TestOverlapShapes(shapeA ShapeInterface, indexA int, shapeB ShapeInterface, indexB int, xfA Transform, xfB Transform) bool {
	input := MakeDistanceInput()
	input.ProxyA.Set(shapeA, indexA)
	input.ProxyB.Set(shapeB, indexB)
	input.TransformA = xfA
	input.TransformB = xfB
	input.UseRadii = true

	cache := MakeSimplexCache()
	cache.Count = 0

	output := MakeDistanceOutput()

	Distance(&output, &cache, &input)

	return output.Distance < 10.0*epsilon
}
