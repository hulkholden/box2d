package box2d

import (
	"math"
)

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// b2Math.h
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// B2IsValid is used to ensure that a floating point number is not a NaN or infinity.
func B2IsValid(x float64) bool {
	return !math.IsNaN(x) && !math.IsInf(x, 0)
}

// This is a approximate yet fast inverse square-root.
func B2InvSqrt(x float64) float64 {
	// https://groups.google.com/forum/#!topic/golang-nuts/8vaZ1ERYIQ0
	// Faster with math.Sqrt
	return 1.0 / math.Sqrt(x)
}

// Vec2 is a 2D column vector.
type Vec2 struct {
	X, Y float64
}

// MakeVec2 constructs a Vec2 using the provided coordinates.
func MakeVec2(xIn, yIn float64) Vec2 {
	return Vec2{
		X: xIn,
		Y: yIn,
	}
}

// NewVec2 constructs a new Vec2 using the provided coordinates.
func NewVec2(xIn, yIn float64) *Vec2 {
	return &Vec2{
		X: xIn,
		Y: yIn,
	}
}

// SetZero sets the vector to all zeros.
func (v *Vec2) SetZero() {
	v.X = 0.0
	v.Y = 0.0
}

// Set sets this vector to some specified coordinates.
func (v *Vec2) Set(x, y float64) {
	v.X = x
	v.Y = y
}

// OperatorNegate negates this vector.
func (v Vec2) OperatorNegate() Vec2 {
	return MakeVec2(
		-v.X,
		-v.Y,
	)
}

// OperatorIndexGet returns an indexed element.
func (v Vec2) OperatorIndexGet(i int) float64 {
	if i == 0 {
		return v.X
	}

	return v.Y
}

// OperatorIndexSet sets an indexed element.
func (v *Vec2) OperatorIndexSet(i int, value float64) {
	if i == 0 {
		v.X = value
		return
	}

	v.Y = value
}

// OperatorPlusInplace adds a vector to this vector.
func (v *Vec2) OperatorPlusInplace(other Vec2) {
	v.X += other.X
	v.Y += other.Y
}

// OperatorMinusInplace subtracts a vector from this vector.
func (v *Vec2) OperatorMinusInplace(other Vec2) {
	v.X -= other.X
	v.Y -= other.Y
}

// OperatorScalarMulInplace multiplies this vector by a scalar.
func (v *Vec2) OperatorScalarMulInplace(a float64) {
	v.X *= a
	v.Y *= a
}

// Length returns length of this vector (the norm).
func (v Vec2) Length() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y)
}

// LengthSquared returns the length squared.
// For performance, use this instead of Vec2::Length (if possible).
func (v Vec2) LengthSquared() float64 {
	return v.X*v.X + v.Y*v.Y
}

// Normalize converts this vector into a unit vector. Returns the length.
func (v *Vec2) Normalize() float64 {
	length := v.Length()

	if length < B2_epsilon {
		return 0.0
	}

	invLength := 1.0 / length
	v.X *= invLength
	v.Y *= invLength

	return length
}

// IsValid returns whether this vector contain finite coordinates.
func (v Vec2) IsValid() bool {
	return B2IsValid(v.X) && B2IsValid(v.Y)
}

// Skew returns skew vector such that dot(skew_vec, other) == cross(vec, other)
func (v Vec2) Skew() Vec2 {
	return MakeVec2(-v.Y, v.X)
}

// B2Vec3 is 2D column vector with 3 elements.
type B2Vec3 struct {
	X, Y, Z float64
}

// MakeB2Vec3 constructs a B2Vec3 using the provided coordinates.
func MakeB2Vec3(xIn, yIn, zIn float64) B2Vec3 {
	return B2Vec3{
		X: xIn,
		Y: yIn,
		Z: zIn,
	}
}

// NewB2Vec3 returns a reference to a B2Vec3 using the provided coordinates.
func NewB2Vec3(xIn, yIn, zIn float64) *B2Vec3 {
	res := MakeB2Vec3(xIn, yIn, zIn)
	return &res
}

// SetZero sets this vector to all zeros.
func (v *B2Vec3) SetZero() {
	v.X = 0.0
	v.Y = 0.0
	v.Z = 0.0
}

// Set sets this vector to some specified coordinates.
func (v *B2Vec3) Set(x, y, z float64) {
	v.X = x
	v.Y = y
	v.Z = z
}

// OperatorNegate negates this vector.
func (v B2Vec3) OperatorNegate() B2Vec3 {
	return MakeB2Vec3(
		-v.X,
		-v.Y,
		-v.Z,
	)
}

// OperatorPlusInplace adds a vector to this vector.
func (v *B2Vec3) OperatorPlusInplace(other B2Vec3) {
	v.X += other.X
	v.Y += other.Y
	v.Z += other.Z
}

// OperatorMinusInplace subtracts a vector from this vector.
func (v *B2Vec3) OperatorMinusInplace(other B2Vec3) {
	v.X -= other.X
	v.Y -= other.Y
	v.Z -= other.Z
}

// OperatorScalarMultInplace multiplies this vector by a scalar.
func (v *B2Vec3) OperatorScalarMultInplace(a float64) {
	v.X *= a
	v.Y *= a
	v.Z *= a
}

// B2Mat22 is a 2-by-2 matrix. Stored in column-major order.
type B2Mat22 struct {
	Ex, Ey Vec2
}

// The default constructor does nothing
func MakeB2Mat22() B2Mat22 { return B2Mat22{} }
func NewB2Mat22() *B2Mat22 { return &B2Mat22{} }

// Construct this matrix using columns.
func MakeB2Mat22FromColumns(c1, c2 Vec2) B2Mat22 {
	return B2Mat22{
		Ex: c1,
		Ey: c2,
	}
}

func NewB2Mat22FromColumns(c1, c2 Vec2) *B2Mat22 {
	res := MakeB2Mat22FromColumns(c1, c2)
	return &res
}

// Construct this matrix using scalars.
func MakeB2Mat22FromScalars(a11, a12, a21, a22 float64) B2Mat22 {
	return B2Mat22{
		Ex: MakeVec2(a11, a21),
		Ey: MakeVec2(a12, a22),
	}
}

func NewB2Mat22FromScalars(a11, a12, a21, a22 float64) *B2Mat22 {
	res := MakeB2Mat22FromScalars(a11, a12, a21, a22)
	return &res
}

// Initialize this matrix using columns.
func (m *B2Mat22) Set(c1 Vec2, c2 Vec2) {
	m.Ex = c1
	m.Ey = c2
}

// Set this to the identity matrix.
func (m *B2Mat22) SetIdentity() {
	m.Ex.X = 1.0
	m.Ey.X = 0.0
	m.Ex.Y = 0.0
	m.Ey.Y = 1.0
}

// Set this matrix to all zeros.
func (m *B2Mat22) SetZero() {
	m.Ex.X = 0.0
	m.Ey.X = 0.0
	m.Ex.Y = 0.0
	m.Ey.Y = 0.0
}

func (m B2Mat22) GetInverse() B2Mat22 {
	a := m.Ex.X
	b := m.Ey.X
	c := m.Ex.Y
	d := m.Ey.Y

	B := MakeB2Mat22()

	det := a*d - b*c
	if det != 0.0 {
		det = 1.0 / det
	}

	B.Ex.X = det * d
	B.Ey.X = -det * b
	B.Ex.Y = -det * c
	B.Ey.Y = det * a

	return B
}

// Solve A * x = b, where b is a column vector. This is more efficient
// than computing the inverse in one-shot cases.
func (m B2Mat22) Solve(b Vec2) Vec2 {
	a11 := m.Ex.X
	a12 := m.Ey.X
	a21 := m.Ex.Y
	a22 := m.Ey.Y
	det := a11*a22 - a12*a21

	if det != 0.0 {
		det = 1.0 / det
	}

	return MakeVec2(
		det*(a22*b.X-a12*b.Y),
		det*(a11*b.Y-a21*b.X),
	)
}

// B2Mat33 is a 3-by-3 matrix. Stored in column-major order.
type B2Mat33 struct {
	Ex, Ey, Ez B2Vec3
}

// The default constructor does nothing (for performance).
func MakeB2Mat33() B2Mat33 { return B2Mat33{} }
func NewB2Mat33() *B2Mat33 { return &B2Mat33{} }

// Construct this matrix using columns.
func MakeB2Mat33FromColumns(c1, c2, c3 B2Vec3) B2Mat33 {
	return B2Mat33{
		Ex: c1,
		Ey: c2,
		Ez: c3,
	}
}

func NewB2Mat33FromColumns(c1, c2, c3 B2Vec3) *B2Mat33 {
	res := MakeB2Mat33FromColumns(c1, c2, c3)
	return &res
}

// Set this matrix to all zeros.
func (m *B2Mat33) SetZero() {
	m.Ex.SetZero()
	m.Ey.SetZero()
	m.Ez.SetZero()
}

// B2Rot represents a rotation.
type B2Rot struct {
	/// Sine and cosine
	S, C float64
}

func MakeB2Rot() B2Rot { return B2Rot{} }
func NewB2Rot() *B2Rot { return &B2Rot{} }

// Initialize from an angle in radians
func MakeB2RotFromAngle(anglerad float64) B2Rot {
	return B2Rot{
		S: math.Sin(anglerad),
		C: math.Cos(anglerad),
	}
}

func NewB2RotFromAngle(anglerad float64) *B2Rot {
	res := MakeB2RotFromAngle(anglerad)
	return &res
}

// Set using an angle in radians.
func (r *B2Rot) Set(anglerad float64) {
	r.S = math.Sin(anglerad)
	r.C = math.Cos(anglerad)
}

// Set to the identity rotation
func (r *B2Rot) SetIdentity() {
	r.S = 0.0
	r.C = 1.0
}

// Get the angle in radians
func (r B2Rot) GetAngle() float64 {
	return math.Atan2(r.S, r.C)
}

// Get the x-axis
func (r B2Rot) GetXAxis() Vec2 {
	return MakeVec2(r.C, r.S)
}

// Get the u-axis
func (r B2Rot) GetYAxis() Vec2 {
	return MakeVec2(-r.S, r.C)
}

// B2Transform contains translation and rotation. It is used to represent
// the position and orientation of rigid frames.
type B2Transform struct {
	P Vec2
	Q B2Rot
}

// The default constructor does nothing.
func MakeB2Transform() B2Transform { return B2Transform{} }
func NewB2Transform() *B2Transform { return &B2Transform{} }

// Initialize using a position vector and a rotation.
func MakeB2TransformByPositionAndRotation(position Vec2, rotation B2Rot) B2Transform {
	return B2Transform{
		P: position,
		Q: rotation,
	}
}

func NewB2TransformByPositionAndRotation(position Vec2, rotation B2Rot) *B2Transform {
	res := MakeB2TransformByPositionAndRotation(position, rotation)
	return &res
}

// Set this to the identity transform.
func (t *B2Transform) SetIdentity() {
	t.P.SetZero()
	t.Q.SetIdentity()
}

// Set this based on the position and angle.
func (t *B2Transform) Set(position Vec2, anglerad float64) {
	t.P = position
	t.Q.Set(anglerad)
}

///////////////////////////////////////////////////////////////////////////////
/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
///////////////////////////////////////////////////////////////////////////////

type B2Sweep struct {
	LocalCenter Vec2    ///< local center of mass position
	C0, C       Vec2    ///< center world positions
	A0, A       float64 ///< world angles

	/// Fraction of the current time step in the range [0,1]
	/// c0 and a0 are the positions at alpha0.
	Alpha0 float64
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Perform the dot product on two vectors.
func Vec2Dot(a, b Vec2) float64 {
	return a.X*b.X + a.Y*b.Y
}

// Perform the cross product on two vectors. In 2D this produces a scalar.
func Vec2Cross(a, b Vec2) float64 {
	return a.X*b.Y - a.Y*b.X
}

// Perform the cross product on a vector and a scalar. In 2D this produces
// a vector.
func Vec2CrossVectorScalar(a Vec2, s float64) Vec2 {
	return MakeVec2(s*a.Y, -s*a.X)
}

// Perform the cross product on a scalar and a vector. In 2D this produces
// a vector.
func Vec2CrossScalarVector(s float64, a Vec2) Vec2 {
	return MakeVec2(-s*a.Y, s*a.X)
}

// Multiply a matrix times a vector. If a rotation matrix is provided,
// then this transforms the vector from one frame to another.
func Vec2Mat22Mul(A B2Mat22, v Vec2) Vec2 {
	return MakeVec2(A.Ex.X*v.X+A.Ey.X*v.Y, A.Ex.Y*v.X+A.Ey.Y*v.Y)
}

// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
// then this transforms the vector from one frame to another (inverse transform).
func Vec2Mat22MulT(A B2Mat22, v Vec2) Vec2 {
	return MakeVec2(Vec2Dot(v, A.Ex), Vec2Dot(v, A.Ey))
}

// Add two vectors component-wise.
func Vec2Add(a, b Vec2) Vec2 {
	return MakeVec2(a.X+b.X, a.Y+b.Y)
}

// Subtract two vectors component-wise.
func Vec2Sub(a, b Vec2) Vec2 {
	return MakeVec2(a.X-b.X, a.Y-b.Y)
}

func Vec2MulScalar(s float64, a Vec2) Vec2 {
	return MakeVec2(s*a.X, s*a.Y)
}

func Vec2Equals(a, b Vec2) bool {
	return a.X == b.X && a.Y == b.Y
}

func Vec2NotEquals(a, b Vec2) bool {
	return a.X != b.X || a.Y != b.Y
}

func Vec2Distance(a, b Vec2) float64 {
	return Vec2Sub(a, b).Length()
}

func Vec2DistanceSquared(a, b Vec2) float64 {
	c := Vec2Sub(a, b)
	return Vec2Dot(c, c)
}

func B2Vec3MultScalar(s float64, a B2Vec3) B2Vec3 {
	return MakeB2Vec3(s*a.X, s*a.Y, s*a.Z)
}

// Add two vectors component-wise.
func B2Vec3Add(a, b B2Vec3) B2Vec3 {
	return MakeB2Vec3(a.X+b.X, a.Y+b.Y, a.Z+b.Z)
}

// Subtract two vectors component-wise.
func B2Vec3Sub(a, b B2Vec3) B2Vec3 {
	return MakeB2Vec3(a.X-b.X, a.Y-b.Y, a.Z-b.Z)
}

// Perform the dot product on two vectors.
func B2Vec3Dot(a, b B2Vec3) float64 {
	return a.X*b.X + a.Y*b.Y + a.Z*b.Z
}

// Perform the cross product on two vectors.
func B2Vec3Cross(a, b B2Vec3) B2Vec3 {
	return MakeB2Vec3(a.Y*b.Z-a.Z*b.Y, a.Z*b.X-a.X*b.Z, a.X*b.Y-a.Y*b.X)
}

func B2Mat22Add(A, B B2Mat22) B2Mat22 {
	return MakeB2Mat22FromColumns(
		Vec2Add(A.Ex, B.Ex),
		Vec2Add(A.Ey, B.Ey),
	)
}

// A * B
func B2Mat22Mul(A, B B2Mat22) B2Mat22 {
	return MakeB2Mat22FromColumns(
		Vec2Mat22Mul(A, B.Ex),
		Vec2Mat22Mul(A, B.Ey),
	)
}

// A^T * B
func B2Mat22MulT(A, B B2Mat22) B2Mat22 {
	c1 := MakeVec2(
		Vec2Dot(A.Ex, B.Ex),
		Vec2Dot(A.Ey, B.Ex),
	)

	c2 := MakeVec2(
		Vec2Dot(A.Ex, B.Ey),
		Vec2Dot(A.Ey, B.Ey),
	)

	return MakeB2Mat22FromColumns(c1, c2)
}

// Multiply a matrix times a vector.
func B2Vec3Mat33Mul(A B2Mat33, v B2Vec3) B2Vec3 {
	one := B2Vec3MultScalar(v.X, A.Ex)
	two := B2Vec3MultScalar(v.Y, A.Ey)
	three := B2Vec3MultScalar(v.Z, A.Ez)

	return B2Vec3Add(
		B2Vec3Add(
			one,
			two,
		),
		three,
	)
}

// Multiply a matrix times a vector.
func Vec2Mul22(A B2Mat33, v Vec2) Vec2 {
	return MakeVec2(A.Ex.X*v.X+A.Ey.X*v.Y, A.Ex.Y*v.X+A.Ey.Y*v.Y)
}

// Multiply two rotations: q * r
func B2RotMul(q, r B2Rot) B2Rot {
	return B2Rot{
		S: q.S*r.C + q.C*r.S,
		C: q.C*r.C - q.S*r.S,
	}
}

// Transpose multiply two rotations: qT * r
func B2RotMulT(q, r B2Rot) B2Rot {
	return B2Rot{
		S: q.C*r.S - q.S*r.C,
		C: q.C*r.C + q.S*r.S,
	}
}

// Rotate a vector
func B2RotVec2Mul(q B2Rot, v Vec2) Vec2 {
	return MakeVec2(
		q.C*v.X-q.S*v.Y,
		q.S*v.X+q.C*v.Y,
	)
}

// Inverse rotate a vector
func B2RotVec2MulT(q B2Rot, v Vec2) Vec2 {
	return MakeVec2(
		q.C*v.X+q.S*v.Y,
		-q.S*v.X+q.C*v.Y,
	)
}

func B2TransformVec2Mul(T B2Transform, v Vec2) Vec2 {
	return MakeVec2(
		(T.Q.C*v.X-T.Q.S*v.Y)+T.P.X,
		(T.Q.S*v.X+T.Q.C*v.Y)+T.P.Y,
	)
}

func B2TransformVec2MulT(T B2Transform, v Vec2) Vec2 {
	px := v.X - T.P.X
	py := v.Y - T.P.Y
	x := (T.Q.C*px + T.Q.S*py)
	y := (-T.Q.S*px + T.Q.C*py)

	return MakeVec2(x, y)
}

func B2TransformMul(A, B B2Transform) B2Transform {
	q := B2RotMul(A.Q, B.Q)
	p := Vec2Add(B2RotVec2Mul(A.Q, B.P), A.P)

	return MakeB2TransformByPositionAndRotation(p, q)
}

func B2TransformMulT(A, B B2Transform) B2Transform {
	q := B2RotMulT(A.Q, B.Q)
	p := B2RotVec2MulT(A.Q, Vec2Sub(B.P, A.P))

	return MakeB2TransformByPositionAndRotation(p, q)
}

// Check if the projected testpoint onto the line is on the line segment
func B2IsProjectedPointOnLineSegment(v1 Vec2, v2 Vec2, p Vec2) bool {
	e1 := Vec2{v2.X - v1.X, v2.Y - v1.Y}
	recArea := Vec2Dot(e1, e1)
	e2 := Vec2{p.X - v1.X, p.Y - v1.Y}
	v := Vec2Dot(e1, e2)
	return v >= 0.0 && v <= recArea
}

// Get projected point p' of p on line v1,v2
func B2ProjectPointOnLine(v1 Vec2, v2 Vec2, p Vec2) Vec2 {
	e1 := Vec2{v2.X - v1.X, v2.Y - v1.Y}
	e2 := Vec2{p.X - v1.X, p.Y - v1.Y}
	valDp := Vec2Dot(e1, e2)
	len2 := e1.X*e1.X + e1.Y*e1.Y
	p1 := Vec2{
		v1.X + (valDp*e1.X)/len2,
		v1.Y + (valDp*e1.Y)/len2,
	}
	return p1
}

func Vec2Abs(a Vec2) Vec2 {
	return MakeVec2(math.Abs(a.X), math.Abs(a.Y))
}

func B2Mat22Abs(A B2Mat22) B2Mat22 {
	return MakeB2Mat22FromColumns(
		Vec2Abs(A.Ex),
		Vec2Abs(A.Ey),
	)
}

func Vec2Min(a, b Vec2) Vec2 {
	return MakeVec2(
		fastMin(a.X, b.X),
		fastMin(a.Y, b.Y),
	)
}

func Vec2Max(a, b Vec2) Vec2 {
	return MakeVec2(
		fastMax(a.X, b.X),
		fastMax(a.Y, b.Y),
	)
}

func Vec2Clamp(a, low, high Vec2) Vec2 {
	return Vec2Max(
		low,
		Vec2Min(a, high),
	)
}

func B2FloatClamp(a, low, high float64) float64 {
	var b, c float64
	if B2IsValid(high) {
		b = math.Min(a, high)
	} else {
		b = a
	}
	if B2IsValid(low) {
		c = math.Max(b, low)
	} else {
		c = b
	}
	return c
}

// "Next Largest Power of 2
// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
// largest power of 2. For a 32-bit value:"
func B2NextPowerOfTwo(x uint32) uint32 {
	x |= (x >> 1)
	x |= (x >> 2)
	x |= (x >> 4)
	x |= (x >> 8)
	x |= (x >> 16)
	return x + 1
}

func B2IsPowerOfTwo(x uint32) bool {
	return x > 0 && (x&(x-1)) == 0
}

func (sweep B2Sweep) GetTransform(xf *B2Transform, beta float64) {
	xf.P = Vec2Add(sweep.C0, Vec2MulScalar(beta, Vec2Sub(sweep.C, sweep.C0)))
	angle := sweep.A0 + (sweep.A-sweep.A0)*beta
	xf.Q.Set(angle)

	// Shift to origin
	xf.P.OperatorMinusInplace(B2RotVec2Mul(xf.Q, sweep.LocalCenter))
}

func (sweep *B2Sweep) Advance(alpha float64) {
	B2Assert(sweep.Alpha0 < 1.0)
	beta := (alpha - sweep.Alpha0) / (1.0 - sweep.Alpha0)
	sweep.C0.OperatorPlusInplace(Vec2MulScalar(beta, Vec2Sub(sweep.C, sweep.C0)))
	sweep.A0 += beta * (sweep.A - sweep.A0)
	sweep.Alpha0 = alpha
}

// Normalize an angle in radians to be between -pi and pi
func (sweep *B2Sweep) Normalize() {
	twoPi := 2.0 * B2_pi
	d := twoPi * math.Floor(sweep.A0/twoPi)
	sweep.A0 -= d
	sweep.A -= d
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// b2Math.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Solve A * x = b, where b is a column vector. This is more efficient
// than computing the inverse in one-shot cases.
func (mat B2Mat33) Solve33(b B2Vec3) B2Vec3 {
	det := B2Vec3Dot(mat.Ex, B2Vec3Cross(mat.Ey, mat.Ez))
	if det != 0.0 {
		det = 1.0 / det
	}

	// b2Vec3 x;
	// x.x = det * b2Dot(b, b2Cross(ey, ez));
	// x.y = det * b2Dot(ex, b2Cross(b, ez));
	// x.z = det * b2Dot(ex, b2Cross(ey, b));
	// return x;

	x := det * B2Vec3Dot(b, B2Vec3Cross(mat.Ey, mat.Ez))
	y := det * B2Vec3Dot(mat.Ex, B2Vec3Cross(b, mat.Ez))
	z := det * B2Vec3Dot(mat.Ex, B2Vec3Cross(mat.Ey, b))

	return MakeB2Vec3(x, y, z)
}

// Solve A * x = b, where b is a column vector. This is more efficient
// than computing the inverse in one-shot cases.
func (mat B2Mat33) Solve22(b Vec2) Vec2 {
	a11 := mat.Ex.X
	a12 := mat.Ey.X
	a21 := mat.Ex.Y
	a22 := mat.Ey.Y

	det := a11*a22 - a12*a21
	if det != 0.0 {
		det = 1.0 / det
	}

	x := det * (a22*b.X - a12*b.Y)
	y := det * (a11*b.Y - a21*b.X)

	return MakeVec2(x, y)
}

func (mat B2Mat33) GetInverse22(M *B2Mat33) {
	a := mat.Ex.X
	b := mat.Ey.X
	c := mat.Ex.Y
	d := mat.Ey.Y

	det := a*d - b*c
	if det != 0.0 {
		det = 1.0 / det
	}

	M.Ex.X = det * d
	M.Ey.X = -det * b
	M.Ex.Z = 0.0
	M.Ex.Y = -det * c
	M.Ey.Y = det * a
	M.Ey.Z = 0.0
	M.Ez.X = 0.0
	M.Ez.Y = 0.0
	M.Ez.Z = 0.0
}

// Returns the zero matrix if singular.
func (mat B2Mat33) GetSymInverse33(M *B2Mat33) {
	det := B2Vec3Dot(mat.Ex, B2Vec3Cross(mat.Ey, mat.Ez))

	if det != 0.0 {
		det = 1.0 / det
	}

	a11 := mat.Ex.X
	a12 := mat.Ey.X
	a13 := mat.Ez.X
	a22 := mat.Ey.Y
	a23 := mat.Ez.Y
	a33 := mat.Ez.Z

	M.Ex.X = det * (a22*a33 - a23*a23)
	M.Ex.Y = det * (a13*a23 - a12*a33)
	M.Ex.Z = det * (a12*a23 - a13*a22)

	M.Ey.X = M.Ex.Y
	M.Ey.Y = det * (a11*a33 - a13*a13)
	M.Ey.Z = det * (a13*a12 - a11*a23)

	M.Ez.X = M.Ex.Z
	M.Ez.Y = M.Ey.Z
	M.Ez.Z = det * (a11*a22 - a12*a12)
}

func fastMin(a, b float64) float64 {
	if a < b {
		return a
	}
	return b
}

func fastMax(a, b float64) float64 {
	if a > b {
		return a
	}
	return b
}

// Legacy type names.
// TODO: fully migrate to Vec2 and remove.
type B2Vec2 = Vec2

var (
	MakeB2Vec2 = MakeVec2
	NewB2Vec2  = NewVec2

	B2Vec2Dot               = Vec2Dot
	B2Vec2Cross             = Vec2Cross
	B2Vec2CrossVectorScalar = Vec2CrossVectorScalar
	B2Vec2CrossScalarVector = Vec2CrossScalarVector
	B2Vec2Mat22Mul          = Vec2Mat22Mul
	B2Vec2Mat22MulT         = Vec2Mat22MulT
	B2Vec2Add               = Vec2Add
	B2Vec2Sub               = Vec2Sub
	B2Vec2MulScalar         = Vec2MulScalar
	B2Vec2Equals            = Vec2Equals
	B2Vec2NotEquals         = Vec2NotEquals
	B2Vec2Distance          = Vec2Distance
	B2Vec2DistanceSquared   = Vec2DistanceSquared
	B2Vec2Mul22             = Vec2Mul22
	B2Vec2Abs               = Vec2Abs
	B2Vec2Min               = Vec2Min
	B2Vec2Max               = Vec2Max
	B2Vec2Clamp             = Vec2Clamp

	B2Vec2_zero = MakeVec2(0, 0)
)
