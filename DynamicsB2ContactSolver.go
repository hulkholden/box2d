package box2d

import (
	"math"
)

type VelocityConstraintPoint struct {
	RA             Vec2
	RB             Vec2
	NormalImpulse  float64
	TangentImpulse float64
	NormalMass     float64
	TangentMass    float64
	VelocityBias   float64
}

type ContactVelocityConstraint struct {
	Points             [maxManifoldPoints]VelocityConstraintPoint
	Normal             Vec2
	NormalMass         Mat22
	K                  Mat22
	IndexA             int
	IndexB             int
	InvMassA, InvMassB float64
	InvIA, InvIB       float64
	Friction           float64
	Restitution        float64
	TangentSpeed       float64
	PointCount         int
	ContactIndex       int
}

type ContactSolverDef struct {
	Step       B2TimeStep
	Contacts   []ContactInterface // has to be backed by pointers
	Count      int
	Positions  []B2Position
	Velocities []B2Velocity
}

func MakeContactSolverDef() ContactSolverDef {
	return ContactSolverDef{}
}

type ContactSolver struct {
	M_step                B2TimeStep
	M_positions           []B2Position
	M_velocities          []B2Velocity
	M_positionConstraints []ContactPositionConstraint
	M_velocityConstraints []ContactVelocityConstraint
	M_contacts            []ContactInterface // has to be backed by pointers
	M_count               int
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ContactSolver
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Solver debugging is normally disabled because the block solver sometimes has to deal with a poorly conditioned effective mass matrix.
const B2_DEBUG_SOLVER = 0

var g_blockSolve = true

type ContactPositionConstraint struct {
	LocalPoints                [maxManifoldPoints]Vec2
	LocalNormal                Vec2
	LocalPoint                 Vec2
	IndexA                     int
	IndexB                     int
	InvMassA, InvMassB         float64
	LocalCenterA, LocalCenterB Vec2
	InvIA, InvIB               float64
	Type                       uint8
	RadiusA, RadiusB           float64
	PointCount                 int
}

func MakeContactSolver(def *ContactSolverDef) ContactSolver {
	solver := ContactSolver{}

	solver.M_step = def.Step
	solver.M_count = def.Count
	solver.M_positionConstraints = make([]ContactPositionConstraint, solver.M_count)
	solver.M_velocityConstraints = make([]ContactVelocityConstraint, solver.M_count)
	solver.M_positions = def.Positions
	solver.M_velocities = def.Velocities
	solver.M_contacts = def.Contacts

	// Initialize position independent portions of the constraints.
	for i := 0; i < solver.M_count; i++ {
		contact := solver.M_contacts[i]

		fixtureA := contact.GetFixtureA()
		fixtureB := contact.GetFixtureB()
		shapeA := fixtureA.GetShape()
		shapeB := fixtureB.GetShape()
		radiusA := shapeA.GetRadius()
		radiusB := shapeB.GetRadius()
		bodyA := fixtureA.GetBody()
		bodyB := fixtureB.GetBody()
		manifold := contact.GetManifold()

		pointCount := manifold.PointCount
		assert(pointCount > 0)

		vc := &solver.M_velocityConstraints[i]
		vc.Friction = contact.GetFriction()
		vc.Restitution = contact.GetRestitution()
		vc.TangentSpeed = contact.GetTangentSpeed()
		vc.IndexA = bodyA.M_islandIndex
		vc.IndexB = bodyB.M_islandIndex
		vc.InvMassA = bodyA.M_invMass
		vc.InvMassB = bodyB.M_invMass
		vc.InvIA = bodyA.M_invI
		vc.InvIB = bodyB.M_invI
		vc.ContactIndex = i
		vc.PointCount = pointCount
		vc.K.SetZero()
		vc.NormalMass.SetZero()

		pc := &solver.M_positionConstraints[i]
		pc.IndexA = bodyA.M_islandIndex
		pc.IndexB = bodyB.M_islandIndex
		pc.InvMassA = bodyA.M_invMass
		pc.InvMassB = bodyB.M_invMass
		pc.LocalCenterA = bodyA.M_sweep.LocalCenter
		pc.LocalCenterB = bodyB.M_sweep.LocalCenter
		pc.InvIA = bodyA.M_invI
		pc.InvIB = bodyB.M_invI
		pc.LocalNormal = manifold.LocalNormal
		pc.LocalPoint = manifold.LocalPoint
		pc.PointCount = pointCount
		pc.RadiusA = radiusA
		pc.RadiusB = radiusB
		pc.Type = manifold.Type

		for j := 0; j < pointCount; j++ {
			cp := &manifold.Points[j]
			vcp := &vc.Points[j]

			if solver.M_step.WarmStarting {
				vcp.NormalImpulse = solver.M_step.DtRatio * cp.NormalImpulse
				vcp.TangentImpulse = solver.M_step.DtRatio * cp.TangentImpulse
			} else {
				vcp.NormalImpulse = 0.0
				vcp.TangentImpulse = 0.0
			}

			vcp.RA.SetZero()
			vcp.RB.SetZero()
			vcp.NormalMass = 0.0
			vcp.TangentMass = 0.0
			vcp.VelocityBias = 0.0

			pc.LocalPoints[j] = cp.LocalPoint
		}
	}

	return solver
}

func (solver *ContactSolver) Destroy() {
}

// Initialize position dependent portions of the velocity constraints.
func (solver *ContactSolver) InitializeVelocityConstraints() {
	for i := 0; i < solver.M_count; i++ {
		vc := &solver.M_velocityConstraints[i]
		pc := &solver.M_positionConstraints[i]

		radiusA := pc.RadiusA
		radiusB := pc.RadiusB
		manifold := solver.M_contacts[vc.ContactIndex].GetManifold()

		indexA := vc.IndexA
		indexB := vc.IndexB

		mA := vc.InvMassA
		mB := vc.InvMassB
		iA := vc.InvIA
		iB := vc.InvIB
		localCenterA := pc.LocalCenterA
		localCenterB := pc.LocalCenterB

		cA := solver.M_positions[indexA].C
		aA := solver.M_positions[indexA].A
		vA := solver.M_velocities[indexA].V
		wA := solver.M_velocities[indexA].W

		cB := solver.M_positions[indexB].C
		aB := solver.M_positions[indexB].A
		vB := solver.M_velocities[indexB].V
		wB := solver.M_velocities[indexB].W

		assert(manifold.PointCount > 0)

		xfA := MakeTransform()
		xfB := MakeTransform()
		xfA.Q.Set(aA)
		xfB.Q.Set(aB)
		xfA.P = Vec2Sub(cA, RotVec2Mul(xfA.Q, localCenterA))
		xfB.P = Vec2Sub(cB, RotVec2Mul(xfB.Q, localCenterB))

		worldManifold := MakeWorldManifold()
		worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB)

		vc.Normal = worldManifold.Normal

		pointCount := vc.PointCount
		for j := 0; j < pointCount; j++ {
			vcp := &vc.Points[j]

			vcp.RA = Vec2Sub(worldManifold.Points[j], cA)
			vcp.RB = Vec2Sub(worldManifold.Points[j], cB)

			rnA := Vec2Cross(vcp.RA, vc.Normal)
			rnB := Vec2Cross(vcp.RB, vc.Normal)

			kNormal := mA + mB + iA*rnA*rnA + iB*rnB*rnB

			if kNormal > 0.0 {
				vcp.NormalMass = 1.0 / kNormal
			} else {
				vcp.NormalMass = 0.0
			}

			tangent := Vec2CrossVectorScalar(vc.Normal, 1.0)

			rtA := Vec2Cross(vcp.RA, tangent)
			rtB := Vec2Cross(vcp.RB, tangent)

			kTangent := mA + mB + iA*rtA*rtA + iB*rtB*rtB

			if kTangent > 0.0 {
				vcp.TangentMass = 1.0 / kTangent
			} else {
				vcp.TangentMass = 0.0
			}

			// Setup a velocity bias for restitution.
			vcp.VelocityBias = 0.0
			vRel := Vec2Dot(
				vc.Normal,
				Vec2Sub(
					Vec2Sub(
						Vec2Add(
							vB,
							Vec2CrossScalarVector(wB, vcp.RB),
						),
						vA),
					Vec2CrossScalarVector(wA, vcp.RA),
				),
			)
			if vRel < -velocityThreshold {
				vcp.VelocityBias = -vc.Restitution * vRel
			}
		}

		// If we have two points, then prepare the block solver.
		if vc.PointCount == 2 && g_blockSolve {
			vcp1 := &vc.Points[0]
			vcp2 := &vc.Points[1]

			rn1A := Vec2Cross(vcp1.RA, vc.Normal)
			rn1B := Vec2Cross(vcp1.RB, vc.Normal)
			rn2A := Vec2Cross(vcp2.RA, vc.Normal)
			rn2B := Vec2Cross(vcp2.RB, vc.Normal)

			k11 := mA + mB + iA*rn1A*rn1A + iB*rn1B*rn1B
			k22 := mA + mB + iA*rn2A*rn2A + iB*rn2B*rn2B
			k12 := mA + mB + iA*rn1A*rn2A + iB*rn1B*rn2B

			// Ensure a reasonable condition number.
			k_maxConditionNumber := 1000.0
			if k11*k11 < k_maxConditionNumber*(k11*k22-k12*k12) {
				// K is safe to invert.
				vc.K.Ex.Set(k11, k12)
				vc.K.Ey.Set(k12, k22)
				vc.NormalMass = vc.K.GetInverse()
			} else {
				// The constraints are redundant, just use one.
				// TODO_ERIN use deepest?
				vc.PointCount = 1
			}
		}
	}
}

func (solver *ContactSolver) WarmStart() {
	// Warm start.
	for i := 0; i < solver.M_count; i++ {
		vc := &solver.M_velocityConstraints[i]

		indexA := vc.IndexA
		indexB := vc.IndexB
		mA := vc.InvMassA
		iA := vc.InvIA
		mB := vc.InvMassB
		iB := vc.InvIB
		pointCount := vc.PointCount

		vA := solver.M_velocities[indexA].V
		wA := solver.M_velocities[indexA].W
		vB := solver.M_velocities[indexB].V
		wB := solver.M_velocities[indexB].W

		normal := vc.Normal
		tangent := Vec2CrossVectorScalar(normal, 1.0)

		for j := 0; j < pointCount; j++ {
			vcp := &vc.Points[j]
			P := Vec2Add(Vec2MulScalar(vcp.NormalImpulse, normal), Vec2MulScalar(vcp.TangentImpulse, tangent))
			wA -= iA * Vec2Cross(vcp.RA, P)
			vA.OperatorMinusInplace(Vec2MulScalar(mA, P))
			wB += iB * Vec2Cross(vcp.RB, P)
			vB.OperatorPlusInplace(Vec2MulScalar(mB, P))
		}

		solver.M_velocities[indexA].V = vA
		solver.M_velocities[indexA].W = wA
		solver.M_velocities[indexB].V = vB
		solver.M_velocities[indexB].W = wB
	}
}

func (solver *ContactSolver) SolveVelocityConstraints() {
	for i := 0; i < solver.M_count; i++ {
		vc := &solver.M_velocityConstraints[i]

		indexA := vc.IndexA
		indexB := vc.IndexB
		mA := vc.InvMassA
		iA := vc.InvIA
		mB := vc.InvMassB
		iB := vc.InvIB
		pointCount := vc.PointCount

		vA := solver.M_velocities[indexA].V
		wA := solver.M_velocities[indexA].W
		vB := solver.M_velocities[indexB].V
		wB := solver.M_velocities[indexB].W

		normal := vc.Normal
		tangent := Vec2CrossVectorScalar(normal, 1.0)
		friction := vc.Friction

		assert(pointCount == 1 || pointCount == 2)

		// Solve tangent constraints first because non-penetration is more important
		// than friction.
		for j := 0; j < pointCount; j++ {
			vcp := &vc.Points[j]

			// Relative velocity at contact
			dv := Vec2Add(
				vB,
				Vec2Sub(
					Vec2Sub(
						Vec2CrossScalarVector(wB, vcp.RB),
						vA,
					),
					Vec2CrossScalarVector(wA, vcp.RA),
				),
			)

			// Compute tangent force
			vt := Vec2Dot(dv, tangent) - vc.TangentSpeed
			lambda := vcp.TangentMass * (-vt)

			// b2Clamp the accumulated force
			maxFriction := friction * vcp.NormalImpulse
			newImpulse := FloatClamp(vcp.TangentImpulse+lambda, -maxFriction, maxFriction)
			lambda = newImpulse - vcp.TangentImpulse
			vcp.TangentImpulse = newImpulse

			// Apply contact impulse
			P := Vec2MulScalar(lambda, tangent)

			vA.OperatorMinusInplace(Vec2MulScalar(mA, P))
			wA -= iA * Vec2Cross(vcp.RA, P)

			vB.OperatorPlusInplace(Vec2MulScalar(mB, P))
			wB += iB * Vec2Cross(vcp.RB, P)
		}

		// Solve normal constraints
		if pointCount == 1 || !g_blockSolve {
			for j := 0; j < pointCount; j++ {
				vcp := &vc.Points[j]

				// Relative velocity at contact
				dv := Vec2Add(
					vB,
					Vec2Sub(
						Vec2Sub(
							Vec2CrossScalarVector(wB, vcp.RB),
							vA,
						),
						Vec2CrossScalarVector(wA, vcp.RA),
					),
				)

				// Compute normal impulse
				vn := Vec2Dot(dv, normal)
				lambda := -vcp.NormalMass * (vn - vcp.VelocityBias)

				// b2Clamp the accumulated impulse
				newImpulse := math.Max(vcp.NormalImpulse+lambda, 0.0)
				lambda = newImpulse - vcp.NormalImpulse
				vcp.NormalImpulse = newImpulse

				// Apply contact impulse
				P := Vec2MulScalar(lambda, normal)
				vA.OperatorMinusInplace(Vec2MulScalar(mA, P))
				wA -= iA * Vec2Cross(vcp.RA, P)

				vB.OperatorPlusInplace(Vec2MulScalar(mB, P))
				wB += iB * Vec2Cross(vcp.RB, P)
			}
		} else {
			// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
			// Build the mini LCP for this contact patch
			//
			// vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
			//
			// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
			// b = vn0 - velocityBias
			//
			// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
			// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
			// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
			// solution that satisfies the problem is chosen.
			//
			// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
			// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
			//
			// Substitute:
			//
			// x = a + d
			//
			// a := old total impulse
			// x := new total impulse
			// d := incremental impulse
			//
			// For the current iteration we extend the formula for the incremental impulse
			// to compute the new total impulse:
			//
			// vn = A * d + b
			//    = A * (x - a) + b
			//    = A * x + b - A * a
			//    = A * x + b'
			// b' = b - A * a;

			cp1 := &vc.Points[0]
			cp2 := &vc.Points[1]

			a := MakeVec2(cp1.NormalImpulse, cp2.NormalImpulse)
			assert(a.X >= 0.0 && a.Y >= 0.0)

			// Relative velocity at contact
			dv1 := Vec2Add(vB, Vec2Sub(Vec2Sub(Vec2CrossScalarVector(wB, cp1.RB), vA), Vec2CrossScalarVector(wA, cp1.RA)))
			dv2 := Vec2Add(vB, Vec2Sub(Vec2Sub(Vec2CrossScalarVector(wB, cp2.RB), vA), Vec2CrossScalarVector(wA, cp2.RA)))

			// Compute normal velocity
			vn1 := Vec2Dot(dv1, normal)
			vn2 := Vec2Dot(dv2, normal)

			b := MakeVec2(0, 0)
			b.X = vn1 - cp1.VelocityBias
			b.Y = vn2 - cp2.VelocityBias

			// Compute b'
			b.OperatorMinusInplace(Vec2Mat22Mul(vc.K, a))

			const k_errorTol = 0.001
			// B2_NOT_USED(k_errorTol);

			for {
				//
				// Case 1: vn = 0
				//
				// 0 = A * x + b'
				//
				// Solve for x:
				//
				// x = - inv(A) * b'
				//
				x := Vec2Mat22Mul(vc.NormalMass, b).OperatorNegate()

				if x.X >= 0.0 && x.Y >= 0.0 {
					// Get the incremental impulse
					d := Vec2Sub(x, a)

					// Apply incremental impulse
					P1 := Vec2MulScalar(d.X, normal)
					P2 := Vec2MulScalar(d.Y, normal)
					vA.OperatorMinusInplace(Vec2MulScalar(mA, Vec2Add(P1, P2)))
					wA -= iA * (Vec2Cross(cp1.RA, P1) + Vec2Cross(cp2.RA, P2))

					vB.OperatorPlusInplace(Vec2MulScalar(mB, Vec2Add(P1, P2)))
					wB += iB * (Vec2Cross(cp1.RB, P1) + Vec2Cross(cp2.RB, P2))

					// Accumulate
					cp1.NormalImpulse = x.X
					cp2.NormalImpulse = x.Y

					if B2_DEBUG_SOLVER == 1 {
						// Postconditions
						dv1 = Vec2Add(
							vB,
							Vec2Sub(
								Vec2Sub(
									Vec2CrossScalarVector(wB, cp1.RB),
									vA,
								),
								Vec2CrossScalarVector(wA, cp1.RA),
							),
						)
						dv2 = Vec2Add(
							vB,
							Vec2Sub(
								Vec2Sub(
									Vec2CrossScalarVector(wB, cp2.RB),
									vA,
								),
								Vec2CrossScalarVector(wA, cp2.RA),
							),
						)

						// Compute normal velocity
						vn1 = Vec2Dot(dv1, normal)
						vn2 = Vec2Dot(dv2, normal)

						assert(math.Abs(vn1-cp1.VelocityBias) < k_errorTol)
						assert(math.Abs(vn2-cp2.VelocityBias) < k_errorTol)
					}
					break
				}

				//
				// Case 2: vn1 = 0 and x2 = 0
				//
				//   0 = a11 * x1 + a12 * 0 + b1'
				// vn2 = a21 * x1 + a22 * 0 + b2'
				//
				x.X = -cp1.NormalMass * b.X
				x.Y = 0.0
				vn1 = 0.0
				vn2 = vc.K.Ex.Y*x.X + b.Y
				if x.X >= 0.0 && vn2 >= 0.0 {
					// Get the incremental impulse
					d := Vec2Sub(x, a)

					// Apply incremental impulse
					P1 := Vec2MulScalar(d.X, normal)
					P2 := Vec2MulScalar(d.Y, normal)
					vA.OperatorMinusInplace(Vec2MulScalar(mA, Vec2Add(P1, P2)))
					wA -= iA * (Vec2Cross(cp1.RA, P1) + Vec2Cross(cp2.RA, P2))

					vB.OperatorPlusInplace(Vec2MulScalar(mB, Vec2Add(P1, P2)))
					wB += iB * (Vec2Cross(cp1.RB, P1) + Vec2Cross(cp2.RB, P2))

					// Accumulate
					cp1.NormalImpulse = x.X
					cp2.NormalImpulse = x.Y

					if B2_DEBUG_SOLVER == 1 {
						// Postconditions
						dv1 = Vec2Add(vB, Vec2Sub(Vec2Sub(Vec2CrossScalarVector(wB, cp1.RB), vA), Vec2CrossScalarVector(wA, cp1.RA)))

						// Compute normal velocity
						vn1 = Vec2Dot(dv1, normal)

						assert(math.Abs(vn1-cp1.VelocityBias) < k_errorTol)
					}
					break
				}

				//
				// Case 3: vn2 = 0 and x1 = 0
				//
				// vn1 = a11 * 0 + a12 * x2 + b1'
				//   0 = a21 * 0 + a22 * x2 + b2'
				//
				x.X = 0.0
				x.Y = -cp2.NormalMass * b.Y
				vn1 = vc.K.Ey.X*x.Y + b.X
				vn2 = 0.0

				if x.Y >= 0.0 && vn1 >= 0.0 {
					// Resubstitute for the incremental impulse
					d := Vec2Sub(x, a)

					// Apply incremental impulse
					P1 := Vec2MulScalar(d.X, normal)
					P2 := Vec2MulScalar(d.Y, normal)
					vA.OperatorMinusInplace(Vec2MulScalar(mA, Vec2Add(P1, P2)))
					wA -= iA * (Vec2Cross(cp1.RA, P1) + Vec2Cross(cp2.RA, P2))

					vB.OperatorPlusInplace(Vec2MulScalar(mB, Vec2Add(P1, P2)))
					wB += iB * (Vec2Cross(cp1.RB, P1) + Vec2Cross(cp2.RB, P2))

					// Accumulate
					cp1.NormalImpulse = x.X
					cp2.NormalImpulse = x.Y

					if B2_DEBUG_SOLVER == 1 {
						// Postconditions
						dv2 = Vec2Add(vB, Vec2Sub(Vec2Sub(Vec2CrossScalarVector(wB, cp2.RB), vA), Vec2CrossScalarVector(wA, cp2.RA)))

						// Compute normal velocity
						vn2 = Vec2Dot(dv2, normal)

						assert(math.Abs(vn2-cp2.VelocityBias) < k_errorTol)
					}

					break
				}

				//
				// Case 4: x1 = 0 and x2 = 0
				//
				// vn1 = b1
				// vn2 = b2;
				x.X = 0.0
				x.Y = 0.0
				vn1 = b.X
				vn2 = b.Y

				if vn1 >= 0.0 && vn2 >= 0.0 {
					// Resubstitute for the incremental impulse
					d := Vec2Sub(x, a)

					// Apply incremental impulse
					P1 := Vec2MulScalar(d.X, normal)
					P2 := Vec2MulScalar(d.Y, normal)
					vA.OperatorMinusInplace(Vec2MulScalar(mA, Vec2Add(P1, P2)))
					wA -= iA * (Vec2Cross(cp1.RA, P1) + Vec2Cross(cp2.RA, P2))

					vB.OperatorPlusInplace(Vec2MulScalar(mB, Vec2Add(P1, P2)))
					wB += iB * (Vec2Cross(cp1.RB, P1) + Vec2Cross(cp2.RB, P2))

					// Accumulate
					cp1.NormalImpulse = x.X
					cp2.NormalImpulse = x.Y

					break
				}

				// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
				break
			}
		}

		solver.M_velocities[indexA].V = vA
		solver.M_velocities[indexA].W = wA
		solver.M_velocities[indexB].V = vB
		solver.M_velocities[indexB].W = wB
	}
}

func (solver *ContactSolver) StoreImpulses() {
	for i := 0; i < solver.M_count; i++ {
		vc := &solver.M_velocityConstraints[i]
		manifold := solver.M_contacts[vc.ContactIndex].GetManifold()

		for j := 0; j < vc.PointCount; j++ {
			manifold.Points[j].NormalImpulse = vc.Points[j].NormalImpulse
			manifold.Points[j].TangentImpulse = vc.Points[j].TangentImpulse
		}
	}
}

type PositionSolverManifold struct {
	Normal     Vec2
	Point      Vec2
	Separation float64
}

func MakePositionSolverManifold() PositionSolverManifold {
	return PositionSolverManifold{}
}

func (solvermanifold *PositionSolverManifold) Initialize(pc *ContactPositionConstraint, xfA Transform, xfB Transform, index int) {
	assert(pc.PointCount > 0)

	switch pc.Type {
	case ManifoldType.Circles:
		pointA := TransformVec2Mul(xfA, pc.LocalPoint)
		pointB := TransformVec2Mul(xfB, pc.LocalPoints[0])
		solvermanifold.Normal = Vec2Sub(pointB, pointA)
		solvermanifold.Normal.Normalize()
		solvermanifold.Point = Vec2MulScalar(0.5, Vec2Add(pointA, pointB))
		solvermanifold.Separation = Vec2Dot(Vec2Sub(pointB, pointA), solvermanifold.Normal) - pc.RadiusA - pc.RadiusB

	case ManifoldType.FaceA:
		solvermanifold.Normal = RotVec2Mul(xfA.Q, pc.LocalNormal)
		planePoint := TransformVec2Mul(xfA, pc.LocalPoint)

		clipPoint := TransformVec2Mul(xfB, pc.LocalPoints[index])
		solvermanifold.Separation = Vec2Dot(Vec2Sub(clipPoint, planePoint), solvermanifold.Normal) - pc.RadiusA - pc.RadiusB
		solvermanifold.Point = clipPoint

	case ManifoldType.FaceB:
		solvermanifold.Normal = RotVec2Mul(xfB.Q, pc.LocalNormal)
		planePoint := TransformVec2Mul(xfB, pc.LocalPoint)

		clipPoint := TransformVec2Mul(xfA, pc.LocalPoints[index])
		solvermanifold.Separation = Vec2Dot(Vec2Sub(clipPoint, planePoint), solvermanifold.Normal) - pc.RadiusA - pc.RadiusB
		solvermanifold.Point = clipPoint

		// Ensure normal points from A to B
		solvermanifold.Normal = solvermanifold.Normal.OperatorNegate()
	}
}

// Sequential solver.
func (solver *ContactSolver) SolvePositionConstraints() bool {
	minSeparation := 0.0

	for i := 0; i < solver.M_count; i++ {
		pc := &solver.M_positionConstraints[i]

		indexA := pc.IndexA
		indexB := pc.IndexB
		localCenterA := pc.LocalCenterA
		mA := pc.InvMassA
		iA := pc.InvIA
		localCenterB := pc.LocalCenterB
		mB := pc.InvMassB
		iB := pc.InvIB
		pointCount := pc.PointCount

		cA := solver.M_positions[indexA].C
		aA := solver.M_positions[indexA].A

		cB := solver.M_positions[indexB].C
		aB := solver.M_positions[indexB].A

		// Solve normal constraints
		for j := 0; j < pointCount; j++ {
			xfA := MakeTransform()
			xfB := MakeTransform()

			xfA.Q.Set(aA)
			xfB.Q.Set(aB)
			xfA.P = Vec2Sub(cA, RotVec2Mul(xfA.Q, localCenterA))
			xfB.P = Vec2Sub(cB, RotVec2Mul(xfB.Q, localCenterB))

			psm := MakePositionSolverManifold()
			psm.Initialize(pc, xfA, xfB, j)
			normal := psm.Normal

			point := psm.Point
			separation := psm.Separation

			rA := Vec2Sub(point, cA)
			rB := Vec2Sub(point, cB)

			// Track max constraint error.
			minSeparation = math.Min(minSeparation, separation)

			// Prevent large corrections and allow slop.
			C := FloatClamp(baumgarte*(separation+linearSlop), -maxLinearCorrection, 0.0)

			// Compute the effective mass.
			rnA := Vec2Cross(rA, normal)
			rnB := Vec2Cross(rB, normal)
			K := mA + mB + iA*rnA*rnA + iB*rnB*rnB

			// Compute normal impulse
			impulse := 0.0
			if K > 0.0 {
				impulse = -C / K
			}

			P := Vec2MulScalar(impulse, normal)

			cA.OperatorMinusInplace(Vec2MulScalar(mA, P))
			aA -= iA * Vec2Cross(rA, P)

			cB.OperatorPlusInplace(Vec2MulScalar(mB, P))
			aB += iB * Vec2Cross(rB, P)
		}

		solver.M_positions[indexA].C = cA
		solver.M_positions[indexA].A = aA

		solver.M_positions[indexB].C = cB
		solver.M_positions[indexB].A = aB
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -3.0*linearSlop
}

// Sequential position solver for position constraints.
func (solver *ContactSolver) SolveTOIPositionConstraints(toiIndexA int, toiIndexB int) bool {
	minSeparation := 0.0

	for i := 0; i < solver.M_count; i++ {
		pc := &solver.M_positionConstraints[i]

		indexA := pc.IndexA
		indexB := pc.IndexB
		localCenterA := pc.LocalCenterA
		localCenterB := pc.LocalCenterB
		pointCount := pc.PointCount

		mA := 0.0
		iA := 0.0
		if indexA == toiIndexA || indexA == toiIndexB {
			mA = pc.InvMassA
			iA = pc.InvIA
		}

		mB := 0.0
		iB := 0.0
		if indexB == toiIndexA || indexB == toiIndexB {
			mB = pc.InvMassB
			iB = pc.InvIB
		}

		cA := solver.M_positions[indexA].C
		aA := solver.M_positions[indexA].A

		cB := solver.M_positions[indexB].C
		aB := solver.M_positions[indexB].A

		// Solve normal constraints
		for j := 0; j < pointCount; j++ {
			xfA := MakeTransform()
			xfB := MakeTransform()

			xfA.Q.Set(aA)
			xfB.Q.Set(aB)
			xfB.P = Vec2Sub(cB, RotVec2Mul(xfB.Q, localCenterB))
			xfA.P = Vec2Sub(cA, RotVec2Mul(xfA.Q, localCenterA))

			psm := MakePositionSolverManifold()
			psm.Initialize(pc, xfA, xfB, j)
			normal := psm.Normal

			point := psm.Point
			separation := psm.Separation

			rA := Vec2Sub(point, cA)
			rB := Vec2Sub(point, cB)

			// Track max constraint error.
			minSeparation = math.Min(minSeparation, separation)

			// Prevent large corrections and allow slop.
			C := FloatClamp(toiBaugarte*(separation+linearSlop), -maxLinearCorrection, 0.0)

			// Compute the effective mass.
			rnA := Vec2Cross(rA, normal)
			rnB := Vec2Cross(rB, normal)
			K := mA + mB + iA*rnA*rnA + iB*rnB*rnB

			// Compute normal impulse
			impulse := 0.0
			if K > 0.0 {
				impulse = -C / K
			}

			P := Vec2MulScalar(impulse, normal)

			cA.OperatorMinusInplace(Vec2MulScalar(mA, P))
			aA -= iA * Vec2Cross(rA, P)

			cB.OperatorPlusInplace(Vec2MulScalar(mB, P))
			aB += iB * Vec2Cross(rB, P)
		}

		solver.M_positions[indexA].C = cA
		solver.M_positions[indexA].A = aA

		solver.M_positions[indexB].C = cB
		solver.M_positions[indexB].A = aB
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -1.5*linearSlop
}
