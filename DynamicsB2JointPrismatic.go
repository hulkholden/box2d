package box2d

import (
	"fmt"
	"math"
)

// Prismatic joint definition. This requires defining a line of
// motion using an axis and an anchor point. The definition uses local
// anchor points and a local axis so that the initial configuration
// can violate the constraint slightly. The joint translation is zero
// when the local anchor points coincide in world space. Using local
// anchors and a local axis helps when saving and loading a game.
type PrismaticJointDef struct {
	JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The local translation unit axis in bodyA.
	LocalAxisA Vec2

	/// The constrained angle between the bodies: bodyB_angle - bodyA_angle.
	ReferenceAngle float64

	/// Enable/disable the joint limit.
	EnableLimit bool

	/// The lower translation limit, usually in meters.
	LowerTranslation float64

	/// The upper translation limit, usually in meters.
	UpperTranslation float64

	/// Enable/disable the joint motor.
	EnableMotor bool

	/// The maximum motor torque, usually in N-m.
	MaxMotorForce float64

	/// The desired motor speed in radians per second.
	MotorSpeed float64
}

func MakePrismaticJointDef() PrismaticJointDef {
	res := PrismaticJointDef{
		JointDef: MakeJointDef(),
	}

	res.Type = JointType.Prismatic
	res.LocalAnchorA.SetZero()
	res.LocalAnchorB.SetZero()
	res.LocalAxisA.Set(1.0, 0.0)
	res.ReferenceAngle = 0.0
	res.EnableLimit = false
	res.LowerTranslation = 0.0
	res.UpperTranslation = 0.0
	res.EnableMotor = false
	res.MaxMotorForce = 0.0
	res.MotorSpeed = 0.0

	return res
}

// A prismatic joint. This joint provides one degree of freedom: translation
// along an axis fixed in bodyA. Relative rotation is prevented. You can
// use a joint limit to restrict the range of motion and a joint motor to
// drive the motion or to model joint friction.
type PrismaticJoint struct {
	*Joint

	// Solver shared
	M_localAnchorA     Vec2
	M_localAnchorB     Vec2
	M_localXAxisA      Vec2
	M_localYAxisA      Vec2
	M_referenceAngle   float64
	M_impulse          Vec3
	M_motorImpulse     float64
	M_lowerTranslation float64
	M_upperTranslation float64
	M_maxMotorForce    float64
	M_motorSpeed       float64
	M_enableLimit      bool
	M_enableMotor      bool
	M_limitState       uint8

	// Solver temp
	M_indexA       int
	M_indexB       int
	M_localCenterA Vec2
	M_localCenterB Vec2
	M_invMassA     float64
	M_invMassB     float64
	M_invIA        float64
	M_invIB        float64
	M_axis, M_perp Vec2
	M_s1, M_s2     float64
	M_a1, M_a2     float64
	M_K            Mat33
	M_motorMass    float64
}

// The local anchor point relative to bodyA's origin.
func (joint PrismaticJoint) GetLocalAnchorA() Vec2 {
	return joint.M_localAnchorA
}

// The local anchor point relative to bodyB's origin.
func (joint PrismaticJoint) GetLocalAnchorB() Vec2 {
	return joint.M_localAnchorB
}

// The local joint axis relative to bodyA.
func (joint PrismaticJoint) GetLocalAxisA() Vec2 {
	return joint.M_localXAxisA
}

// Get the reference angle.
func (joint PrismaticJoint) GetReferenceAngle() float64 {
	return joint.M_referenceAngle
}

func (joint PrismaticJoint) GetMaxMotorForce() float64 {
	return joint.M_maxMotorForce
}

func (joint PrismaticJoint) GetMotorSpeed() float64 {
	return joint.M_motorSpeed
}

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)

// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Block Solver
// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
// when the mass has poor distribution (leading to large torques about the joint anchor points).
//
// The Jacobian has 3 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//     [-vT -a1 vT a2] // limit
//
// u = perp
// v = axis
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

// M * (v2 - v1) = JT * df
// J * v2 = bias
//
// v2 = v1 + invM * JT * df
// J * (v1 + invM * JT * df) = bias
// K * df = bias - J * v1 = -Cdot
// K = J * invM * JT
// Cdot = J * v1 - bias
//
// Now solve for f2.
// df = f2 - f1
// K * (f2 - f1) = -Cdot
// f2 = invK * (-Cdot) + f1
//
// Clamp accumulated limit impulse.
// lower: f2(3) = max(f2(3), 0)
// upper: f2(3) = min(f2(3), 0)
//
// Solve for correct f2(1:2)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
//                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
//
// Now compute impulse to be applied:
// df = f2 - f1

func (joint *PrismaticJointDef) Initialize(bA *Body, bB *Body, anchor Vec2, axis Vec2) {
	joint.BodyA = bA
	joint.BodyB = bB
	joint.LocalAnchorA = joint.BodyA.GetLocalPoint(anchor)
	joint.LocalAnchorB = joint.BodyB.GetLocalPoint(anchor)
	joint.LocalAxisA = joint.BodyA.GetLocalVector(axis)
	joint.ReferenceAngle = joint.BodyB.GetAngle() - joint.BodyA.GetAngle()
}

func MakePrismaticJoint(def *PrismaticJointDef) *PrismaticJoint {
	res := PrismaticJoint{
		Joint: MakeJoint(def),
	}

	res.M_localAnchorA = def.LocalAnchorA
	res.M_localAnchorB = def.LocalAnchorB
	res.M_localXAxisA = def.LocalAxisA
	res.M_localXAxisA.Normalize()
	res.M_localYAxisA = Vec2CrossScalarVector(1.0, res.M_localXAxisA)
	res.M_referenceAngle = def.ReferenceAngle

	res.M_impulse.SetZero()
	res.M_motorMass = 0.0
	res.M_motorImpulse = 0.0

	res.M_lowerTranslation = def.LowerTranslation
	res.M_upperTranslation = def.UpperTranslation
	res.M_maxMotorForce = def.MaxMotorForce
	res.M_motorSpeed = def.MotorSpeed
	res.M_enableLimit = def.EnableLimit
	res.M_enableMotor = def.EnableMotor
	res.M_limitState = LimitState.Inactive

	res.M_axis.SetZero()
	res.M_perp.SetZero()

	return &res
}

func (joint *PrismaticJoint) InitVelocityConstraints(data SolverData) {
	joint.M_indexA = joint.M_bodyA.M_islandIndex
	joint.M_indexB = joint.M_bodyB.M_islandIndex
	joint.M_localCenterA = joint.M_bodyA.M_sweep.LocalCenter
	joint.M_localCenterB = joint.M_bodyB.M_sweep.LocalCenter
	joint.M_invMassA = joint.M_bodyA.M_invMass
	joint.M_invMassB = joint.M_bodyB.M_invMass
	joint.M_invIA = joint.M_bodyA.M_invI
	joint.M_invIB = joint.M_bodyB.M_invI

	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W

	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	qA := MakeRotFromAngle(aA)
	qB := MakeRotFromAngle(aB)

	// Compute the effective masses.
	rA := RotVec2Mul(qA, Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	rB := RotVec2Mul(qB, Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))
	d := Vec2Sub(Vec2Add(Vec2Sub(cB, cA), rB), rA)

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	// Compute motor Jacobian and effective mass.
	{
		joint.M_axis = RotVec2Mul(qA, joint.M_localXAxisA)
		joint.M_a1 = Vec2Cross(Vec2Add(d, rA), joint.M_axis)
		joint.M_a2 = Vec2Cross(rB, joint.M_axis)

		joint.M_motorMass = mA + mB + iA*joint.M_a1*joint.M_a1 + iB*joint.M_a2*joint.M_a2
		if joint.M_motorMass > 0.0 {
			joint.M_motorMass = 1.0 / joint.M_motorMass
		}
	}

	// Prismatic constraint.
	{
		joint.M_perp = RotVec2Mul(qA, joint.M_localYAxisA)

		joint.M_s1 = Vec2Cross(Vec2Add(d, rA), joint.M_perp)
		joint.M_s2 = Vec2Cross(rB, joint.M_perp)

		k11 := mA + mB + iA*joint.M_s1*joint.M_s1 + iB*joint.M_s2*joint.M_s2
		k12 := iA*joint.M_s1 + iB*joint.M_s2
		k13 := iA*joint.M_s1*joint.M_a1 + iB*joint.M_s2*joint.M_a2
		k22 := iA + iB
		if k22 == 0.0 {
			// For bodies with fixed rotation.
			k22 = 1.0
		}
		k23 := iA*joint.M_a1 + iB*joint.M_a2
		k33 := mA + mB + iA*joint.M_a1*joint.M_a1 + iB*joint.M_a2*joint.M_a2

		joint.M_K.Ex.Set(k11, k12, k13)
		joint.M_K.Ey.Set(k12, k22, k23)
		joint.M_K.Ez.Set(k13, k23, k33)
	}

	// Compute motor and limit terms.
	if joint.M_enableLimit {
		jointTranslation := Vec2Dot(joint.M_axis, d)
		if math.Abs(joint.M_upperTranslation-joint.M_lowerTranslation) < 2.0*linearSlop {
			joint.M_limitState = LimitState.Equal
		} else if jointTranslation <= joint.M_lowerTranslation {
			if joint.M_limitState != LimitState.AtLowerLimit {
				joint.M_limitState = LimitState.AtLowerLimit
				joint.M_impulse.Z = 0.0
			}
		} else if jointTranslation >= joint.M_upperTranslation {
			if joint.M_limitState != LimitState.AtUpperLimit {
				joint.M_limitState = LimitState.AtUpperLimit
				joint.M_impulse.Z = 0.0
			}
		} else {
			joint.M_limitState = LimitState.Inactive
			joint.M_impulse.Z = 0.0
		}
	} else {
		joint.M_limitState = LimitState.Inactive
		joint.M_impulse.Z = 0.0
	}

	if !joint.M_enableMotor {
		joint.M_motorImpulse = 0.0
	}

	if data.Step.WarmStarting {
		// Account for variable time step.
		joint.M_impulse.OperatorScalarMultInplace(data.Step.DtRatio)
		joint.M_motorImpulse *= data.Step.DtRatio

		P := Vec2Add(Vec2MulScalar(joint.M_impulse.X, joint.M_perp), Vec2MulScalar(joint.M_motorImpulse+joint.M_impulse.Z, joint.M_axis))
		LA := joint.M_impulse.X*joint.M_s1 + joint.M_impulse.Y + (joint.M_motorImpulse+joint.M_impulse.Z)*joint.M_a1
		LB := joint.M_impulse.X*joint.M_s2 + joint.M_impulse.Y + (joint.M_motorImpulse+joint.M_impulse.Z)*joint.M_a2

		vA.OperatorMinusInplace(Vec2MulScalar(mA, P))
		wA -= iA * LA

		vB.OperatorPlusInplace(Vec2MulScalar(mB, P))
		wB += iB * LB
	} else {
		joint.M_impulse.SetZero()
		joint.M_motorImpulse = 0.0
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *PrismaticJoint) SolveVelocityConstraints(data SolverData) {
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	// Solve linear motor constraint.
	if joint.M_enableMotor && joint.M_limitState != LimitState.Equal {
		Cdot := Vec2Dot(joint.M_axis, Vec2Sub(vB, vA)) + joint.M_a2*wB - joint.M_a1*wA
		impulse := joint.M_motorMass * (joint.M_motorSpeed - Cdot)
		oldImpulse := joint.M_motorImpulse
		maxImpulse := data.Step.Dt * joint.M_maxMotorForce
		joint.M_motorImpulse = FloatClamp(joint.M_motorImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = joint.M_motorImpulse - oldImpulse

		P := Vec2MulScalar(impulse, joint.M_axis)
		LA := impulse * joint.M_a1
		LB := impulse * joint.M_a2

		vA.OperatorMinusInplace(Vec2MulScalar(mA, P))
		wA -= iA * LA

		vB.OperatorPlusInplace(Vec2MulScalar(mB, P))
		wB += iB * LB
	}

	var Cdot1 Vec2
	Cdot1.X = Vec2Dot(joint.M_perp, Vec2Sub(vB, vA)) + joint.M_s2*wB - joint.M_s1*wA
	Cdot1.Y = wB - wA

	if joint.M_enableLimit && joint.M_limitState != LimitState.Inactive {
		// Solve prismatic and limit constraint in block form.
		Cdot2 := 0.0
		Cdot2 = Vec2Dot(joint.M_axis, Vec2Sub(vB, vA)) + joint.M_a2*wB - joint.M_a1*wA
		Cdot := MakeVec3(Cdot1.X, Cdot1.Y, Cdot2)

		f1 := joint.M_impulse
		df := joint.M_K.Solve33(Cdot.OperatorNegate())
		joint.M_impulse.OperatorPlusInplace(df)

		if joint.M_limitState == LimitState.AtLowerLimit {
			joint.M_impulse.Z = math.Max(joint.M_impulse.Z, 0.0)
		} else if joint.M_limitState == LimitState.AtUpperLimit {
			joint.M_impulse.Z = math.Min(joint.M_impulse.Z, 0.0)
		}

		// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
		b := Vec2Sub(Cdot1.OperatorNegate(), Vec2MulScalar(joint.M_impulse.Z-f1.Z, MakeVec2(joint.M_K.Ez.X, joint.M_K.Ez.Y)))
		f2r := Vec2Add(joint.M_K.Solve22(b), MakeVec2(f1.X, f1.Y))
		joint.M_impulse.X = f2r.X
		joint.M_impulse.Y = f2r.Y

		df = Vec3Sub(joint.M_impulse, f1)

		P := Vec2Add(Vec2MulScalar(df.X, joint.M_perp), Vec2MulScalar(df.Z, joint.M_axis))
		LA := df.X*joint.M_s1 + df.Y + df.Z*joint.M_a1
		LB := df.X*joint.M_s2 + df.Y + df.Z*joint.M_a2

		vA.OperatorMinusInplace(Vec2MulScalar(mA, P))
		wA -= iA * LA

		vB.OperatorPlusInplace(Vec2MulScalar(mB, P))
		wB += iB * LB
	} else {
		// Limit is inactive, just solve the prismatic constraint in block form.
		df := joint.M_K.Solve22(Cdot1.OperatorNegate())
		joint.M_impulse.X += df.X
		joint.M_impulse.Y += df.Y

		P := Vec2MulScalar(df.X, joint.M_perp)
		LA := df.X*joint.M_s1 + df.Y
		LB := df.X*joint.M_s2 + df.Y

		vA.OperatorMinusInplace(Vec2MulScalar(mA, P))
		wA -= iA * LA

		vB.OperatorPlusInplace(Vec2MulScalar(mB, P))
		wB += iB * LB
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

// A velocity based solver computes reaction forces(impulses) using the velocity constraint solver.Under this context,
// the position solver is not there to resolve forces.It is only there to cope with integration error.
//
// Therefore, the pseudo impulses in the position solver do not have any physical meaning.Thus it is okay if they suck.
//
// We could take the active state from the velocity solver.However, the joint might push past the limit when the velocity
// solver indicates the limit is inactive.
func (joint *PrismaticJoint) SolvePositionConstraints(data SolverData) bool {
	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A

	qA := MakeRotFromAngle(aA)
	qB := MakeRotFromAngle(aB)

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	// Compute fresh Jacobians
	rA := RotVec2Mul(qA, Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	rB := RotVec2Mul(qB, Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))
	d := Vec2Sub(Vec2Sub(Vec2Add(cB, rB), cA), rA)

	axis := RotVec2Mul(qA, joint.M_localXAxisA)
	a1 := Vec2Cross(Vec2Add(d, rA), axis)
	a2 := Vec2Cross(rB, axis)
	perp := RotVec2Mul(qA, joint.M_localYAxisA)

	s1 := Vec2Cross(Vec2Add(d, rA), perp)
	s2 := Vec2Cross(rB, perp)

	impulse := MakeVec3(0, 0, 0)
	C1 := MakeVec2(0, 0)
	C1.X = Vec2Dot(perp, d)
	C1.Y = aB - aA - joint.M_referenceAngle

	linearError := math.Abs(C1.X)
	angularError := math.Abs(C1.Y)

	active := false
	C2 := 0.0
	if joint.M_enableLimit {
		translation := Vec2Dot(axis, d)
		if math.Abs(joint.M_upperTranslation-joint.M_lowerTranslation) < 2.0*linearSlop {
			// Prevent large angular corrections
			C2 = FloatClamp(translation, -maxLinearCorrection, maxLinearCorrection)
			linearError = math.Max(linearError, math.Abs(translation))
			active = true
		} else if translation <= joint.M_lowerTranslation {
			// Prevent large linear corrections and allow some slop.
			C2 = FloatClamp(translation-joint.M_lowerTranslation+linearSlop, -maxLinearCorrection, 0.0)
			linearError = math.Max(linearError, joint.M_lowerTranslation-translation)
			active = true
		} else if translation >= joint.M_upperTranslation {
			// Prevent large linear corrections and allow some slop.
			C2 = FloatClamp(translation-joint.M_upperTranslation-linearSlop, 0.0, maxLinearCorrection)
			linearError = math.Max(linearError, translation-joint.M_upperTranslation)
			active = true
		}
	}

	if active {
		k11 := mA + mB + iA*s1*s1 + iB*s2*s2
		k12 := iA*s1 + iB*s2
		k13 := iA*s1*a1 + iB*s2*a2
		k22 := iA + iB
		if k22 == 0.0 {
			// For fixed rotation
			k22 = 1.0
		}
		k23 := iA*a1 + iB*a2
		k33 := mA + mB + iA*a1*a1 + iB*a2*a2

		K := MakeMat33()
		K.Ex.Set(k11, k12, k13)
		K.Ey.Set(k12, k22, k23)
		K.Ez.Set(k13, k23, k33)

		C := MakeVec3(0, 0, 0)
		C.X = C1.X
		C.Y = C1.Y
		C.Z = C2

		impulse = K.Solve33(C.OperatorNegate())
	} else {
		k11 := mA + mB + iA*s1*s1 + iB*s2*s2
		k12 := iA*s1 + iB*s2
		k22 := iA + iB
		if k22 == 0.0 {
			k22 = 1.0
		}

		K := MakeMat22()
		K.Ex.Set(k11, k12)
		K.Ey.Set(k12, k22)

		impulse1 := K.Solve(C1.OperatorNegate())
		impulse.X = impulse1.X
		impulse.Y = impulse1.Y
		impulse.Z = 0.0
	}

	P := Vec2Add(Vec2MulScalar(impulse.X, perp), Vec2MulScalar(impulse.Z, axis))
	LA := impulse.X*s1 + impulse.Y + impulse.Z*a1
	LB := impulse.X*s2 + impulse.Y + impulse.Z*a2

	cA.OperatorMinusInplace(Vec2MulScalar(mA, P))
	aA -= iA * LA
	cB.OperatorPlusInplace(Vec2MulScalar(mB, P))
	aB += iB * LB

	data.Positions[joint.M_indexA].C = cA
	data.Positions[joint.M_indexA].A = aA
	data.Positions[joint.M_indexB].C = cB
	data.Positions[joint.M_indexB].A = aB

	return linearError <= linearSlop && angularError <= angularSlop
}

func (joint PrismaticJoint) GetAnchorA() Vec2 {
	return joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
}

func (joint PrismaticJoint) GetAnchorB() Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint PrismaticJoint) GetReactionForce(inv_dt float64) Vec2 {
	return Vec2MulScalar(inv_dt, Vec2Add(Vec2MulScalar(joint.M_impulse.X, joint.M_perp), Vec2MulScalar(joint.M_motorImpulse+joint.M_impulse.Z, joint.M_axis)))
}

func (joint PrismaticJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * joint.M_impulse.Y
}

func (joint PrismaticJoint) GetJointTranslation() float64 {
	pA := joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
	pB := joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
	d := Vec2Sub(pB, pA)
	axis := joint.M_bodyA.GetWorldVector(joint.M_localXAxisA)

	translation := Vec2Dot(d, axis)
	return translation
}

func (joint PrismaticJoint) GetJointSpeed() float64 {
	bA := joint.M_bodyA
	bB := joint.M_bodyB

	rA := RotVec2Mul(bA.M_xf.Q, Vec2Sub(joint.M_localAnchorA, bA.M_sweep.LocalCenter))
	rB := RotVec2Mul(bB.M_xf.Q, Vec2Sub(joint.M_localAnchorB, bB.M_sweep.LocalCenter))
	p1 := Vec2Add(bA.M_sweep.C, rA)
	p2 := Vec2Add(bB.M_sweep.C, rB)
	d := Vec2Sub(p2, p1)
	axis := RotVec2Mul(bA.M_xf.Q, joint.M_localXAxisA)

	vA := bA.M_linearVelocity
	vB := bB.M_linearVelocity
	wA := bA.M_angularVelocity
	wB := bB.M_angularVelocity

	speed := Vec2Dot(d, Vec2CrossScalarVector(wA, axis)) +
		Vec2Dot(axis, Vec2Sub(Vec2Sub(Vec2Add(vB, Vec2CrossScalarVector(wB, rB)), vA), Vec2CrossScalarVector(wA, rA)))
	return speed
}

func (joint PrismaticJoint) IsLimitEnabled() bool {
	return joint.M_enableLimit
}

func (joint *PrismaticJoint) EnableLimit(flag bool) {
	if flag != joint.M_enableLimit {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_enableLimit = flag
		joint.M_impulse.Z = 0.0
	}
}

func (joint PrismaticJoint) GetLowerLimit() float64 {
	return joint.M_lowerTranslation
}

func (joint PrismaticJoint) GetUpperLimit() float64 {
	return joint.M_upperTranslation
}

func (joint *PrismaticJoint) SetLimits(lower float64, upper float64) {
	assert(lower <= upper)
	if lower != joint.M_lowerTranslation || upper != joint.M_upperTranslation {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_lowerTranslation = lower
		joint.M_upperTranslation = upper
		joint.M_impulse.Z = 0.0
	}
}

func (joint PrismaticJoint) IsMotorEnabled() bool {
	return joint.M_enableMotor
}

func (joint *PrismaticJoint) EnableMotor(flag bool) {
	if flag != joint.M_enableMotor {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_enableMotor = flag
	}
}

func (joint *PrismaticJoint) SetMotorSpeed(speed float64) {
	if speed != joint.M_motorSpeed {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_motorSpeed = speed
	}
}

func (joint *PrismaticJoint) SetMaxMotorForce(force float64) {
	if force != joint.M_maxMotorForce {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_maxMotorForce = force
	}
}

func (joint PrismaticJoint) GetMotorForce(inv_dt float64) float64 {
	return inv_dt * joint.M_motorImpulse
}

func (joint *PrismaticJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	fmt.Printf("  b2PrismaticJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.localAnchorA.Set(%.15f, %.15f);\n", joint.M_localAnchorA.X, joint.M_localAnchorA.Y)
	fmt.Printf("  jd.localAnchorB.Set(%.15f, %.15f);\n", joint.M_localAnchorB.X, joint.M_localAnchorB.Y)
	fmt.Printf("  jd.localAxisA.Set(%.15f, %.15f);\n", joint.M_localXAxisA.X, joint.M_localXAxisA.Y)
	fmt.Printf("  jd.referenceAngle = %.15f;\n", joint.M_referenceAngle)
	fmt.Printf("  jd.enableLimit = bool(%v);\n", joint.M_enableLimit)
	fmt.Printf("  jd.lowerTranslation = %.15f;\n", joint.M_lowerTranslation)
	fmt.Printf("  jd.upperTranslation = %.15f;\n", joint.M_upperTranslation)
	fmt.Printf("  jd.enableMotor = bool(%v);\n", joint.M_enableMotor)
	fmt.Printf("  jd.motorSpeed = %.15f;\n", joint.M_motorSpeed)
	fmt.Printf("  jd.maxMotorForce = %.15f;\n", joint.M_maxMotorForce)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}
