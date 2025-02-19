package box2d

import (
	"fmt"
	"math"
)

// Revolute joint definition. This requires defining an
// anchor point where the bodies are joined. The definition
// uses local anchor points so that the initial configuration
// can violate the constraint slightly. You also need to
// specify the initial relative angle for joint limits. This
// helps when saving and loading a game.
// The local anchor points are measured from the body's origin
// rather than the center of mass because:
//  1. you might not know where the center of mass will be.
//  2. if you add/remove shapes from a body and recompute the mass,
//     the joints will be broken.
type RevoluteJointDef struct {
	JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	ReferenceAngle float64

	/// A flag to enable joint limits.
	EnableLimit bool

	/// The lower angle for the joint limit (radians).
	LowerAngle float64

	/// The upper angle for the joint limit (radians).
	UpperAngle float64

	/// A flag to enable the joint motor.
	EnableMotor bool

	/// The desired motor speed. Usually in radians per second.
	MotorSpeed float64

	/// The maximum motor torque used to achieve the desired motor speed.
	/// Usually in N-m.
	MaxMotorTorque float64
}

func MakeRevoluteJointDef() RevoluteJointDef {
	res := RevoluteJointDef{
		JointDef: MakeJointDef(),
	}

	res.Type = JointType.Revolute
	res.LocalAnchorA.Set(0.0, 0.0)
	res.LocalAnchorB.Set(0.0, 0.0)
	res.ReferenceAngle = 0.0
	res.LowerAngle = 0.0
	res.UpperAngle = 0.0
	res.MaxMotorTorque = 0.0
	res.MotorSpeed = 0.0
	res.EnableLimit = false
	res.EnableMotor = false

	return res
}

// A revolute joint constrains two bodies to share a common point while they
// are free to rotate about the point. The relative rotation about the shared
// point is the joint angle. You can limit the relative rotation with
// a joint limit that specifies a lower and upper angle. You can use a motor
// to drive the relative rotation about the shared point. A maximum motor torque
// is provided so that infinite forces are not generated.
type RevoluteJoint struct {
	*Joint

	// Solver shared
	M_localAnchorA Vec2
	M_localAnchorB Vec2
	M_impulse      Vec3
	M_motorImpulse float64

	M_enableMotor    bool
	M_maxMotorTorque float64
	M_motorSpeed     float64

	M_enableLimit    bool
	M_referenceAngle float64
	M_lowerAngle     float64
	M_upperAngle     float64

	// Solver temp
	M_indexA       int
	M_indexB       int
	M_rA           Vec2
	M_rB           Vec2
	M_localCenterA Vec2
	M_localCenterB Vec2
	M_invMassA     float64
	M_invMassB     float64
	M_invIA        float64
	M_invIB        float64
	M_mass         Mat33   // effective mass for point-to-point constraint.
	M_motorMass    float64 // effective mass for motor/limit angular constraint.
	M_limitState   uint8
}

// The local anchor point relative to bodyA's origin.
func (joint RevoluteJoint) GetLocalAnchorA() Vec2 {
	return joint.M_localAnchorA
}

// The local anchor point relative to bodyB's origin.
func (joint RevoluteJoint) GetLocalAnchorB() Vec2 {
	return joint.M_localAnchorB
}

// Get the reference angle.
func (joint RevoluteJoint) GetReferenceAngle() float64 {
	return joint.M_referenceAngle
}

func (joint RevoluteJoint) GetMaxMotorTorque() float64 {
	return joint.M_maxMotorTorque
}

func (joint RevoluteJoint) GetMotorSpeed() float64 {
	return joint.M_motorSpeed
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

func (def *RevoluteJointDef) Initialize(bA *Body, bB *Body, anchor Vec2) {
	def.BodyA = bA
	def.BodyB = bB
	def.LocalAnchorA = def.BodyA.GetLocalPoint(anchor)
	def.LocalAnchorB = def.BodyB.GetLocalPoint(anchor)
	def.ReferenceAngle = def.BodyB.GetAngle() - def.BodyA.GetAngle()
}

func MakeRevoluteJoint(def *RevoluteJointDef) *RevoluteJoint {
	res := RevoluteJoint{
		Joint: MakeJoint(def),
	}

	res.M_localAnchorA = def.LocalAnchorA
	res.M_localAnchorB = def.LocalAnchorB
	res.M_referenceAngle = def.ReferenceAngle

	res.M_impulse.SetZero()
	res.M_motorImpulse = 0.0

	res.M_lowerAngle = def.LowerAngle
	res.M_upperAngle = def.UpperAngle
	res.M_maxMotorTorque = def.MaxMotorTorque
	res.M_motorSpeed = def.MotorSpeed
	res.M_enableLimit = def.EnableLimit
	res.M_enableMotor = def.EnableMotor
	res.M_limitState = LimitState.Inactive

	return &res
}

func (joint *RevoluteJoint) InitVelocityConstraints(data SolverData) {
	joint.M_indexA = joint.M_bodyA.M_islandIndex
	joint.M_indexB = joint.M_bodyB.M_islandIndex
	joint.M_localCenterA = joint.M_bodyA.M_sweep.LocalCenter
	joint.M_localCenterB = joint.M_bodyB.M_sweep.LocalCenter
	joint.M_invMassA = joint.M_bodyA.M_invMass
	joint.M_invMassB = joint.M_bodyB.M_invMass
	joint.M_invIA = joint.M_bodyA.M_invI
	joint.M_invIB = joint.M_bodyB.M_invI

	aA := data.Positions[joint.M_indexA].A
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W

	aB := data.Positions[joint.M_indexB].A
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	qA := MakeRotFromAngle(aA)
	qB := MakeRotFromAngle(aB)

	joint.M_rA = RotVec2Mul(qA, Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	joint.M_rB = RotVec2Mul(qB, Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	fixedRotation := (iA+iB == 0.0)

	joint.M_mass.Ex.X = mA + mB + joint.M_rA.Y*joint.M_rA.Y*iA + joint.M_rB.Y*joint.M_rB.Y*iB
	joint.M_mass.Ey.X = -joint.M_rA.Y*joint.M_rA.X*iA - joint.M_rB.Y*joint.M_rB.X*iB
	joint.M_mass.Ez.X = -joint.M_rA.Y*iA - joint.M_rB.Y*iB
	joint.M_mass.Ex.Y = joint.M_mass.Ey.X
	joint.M_mass.Ey.Y = mA + mB + joint.M_rA.X*joint.M_rA.X*iA + joint.M_rB.X*joint.M_rB.X*iB
	joint.M_mass.Ez.Y = joint.M_rA.X*iA + joint.M_rB.X*iB
	joint.M_mass.Ex.Z = joint.M_mass.Ez.X
	joint.M_mass.Ey.Z = joint.M_mass.Ez.Y
	joint.M_mass.Ez.Z = iA + iB

	joint.M_motorMass = iA + iB
	if joint.M_motorMass > 0.0 {
		joint.M_motorMass = 1.0 / joint.M_motorMass
	}

	if !joint.M_enableMotor || fixedRotation {
		joint.M_motorImpulse = 0.0
	}

	if joint.M_enableLimit && !fixedRotation {
		jointAngle := aB - aA - joint.M_referenceAngle
		if math.Abs(joint.M_upperAngle-joint.M_lowerAngle) < 2.0*angularSlop {
			joint.M_limitState = LimitState.Equal
		} else if jointAngle <= joint.M_lowerAngle {
			if joint.M_limitState != LimitState.AtLowerLimit {
				joint.M_impulse.Z = 0.0
			}
			joint.M_limitState = LimitState.AtLowerLimit
		} else if jointAngle >= joint.M_upperAngle {
			if joint.M_limitState != LimitState.AtUpperLimit {
				joint.M_impulse.Z = 0.0
			}
			joint.M_limitState = LimitState.AtUpperLimit
		} else {
			joint.M_limitState = LimitState.Inactive
			joint.M_impulse.Z = 0.0
		}
	} else {
		joint.M_limitState = LimitState.Inactive
	}

	if data.Step.WarmStarting {
		// Scale impulses to support a variable time step.
		joint.M_impulse.OperatorScalarMultInplace(data.Step.DtRatio)
		joint.M_motorImpulse *= data.Step.DtRatio

		P := MakeVec2(joint.M_impulse.X, joint.M_impulse.Y)

		vA.OperatorMinusInplace(Vec2MulScalar(mA, P))
		wA -= iA * (Vec2Cross(joint.M_rA, P) + joint.M_motorImpulse + joint.M_impulse.Z)

		vB.OperatorPlusInplace(Vec2MulScalar(mB, P))
		wB += iB * (Vec2Cross(joint.M_rB, P) + joint.M_motorImpulse + joint.M_impulse.Z)
	} else {
		joint.M_impulse.SetZero()
		joint.M_motorImpulse = 0.0
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *RevoluteJoint) SolveVelocityConstraints(data SolverData) {
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	fixedRotation := (iA+iB == 0.0)

	// Solve motor constraint.
	if joint.M_enableMotor && joint.M_limitState != LimitState.Equal && !fixedRotation {
		Cdot := wB - wA - joint.M_motorSpeed
		impulse := -joint.M_motorMass * Cdot
		oldImpulse := joint.M_motorImpulse
		maxImpulse := data.Step.Dt * joint.M_maxMotorTorque
		joint.M_motorImpulse = FloatClamp(joint.M_motorImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = joint.M_motorImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	// Solve limit constraint.
	if joint.M_enableLimit && joint.M_limitState != LimitState.Inactive && !fixedRotation {
		Cdot1 := Vec2Sub(Vec2Sub(Vec2Add(vB, Vec2CrossScalarVector(wB, joint.M_rB)), vA), Vec2CrossScalarVector(wA, joint.M_rA))
		Cdot2 := wB - wA
		Cdot := MakeVec3(Cdot1.X, Cdot1.Y, Cdot2)

		impulse := joint.M_mass.Solve33(Cdot).OperatorNegate()

		if joint.M_limitState == LimitState.Equal {
			joint.M_impulse.OperatorPlusInplace(impulse)
		} else if joint.M_limitState == LimitState.AtLowerLimit {
			newImpulse := joint.M_impulse.Z + impulse.Z
			if newImpulse < 0.0 {
				rhs := Vec2Add(Cdot1.OperatorNegate(), Vec2MulScalar(joint.M_impulse.Z, MakeVec2(joint.M_mass.Ez.X, joint.M_mass.Ez.Y)))
				reduced := joint.M_mass.Solve22(rhs)
				impulse.X = reduced.X
				impulse.Y = reduced.Y
				impulse.Z = -joint.M_impulse.Z
				joint.M_impulse.X += reduced.X
				joint.M_impulse.Y += reduced.Y
				joint.M_impulse.Z = 0.0
			} else {
				joint.M_impulse.OperatorPlusInplace(impulse)
			}
		} else if joint.M_limitState == LimitState.AtUpperLimit {
			newImpulse := joint.M_impulse.Z + impulse.Z
			if newImpulse > 0.0 {
				rhs := Vec2Add(Cdot1.OperatorNegate(), Vec2MulScalar(joint.M_impulse.Z, MakeVec2(joint.M_mass.Ez.X, joint.M_mass.Ez.Y)))
				reduced := joint.M_mass.Solve22(rhs)
				impulse.X = reduced.X
				impulse.Y = reduced.Y
				impulse.Z = -joint.M_impulse.Z
				joint.M_impulse.X += reduced.X
				joint.M_impulse.Y += reduced.Y
				joint.M_impulse.Z = 0.0
			} else {
				joint.M_impulse.OperatorPlusInplace(impulse)
			}
		}

		P := MakeVec2(impulse.X, impulse.Y)

		vA.OperatorMinusInplace(Vec2MulScalar(mA, P))
		wA -= iA * (Vec2Cross(joint.M_rA, P) + impulse.Z)

		vB.OperatorPlusInplace(Vec2MulScalar(mB, P))
		wB += iB * (Vec2Cross(joint.M_rB, P) + impulse.Z)
	} else {
		// Solve point-to-point constraint
		Cdot := Vec2Sub(Vec2Sub(Vec2Add(vB, Vec2CrossScalarVector(wB, joint.M_rB)), vA), Vec2CrossScalarVector(wA, joint.M_rA))
		impulse := joint.M_mass.Solve22(Cdot.OperatorNegate())

		joint.M_impulse.X += impulse.X
		joint.M_impulse.Y += impulse.Y

		vA.OperatorMinusInplace(Vec2MulScalar(mA, impulse))
		wA -= iA * Vec2Cross(joint.M_rA, impulse)

		vB.OperatorPlusInplace(Vec2MulScalar(mB, impulse))
		wB += iB * Vec2Cross(joint.M_rB, impulse)
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *RevoluteJoint) SolvePositionConstraints(data SolverData) bool {
	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A

	qA := MakeRotFromAngle(aA)
	qB := MakeRotFromAngle(aB)

	angularError := 0.0
	positionError := 0.0

	fixedRotation := (joint.M_invIA+joint.M_invIB == 0.0)

	// Solve angular limit constraint.
	if joint.M_enableLimit && joint.M_limitState != LimitState.Inactive && !fixedRotation {
		angle := aB - aA - joint.M_referenceAngle
		limitImpulse := 0.0

		if joint.M_limitState == LimitState.Equal {
			// Prevent large angular corrections
			C := FloatClamp(angle-joint.M_lowerAngle, -maxAngularCorrection, maxAngularCorrection)
			limitImpulse = -joint.M_motorMass * C
			angularError = math.Abs(C)
		} else if joint.M_limitState == LimitState.AtLowerLimit {
			C := angle - joint.M_lowerAngle
			angularError = -C

			// Prevent large angular corrections and allow some slop.
			C = FloatClamp(C+angularSlop, -maxAngularCorrection, 0.0)
			limitImpulse = -joint.M_motorMass * C
		} else if joint.M_limitState == LimitState.AtUpperLimit {
			C := angle - joint.M_upperAngle
			angularError = C

			// Prevent large angular corrections and allow some slop.
			C = FloatClamp(C-angularSlop, 0.0, maxAngularCorrection)
			limitImpulse = -joint.M_motorMass * C
		}

		aA -= joint.M_invIA * limitImpulse
		aB += joint.M_invIB * limitImpulse
	}

	// Solve point-to-point constraint.
	{
		qA.Set(aA)
		qB.Set(aB)
		rA := RotVec2Mul(qA, Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
		rB := RotVec2Mul(qB, Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))

		C := Vec2Sub(Vec2Sub(Vec2Add(cB, rB), cA), rA)
		positionError = C.Length()

		mA := joint.M_invMassA
		mB := joint.M_invMassB
		iA := joint.M_invIA
		iB := joint.M_invIB

		var K Mat22
		K.Ex.X = mA + mB + iA*rA.Y*rA.Y + iB*rB.Y*rB.Y
		K.Ex.Y = -iA*rA.X*rA.Y - iB*rB.X*rB.Y
		K.Ey.X = K.Ex.Y
		K.Ey.Y = mA + mB + iA*rA.X*rA.X + iB*rB.X*rB.X

		impulse := K.Solve(C).OperatorNegate()

		cA.OperatorMinusInplace(Vec2MulScalar(mA, impulse))
		aA -= iA * Vec2Cross(rA, impulse)

		cB.OperatorPlusInplace(Vec2MulScalar(mB, impulse))
		aB += iB * Vec2Cross(rB, impulse)
	}

	data.Positions[joint.M_indexA].C = cA
	data.Positions[joint.M_indexA].A = aA
	data.Positions[joint.M_indexB].C = cB
	data.Positions[joint.M_indexB].A = aB

	return positionError <= linearSlop && angularError <= angularSlop
}

func (joint RevoluteJoint) GetAnchorA() Vec2 {
	return joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
}

func (joint RevoluteJoint) GetAnchorB() Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint RevoluteJoint) GetReactionForce(inv_dt float64) Vec2 {
	P := MakeVec2(joint.M_impulse.X, joint.M_impulse.Y)
	return Vec2MulScalar(inv_dt, P)
}

func (joint RevoluteJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * joint.M_impulse.Z
}

func (joint RevoluteJoint) GetJointAngle() float64 {
	bA := joint.M_bodyA
	bB := joint.M_bodyB
	return bB.M_sweep.A - bA.M_sweep.A - joint.M_referenceAngle
}

func (joint *RevoluteJoint) GetJointSpeed() float64 {
	bA := joint.M_bodyA
	bB := joint.M_bodyB
	return bB.M_angularVelocity - bA.M_angularVelocity
}

func (joint RevoluteJoint) IsMotorEnabled() bool {
	return joint.M_enableMotor
}

func (joint *RevoluteJoint) EnableMotor(flag bool) {
	if flag != joint.M_enableMotor {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_enableMotor = flag
	}
}

func (joint RevoluteJoint) GetMotorTorque(inv_dt float64) float64 {
	return inv_dt * joint.M_motorImpulse
}

func (joint *RevoluteJoint) SetMotorSpeed(speed float64) {
	if speed != joint.M_motorSpeed {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_motorSpeed = speed
	}
}

func (joint *RevoluteJoint) SetMaxMotorTorque(torque float64) {
	if torque != joint.M_maxMotorTorque {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_maxMotorTorque = torque
	}
}

func (joint RevoluteJoint) IsLimitEnabled() bool {
	return joint.M_enableLimit
}

func (joint *RevoluteJoint) EnableLimit(flag bool) {
	if flag != joint.M_enableLimit {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_enableLimit = flag
		joint.M_impulse.Z = 0.0
	}
}

func (joint RevoluteJoint) GetLowerLimit() float64 {
	return joint.M_lowerAngle
}

func (joint RevoluteJoint) GetUpperLimit() float64 {
	return joint.M_upperAngle
}

func (joint *RevoluteJoint) SetLimits(lower float64, upper float64) {
	assert(lower <= upper)

	if lower != joint.M_lowerAngle || upper != joint.M_upperAngle {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_impulse.Z = 0.0
		joint.M_lowerAngle = lower
		joint.M_upperAngle = upper
	}
}

func (joint *RevoluteJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	fmt.Printf("  b2RevoluteJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.localAnchorA.Set(%.15f, %.15f);\n", joint.M_localAnchorA.X, joint.M_localAnchorA.Y)
	fmt.Printf("  jd.localAnchorB.Set(%.15f, %.15f);\n", joint.M_localAnchorB.X, joint.M_localAnchorB.Y)
	fmt.Printf("  jd.referenceAngle = %.15f;\n", joint.M_referenceAngle)
	fmt.Printf("  jd.enableLimit = bool(%v);\n", joint.M_enableLimit)
	fmt.Printf("  jd.lowerAngle = %.15f;\n", joint.M_lowerAngle)
	fmt.Printf("  jd.upperAngle = %.15f;\n", joint.M_upperAngle)
	fmt.Printf("  jd.enableMotor = bool(%v);\n", joint.M_enableMotor)
	fmt.Printf("  jd.motorSpeed = %.15f;\n", joint.M_motorSpeed)
	fmt.Printf("  jd.maxMotorTorque = %.15f;\n", joint.M_maxMotorTorque)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}
