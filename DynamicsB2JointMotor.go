package box2d

import (
	"fmt"
)

// Motor joint definition.
type MotorJointDef struct {
	JointDef

	/// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
	LinearOffset Vec2

	/// The bodyB angle minus bodyA angle in radians.
	AngularOffset float64

	/// The maximum motor force in N.
	MaxForce float64

	/// The maximum motor torque in N-m.
	MaxTorque float64

	/// Position correction factor in the range [0,1].
	CorrectionFactor float64
}

func MakeMotorJointDef() MotorJointDef {
	res := MotorJointDef{}
	res.Type = JointType.Motor
	res.LinearOffset.SetZero()
	res.AngularOffset = 0.0
	res.MaxForce = 1.0
	res.MaxTorque = 1.0
	res.CorrectionFactor = 0.3
	return res
}

// A motor joint is used to control the relative motion
// between two bodies. A typical usage is to control the movement
// of a dynamic body with respect to the ground.
type MotorJoint struct {
	*Joint

	// Solver shared
	M_linearOffset     Vec2
	M_angularOffset    float64
	M_linearImpulse    Vec2
	M_angularImpulse   float64
	M_maxForce         float64
	M_maxTorque        float64
	M_correctionFactor float64

	// Solver temp
	M_indexA       int
	M_indexB       int
	M_rA           Vec2
	M_rB           Vec2
	M_localCenterA Vec2
	M_localCenterB Vec2
	M_linearError  Vec2
	M_angularError float64
	M_invMassA     float64
	M_invMassB     float64
	M_invIA        float64
	M_invIB        float64
	M_linearMass   Mat22
	M_angularMass  float64
}

// Point-to-point constraint
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)
//
// r1 = offset - c1
// r2 = -c2

// Angle constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

func (def *MotorJointDef) Initialize(bA *Body, bB *Body) {
	def.BodyA = bA
	def.BodyB = bB
	xB := def.BodyB.GetPosition()
	def.LinearOffset = def.BodyA.GetLocalPoint(xB)

	angleA := def.BodyA.GetAngle()
	angleB := def.BodyB.GetAngle()
	def.AngularOffset = angleB - angleA
}

func MakeMotorJoint(def *MotorJointDef) *MotorJoint {
	res := MotorJoint{
		Joint: MakeJoint(def),
	}

	res.M_linearOffset = def.LinearOffset
	res.M_angularOffset = def.AngularOffset

	res.M_linearImpulse.SetZero()
	res.M_angularImpulse = 0.0

	res.M_maxForce = def.MaxForce
	res.M_maxTorque = def.MaxTorque
	res.M_correctionFactor = def.CorrectionFactor

	return &res
}

func (joint *MotorJoint) InitVelocityConstraints(data SolverData) {
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

	// Compute the effective mass matrix.
	joint.M_rA = RotVec2Mul(qA, Vec2Sub(joint.M_linearOffset, joint.M_localCenterA))
	joint.M_rB = RotVec2Mul(qB, joint.M_localCenterB.OperatorNegate())

	// J = [-I -r1_skew I r2_skew]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	// Upper 2 by 2 of K for point to point
	var K Mat22
	K.Ex.X = mA + mB + iA*joint.M_rA.Y*joint.M_rA.Y + iB*joint.M_rB.Y*joint.M_rB.Y
	K.Ex.Y = -iA*joint.M_rA.X*joint.M_rA.Y - iB*joint.M_rB.X*joint.M_rB.Y
	K.Ey.X = K.Ex.Y
	K.Ey.Y = mA + mB + iA*joint.M_rA.X*joint.M_rA.X + iB*joint.M_rB.X*joint.M_rB.X

	joint.M_linearMass = K.GetInverse()

	joint.M_angularMass = iA + iB
	if joint.M_angularMass > 0.0 {
		joint.M_angularMass = 1.0 / joint.M_angularMass
	}

	joint.M_linearError = Vec2Sub(Vec2Sub(Vec2Add(cB, joint.M_rB), cA), joint.M_rA)
	joint.M_angularError = aB - aA - joint.M_angularOffset

	if data.Step.WarmStarting {
		// Scale impulses to support a variable time step.
		joint.M_linearImpulse.OperatorScalarMulInplace(data.Step.DtRatio)
		joint.M_angularImpulse *= data.Step.DtRatio

		P := MakeVec2(joint.M_linearImpulse.X, joint.M_linearImpulse.Y)
		vA.OperatorMinusInplace(Vec2MulScalar(mA, P))
		wA -= iA * (Vec2Cross(joint.M_rA, P) + joint.M_angularImpulse)
		vB.OperatorPlusInplace(Vec2MulScalar(mB, P))
		wB += iB * (Vec2Cross(joint.M_rB, P) + joint.M_angularImpulse)
	} else {
		joint.M_linearImpulse.SetZero()
		joint.M_angularImpulse = 0.0
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *MotorJoint) SolveVelocityConstraints(data SolverData) {
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	h := data.Step.Dt
	inv_h := data.Step.Inv_dt

	// Solve angular friction
	{
		Cdot := wB - wA + inv_h*joint.M_correctionFactor*joint.M_angularError
		impulse := -joint.M_angularMass * Cdot

		oldImpulse := joint.M_angularImpulse
		maxImpulse := h * joint.M_maxTorque
		joint.M_angularImpulse = FloatClamp(joint.M_angularImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = joint.M_angularImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	// Solve linear friction
	{
		Cdot := Vec2Add(Vec2Sub(Vec2Sub(Vec2Add(vB, Vec2CrossScalarVector(wB, joint.M_rB)), vA), Vec2CrossScalarVector(wA, joint.M_rA)), Vec2MulScalar(inv_h*joint.M_correctionFactor, joint.M_linearError))

		impulse := Vec2Mat22Mul(joint.M_linearMass, Cdot).OperatorNegate()
		oldImpulse := joint.M_linearImpulse
		joint.M_linearImpulse.OperatorPlusInplace(impulse)

		maxImpulse := h * joint.M_maxForce

		if joint.M_linearImpulse.LengthSquared() > maxImpulse*maxImpulse {
			joint.M_linearImpulse.Normalize()
			joint.M_linearImpulse.OperatorScalarMulInplace(maxImpulse)
		}

		impulse = Vec2Sub(joint.M_linearImpulse, oldImpulse)

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

func (joint *MotorJoint) SolvePositionConstraints(data SolverData) bool {
	return true
}

func (joint MotorJoint) GetAnchorA() Vec2 {
	return joint.M_bodyA.GetPosition()
}

func (joint MotorJoint) GetAnchorB() Vec2 {
	return joint.M_bodyB.GetPosition()
}

func (joint MotorJoint) GetReactionForce(inv_dt float64) Vec2 {
	return Vec2MulScalar(inv_dt, joint.M_linearImpulse)
}

func (joint MotorJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * joint.M_angularImpulse
}

func (joint *MotorJoint) SetMaxForce(force float64) {
	assert(IsValid(force) && force >= 0.0)
	joint.M_maxForce = force
}

func (joint MotorJoint) GetMaxForce() float64 {
	return joint.M_maxForce
}

func (joint *MotorJoint) SetMaxTorque(torque float64) {
	assert(IsValid(torque) && torque >= 0.0)
	joint.M_maxTorque = torque
}

func (joint MotorJoint) GetMaxTorque() float64 {
	return joint.M_maxTorque
}

func (joint *MotorJoint) SetCorrectionFactor(factor float64) {
	assert(IsValid(factor) && 0.0 <= factor && factor <= 1.0)
	joint.M_correctionFactor = factor
}

func (joint MotorJoint) GetCorrectionFactor() float64 {
	return joint.M_correctionFactor
}

func (joint *MotorJoint) SetLinearOffset(linearOffset Vec2) {
	if linearOffset.X != joint.M_linearOffset.X || linearOffset.Y != joint.M_linearOffset.Y {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_linearOffset = linearOffset
	}
}

func (joint MotorJoint) GetLinearOffset() Vec2 {
	return joint.M_linearOffset
}

func (joint *MotorJoint) SetAngularOffset(angularOffset float64) {
	if angularOffset != joint.M_angularOffset {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_angularOffset = angularOffset
	}
}

func (joint MotorJoint) GetAngularOffset() float64 {
	return joint.M_angularOffset
}

func (joint *MotorJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	fmt.Printf("  b2MotorJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.linearOffset.Set(%.15f, %.15f);\n", joint.M_linearOffset.X, joint.M_linearOffset.Y)
	fmt.Printf("  jd.angularOffset = %.15f;\n", joint.M_angularOffset)
	fmt.Printf("  jd.maxForce = %.15f;\n", joint.M_maxForce)
	fmt.Printf("  jd.maxTorque = %.15f;\n", joint.M_maxTorque)
	fmt.Printf("  jd.correctionFactor = %.15f;\n", joint.M_correctionFactor)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}
