package box2d

import (
	"fmt"
	"math"
)

// Weld joint definition. You need to specify local anchor points
// where they are attached and the relative body angle. The position
// of the anchor points is important for computing the reaction torque.
type WeldJointDef struct {
	JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	ReferenceAngle float64

	/// The mass-spring-damper frequency in Hertz. Rotation only.
	/// Disable softness with a value of 0.
	FrequencyHz float64

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	DampingRatio float64
}

func MakeWeldJointDef() WeldJointDef {
	res := WeldJointDef{
		JointDef: MakeJointDef(),
	}

	res.Type = JointType.Weld
	res.LocalAnchorA.Set(0.0, 0.0)
	res.LocalAnchorB.Set(0.0, 0.0)
	res.ReferenceAngle = 0.0
	res.FrequencyHz = 0.0
	res.DampingRatio = 0.0

	return res
}

// A weld joint essentially glues two bodies together. A weld joint may
// distort somewhat because the island constraint solver is approximate.
type WeldJoint struct {
	*Joint

	M_frequencyHz  float64
	M_dampingRatio float64
	M_bias         float64

	// Solver shared
	M_localAnchorA   Vec2
	M_localAnchorB   Vec2
	M_referenceAngle float64
	M_gamma          float64
	M_impulse        Vec3

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
	M_mass         Mat33
}

// The local anchor point relative to bodyA's origin.
func (joint WeldJoint) GetLocalAnchorA() Vec2 {
	return joint.M_localAnchorA
}

// The local anchor point relative to bodyB's origin.
func (joint WeldJoint) GetLocalAnchorB() Vec2 {
	return joint.M_localAnchorB
}

// Get the reference angle.
func (joint WeldJoint) GetReferenceAngle() float64 {
	return joint.M_referenceAngle
}

// Set/get frequency in Hz.
func (joint *WeldJoint) SetFrequency(hz float64) {
	joint.M_frequencyHz = hz
}

func (joint WeldJoint) GetFrequency() float64 {
	return joint.M_frequencyHz
}

// Set/get damping ratio.
func (joint *WeldJoint) SetDampingRatio(ratio float64) {
	joint.M_dampingRatio = ratio
}

func (joint WeldJoint) GetDampingRatio() float64 {
	return joint.M_dampingRatio
}

/// Point-to-point constraint
/// C = p2 - p1
/// Cdot = v2 - v1
///      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
/// J = [-I -r1_skew I r2_skew ]
/// Identity used:
/// w k % (rx i + ry j) = w * (-ry i + rx j)

/// Angle constraint
/// C = angle2 - angle1 - referenceAngle
/// Cdot = w2 - w1
/// J = [0 0 -1 0 0 1]
/// K = invI1 + invI2

func (def *WeldJointDef) Initialize(bA *Body, bB *Body, anchor Vec2) {
	def.BodyA = bA
	def.BodyB = bB
	def.LocalAnchorA = def.BodyA.GetLocalPoint(anchor)
	def.LocalAnchorB = def.BodyB.GetLocalPoint(anchor)
	def.ReferenceAngle = def.BodyB.GetAngle() - def.BodyA.GetAngle()
}

func MakeWeldJoint(def *WeldJointDef) *WeldJoint {
	res := WeldJoint{
		Joint: MakeJoint(def),
	}

	res.M_localAnchorA = def.LocalAnchorA
	res.M_localAnchorB = def.LocalAnchorB
	res.M_referenceAngle = def.ReferenceAngle
	res.M_frequencyHz = def.FrequencyHz
	res.M_dampingRatio = def.DampingRatio

	res.M_impulse.SetZero()

	return &res
}

func (joint *WeldJoint) InitVelocityConstraints(data SolverData) {
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

	var K Mat33
	K.Ex.X = mA + mB + joint.M_rA.Y*joint.M_rA.Y*iA + joint.M_rB.Y*joint.M_rB.Y*iB
	K.Ey.X = -joint.M_rA.Y*joint.M_rA.X*iA - joint.M_rB.Y*joint.M_rB.X*iB
	K.Ez.X = -joint.M_rA.Y*iA - joint.M_rB.Y*iB
	K.Ex.Y = K.Ey.X
	K.Ey.Y = mA + mB + joint.M_rA.X*joint.M_rA.X*iA + joint.M_rB.X*joint.M_rB.X*iB
	K.Ez.Y = joint.M_rA.X*iA + joint.M_rB.X*iB
	K.Ex.Z = K.Ez.X
	K.Ey.Z = K.Ez.Y
	K.Ez.Z = iA + iB

	if joint.M_frequencyHz > 0.0 {
		K.GetInverse22(&joint.M_mass)

		invM := iA + iB
		m := 0.0
		if invM > 0.0 {
			m = 1.0 / invM
		}

		C := aB - aA - joint.M_referenceAngle

		// Frequency
		omega := 2.0 * Pi * joint.M_frequencyHz

		// Damping coefficient
		d := 2.0 * m * joint.M_dampingRatio * omega

		// Spring stiffness
		k := m * omega * omega

		// magic formulas
		h := data.Step.Dt
		joint.M_gamma = h * (d + h*k)
		if joint.M_gamma != 0.0 {
			joint.M_gamma = 1.0 / joint.M_gamma
		} else {
			joint.M_gamma = 0.0
		}
		joint.M_bias = C * h * k * joint.M_gamma

		invM += joint.M_gamma
		if invM != 0.0 {
			joint.M_mass.Ez.Z = 1.0 / invM
		} else {
			joint.M_mass.Ez.Z = 0.0
		}
	} else if K.Ez.Z == 0.0 {
		K.GetInverse22(&joint.M_mass)
		joint.M_gamma = 0.0
		joint.M_bias = 0.0
	} else {
		K.GetSymInverse33(&joint.M_mass)
		joint.M_gamma = 0.0
		joint.M_bias = 0.0
	}

	if data.Step.WarmStarting {
		// Scale impulses to support a variable time step.
		joint.M_impulse.OperatorScalarMultInplace(data.Step.DtRatio)

		P := MakeVec2(joint.M_impulse.X, joint.M_impulse.Y)

		vA.OperatorMinusInplace(Vec2MulScalar(mA, P))
		wA -= iA * (Vec2Cross(joint.M_rA, P) + joint.M_impulse.Z)

		vB.OperatorPlusInplace(Vec2MulScalar(mB, P))
		wB += iB * (Vec2Cross(joint.M_rB, P) + joint.M_impulse.Z)
	} else {
		joint.M_impulse.SetZero()
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *WeldJoint) SolveVelocityConstraints(data SolverData) {
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	if joint.M_frequencyHz > 0.0 {
		Cdot2 := wB - wA

		impulse2 := -joint.M_mass.Ez.Z * (Cdot2 + joint.M_bias + joint.M_gamma*joint.M_impulse.Z)
		joint.M_impulse.Z += impulse2

		wA -= iA * impulse2
		wB += iB * impulse2

		Cdot1 := Vec2Sub(Vec2Sub(Vec2Add(vB, Vec2CrossScalarVector(wB, joint.M_rB)), vA), Vec2CrossScalarVector(wA, joint.M_rA))

		impulse1 := Vec2Mul22(joint.M_mass, Cdot1).OperatorNegate()
		joint.M_impulse.X += impulse1.X
		joint.M_impulse.Y += impulse1.Y

		P := impulse1

		vA.OperatorMinusInplace(Vec2MulScalar(mA, P))
		wA -= iA * Vec2Cross(joint.M_rA, P)

		vB.OperatorPlusInplace(Vec2MulScalar(mB, P))
		wB += iB * Vec2Cross(joint.M_rB, P)
	} else {
		Cdot1 := Vec2Sub(Vec2Sub(Vec2Add(vB, Vec2CrossScalarVector(wB, joint.M_rB)), vA), Vec2CrossScalarVector(wA, joint.M_rA))
		Cdot2 := wB - wA
		Cdot := MakeVec3(Cdot1.X, Cdot1.Y, Cdot2)

		impulse := Vec3Mat33Mul(joint.M_mass, Cdot).OperatorNegate()
		joint.M_impulse.OperatorPlusInplace(impulse)

		P := MakeVec2(impulse.X, impulse.Y)

		vA.OperatorMinusInplace(Vec2MulScalar(mA, P))
		wA -= iA * (Vec2Cross(joint.M_rA, P) + impulse.Z)

		vB.OperatorPlusInplace(Vec2MulScalar(mB, P))
		wB += iB * (Vec2Cross(joint.M_rB, P) + impulse.Z)
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *WeldJoint) SolvePositionConstraints(data SolverData) bool {
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

	rA := RotVec2Mul(qA, Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	rB := RotVec2Mul(qB, Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))

	positionError := 0.0
	angularError := 0.0

	var K Mat33
	K.Ex.X = mA + mB + rA.Y*rA.Y*iA + rB.Y*rB.Y*iB
	K.Ey.X = -rA.Y*rA.X*iA - rB.Y*rB.X*iB
	K.Ez.X = -rA.Y*iA - rB.Y*iB
	K.Ex.Y = K.Ey.X
	K.Ey.Y = mA + mB + rA.X*rA.X*iA + rB.X*rB.X*iB
	K.Ez.Y = rA.X*iA + rB.X*iB
	K.Ex.Z = K.Ez.X
	K.Ey.Z = K.Ez.Y
	K.Ez.Z = iA + iB

	if joint.M_frequencyHz > 0.0 {
		C1 := Vec2Sub(Vec2Sub(Vec2Add(cB, rB), cA), rA)

		positionError = C1.Length()
		angularError = 0.0

		P := K.Solve22(C1).OperatorNegate()

		cA.OperatorMinusInplace(Vec2MulScalar(mA, P))
		aA -= iA * Vec2Cross(rA, P)

		cB.OperatorPlusInplace(Vec2MulScalar(mB, P))
		aB += iB * Vec2Cross(rB, P)
	} else {
		C1 := Vec2Sub(Vec2Sub(Vec2Add(cB, rB), cA), rA)
		C2 := aB - aA - joint.M_referenceAngle

		positionError = C1.Length()
		angularError = math.Abs(C2)

		C := MakeVec3(C1.X, C1.Y, C2)

		var impulse Vec3
		if K.Ez.Z > 0.0 {
			impulse = K.Solve33(C).OperatorNegate()
		} else {
			impulse2 := K.Solve22(C1).OperatorNegate()
			impulse.Set(impulse2.X, impulse2.Y, 0.0)
		}

		P := MakeVec2(impulse.X, impulse.Y)

		cA.OperatorMinusInplace(Vec2MulScalar(mA, P))
		aA -= iA * (Vec2Cross(rA, P) + impulse.Z)

		cB.OperatorPlusInplace(Vec2MulScalar(mB, P))
		aB += iB * (Vec2Cross(rB, P) + impulse.Z)
	}

	data.Positions[joint.M_indexA].C = cA
	data.Positions[joint.M_indexA].A = aA
	data.Positions[joint.M_indexB].C = cB
	data.Positions[joint.M_indexB].A = aB

	return positionError <= linearSlop && angularError <= angularSlop
}

func (joint WeldJoint) GetAnchorA() Vec2 {
	return joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
}

func (joint WeldJoint) GetAnchorB() Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint WeldJoint) GetReactionForce(inv_dt float64) Vec2 {
	P := MakeVec2(joint.M_impulse.X, joint.M_impulse.Y)
	return Vec2MulScalar(inv_dt, P)
}

func (joint WeldJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * joint.M_impulse.Z
}

func (joint *WeldJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	fmt.Printf("  b2WeldJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.localAnchorA.Set(%.15f, %.15f);\n", joint.M_localAnchorA.X, joint.M_localAnchorA.Y)
	fmt.Printf("  jd.localAnchorB.Set(%.15f, %.15f);\n", joint.M_localAnchorB.X, joint.M_localAnchorB.Y)
	fmt.Printf("  jd.referenceAngle = %.15f;\n", joint.M_referenceAngle)
	fmt.Printf("  jd.frequencyHz = %.15f;\n", joint.M_frequencyHz)
	fmt.Printf("  jd.dampingRatio = %.15f;\n", joint.M_dampingRatio)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}
