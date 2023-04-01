package box2d

import (
	"fmt"
	"math"
)

const B2MinPulleyLength = 2.0

// Pulley joint definition. This requires two ground anchors,
// two dynamic body anchor points, and a pulley ratio.
type PulleyJointDef struct {
	JointDef

	/// The first ground anchor in world coordinates. This point never moves.
	GroundAnchorA Vec2

	/// The second ground anchor in world coordinates. This point never moves.
	GroundAnchorB Vec2

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The a reference length for the segment attached to bodyA.
	LengthA float64

	/// The a reference length for the segment attached to bodyB.
	LengthB float64

	/// The pulley ratio, used to simulate a block-and-tackle.
	Ratio float64
}

func MakeB2PulleyJointDef() PulleyJointDef {
	res := PulleyJointDef{
		JointDef: MakeJointDef(),
	}

	res.Type = JointType.Pulley
	res.GroundAnchorA.Set(-1.0, 1.0)
	res.GroundAnchorB.Set(1.0, 1.0)
	res.LocalAnchorA.Set(-1.0, 0.0)
	res.LocalAnchorB.Set(1.0, 0.0)
	res.LengthA = 0.0
	res.LengthB = 0.0
	res.Ratio = 1.0
	res.CollideConnected = true

	return res
}

// The pulley joint is connected to two bodies and two fixed ground points.
// The pulley supports a ratio such that:
// length1 + ratio * length2 <= constant
// Yes, the force transmitted is scaled by the ratio.
// Warning: the pulley joint can get a bit squirrelly by itself. They often
// work better when combined with prismatic joints. You should also cover the
// the anchor points with static shapes to prevent one side from going to
// zero length.
type PulleyJoint struct {
	*Joint

	M_groundAnchorA Vec2
	M_groundAnchorB Vec2
	M_lengthA       float64
	M_lengthB       float64

	// Solver shared
	M_localAnchorA Vec2
	M_localAnchorB Vec2
	M_constant     float64
	M_ratio        float64
	M_impulse      float64

	// Solver temp
	M_indexA       int
	M_indexB       int
	M_uA           Vec2
	M_uB           Vec2
	M_rA           Vec2
	M_rB           Vec2
	M_localCenterA Vec2
	M_localCenterB Vec2
	M_invMassA     float64
	M_invMassB     float64
	M_invIA        float64
	M_invIB        float64
	M_mass         float64
}

// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C0 = (length1 + ratio * length2)_initial
// C = C0 - (length1 + ratio * length2)
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

func (def *PulleyJointDef) Initialize(bA *Body, bB *Body, groundA Vec2, groundB Vec2, anchorA Vec2, anchorB Vec2, r float64) {
	def.BodyA = bA
	def.BodyB = bB
	def.GroundAnchorA = groundA
	def.GroundAnchorB = groundB
	def.LocalAnchorA = def.BodyA.GetLocalPoint(anchorA)
	def.LocalAnchorB = def.BodyB.GetLocalPoint(anchorB)
	dA := Vec2Sub(anchorA, groundA)
	def.LengthA = dA.Length()
	dB := Vec2Sub(anchorB, groundB)
	def.LengthB = dB.Length()
	def.Ratio = r
	assert(def.Ratio > epsilon)
}

func MakePulleyJoint(def *PulleyJointDef) *PulleyJoint {
	res := PulleyJoint{
		Joint: MakeJoint(def),
	}

	res.M_groundAnchorA = def.GroundAnchorA
	res.M_groundAnchorB = def.GroundAnchorB
	res.M_localAnchorA = def.LocalAnchorA
	res.M_localAnchorB = def.LocalAnchorB

	res.M_lengthA = def.LengthA
	res.M_lengthB = def.LengthB

	assert(def.Ratio != 0.0)
	res.M_ratio = def.Ratio

	res.M_constant = def.LengthA + res.M_ratio*def.LengthB

	res.M_impulse = 0.0

	return &res
}

func (joint *PulleyJoint) InitVelocityConstraints(data SolverData) {
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

	joint.M_rA = RotVec2Mul(qA, Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	joint.M_rB = RotVec2Mul(qB, Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))

	// Get the pulley axes.
	joint.M_uA = Vec2Sub(Vec2Add(cA, joint.M_rA), joint.M_groundAnchorA)
	joint.M_uB = Vec2Sub(Vec2Add(cB, joint.M_rB), joint.M_groundAnchorB)

	lengthA := joint.M_uA.Length()
	lengthB := joint.M_uB.Length()

	if lengthA > 10.0*linearSlop {
		joint.M_uA.OperatorScalarMulInplace(1.0 / lengthA)
	} else {
		joint.M_uA.SetZero()
	}

	if lengthB > 10.0*linearSlop {
		joint.M_uB.OperatorScalarMulInplace(1.0 / lengthB)
	} else {
		joint.M_uB.SetZero()
	}

	// Compute effective mass.
	ruA := Vec2Cross(joint.M_rA, joint.M_uA)
	ruB := Vec2Cross(joint.M_rB, joint.M_uB)

	mA := joint.M_invMassA + joint.M_invIA*ruA*ruA
	mB := joint.M_invMassB + joint.M_invIB*ruB*ruB

	joint.M_mass = mA + joint.M_ratio*joint.M_ratio*mB

	if joint.M_mass > 0.0 {
		joint.M_mass = 1.0 / joint.M_mass
	}

	if data.Step.WarmStarting {
		// Scale impulses to support variable time steps.
		joint.M_impulse *= data.Step.DtRatio

		// Warm starting.
		PA := Vec2MulScalar(-(joint.M_impulse), joint.M_uA)
		PB := Vec2MulScalar(-joint.M_ratio*joint.M_impulse, joint.M_uB)

		vA.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassA, PA))
		wA += joint.M_invIA * Vec2Cross(joint.M_rA, PA)
		vB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, PB))
		wB += joint.M_invIB * Vec2Cross(joint.M_rB, PB)
	} else {
		joint.M_impulse = 0.0
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *PulleyJoint) SolveVelocityConstraints(data SolverData) {
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	vpA := Vec2Add(vA, Vec2CrossScalarVector(wA, joint.M_rA))
	vpB := Vec2Add(vB, Vec2CrossScalarVector(wB, joint.M_rB))

	Cdot := -Vec2Dot(joint.M_uA, vpA) - joint.M_ratio*Vec2Dot(joint.M_uB, vpB)
	impulse := -joint.M_mass * Cdot
	joint.M_impulse += impulse

	PA := Vec2MulScalar(-impulse, joint.M_uA)
	PB := Vec2MulScalar(-joint.M_ratio*impulse, joint.M_uB)
	vA.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassA, PA))
	wA += joint.M_invIA * Vec2Cross(joint.M_rA, PA)
	vB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, PB))
	wB += joint.M_invIB * Vec2Cross(joint.M_rB, PB)

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *PulleyJoint) SolvePositionConstraints(data SolverData) bool {
	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A

	qA := MakeRotFromAngle(aA)
	qB := MakeRotFromAngle(aB)

	rA := RotVec2Mul(qA, Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	rB := RotVec2Mul(qB, Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))

	// Get the pulley axes.
	uA := Vec2Sub(Vec2Add(cA, rA), joint.M_groundAnchorA)
	uB := Vec2Sub(Vec2Add(cB, rB), joint.M_groundAnchorB)

	lengthA := uA.Length()
	lengthB := uB.Length()

	if lengthA > 10.0*linearSlop {
		uA.OperatorScalarMulInplace(1.0 / lengthA)
	} else {
		uA.SetZero()
	}

	if lengthB > 10.0*linearSlop {
		uB.OperatorScalarMulInplace(1.0 / lengthB)
	} else {
		uB.SetZero()
	}

	// Compute effective mass.
	ruA := Vec2Cross(rA, uA)
	ruB := Vec2Cross(rB, uB)

	mA := joint.M_invMassA + joint.M_invIA*ruA*ruA
	mB := joint.M_invMassB + joint.M_invIB*ruB*ruB

	mass := mA + joint.M_ratio*joint.M_ratio*mB

	if mass > 0.0 {
		mass = 1.0 / mass
	}

	C := joint.M_constant - lengthA - joint.M_ratio*lengthB
	linearError := math.Abs(C)

	impulse := -mass * C

	PA := Vec2MulScalar(-impulse, uA)
	PB := Vec2MulScalar(-joint.M_ratio*impulse, uB)

	cA.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassA, PA))
	aA += joint.M_invIA * Vec2Cross(rA, PA)
	cB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, PB))
	aB += joint.M_invIB * Vec2Cross(rB, PB)

	data.Positions[joint.M_indexA].C = cA
	data.Positions[joint.M_indexA].A = aA
	data.Positions[joint.M_indexB].C = cB
	data.Positions[joint.M_indexB].A = aB

	return linearError < linearSlop
}

func (joint PulleyJoint) GetAnchorA() Vec2 {
	return joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
}

func (joint PulleyJoint) GetAnchorB() Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint PulleyJoint) GetReactionForce(inv_dt float64) Vec2 {
	P := Vec2MulScalar(joint.M_impulse, joint.M_uB)
	return Vec2MulScalar(inv_dt, P)
}

func (joint PulleyJoint) GetReactionTorque(inv_dt float64) float64 {
	return 0.0
}

func (joint PulleyJoint) GetGroundAnchorA() Vec2 {
	return joint.M_groundAnchorA
}

func (joint PulleyJoint) GetGroundAnchorB() Vec2 {
	return joint.M_groundAnchorB
}

func (joint PulleyJoint) GetLengthA() float64 {
	return joint.M_lengthA
}

func (joint PulleyJoint) GetLengthB() float64 {
	return joint.M_lengthB
}

func (joint PulleyJoint) GetRatio() float64 {
	return joint.M_ratio
}

func (joint PulleyJoint) GetCurrentLengthA() float64 {
	p := joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
	s := joint.M_groundAnchorA
	d := Vec2Sub(p, s)
	return d.Length()
}

func (joint PulleyJoint) GetCurrentLengthB() float64 {
	p := joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
	s := joint.M_groundAnchorB
	d := Vec2Sub(p, s)
	return d.Length()
}

func (joint *PulleyJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	fmt.Printf("  b2PulleyJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.groundAnchorA.Set(%.15f, %.15f);\n", joint.M_groundAnchorA.X, joint.M_groundAnchorA.Y)
	fmt.Printf("  jd.groundAnchorB.Set(%.15f, %.15f);\n", joint.M_groundAnchorB.X, joint.M_groundAnchorB.Y)
	fmt.Printf("  jd.localAnchorA.Set(%.15f, %.15f);\n", joint.M_localAnchorA.X, joint.M_localAnchorA.Y)
	fmt.Printf("  jd.localAnchorB.Set(%.15f, %.15f);\n", joint.M_localAnchorB.X, joint.M_localAnchorB.Y)
	fmt.Printf("  jd.lengthA = %.15f;\n", joint.M_lengthA)
	fmt.Printf("  jd.lengthB = %.15f;\n", joint.M_lengthB)
	fmt.Printf("  jd.ratio = %.15f;\n", joint.M_ratio)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}

func (joint *PulleyJoint) ShiftOrigin(newOrigin Vec2) {
	joint.M_groundAnchorA.OperatorMinusInplace(newOrigin)
	joint.M_groundAnchorB.OperatorMinusInplace(newOrigin)
}
