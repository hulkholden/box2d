package box2d

import (
	"fmt"
	"math"
)

// Rope joint definition. This requires two body anchor points and
// a maximum lengths.
// Note: by default the connected objects will not collide.
// see collideConnected in b2JointDef.
type RopeJointDef struct {
	JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The maximum length of the rope.
	/// Warning: this must be larger than b2_linearSlop or
	/// the joint will have no effect.
	MaxLength float64
}

func MakeRopeJointDef() RopeJointDef {
	res := RopeJointDef{
		JointDef: MakeJointDef(),
	}
	res.Type = JointType.Rope
	res.LocalAnchorA.Set(-1.0, 0.0)
	res.LocalAnchorB.Set(1.0, 0.0)
	res.MaxLength = 0.0
	return res
}

// A rope joint enforces a maximum distance between two points
// on two bodies. It has no other effect.
// Warning: if you attempt to change the maximum length during
// the simulation you will get some non-physical behavior.
// A model that would allow you to dynamically modify the length
// would have some sponginess, so I chose not to implement it
// that way. See DistanceJoint if you want to dynamically
// control length.
type RopeJoint struct {
	*Joint

	// Solver shared
	M_localAnchorA Vec2
	M_localAnchorB Vec2
	M_maxLength    float64
	M_length       float64
	M_impulse      float64

	// Solver temp
	M_indexA       int
	M_indexB       int
	M_u            Vec2
	M_rA           Vec2
	M_rB           Vec2
	M_localCenterA Vec2
	M_localCenterB Vec2
	M_invMassA     float64
	M_invMassB     float64
	M_invIA        float64
	M_invIB        float64
	M_mass         float64
	M_state        uint8
}

// The local anchor point relative to bodyA's origin.
func (joint RopeJoint) GetLocalAnchorA() Vec2 {
	return joint.M_localAnchorA
}

// The local anchor point relative to bodyB's origin.
func (joint RopeJoint) GetLocalAnchorB() Vec2 {
	return joint.M_localAnchorB
}

// Set/Get the maximum length of the rope.
func (joint *RopeJoint) SetMaxLength(length float64) {
	joint.M_maxLength = length
}

/// Limit:
/// C = norm(pB - pA) - L
/// u = (pB - pA) / norm(pB - pA)
/// Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
/// J = [-u -cross(rA, u) u cross(rB, u)]
/// K = J * invM * JT
///   = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2

func MakeRopeJoint(def *RopeJointDef) *RopeJoint {
	res := RopeJoint{
		Joint: MakeJoint(def),
	}

	res.M_localAnchorA = def.LocalAnchorA
	res.M_localAnchorB = def.LocalAnchorB

	res.M_maxLength = def.MaxLength

	res.M_mass = 0.0
	res.M_impulse = 0.0
	res.M_state = LimitState.Inactive
	res.M_length = 0.0

	return &res
}

func (joint *RopeJoint) InitVelocityConstraints(data SolverData) {
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
	joint.M_u = Vec2Sub(Vec2Sub(Vec2Add(cB, joint.M_rB), cA), joint.M_rA)

	joint.M_length = joint.M_u.Length()

	C := joint.M_length - joint.M_maxLength
	if C > 0.0 {
		joint.M_state = LimitState.AtUpperLimit
	} else {
		joint.M_state = LimitState.Inactive
	}

	if joint.M_length > linearSlop {
		joint.M_u.OperatorScalarMulInplace(1.0 / joint.M_length)
	} else {
		joint.M_u.SetZero()
		joint.M_mass = 0.0
		joint.M_impulse = 0.0
		return
	}

	// Compute effective mass.
	crA := Vec2Cross(joint.M_rA, joint.M_u)
	crB := Vec2Cross(joint.M_rB, joint.M_u)
	invMass := joint.M_invMassA + joint.M_invIA*crA*crA + joint.M_invMassB + joint.M_invIB*crB*crB

	if invMass != 0.0 {
		joint.M_mass = 1.0 / invMass
	} else {
		joint.M_mass = 0.0
	}

	if data.Step.WarmStarting {
		// Scale the impulse to support a variable time step.
		joint.M_impulse *= data.Step.DtRatio

		P := Vec2MulScalar(joint.M_impulse, joint.M_u)
		vA.OperatorMinusInplace(Vec2MulScalar(joint.M_invMassA, P))
		wA -= joint.M_invIA * Vec2Cross(joint.M_rA, P)
		vB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, P))
		wB += joint.M_invIB * Vec2Cross(joint.M_rB, P)
	} else {
		joint.M_impulse = 0.0
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *RopeJoint) SolveVelocityConstraints(data SolverData) {
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	// Cdot = dot(u, v + cross(w, r))
	vpA := Vec2Add(vA, Vec2CrossScalarVector(wA, joint.M_rA))
	vpB := Vec2Add(vB, Vec2CrossScalarVector(wB, joint.M_rB))
	C := joint.M_length - joint.M_maxLength
	Cdot := Vec2Dot(joint.M_u, Vec2Sub(vpB, vpA))

	// Predictive constraint.
	if C < 0.0 {
		Cdot += data.Step.Inv_dt * C
	}

	impulse := -joint.M_mass * Cdot
	oldImpulse := joint.M_impulse
	joint.M_impulse = math.Min(0.0, joint.M_impulse+impulse)
	impulse = joint.M_impulse - oldImpulse

	P := Vec2MulScalar(impulse, joint.M_u)
	vA.OperatorMinusInplace(Vec2MulScalar(joint.M_invMassA, P))
	wA -= joint.M_invIA * Vec2Cross(joint.M_rA, P)
	vB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, P))
	wB += joint.M_invIB * Vec2Cross(joint.M_rB, P)

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *RopeJoint) SolvePositionConstraints(data SolverData) bool {
	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A

	qA := MakeRotFromAngle(aA)
	qB := MakeRotFromAngle(aB)

	rA := RotVec2Mul(qA, Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	rB := RotVec2Mul(qB, Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))
	u := Vec2Sub(Vec2Sub(Vec2Add(cB, rB), cA), rA)

	length := u.Normalize()
	C := length - joint.M_maxLength

	C = FloatClamp(C, 0.0, maxLinearCorrection)

	impulse := -joint.M_mass * C
	P := Vec2MulScalar(impulse, u)

	cA.OperatorMinusInplace(Vec2MulScalar(joint.M_invMassA, P))
	aA -= joint.M_invIA * Vec2Cross(rA, P)
	cB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, P))
	aB += joint.M_invIB * Vec2Cross(rB, P)

	data.Positions[joint.M_indexA].C = cA
	data.Positions[joint.M_indexA].A = aA
	data.Positions[joint.M_indexB].C = cB
	data.Positions[joint.M_indexB].A = aB

	return length-joint.M_maxLength < linearSlop
}

func (joint RopeJoint) GetAnchorA() Vec2 {
	return joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
}

func (joint RopeJoint) GetAnchorB() Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint RopeJoint) GetReactionForce(inv_dt float64) Vec2 {
	F := Vec2MulScalar((inv_dt * joint.M_impulse), joint.M_u)
	return F
}

func (joint RopeJoint) GetReactionTorque(inv_dt float64) float64 {
	return 0.0
}

func (joint RopeJoint) GetMaxLength() float64 {
	return joint.M_maxLength
}

func (joint RopeJoint) GetLimitState() uint8 {
	return joint.M_state
}

func (joint *RopeJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	fmt.Printf("  b2RopeJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.localAnchorA.Set(%.15f, %.15f);\n", joint.M_localAnchorA.X, joint.M_localAnchorA.Y)
	fmt.Printf("  jd.localAnchorB.Set(%.15f, %.15f);\n", joint.M_localAnchorB.X, joint.M_localAnchorB.Y)
	fmt.Printf("  jd.maxLength = %.15f;\n", joint.M_maxLength)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}
