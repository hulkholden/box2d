package box2d

import (
	"fmt"
	"math"
)

// Distance joint definition. This requires defining an
// anchor point on both bodies and the non-zero length of the
// distance joint. The definition uses local anchor points
// so that the initial configuration can violate the constraint
// slightly. This helps when saving and loading a game.
// @warning Do not use a zero or short length.
type DistanceJointDef struct {
	JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The natural length between the anchor points.
	Length float64

	/// The mass-spring-damper frequency in Hertz. A value of 0
	/// disables softness.
	FrequencyHz float64

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	DampingRatio float64
}

func MakeDistanceJointDef() DistanceJointDef {
	res := DistanceJointDef{
		JointDef: MakeJointDef(),
	}

	res.Type = JointType.Distance
	res.LocalAnchorA.Set(0.0, 0.0)
	res.LocalAnchorB.Set(0.0, 0.0)
	res.Length = 1.0
	res.FrequencyHz = 0.0
	res.DampingRatio = 0.0

	return res
}

// A distance joint constrains two points on two bodies
// to remain at a fixed distance from each other. You can view
// this as a massless, rigid rod.
type DistanceJoint struct {
	*Joint

	M_frequencyHz  float64
	M_dampingRatio float64
	M_bias         float64

	// Solver shared
	M_localAnchorA Vec2
	M_localAnchorB Vec2
	M_gamma        float64
	M_impulse      float64
	M_length       float64

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
}

// The local anchor point relative to bodyA's origin.
func (joint DistanceJoint) GetLocalAnchorA() Vec2 {
	return joint.M_localAnchorA
}

// The local anchor point relative to bodyB's origin.
func (joint DistanceJoint) GetLocalAnchorB() Vec2 {
	return joint.M_localAnchorB
}

func (joint *DistanceJoint) SetLength(length float64) {
	joint.M_length = length
}

func (joint DistanceJoint) GetLength() float64 {
	return joint.M_length
}

func (joint *DistanceJoint) SetFrequency(hz float64) {
	joint.M_frequencyHz = hz
}

func (joint DistanceJoint) GetFrequency() float64 {
	return joint.M_frequencyHz
}

func (joint *DistanceJoint) SetDampingRatio(ratio float64) {
	joint.M_dampingRatio = ratio
}

func (joint DistanceJoint) GetDampingRatio() float64 {
	return joint.M_dampingRatio
}

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

func (joint *DistanceJointDef) Initialize(b1 *Body, b2 *Body, anchor1 Vec2, anchor2 Vec2) {
	joint.BodyA = b1
	joint.BodyB = b2
	joint.LocalAnchorA = joint.BodyA.GetLocalPoint(anchor1)
	joint.LocalAnchorB = joint.BodyB.GetLocalPoint(anchor2)
	d := Vec2Sub(anchor2, anchor1)
	joint.Length = d.Length()
}

func MakeDistanceJoint(def *DistanceJointDef) *DistanceJoint {
	res := DistanceJoint{
		Joint: MakeJoint(def),
	}

	res.M_localAnchorA = def.LocalAnchorA
	res.M_localAnchorB = def.LocalAnchorB
	res.M_length = def.Length
	res.M_frequencyHz = def.FrequencyHz
	res.M_dampingRatio = def.DampingRatio
	res.M_impulse = 0.0
	res.M_gamma = 0.0
	res.M_bias = 0.0

	return &res
}

func (joint *DistanceJoint) InitVelocityConstraints(data SolverData) {
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

	// Handle singularity.
	length := joint.M_u.Length()
	if length > linearSlop {
		joint.M_u.OperatorScalarMulInplace(1.0 / length)
	} else {
		joint.M_u.Set(0.0, 0.0)
	}

	crAu := Vec2Cross(joint.M_rA, joint.M_u)
	crBu := Vec2Cross(joint.M_rB, joint.M_u)
	invMass := joint.M_invMassA + joint.M_invIA*crAu*crAu + joint.M_invMassB + joint.M_invIB*crBu*crBu

	// Compute the effective mass matrix.
	if invMass != 0.0 {
		joint.M_mass = 1.0 / invMass
	} else {
		joint.M_mass = 0
	}

	if joint.M_frequencyHz > 0.0 {
		C := length - joint.M_length

		// Frequency
		omega := 2.0 * Pi * joint.M_frequencyHz

		// Damping coefficient
		d := 2.0 * joint.M_mass * joint.M_dampingRatio * omega

		// Spring stiffness
		k := joint.M_mass * omega * omega

		// magic formulas
		h := data.Step.Dt
		joint.M_gamma = h * (d + h*k)
		if joint.M_gamma != 0.0 {
			joint.M_gamma = 1.0 / joint.M_gamma
		} else {
			joint.M_gamma = 0.0
		}
		joint.M_bias = C * h * k * joint.M_gamma

		invMass += joint.M_gamma
		if invMass != 0.0 {
			joint.M_mass = 1.0 / invMass
		} else {
			joint.M_mass = 0.0
		}
	} else {
		joint.M_gamma = 0.0
		joint.M_bias = 0.0
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

	// Note: mutation on value, not ref; but OK because Velocities is an array
	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *DistanceJoint) SolveVelocityConstraints(data SolverData) {
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	// Cdot = dot(u, v + cross(w, r))
	vpA := Vec2Add(vA, Vec2CrossScalarVector(wA, joint.M_rA))
	vpB := Vec2Add(vB, Vec2CrossScalarVector(wB, joint.M_rB))
	Cdot := Vec2Dot(joint.M_u, Vec2Sub(vpB, vpA))

	impulse := -joint.M_mass * (Cdot + joint.M_bias + joint.M_gamma*joint.M_impulse)
	joint.M_impulse += impulse

	P := Vec2MulScalar(impulse, joint.M_u)
	vA.OperatorMinusInplace(Vec2MulScalar(joint.M_invMassA, P))
	wA -= joint.M_invIA * Vec2Cross(joint.M_rA, P)
	vB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, P))
	wB += joint.M_invIB * Vec2Cross(joint.M_rB, P)

	// Note: mutation on value, not ref; but OK because Velocities is an array
	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *DistanceJoint) SolvePositionConstraints(data SolverData) bool {
	if joint.M_frequencyHz > 0.0 {
		// There is no position correction for soft distance constraints.
		return true
	}

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
	C := length - joint.M_length
	C = FloatClamp(C, -maxLinearCorrection, maxLinearCorrection)

	impulse := -joint.M_mass * C
	P := Vec2MulScalar(impulse, u)

	cA.OperatorMinusInplace(Vec2MulScalar(joint.M_invMassA, P))
	aA -= joint.M_invIA * Vec2Cross(rA, P)
	cB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, P))
	aB += joint.M_invIB * Vec2Cross(rB, P)

	// Note: mutation on value, not ref; but OK because Positions is an array
	data.Positions[joint.M_indexA].C = cA
	data.Positions[joint.M_indexA].A = aA
	data.Positions[joint.M_indexB].C = cB
	data.Positions[joint.M_indexB].A = aB

	return math.Abs(C) < linearSlop
}

func (joint DistanceJoint) GetAnchorA() Vec2 {
	return joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
}

func (joint DistanceJoint) GetAnchorB() Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint DistanceJoint) GetReactionForce(inv_dt float64) Vec2 {
	return Vec2MulScalar((inv_dt * joint.M_impulse), joint.M_u)
}

func (joint DistanceJoint) GetReactionTorque(inv_dt float64) float64 {
	return 0.0
}

func (joint DistanceJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	fmt.Printf("  DistanceJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.localAnchorA.Set(%.15f, %.15f);\n", joint.M_localAnchorA.X, joint.M_localAnchorA.Y)
	fmt.Printf("  jd.localAnchorB.Set(%.15f, %.15f);\n", joint.M_localAnchorB.X, joint.M_localAnchorB.Y)
	fmt.Printf("  jd.length = %.15f;\n", joint.M_length)
	fmt.Printf("  jd.frequencyHz = %.15f;\n", joint.M_frequencyHz)
	fmt.Printf("  jd.dampingRatio = %.15f;\n", joint.M_dampingRatio)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}
