package box2d

import (
	"fmt"
	"math"
)

// Wheel joint definition. This requires defining a line of
// motion using an axis and an anchor point. The definition uses local
// anchor points and a local axis so that the initial configuration
// can violate the constraint slightly. The joint translation is zero
// when the local anchor points coincide in world space. Using local
// anchors and a local axis helps when saving and loading a game.
type WheelJointDef struct {
	JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The local translation axis in bodyA.
	LocalAxisA Vec2

	/// Enable/disable the joint motor.
	EnableMotor bool

	/// The maximum motor torque, usually in N-m.
	MaxMotorTorque float64

	/// The desired motor speed in radians per second.
	MotorSpeed float64

	/// Suspension frequency, zero indicates no suspension
	FrequencyHz float64

	/// Suspension damping ratio, one indicates critical damping
	DampingRatio float64
}

func MakeWheelJointDef() WheelJointDef {
	res := WheelJointDef{
		JointDef: MakeJointDef(),
	}

	res.Type = JointType.Wheel
	res.LocalAnchorA.SetZero()
	res.LocalAnchorB.SetZero()
	res.LocalAxisA.Set(1.0, 0.0)
	res.EnableMotor = false
	res.MaxMotorTorque = 0.0
	res.MotorSpeed = 0.0
	res.FrequencyHz = 2.0
	res.DampingRatio = 0.7

	return res
}

// A wheel joint. This joint provides two degrees of freedom: translation
// along an axis fixed in bodyA and rotation in the plane. In other words, it is a point to
// line constraint with a rotational motor and a linear spring/damper.
// This joint is designed for vehicle suspensions.
type WheelJoint struct {
	*Joint

	M_frequencyHz  float64
	M_dampingRatio float64

	// Solver shared
	M_localAnchorA Vec2
	M_localAnchorB Vec2
	M_localXAxisA  Vec2
	M_localYAxisA  Vec2

	M_impulse       float64
	M_motorImpulse  float64
	M_springImpulse float64

	M_maxMotorTorque float64
	M_motorSpeed     float64
	M_enableMotor    bool

	// Solver temp
	M_indexA       int
	M_indexB       int
	M_localCenterA Vec2
	M_localCenterB Vec2
	M_invMassA     float64
	M_invMassB     float64
	M_invIA        float64
	M_invIB        float64

	M_ax  Vec2
	M_ay  Vec2
	M_sAx float64
	M_sBx float64
	M_sAy float64
	M_sBy float64

	M_mass       float64
	M_motorMass  float64
	M_springMass float64

	M_bias  float64
	M_gamma float64
}

// The local anchor point relative to bodyA's origin.
func (joint WheelJoint) GetLocalAnchorA() Vec2 {
	return joint.M_localAnchorA
}

// The local anchor point relative to bodyB's origin.
func (joint WheelJoint) GetLocalAnchorB() Vec2 {
	return joint.M_localAnchorB
}

// The local joint axis relative to bodyA.
func (joint WheelJoint) GetLocalAxisA() Vec2 {
	return joint.M_localXAxisA
}

func (joint WheelJoint) GetMotorSpeed() float64 {
	return joint.M_motorSpeed
}

func (joint WheelJoint) GetMaxMotorTorque() float64 {
	return joint.M_maxMotorTorque
}

func (joint *WheelJoint) SetSpringFrequencyHz(hz float64) {
	joint.M_frequencyHz = hz
}

func (joint WheelJoint) GetSpringFrequencyHz() float64 {
	return joint.M_frequencyHz
}

func (joint *WheelJoint) SetSpringDampingRatio(ratio float64) {
	joint.M_dampingRatio = ratio
}

func (joint WheelJoint) GetSpringDampingRatio() float64 {
	return joint.M_dampingRatio
}

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

func (def *WheelJointDef) Initialize(bA *Body, bB *Body, anchor Vec2, axis Vec2) {
	def.BodyA = bA
	def.BodyB = bB
	def.LocalAnchorA = def.BodyA.GetLocalPoint(anchor)
	def.LocalAnchorB = def.BodyB.GetLocalPoint(anchor)
	def.LocalAxisA = def.BodyA.GetLocalVector(axis)
}

func MakeWheelJoint(def *WheelJointDef) *WheelJoint {
	res := WheelJoint{
		Joint: MakeJoint(def),
	}

	res.M_localAnchorA = def.LocalAnchorA
	res.M_localAnchorB = def.LocalAnchorB
	res.M_localXAxisA = def.LocalAxisA
	res.M_localYAxisA = Vec2CrossScalarVector(1.0, res.M_localXAxisA)

	res.M_mass = 0.0
	res.M_impulse = 0.0
	res.M_motorMass = 0.0
	res.M_motorImpulse = 0.0
	res.M_springMass = 0.0
	res.M_springImpulse = 0.0

	res.M_maxMotorTorque = def.MaxMotorTorque
	res.M_motorSpeed = def.MotorSpeed
	res.M_enableMotor = def.EnableMotor

	res.M_frequencyHz = def.FrequencyHz
	res.M_dampingRatio = def.DampingRatio

	res.M_bias = 0.0
	res.M_gamma = 0.0

	res.M_ax.SetZero()
	res.M_ay.SetZero()

	return &res
}

func (joint *WheelJoint) InitVelocityConstraints(data SolverData) {
	joint.M_indexA = joint.M_bodyA.M_islandIndex
	joint.M_indexB = joint.M_bodyB.M_islandIndex
	joint.M_localCenterA = joint.M_bodyA.M_sweep.LocalCenter
	joint.M_localCenterB = joint.M_bodyB.M_sweep.LocalCenter
	joint.M_invMassA = joint.M_bodyA.M_invMass
	joint.M_invMassB = joint.M_bodyB.M_invMass
	joint.M_invIA = joint.M_bodyA.M_invI
	joint.M_invIB = joint.M_bodyB.M_invI

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

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
	d := Vec2Sub(Vec2Sub(Vec2Add(cB, rB), cA), rA)

	// Point to line constraint
	{
		joint.M_ay = RotVec2Mul(qA, joint.M_localYAxisA)
		joint.M_sAy = Vec2Cross(Vec2Add(d, rA), joint.M_ay)
		joint.M_sBy = Vec2Cross(rB, joint.M_ay)

		joint.M_mass = mA + mB + iA*joint.M_sAy*joint.M_sAy + iB*joint.M_sBy*joint.M_sBy

		if joint.M_mass > 0.0 {
			joint.M_mass = 1.0 / joint.M_mass
		}
	}

	// Spring constraint
	joint.M_springMass = 0.0
	joint.M_bias = 0.0
	joint.M_gamma = 0.0
	if joint.M_frequencyHz > 0.0 {
		joint.M_ax = RotVec2Mul(qA, joint.M_localXAxisA)
		joint.M_sAx = Vec2Cross(Vec2Add(d, rA), joint.M_ax)
		joint.M_sBx = Vec2Cross(rB, joint.M_ax)

		invMass := mA + mB + iA*joint.M_sAx*joint.M_sAx + iB*joint.M_sBx*joint.M_sBx

		if invMass > 0.0 {
			joint.M_springMass = 1.0 / invMass

			C := Vec2Dot(d, joint.M_ax)

			// Frequency
			omega := 2.0 * Pi * joint.M_frequencyHz

			// Damping coefficient
			damp := 2.0 * joint.M_springMass * joint.M_dampingRatio * omega

			// Spring stiffness
			k := joint.M_springMass * omega * omega

			// magic formulas
			h := data.Step.Dt
			joint.M_gamma = h * (damp + h*k)
			if joint.M_gamma > 0.0 {
				joint.M_gamma = 1.0 / joint.M_gamma
			}

			joint.M_bias = C * h * k * joint.M_gamma

			joint.M_springMass = invMass + joint.M_gamma
			if joint.M_springMass > 0.0 {
				joint.M_springMass = 1.0 / joint.M_springMass
			}
		}
	} else {
		joint.M_springImpulse = 0.0
	}

	// Rotational motor
	if joint.M_enableMotor {
		joint.M_motorMass = iA + iB
		if joint.M_motorMass > 0.0 {
			joint.M_motorMass = 1.0 / joint.M_motorMass
		}
	} else {
		joint.M_motorMass = 0.0
		joint.M_motorImpulse = 0.0
	}

	if data.Step.WarmStarting {
		// Account for variable time step.
		joint.M_impulse *= data.Step.DtRatio
		joint.M_springImpulse *= data.Step.DtRatio
		joint.M_motorImpulse *= data.Step.DtRatio

		P := Vec2Add(Vec2MulScalar(joint.M_impulse, joint.M_ay), Vec2MulScalar(joint.M_springImpulse, joint.M_ax))
		LA := joint.M_impulse*joint.M_sAy + joint.M_springImpulse*joint.M_sAx + joint.M_motorImpulse
		LB := joint.M_impulse*joint.M_sBy + joint.M_springImpulse*joint.M_sBx + joint.M_motorImpulse

		vA.OperatorMinusInplace(Vec2MulScalar(joint.M_invMassA, P))
		wA -= joint.M_invIA * LA

		vB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, P))
		wB += joint.M_invIB * LB
	} else {
		joint.M_impulse = 0.0
		joint.M_springImpulse = 0.0
		joint.M_motorImpulse = 0.0
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *WheelJoint) SolveVelocityConstraints(data SolverData) {
	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	// Solve spring constraint
	{
		Cdot := Vec2Dot(joint.M_ax, Vec2Sub(vB, vA)) + joint.M_sBx*wB - joint.M_sAx*wA
		impulse := -joint.M_springMass * (Cdot + joint.M_bias + joint.M_gamma*joint.M_springImpulse)
		joint.M_springImpulse += impulse

		P := Vec2MulScalar(impulse, joint.M_ax)
		LA := impulse * joint.M_sAx
		LB := impulse * joint.M_sBx

		vA.OperatorMinusInplace(Vec2MulScalar(mA, P))
		wA -= iA * LA

		vB.OperatorPlusInplace(Vec2MulScalar(mB, P))
		wB += iB * LB
	}

	// Solve rotational motor constraint
	{
		Cdot := wB - wA - joint.M_motorSpeed
		impulse := -joint.M_motorMass * Cdot

		oldImpulse := joint.M_motorImpulse
		maxImpulse := data.Step.Dt * joint.M_maxMotorTorque
		joint.M_motorImpulse = FloatClamp(joint.M_motorImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = joint.M_motorImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	// Solve point to line constraint
	{
		Cdot := Vec2Dot(joint.M_ay, Vec2Sub(vB, vA)) + joint.M_sBy*wB - joint.M_sAy*wA
		impulse := -joint.M_mass * Cdot
		joint.M_impulse += impulse

		P := Vec2MulScalar(impulse, joint.M_ay)
		LA := impulse * joint.M_sAy
		LB := impulse * joint.M_sBy

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

func (joint *WheelJoint) SolvePositionConstraints(data SolverData) bool {
	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A

	qA := MakeRotFromAngle(aA)
	qB := MakeRotFromAngle(aB)

	rA := RotVec2Mul(qA, Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	rB := RotVec2Mul(qB, Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))
	d := Vec2Sub(Vec2Add(Vec2Sub(cB, cA), rB), rA)

	ay := RotVec2Mul(qA, joint.M_localYAxisA)

	sAy := Vec2Cross(Vec2Add(d, rA), ay)
	sBy := Vec2Cross(rB, ay)

	C := Vec2Dot(d, ay)

	k := joint.M_invMassA + joint.M_invMassB + joint.M_invIA*joint.M_sAy*joint.M_sAy + joint.M_invIB*joint.M_sBy*joint.M_sBy

	impulse := 0.0
	if k != 0.0 {
		impulse = -C / k
	} else {
		impulse = 0.0
	}

	P := Vec2MulScalar(impulse, ay)
	LA := impulse * sAy
	LB := impulse * sBy

	cA.OperatorMinusInplace(Vec2MulScalar(joint.M_invMassA, P))
	aA -= joint.M_invIA * LA
	cB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, P))
	aB += joint.M_invIB * LB

	data.Positions[joint.M_indexA].C = cA
	data.Positions[joint.M_indexA].A = aA
	data.Positions[joint.M_indexB].C = cB
	data.Positions[joint.M_indexB].A = aB

	return math.Abs(C) <= linearSlop
}

func (joint WheelJoint) GetAnchorA() Vec2 {
	return joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
}

func (joint WheelJoint) GetAnchorB() Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint WheelJoint) GetReactionForce(inv_dt float64) Vec2 {
	return Vec2MulScalar(inv_dt, Vec2Add(Vec2MulScalar(joint.M_impulse, joint.M_ay), Vec2MulScalar(joint.M_springImpulse, joint.M_ax)))
}

func (joint WheelJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * joint.M_motorImpulse
}

func (joint WheelJoint) GetJointTranslation() float64 {
	bA := joint.M_bodyA
	bB := joint.M_bodyB

	pA := bA.GetWorldPoint(joint.M_localAnchorA)
	pB := bB.GetWorldPoint(joint.M_localAnchorB)
	d := Vec2Sub(pB, pA)
	axis := bA.GetWorldVector(joint.M_localXAxisA)

	translation := Vec2Dot(d, axis)
	return translation
}

func (joint WheelJoint) GetJointLinearSpeed() float64 {
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

	speed := Vec2Dot(d, Vec2CrossScalarVector(wA, axis)) + Vec2Dot(axis, Vec2Sub(Vec2Sub(Vec2Add(vB, Vec2CrossScalarVector(wB, rB)), vA), Vec2CrossScalarVector(wA, rA)))
	return speed
}

func (joint WheelJoint) GetJointAngle() float64 {
	bA := joint.M_bodyA
	bB := joint.M_bodyB
	return bB.M_sweep.A - bA.M_sweep.A
}

func (joint WheelJoint) GetJointAngularSpeed() float64 {
	wA := joint.M_bodyA.M_angularVelocity
	wB := joint.M_bodyB.M_angularVelocity
	return wB - wA
}

func (joint WheelJoint) IsMotorEnabled() bool {
	return joint.M_enableMotor
}

func (joint *WheelJoint) EnableMotor(flag bool) {
	if flag != joint.M_enableMotor {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_enableMotor = flag
	}
}

func (joint *WheelJoint) SetMotorSpeed(speed float64) {
	if speed != joint.M_motorSpeed {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_motorSpeed = speed
	}
}

func (joint *WheelJoint) SetMaxMotorTorque(torque float64) {
	if torque != joint.M_maxMotorTorque {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_maxMotorTorque = torque
	}
}

func (joint WheelJoint) GetMotorTorque(inv_dt float64) float64 {
	return inv_dt * joint.M_motorImpulse
}

func (joint *WheelJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	fmt.Printf("  b2WheelJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.localAnchorA.Set(%.15f, %.15f);\n", joint.M_localAnchorA.X, joint.M_localAnchorA.Y)
	fmt.Printf("  jd.localAnchorB.Set(%.15f, %.15f);\n", joint.M_localAnchorB.X, joint.M_localAnchorB.Y)
	fmt.Printf("  jd.localAxisA.Set(%.15f, %.15f);\n", joint.M_localXAxisA.X, joint.M_localXAxisA.Y)
	fmt.Printf("  jd.enableMotor = bool(%v);\n", joint.M_enableMotor)
	fmt.Printf("  jd.motorSpeed = %.15f;\n", joint.M_motorSpeed)
	fmt.Printf("  jd.maxMotorTorque = %.15f;\n", joint.M_maxMotorTorque)
	fmt.Printf("  jd.frequencyHz = %.15f;\n", joint.M_frequencyHz)
	fmt.Printf("  jd.dampingRatio = %.15f;\n", joint.M_dampingRatio)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}
