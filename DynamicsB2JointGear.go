package box2d

import (
	"fmt"
)

// Gear joint definition. This definition requires two existing
// revolute or prismatic joints (any combination will work).
type GearJointDef struct {
	JointDef

	/// The first revolute/prismatic joint attached to the gear joint.
	Joint1 JointInterface // has to be backed by pointer

	/// The second revolute/prismatic joint attached to the gear joint.
	Joint2 JointInterface // has to be backed by pointer

	/// The gear ratio.
	/// @see b2GearJoint for explanation.
	Ratio float64
}

func MakeGearJointDef() GearJointDef {
	res := GearJointDef{
		JointDef: MakeJointDef(),
	}

	res.Type = JointType.Gear
	res.Joint1 = nil
	res.Joint2 = nil
	res.Ratio = 1.0

	return res
}

// A gear joint is used to connect two joints together. Either joint
// can be a revolute or prismatic joint. You specify a gear ratio
// to bind the motions together:
// coordinate1 + ratio * coordinate2 = constant
// The ratio can be negative or positive. If one joint is a revolute joint
// and the other joint is a prismatic joint, then the ratio will have units
// of length or units of 1/length.
// @warning You have to manually destroy the gear joint if joint1 or joint2
// is destroyed.
type GearJoint struct {
	*Joint

	M_joint1 JointInterface // backed by pointer
	M_joint2 JointInterface // backed by pointer

	M_typeA uint8
	M_typeB uint8

	// Body A is connected to body C
	// Body B is connected to body D
	M_bodyC *Body
	M_bodyD *Body

	// Solver shared
	M_localAnchorA Vec2
	M_localAnchorB Vec2
	M_localAnchorC Vec2
	M_localAnchorD Vec2

	M_localAxisC Vec2
	M_localAxisD Vec2

	M_referenceAngleA float64
	M_referenceAngleB float64

	M_constant float64
	M_ratio    float64

	M_impulse float64

	// Solver temp
	M_indexA, M_indexB, M_indexC, M_indexD int
	M_lcA, M_lcB, M_lcC, M_lcD             Vec2
	M_mA, M_mB, M_mC, M_mD                 float64
	M_iA, M_iB, M_iC, M_iD                 float64
	M_JvAC, M_JvBD                         Vec2
	M_JwA, M_JwB, M_JwC, M_JwD             float64
	M_mass                                 float64
}

// Get the first joint.
func (joint GearJoint) GetJoint1() JointInterface { // returns a pointer
	return joint.M_joint1
}

// Get the second joint.
func (joint GearJoint) GetJoint2() JointInterface { // returns a pointer
	return joint.M_joint2
}

// Gear Joint:
// C0 = (coordinate1 + ratio * coordinate2)_initial
// C = (coordinate1 + ratio * coordinate2) - C0 = 0
// J = [J1 ratio * J2]
// K = J * invM * JT
//   = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
// Revolute:
// coordinate = rotation
// Cdot = angularVelocity
// J = [0 0 1]
// K = J * invM * JT = invI
//
// Prismatic:
// coordinate = dot(p - pg, ug)
// Cdot = dot(v + cross(w, r), ug)
// J = [ug cross(r, ug)]
// K = J * invM * JT = invMass + invI * cross(r, ug)^2

func MakeGearJoint(def *GearJointDef) *GearJoint {
	res := GearJoint{
		Joint: MakeJoint(def),
	}

	res.M_joint1 = def.Joint1
	res.M_joint2 = def.Joint2

	res.M_typeA = res.M_joint1.GetType()
	res.M_typeB = res.M_joint2.GetType()

	assert(res.M_typeA == JointType.Revolute || res.M_typeA == JointType.Prismatic)
	assert(res.M_typeB == JointType.Revolute || res.M_typeB == JointType.Prismatic)

	coordinateA := 0.0
	coordinateB := 0.0

	// TODO_ERIN there might be some problem with the joint edges in b2Joint.

	res.M_bodyC = res.M_joint1.GetBodyA()
	res.M_bodyA = res.M_joint1.GetBodyB()

	// Get geometry of joint1
	xfA := res.M_bodyA.M_xf
	aA := res.M_bodyA.M_sweep.A
	xfC := res.M_bodyC.M_xf
	aC := res.M_bodyC.M_sweep.A

	if res.M_typeA == JointType.Revolute {
		revolute := def.Joint1.(*RevoluteJoint)
		res.M_localAnchorC = revolute.M_localAnchorA
		res.M_localAnchorA = revolute.M_localAnchorB
		res.M_referenceAngleA = revolute.M_referenceAngle
		res.M_localAxisC.SetZero()

		coordinateA = aA - aC - res.M_referenceAngleA
	} else {
		prismatic := def.Joint1.(*PrismaticJoint)
		res.M_localAnchorC = prismatic.M_localAnchorA
		res.M_localAnchorA = prismatic.M_localAnchorB
		res.M_referenceAngleA = prismatic.M_referenceAngle
		res.M_localAxisC = prismatic.M_localXAxisA

		pC := res.M_localAnchorC
		pA := RotVec2MulT(xfC.Q, Vec2Add(RotVec2Mul(xfA.Q, res.M_localAnchorA), Vec2Sub(xfA.P, xfC.P)))
		coordinateA = Vec2Dot(Vec2Sub(pA, pC), res.M_localAxisC)
	}

	res.M_bodyD = res.M_joint2.GetBodyA()
	res.M_bodyB = res.M_joint2.GetBodyB()

	// Get geometry of joint2
	xfB := res.M_bodyB.M_xf
	aB := res.M_bodyB.M_sweep.A
	xfD := res.M_bodyD.M_xf
	aD := res.M_bodyD.M_sweep.A

	if res.M_typeB == JointType.Revolute {
		revolute := def.Joint2.(*RevoluteJoint)
		res.M_localAnchorD = revolute.M_localAnchorA
		res.M_localAnchorB = revolute.M_localAnchorB
		res.M_referenceAngleB = revolute.M_referenceAngle
		res.M_localAxisD.SetZero()

		coordinateB = aB - aD - res.M_referenceAngleB
	} else {
		prismatic := def.Joint2.(*PrismaticJoint)
		res.M_localAnchorD = prismatic.M_localAnchorA
		res.M_localAnchorB = prismatic.M_localAnchorB
		res.M_referenceAngleB = prismatic.M_referenceAngle
		res.M_localAxisD = prismatic.M_localXAxisA

		pD := res.M_localAnchorD
		pB := RotVec2MulT(xfD.Q, Vec2Add(RotVec2Mul(xfB.Q, res.M_localAnchorB), Vec2Sub(xfB.P, xfD.P)))
		coordinateB = Vec2Dot(Vec2Sub(pB, pD), res.M_localAxisD)
	}

	res.M_ratio = def.Ratio

	res.M_constant = coordinateA + res.M_ratio*coordinateB

	res.M_impulse = 0.0

	return &res
}

func (joint *GearJoint) InitVelocityConstraints(data SolverData) {
	joint.M_indexA = joint.M_bodyA.M_islandIndex
	joint.M_indexB = joint.M_bodyB.M_islandIndex
	joint.M_indexC = joint.M_bodyC.M_islandIndex
	joint.M_indexD = joint.M_bodyD.M_islandIndex
	joint.M_lcA = joint.M_bodyA.M_sweep.LocalCenter
	joint.M_lcB = joint.M_bodyB.M_sweep.LocalCenter
	joint.M_lcC = joint.M_bodyC.M_sweep.LocalCenter
	joint.M_lcD = joint.M_bodyD.M_sweep.LocalCenter
	joint.M_mA = joint.M_bodyA.M_invMass
	joint.M_mB = joint.M_bodyB.M_invMass
	joint.M_mC = joint.M_bodyC.M_invMass
	joint.M_mD = joint.M_bodyD.M_invMass
	joint.M_iA = joint.M_bodyA.M_invI
	joint.M_iB = joint.M_bodyB.M_invI
	joint.M_iC = joint.M_bodyC.M_invI
	joint.M_iD = joint.M_bodyD.M_invI

	aA := data.Positions[joint.M_indexA].A
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W

	aB := data.Positions[joint.M_indexB].A
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	aC := data.Positions[joint.M_indexC].A
	vC := data.Velocities[joint.M_indexC].V
	wC := data.Velocities[joint.M_indexC].W

	aD := data.Positions[joint.M_indexD].A
	vD := data.Velocities[joint.M_indexD].V
	wD := data.Velocities[joint.M_indexD].W

	qA := MakeRotFromAngle(aA)
	qB := MakeRotFromAngle(aB)
	qC := MakeRotFromAngle(aC)
	qD := MakeRotFromAngle(aD)

	joint.M_mass = 0.0

	if joint.M_typeA == JointType.Revolute {
		joint.M_JvAC.SetZero()
		joint.M_JwA = 1.0
		joint.M_JwC = 1.0
		joint.M_mass += joint.M_iA + joint.M_iC
	} else {
		u := RotVec2Mul(qC, joint.M_localAxisC)
		rC := RotVec2Mul(qC, Vec2Sub(joint.M_localAnchorC, joint.M_lcC))
		rA := RotVec2Mul(qA, Vec2Sub(joint.M_localAnchorA, joint.M_lcA))
		joint.M_JvAC = u
		joint.M_JwC = Vec2Cross(rC, u)
		joint.M_JwA = Vec2Cross(rA, u)
		joint.M_mass += joint.M_mC + joint.M_mA + joint.M_iC*joint.M_JwC*joint.M_JwC + joint.M_iA*joint.M_JwA*joint.M_JwA
	}

	if joint.M_typeB == JointType.Revolute {
		joint.M_JvBD.SetZero()
		joint.M_JwB = joint.M_ratio
		joint.M_JwD = joint.M_ratio
		joint.M_mass += joint.M_ratio * joint.M_ratio * (joint.M_iB + joint.M_iD)
	} else {
		u := RotVec2Mul(qD, joint.M_localAxisD)
		rD := RotVec2Mul(qD, Vec2Sub(joint.M_localAnchorD, joint.M_lcD))
		rB := RotVec2Mul(qB, Vec2Sub(joint.M_localAnchorB, joint.M_lcB))
		joint.M_JvBD = Vec2MulScalar(joint.M_ratio, u)
		joint.M_JwD = joint.M_ratio * Vec2Cross(rD, u)
		joint.M_JwB = joint.M_ratio * Vec2Cross(rB, u)
		joint.M_mass += joint.M_ratio*joint.M_ratio*(joint.M_mD+joint.M_mB) + joint.M_iD*joint.M_JwD*joint.M_JwD + joint.M_iB*joint.M_JwB*joint.M_JwB
	}

	// Compute effective mass.
	if joint.M_mass > 0.0 {
		joint.M_mass = 1.0 / joint.M_mass
	} else {
		joint.M_mass = 0.0
	}

	if data.Step.WarmStarting {
		vA.OperatorPlusInplace(Vec2MulScalar(joint.M_mA*joint.M_impulse, joint.M_JvAC))
		wA += joint.M_iA * joint.M_impulse * joint.M_JwA
		vB.OperatorPlusInplace(Vec2MulScalar(joint.M_mB*joint.M_impulse, joint.M_JvBD))
		wB += joint.M_iB * joint.M_impulse * joint.M_JwB
		vC.OperatorMinusInplace(Vec2MulScalar(joint.M_mC*joint.M_impulse, joint.M_JvAC))
		wC -= joint.M_iC * joint.M_impulse * joint.M_JwC
		vD.OperatorMinusInplace(Vec2MulScalar(joint.M_mD*joint.M_impulse, joint.M_JvBD))
		wD -= joint.M_iD * joint.M_impulse * joint.M_JwD
	} else {
		joint.M_impulse = 0.0
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
	data.Velocities[joint.M_indexC].V = vC
	data.Velocities[joint.M_indexC].W = wC
	data.Velocities[joint.M_indexD].V = vD
	data.Velocities[joint.M_indexD].W = wD
}

func (joint *GearJoint) SolveVelocityConstraints(data SolverData) {
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W
	vC := data.Velocities[joint.M_indexC].V
	wC := data.Velocities[joint.M_indexC].W
	vD := data.Velocities[joint.M_indexD].V
	wD := data.Velocities[joint.M_indexD].W

	Cdot := Vec2Dot(joint.M_JvAC, Vec2Sub(vA, vC)) + Vec2Dot(joint.M_JvBD, Vec2Sub(vB, vD))
	Cdot += (joint.M_JwA*wA - joint.M_JwC*wC) + (joint.M_JwB*wB - joint.M_JwD*wD)

	impulse := -joint.M_mass * Cdot
	joint.M_impulse += impulse

	vA.OperatorPlusInplace(Vec2MulScalar(joint.M_mA*impulse, joint.M_JvAC))
	wA += joint.M_iA * impulse * joint.M_JwA
	vB.OperatorPlusInplace(Vec2MulScalar(joint.M_mB*impulse, joint.M_JvBD))
	wB += joint.M_iB * impulse * joint.M_JwB
	vC.OperatorMinusInplace(Vec2MulScalar(joint.M_mC*impulse, joint.M_JvAC))
	wC -= joint.M_iC * impulse * joint.M_JwC
	vD.OperatorMinusInplace(Vec2MulScalar(joint.M_mD*impulse, joint.M_JvBD))
	wD -= joint.M_iD * impulse * joint.M_JwD

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
	data.Velocities[joint.M_indexC].V = vC
	data.Velocities[joint.M_indexC].W = wC
	data.Velocities[joint.M_indexD].V = vD
	data.Velocities[joint.M_indexD].W = wD
}

func (joint *GearJoint) SolvePositionConstraints(data SolverData) bool {
	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A
	cC := data.Positions[joint.M_indexC].C
	aC := data.Positions[joint.M_indexC].A
	cD := data.Positions[joint.M_indexD].C
	aD := data.Positions[joint.M_indexD].A

	qA := MakeRotFromAngle(aA)
	qB := MakeRotFromAngle(aB)
	qC := MakeRotFromAngle(aC)
	qD := MakeRotFromAngle(aD)

	linearError := 0.0

	coordinateA := 0.0
	coordinateB := 0.0

	var JvAC Vec2
	var JvBD Vec2
	var JwA, JwB, JwC, JwD float64
	mass := 0.0

	if joint.M_typeA == JointType.Revolute {
		JvAC.SetZero()
		JwA = 1.0
		JwC = 1.0
		mass += joint.M_iA + joint.M_iC

		coordinateA = aA - aC - joint.M_referenceAngleA
	} else {
		u := RotVec2Mul(qC, joint.M_localAxisC)
		rC := RotVec2Mul(qC, Vec2Sub(joint.M_localAnchorC, joint.M_lcC))
		rA := RotVec2Mul(qA, Vec2Sub(joint.M_localAnchorA, joint.M_lcA))
		JvAC = u
		JwC = Vec2Cross(rC, u)
		JwA = Vec2Cross(rA, u)
		mass += joint.M_mC + joint.M_mA + joint.M_iC*JwC*JwC + joint.M_iA*JwA*JwA

		pC := Vec2Sub(joint.M_localAnchorC, joint.M_lcC)
		pA := RotVec2MulT(qC, Vec2Add(rA, Vec2Sub(cA, cC)))
		coordinateA = Vec2Dot(Vec2Sub(pA, pC), joint.M_localAxisC)
	}

	if joint.M_typeB == JointType.Revolute {
		JvBD.SetZero()
		JwB = joint.M_ratio
		JwD = joint.M_ratio
		mass += joint.M_ratio * joint.M_ratio * (joint.M_iB + joint.M_iD)

		coordinateB = aB - aD - joint.M_referenceAngleB
	} else {
		u := RotVec2Mul(qD, joint.M_localAxisD)
		rD := RotVec2Mul(qD, Vec2Sub(joint.M_localAnchorD, joint.M_lcD))
		rB := RotVec2Mul(qB, Vec2Sub(joint.M_localAnchorB, joint.M_lcB))
		JvBD = Vec2MulScalar(joint.M_ratio, u)
		JwD = joint.M_ratio * Vec2Cross(rD, u)
		JwB = joint.M_ratio * Vec2Cross(rB, u)
		mass += joint.M_ratio*joint.M_ratio*(joint.M_mD+joint.M_mB) + joint.M_iD*JwD*JwD + joint.M_iB*JwB*JwB

		pD := Vec2Sub(joint.M_localAnchorD, joint.M_lcD)
		pB := RotVec2MulT(qD, Vec2Add(rB, Vec2Sub(cB, cD)))
		coordinateB = Vec2Dot(Vec2Sub(pB, pD), joint.M_localAxisD)
	}

	C := (coordinateA + joint.M_ratio*coordinateB) - joint.M_constant

	impulse := 0.0
	if mass > 0.0 {
		impulse = -C / mass
	}

	cA.OperatorPlusInplace(Vec2MulScalar(joint.M_mA*impulse, JvAC))
	aA += joint.M_iA * impulse * JwA
	cB.OperatorPlusInplace(Vec2MulScalar(joint.M_mB*impulse, JvBD))
	aB += joint.M_iB * impulse * JwB
	cC.OperatorMinusInplace(Vec2MulScalar(joint.M_mC*impulse, JvAC))
	aC -= joint.M_iC * impulse * JwC
	cD.OperatorMinusInplace(Vec2MulScalar(joint.M_mD*impulse, JvBD))
	aD -= joint.M_iD * impulse * JwD

	data.Positions[joint.M_indexA].C = cA
	data.Positions[joint.M_indexA].A = aA
	data.Positions[joint.M_indexB].C = cB
	data.Positions[joint.M_indexB].A = aB
	data.Positions[joint.M_indexC].C = cC
	data.Positions[joint.M_indexC].A = aC
	data.Positions[joint.M_indexD].C = cD
	data.Positions[joint.M_indexD].A = aD

	// TODO_ERIN not implemented
	return linearError < linearSlop
}

func (joint GearJoint) GetAnchorA() Vec2 {
	return joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
}

func (joint GearJoint) GetAnchorB() Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint GearJoint) GetReactionForce(inv_dt float64) Vec2 {
	P := Vec2MulScalar(joint.M_impulse, joint.M_JvAC)
	return Vec2MulScalar(inv_dt, P)
}

func (joint GearJoint) GetReactionTorque(inv_dt float64) float64 {
	L := joint.M_impulse * joint.M_JwA
	return inv_dt * L
}

func (joint *GearJoint) SetRatio(ratio float64) {
	assert(IsValid(ratio))
	joint.M_ratio = ratio
}

func (joint GearJoint) GetRatio() float64 {
	return joint.M_ratio
}

func (joint *GearJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	index1 := joint.GetJoint1().GetIndex()
	index2 := joint.GetJoint2().GetIndex()

	fmt.Printf("  b2GearJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.joint1 = joints[%d];\n", index1)
	fmt.Printf("  jd.joint2 = joints[%d];\n", index2)
	fmt.Printf("  jd.ratio = %.15f;\n", joint.M_ratio)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}
