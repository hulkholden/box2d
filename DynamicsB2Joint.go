package box2d

var B2JointType = struct {
	E_unknownJoint   uint8
	E_revoluteJoint  uint8
	E_prismaticJoint uint8
	E_distanceJoint  uint8
	E_pulleyJoint    uint8
	E_mouseJoint     uint8
	E_gearJoint      uint8
	E_wheelJoint     uint8
	E_weldJoint      uint8
	E_frictionJoint  uint8
	E_ropeJoint      uint8
	E_motorJoint     uint8
}{
	E_unknownJoint:   1,
	E_revoluteJoint:  2,
	E_prismaticJoint: 3,
	E_distanceJoint:  4,
	E_pulleyJoint:    5,
	E_mouseJoint:     6,
	E_gearJoint:      7,
	E_wheelJoint:     8,
	E_weldJoint:      9,
	E_frictionJoint:  10,
	E_ropeJoint:      11,
	E_motorJoint:     12,
}

var B2LimitState = struct {
	E_inactiveLimit uint8
	E_atLowerLimit  uint8
	E_atUpperLimit  uint8
	E_equalLimits   uint8
}{
	E_inactiveLimit: 1,
	E_atLowerLimit:  2,
	E_atUpperLimit:  3,
	E_equalLimits:   4,
}

type Jacobian struct {
	Linear   Vec2
	AngularA float64
	AngularB float64
}

// A joint edge is used to connect bodies and joints together
// in a joint graph where each body is a node and each joint
// is an edge. A joint edge belongs to a doubly linked list
// maintained in each attached body. Each joint has two joint
// nodes, one for each attached body.
type JointEdge struct {
	Other *Body            ///< provides quick access to the other body attached.
	Joint B2JointInterface ///< the joint; backed by pointer
	Prev  *JointEdge       ///< the previous joint edge in the body's joint list
	Next  *JointEdge       ///< the next joint edge in the body's joint list
}

// Joint definitions are used to construct joints.
type JointDef struct {
	/// The joint type is set automatically for concrete joint types.
	Type uint8

	/// Use this to attach application specific data to your joints.
	UserData interface{}

	/// The first attached body.
	BodyA *Body

	/// The second attached body.
	BodyB *Body

	/// Set this flag to true if the attached bodies should collide.
	CollideConnected bool
}

type JointDefInterface interface {
	GetType() uint8
	SetType(t uint8)
	GetUserData() interface{}
	SetUserData(userdata interface{})
	GetBodyA() *Body
	SetBodyA(body *Body)
	GetBodyB() *Body
	SetBodyB(body *Body)
	IsCollideConnected() bool
	SetCollideConnected(flag bool)
}

// Implementing JointDefInterface on B2Joint (used as a base struct)
func (def JointDef) GetType() uint8 {
	return def.Type
}

func (def *JointDef) SetType(t uint8) {
	def.Type = t
}

func (def JointDef) GetUserData() interface{} {
	return def.UserData
}

func (def *JointDef) SetUserData(userdata interface{}) {
	def.UserData = userdata
}

func (def JointDef) GetBodyA() *Body {
	return def.BodyA
}

func (def *JointDef) SetBodyA(body *Body) {
	def.BodyA = body
}

func (def JointDef) GetBodyB() *Body {
	return def.BodyB
}

func (def *JointDef) SetBodyB(body *Body) {
	def.BodyB = body
}

func (def JointDef) IsCollideConnected() bool {
	return def.CollideConnected
}

func (def *JointDef) SetCollideConnected(flag bool) {
	def.CollideConnected = flag
}

func MakeJointDef() JointDef {
	res := JointDef{}
	res.Type = B2JointType.E_unknownJoint
	res.UserData = nil
	res.BodyA = nil
	res.BodyB = nil
	res.CollideConnected = false

	return res
}

// The base joint class. Joints are used to constraint two bodies together in
// various fashions. Some joints also feature limits and motors.
type B2Joint struct {
	M_type             uint8
	M_prev             B2JointInterface // has to be backed by pointer
	M_next             B2JointInterface // has to be backed by pointer
	M_edgeA            *JointEdge
	M_edgeB            *JointEdge
	M_bodyA            *Body
	M_bodyB            *Body
	M_index            int
	M_islandFlag       bool
	M_collideConnected bool
	M_userData         interface{}
}

// Dump this joint to the log file.
func (j B2Joint) Dump() {}

// Shift the origin for any points stored in world coordinates.
func (j B2Joint) ShiftOrigin(newOrigin Vec2) {}

func (j B2Joint) GetType() uint8 {
	return j.M_type
}

// @goadd
func (j *B2Joint) SetType(t uint8) {
	j.M_type = t
}

func (j B2Joint) GetBodyA() *Body {
	return j.M_bodyA
}

// @goadd
func (j *B2Joint) SetBodyA(body *Body) {
	j.M_bodyA = body
}

func (j B2Joint) GetBodyB() *Body {
	return j.M_bodyB
}

// @goadd
func (j *B2Joint) SetBodyB(body *Body) {
	j.M_bodyB = body
}

func (j B2Joint) GetNext() B2JointInterface { // returns pointer
	return j.M_next
}

// @goadd
func (j *B2Joint) SetNext(next B2JointInterface) { // has to be backed by pointer
	j.M_next = next
}

func (j B2Joint) GetPrev() B2JointInterface { // returns pointer
	return j.M_prev
}

// @goadd
func (j *B2Joint) SetPrev(prev B2JointInterface) { // prev has to be backed by pointer
	j.M_prev = prev
}

func (j B2Joint) GetUserData() interface{} {
	return j.M_userData
}

func (j *B2Joint) SetUserData(data interface{}) {
	j.M_userData = data
}

func (j B2Joint) IsCollideConnected() bool {
	return j.M_collideConnected
}

// @goadd
func (j *B2Joint) SetCollideConnected(flag bool) {
	j.M_collideConnected = flag
}

// @goadd
func (j B2Joint) GetEdgeA() *JointEdge {
	return j.M_edgeA
}

// @goadd
func (j *B2Joint) SetEdgeA(edge *JointEdge) {
	j.M_edgeA = edge
}

// @goadd
func (j B2Joint) GetEdgeB() *JointEdge {
	return j.M_edgeB
}

// @goadd
func (j *B2Joint) SetEdgeB(edge *JointEdge) {
	j.M_edgeB = edge
}

func B2JointCreate(def JointDefInterface) B2JointInterface { // def should be back by pointer; a pointer is returned

	var joint *B2Joint = nil

	switch def.GetType() {
	case B2JointType.E_distanceJoint:
		if typeddef, ok := def.(*B2DistanceJointDef); ok {
			return MakeB2DistanceJoint(typeddef)
		}
		assert(false)

	case B2JointType.E_mouseJoint:
		if typeddef, ok := def.(*B2MouseJointDef); ok {
			return MakeB2MouseJoint(typeddef)
		}
		assert(false)

	case B2JointType.E_prismaticJoint:
		if typeddef, ok := def.(*B2PrismaticJointDef); ok {
			return MakeB2PrismaticJoint(typeddef)
		}
		assert(false)

	case B2JointType.E_revoluteJoint:
		if typeddef, ok := def.(*B2RevoluteJointDef); ok {
			return MakeB2RevoluteJoint(typeddef)
		}
		assert(false)

	case B2JointType.E_pulleyJoint:
		if typeddef, ok := def.(*B2PulleyJointDef); ok {
			return MakeB2PulleyJoint(typeddef)
		}
		assert(false)

	case B2JointType.E_gearJoint:
		if typeddef, ok := def.(*B2GearJointDef); ok {
			return MakeB2GearJoint(typeddef)
		}
		assert(false)

	case B2JointType.E_wheelJoint:
		if typeddef, ok := def.(*B2WheelJointDef); ok {
			return MakeB2WheelJoint(typeddef)
		}
		assert(false)

	case B2JointType.E_weldJoint:
		if typeddef, ok := def.(*B2WeldJointDef); ok {
			return MakeB2WeldJoint(typeddef)
		}
		assert(false)

	case B2JointType.E_frictionJoint:
		if typeddef, ok := def.(*B2FrictionJointDef); ok {
			return MakeB2FrictionJoint(typeddef)
		}
		assert(false)

	case B2JointType.E_ropeJoint:
		if typeddef, ok := def.(*B2RopeJointDef); ok {
			return MakeB2RopeJoint(typeddef)
		}
		assert(false)

	case B2JointType.E_motorJoint:
		if typeddef, ok := def.(*B2MotorJointDef); ok {
			return MakeB2MotorJoint(typeddef)
		}
		assert(false)

	default:
		assert(false)
	}

	return joint
}

func B2JointDestroy(joint B2JointInterface) { // has to be backed by pointer
	joint.Destroy()
}

func MakeB2Joint(def JointDefInterface) *B2Joint { // def has to be backed by pointer
	assert(def.GetBodyA() != def.GetBodyB())

	res := B2Joint{}

	res.M_type = def.GetType()
	res.M_prev = nil
	res.M_next = nil
	res.M_bodyA = def.GetBodyA()
	res.M_bodyB = def.GetBodyB()
	res.M_index = 0
	res.M_collideConnected = def.IsCollideConnected()
	res.M_islandFlag = false
	res.M_userData = def.GetUserData()

	res.M_edgeA = &JointEdge{}
	res.M_edgeB = &JointEdge{}

	return &res
}

func (j B2Joint) IsActive() bool {
	return j.M_bodyA.IsActive() && j.M_bodyB.IsActive()
}

// @goadd
func (j *B2Joint) Destroy() {
}

// @goadd
func (j B2Joint) GetIndex() int {
	return j.M_index
}

func (j *B2Joint) SetIndex(index int) {
	j.M_index = index
}

func (j *B2Joint) InitVelocityConstraints(data B2SolverData) {}

func (j *B2Joint) SolveVelocityConstraints(data B2SolverData) {}

func (j *B2Joint) SolvePositionConstraints(data B2SolverData) bool {
	return false
}

func (j B2Joint) GetIslandFlag() bool {
	return j.M_islandFlag
}

func (j *B2Joint) SetIslandFlag(flag bool) {
	j.M_islandFlag = flag
}

type B2JointInterface interface {
	/// Dump this joint to the log file.
	Dump()

	/// Shift the origin for any points stored in world coordinates.
	ShiftOrigin(newOrigin Vec2)

	GetType() uint8
	SetType(t uint8)

	GetBodyA() *Body
	SetBodyA(body *Body)

	GetBodyB() *Body
	SetBodyB(body *Body)

	GetIndex() int
	SetIndex(index int)

	GetNext() B2JointInterface     // backed by pointer
	SetNext(next B2JointInterface) // backed by pointer

	GetPrev() B2JointInterface     // backed by pointer
	SetPrev(prev B2JointInterface) // backed by pointer

	GetEdgeA() *JointEdge
	SetEdgeA(edge *JointEdge)

	GetEdgeB() *JointEdge
	SetEdgeB(edge *JointEdge)

	GetUserData() interface{}
	SetUserData(data interface{})

	IsCollideConnected() bool
	SetCollideConnected(flag bool)

	IsActive() bool

	//@goadd
	Destroy()

	InitVelocityConstraints(data B2SolverData)

	SolveVelocityConstraints(data B2SolverData)

	SolvePositionConstraints(data B2SolverData) bool

	GetIslandFlag() bool
	SetIslandFlag(flag bool)
}
