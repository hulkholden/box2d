package box2d

var JointType = struct {
	Unknown   uint8
	Revolute  uint8
	Prismatic uint8
	Distance  uint8
	Pulley    uint8
	Mouse     uint8
	Gear      uint8
	Wheel     uint8
	Weld      uint8
	Friction  uint8
	Rope      uint8
	Motor     uint8
}{
	Unknown:   1,
	Revolute:  2,
	Prismatic: 3,
	Distance:  4,
	Pulley:    5,
	Mouse:     6,
	Gear:      7,
	Wheel:     8,
	Weld:      9,
	Friction:  10,
	Rope:      11,
	Motor:     12,
}

var LimitState = struct {
	Inactive     uint8
	AtLowerLimit uint8
	AtUpperLimit uint8
	Equal        uint8
}{
	Inactive:     1,
	AtLowerLimit: 2,
	AtUpperLimit: 3,
	Equal:        4,
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
	Other *Body          ///< provides quick access to the other body attached.
	Joint JointInterface ///< the joint; backed by pointer
	Prev  *JointEdge     ///< the previous joint edge in the body's joint list
	Next  *JointEdge     ///< the next joint edge in the body's joint list
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

// Implementing JointDefInterface on Joint (used as a base struct)
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
	res.Type = JointType.Unknown
	res.UserData = nil
	res.BodyA = nil
	res.BodyB = nil
	res.CollideConnected = false

	return res
}

// The base joint class. Joints are used to constraint two bodies together in
// various fashions. Some joints also feature limits and motors.
type Joint struct {
	M_type             uint8
	M_prev             JointInterface // has to be backed by pointer
	M_next             JointInterface // has to be backed by pointer
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
func (j Joint) Dump() {}

// Shift the origin for any points stored in world coordinates.
func (j Joint) ShiftOrigin(newOrigin Vec2) {}

func (j Joint) GetType() uint8 {
	return j.M_type
}

// @goadd
func (j *Joint) SetType(t uint8) {
	j.M_type = t
}

func (j Joint) GetBodyA() *Body {
	return j.M_bodyA
}

// @goadd
func (j *Joint) SetBodyA(body *Body) {
	j.M_bodyA = body
}

func (j Joint) GetBodyB() *Body {
	return j.M_bodyB
}

// @goadd
func (j *Joint) SetBodyB(body *Body) {
	j.M_bodyB = body
}

func (j Joint) GetNext() JointInterface { // returns pointer
	return j.M_next
}

// @goadd
func (j *Joint) SetNext(next JointInterface) { // has to be backed by pointer
	j.M_next = next
}

func (j Joint) GetPrev() JointInterface { // returns pointer
	return j.M_prev
}

// @goadd
func (j *Joint) SetPrev(prev JointInterface) { // prev has to be backed by pointer
	j.M_prev = prev
}

func (j Joint) GetUserData() interface{} {
	return j.M_userData
}

func (j *Joint) SetUserData(data interface{}) {
	j.M_userData = data
}

func (j Joint) IsCollideConnected() bool {
	return j.M_collideConnected
}

// @goadd
func (j *Joint) SetCollideConnected(flag bool) {
	j.M_collideConnected = flag
}

// @goadd
func (j Joint) GetEdgeA() *JointEdge {
	return j.M_edgeA
}

// @goadd
func (j *Joint) SetEdgeA(edge *JointEdge) {
	j.M_edgeA = edge
}

// @goadd
func (j Joint) GetEdgeB() *JointEdge {
	return j.M_edgeB
}

// @goadd
func (j *Joint) SetEdgeB(edge *JointEdge) {
	j.M_edgeB = edge
}

func JointCreate(def JointDefInterface) JointInterface { // def should be back by pointer; a pointer is returned
	var joint *Joint = nil

	switch def.GetType() {
	case JointType.Distance:
		if typeddef, ok := def.(*DistanceJointDef); ok {
			return MakeB2DistanceJoint(typeddef)
		}
		assert(false)

	case JointType.Mouse:
		if typeddef, ok := def.(*MouseJointDef); ok {
			return MakeB2MouseJoint(typeddef)
		}
		assert(false)

	case JointType.Prismatic:
		if typeddef, ok := def.(*PrismaticJointDef); ok {
			return MakeB2PrismaticJoint(typeddef)
		}
		assert(false)

	case JointType.Revolute:
		if typeddef, ok := def.(*RevoluteJointDef); ok {
			return MakeRevoluteJoint(typeddef)
		}
		assert(false)

	case JointType.Pulley:
		if typeddef, ok := def.(*PulleyJointDef); ok {
			return MakePulleyJoint(typeddef)
		}
		assert(false)

	case JointType.Gear:
		if typeddef, ok := def.(*GearJointDef); ok {
			return MakeGearJoint(typeddef)
		}
		assert(false)

	case JointType.Wheel:
		if typeddef, ok := def.(*WheelJointDef); ok {
			return MakeWheelJoint(typeddef)
		}
		assert(false)

	case JointType.Weld:
		if typeddef, ok := def.(*WeldJointDef); ok {
			return MakeWeldJoint(typeddef)
		}
		assert(false)

	case JointType.Friction:
		if typeddef, ok := def.(*B2FrictionJointDef); ok {
			return MakeB2FrictionJoint(typeddef)
		}
		assert(false)

	case JointType.Rope:
		if typeddef, ok := def.(*B2RopeJointDef); ok {
			return MakeB2RopeJoint(typeddef)
		}
		assert(false)

	case JointType.Motor:
		if typeddef, ok := def.(*B2MotorJointDef); ok {
			return MakeB2MotorJoint(typeddef)
		}
		assert(false)

	default:
		assert(false)
	}

	return joint
}

func JointDestroy(joint JointInterface) { // has to be backed by pointer
	joint.Destroy()
}

func MakeJoint(def JointDefInterface) *Joint { // def has to be backed by pointer
	assert(def.GetBodyA() != def.GetBodyB())

	res := Joint{}

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

func (j Joint) IsActive() bool {
	return j.M_bodyA.IsActive() && j.M_bodyB.IsActive()
}

// @goadd
func (j *Joint) Destroy() {
}

// @goadd
func (j Joint) GetIndex() int {
	return j.M_index
}

func (j *Joint) SetIndex(index int) {
	j.M_index = index
}

func (j *Joint) InitVelocityConstraints(data SolverData) {}

func (j *Joint) SolveVelocityConstraints(data SolverData) {}

func (j *Joint) SolvePositionConstraints(data SolverData) bool {
	return false
}

func (j Joint) GetIslandFlag() bool {
	return j.M_islandFlag
}

func (j *Joint) SetIslandFlag(flag bool) {
	j.M_islandFlag = flag
}

type JointInterface interface {
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

	GetNext() JointInterface     // backed by pointer
	SetNext(next JointInterface) // backed by pointer

	GetPrev() JointInterface     // backed by pointer
	SetPrev(prev JointInterface) // backed by pointer

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

	InitVelocityConstraints(data SolverData)

	SolveVelocityConstraints(data SolverData)

	SolvePositionConstraints(data SolverData) bool

	GetIslandFlag() bool
	SetIslandFlag(flag bool)
}
