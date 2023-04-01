package box2d

import (
	"fmt"
)

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver

var BodyType = struct {
	StaticBody    uint8
	KinematicBody uint8
	DynamicBody   uint8
}{
	StaticBody:    0,
	KinematicBody: 1,
	DynamicBody:   2,
}

// A body definition holds all the data needed to construct a rigid body.
// You can safely re-use body definitions. Shapes are added to a body after construction.
type BodyDef struct {
	/// The body type: static, kinematic, or dynamic.
	/// Note: if a dynamic body would have zero mass, the mass is set to one.
	Type uint8

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	Position Vec2

	/// The world angle of the body in radians.
	Angle float64

	/// The linear velocity of the body's origin in world co-ordinates.
	LinearVelocity Vec2

	/// The angular velocity of the body.
	AngularVelocity float64

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1.0 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Units are 1/time
	LinearDamping float64

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Units are 1/time
	AngularDamping float64

	/// Set this flag to false if this body should never fall asleep. Note that
	/// this increases CPU usage.
	AllowSleep bool

	/// Is this body initially awake or sleeping?
	Awake bool

	/// Should this body be prevented from rotating? Useful for characters.
	FixedRotation bool

	/// Is this a fast moving body that should be prevented from tunneling through
	/// other moving bodies? Note that all bodies are prevented from tunneling through
	/// kinematic and static bodies. This setting is only considered on dynamic bodies.
	/// @warning You should use this flag sparingly since it increases processing time.
	Bullet bool

	/// Does this body start out active?
	Active bool

	/// Use this to store application specific body data.
	UserData interface{}

	/// Scale the gravity applied to this body.
	GravityScale float64
}

// This constructor sets the body definition default values.
func MakeBodyDef() BodyDef {
	return BodyDef{
		UserData:        nil,
		Position:        MakeVec2(0, 0),
		Angle:           0.0,
		LinearVelocity:  MakeVec2(0, 0),
		AngularVelocity: 0.0,
		LinearDamping:   0.0,
		AngularDamping:  0.0,
		AllowSleep:      true,
		Awake:           true,
		FixedRotation:   false,
		Bullet:          false,
		Type:            BodyType.StaticBody,
		Active:          true,
		GravityScale:    1.0,
	}
}

func NewBodyDef() *BodyDef {
	res := MakeBodyDef()
	return &res
}

var BodyFlags = struct {
	Island        uint32
	Awake         uint32
	AutoSleep     uint32
	Bullet        uint32
	FixedRotation uint32
	Active        uint32
	TOI           uint32
}{
	Island:        0x0001,
	Awake:         0x0002,
	AutoSleep:     0x0004,
	Bullet:        0x0008,
	FixedRotation: 0x0010,
	Active:        0x0020,
	TOI:           0x0040,
}

type Body struct {
	M_type uint8

	M_flags uint32

	M_islandIndex int

	M_xf    Transform // the body origin transform
	M_sweep Sweep     // the swept motion for CCD

	M_linearVelocity  Vec2
	M_angularVelocity float64

	M_force  Vec2
	M_torque float64

	M_world *B2World
	M_prev  *Body
	M_next  *Body

	M_fixtureList  *B2Fixture // linked list
	M_fixtureCount int

	M_jointList   *B2JointEdge   // linked list
	M_contactList *B2ContactEdge // linked list

	M_mass, M_invMass float64

	// Rotational inertia about the center of mass.
	M_I, M_invI float64

	M_linearDamping  float64
	M_angularDamping float64
	M_gravityScale   float64

	M_sleepTime float64

	M_userData interface{}
}

func (body Body) GetType() uint8 {
	return body.M_type
}

func (body Body) GetTransform() Transform {
	return body.M_xf
}

func (body Body) GetPosition() Vec2 {
	return body.M_xf.P
}

func (body Body) GetAngle() float64 {
	return body.M_sweep.A
}

func (body Body) GetWorldCenter() Vec2 {
	return body.M_sweep.C
}

func (body Body) GetLocalCenter() Vec2 {
	return body.M_sweep.LocalCenter
}

func (body *Body) SetLinearVelocity(v Vec2) {
	if body.M_type == BodyType.StaticBody {
		return
	}

	if Vec2Dot(v, v) > 0.0 {
		body.SetAwake(true)
	}

	body.M_linearVelocity = v
}

func (body Body) GetLinearVelocity() Vec2 {
	return body.M_linearVelocity
}

func (body *Body) SetAngularVelocity(w float64) {
	if body.M_type == BodyType.StaticBody {
		return
	}

	if w*w > 0.0 {
		body.SetAwake(true)
	}

	body.M_angularVelocity = w
}

func (body Body) GetAngularVelocity() float64 {
	return body.M_angularVelocity
}

func (body Body) GetMass() float64 {
	return body.M_mass
}

func (body Body) GetInertia() float64 {
	return body.M_I + body.M_mass*Vec2Dot(body.M_sweep.LocalCenter, body.M_sweep.LocalCenter)
}

func (body Body) GetMassData() B2MassData {
	data := MakeMassData()
	data.Mass = body.M_mass
	data.I = body.M_I + body.M_mass*Vec2Dot(body.M_sweep.LocalCenter, body.M_sweep.LocalCenter)
	data.Center = body.M_sweep.LocalCenter
	return data
}

func (body Body) GetWorldPoint(localPoint Vec2) Vec2 {
	return TransformVec2Mul(body.M_xf, localPoint)
}

func (body Body) GetWorldVector(localVector Vec2) Vec2 {
	return RotVec2Mul(body.M_xf.Q, localVector)
}

func (body Body) GetLocalPoint(worldPoint Vec2) Vec2 {
	return TransformVec2MulT(body.M_xf, worldPoint)
}

func (body Body) GetLocalVector(worldVector Vec2) Vec2 {
	return RotVec2MulT(body.M_xf.Q, worldVector)
}

func (body Body) GetLinearVelocityFromWorldPoint(worldPoint Vec2) Vec2 {
	return Vec2Add(body.M_linearVelocity, Vec2CrossScalarVector(body.M_angularVelocity, Vec2Sub(worldPoint, body.M_sweep.C)))
}

func (body Body) GetLinearVelocityFromLocalPoint(localPoint Vec2) Vec2 {
	return body.GetLinearVelocityFromWorldPoint(body.GetWorldPoint(localPoint))
}

func (body Body) GetLinearDamping() float64 {
	return body.M_linearDamping
}

func (body *Body) SetLinearDamping(linearDamping float64) {
	body.M_linearDamping = linearDamping
}

func (body Body) GetAngularDamping() float64 {
	return body.M_angularDamping
}

func (body *Body) SetAngularDamping(angularDamping float64) {
	body.M_angularDamping = angularDamping
}

func (body Body) GetGravityScale() float64 {
	return body.M_gravityScale
}

func (body *Body) SetGravityScale(scale float64) {
	body.M_gravityScale = scale
}

func (body *Body) SetBullet(flag bool) {
	if flag {
		body.M_flags |= BodyFlags.Bullet
	} else {
		body.M_flags &= ^BodyFlags.Bullet
	}
}

func (body Body) IsBullet() bool {
	return (body.M_flags & BodyFlags.Bullet) == BodyFlags.Bullet
}

func (body *Body) SetAwake(flag bool) {
	if flag {
		body.M_flags |= BodyFlags.Awake
		body.M_sleepTime = 0.0
	} else {
		body.M_flags &= ^BodyFlags.Awake
		body.M_sleepTime = 0.0
		body.M_linearVelocity.SetZero()
		body.M_angularVelocity = 0.0
		body.M_force.SetZero()
		body.M_torque = 0.0
	}
}

func (body Body) IsAwake() bool {
	return (body.M_flags & BodyFlags.Awake) == BodyFlags.Awake
}

func (body Body) IsActive() bool {
	return (body.M_flags & BodyFlags.Active) == BodyFlags.Active
}

func (body Body) IsFixedRotation() bool {
	return (body.M_flags & BodyFlags.FixedRotation) == BodyFlags.FixedRotation
}

func (body *Body) SetSleepingAllowed(flag bool) {
	if flag {
		body.M_flags |= BodyFlags.AutoSleep
	} else {
		body.M_flags &= ^BodyFlags.AutoSleep
		body.SetAwake(true)
	}
}

func (body Body) IsSleepingAllowed() bool {
	return (body.M_flags & BodyFlags.AutoSleep) == BodyFlags.AutoSleep
}

func (body Body) GetFixtureList() *B2Fixture {
	return body.M_fixtureList
}

func (body Body) GetJointList() *B2JointEdge {
	return body.M_jointList
}

func (body Body) GetContactList() *B2ContactEdge {
	return body.M_contactList
}

func (body Body) GetNext() *Body {
	return body.M_next
}

func (body *Body) SetUserData(data interface{}) {
	body.M_userData = data
}

func (body Body) GetUserData() interface{} {
	return body.M_userData
}

func (body *Body) ApplyForce(force Vec2, point Vec2, wake bool) {
	if body.M_type != BodyType.DynamicBody {
		return
	}

	if wake && (body.M_flags&BodyFlags.Awake) == 0 {
		body.SetAwake(true)
	}

	// Don't accumulate a force if the body is sleeping.
	if (body.M_flags & BodyFlags.Awake) != 0x0000 {
		body.M_force.OperatorPlusInplace(force)
		body.M_torque += Vec2Cross(
			Vec2Sub(point, body.M_sweep.C),
			force,
		)
	}
}

func (body *Body) ApplyForceToCenter(force Vec2, wake bool) {
	if body.M_type != BodyType.DynamicBody {
		return
	}

	if wake && (body.M_flags&BodyFlags.Awake) == 0 {
		body.SetAwake(true)
	}

	// Don't accumulate a force if the body is sleeping
	if (body.M_flags & BodyFlags.Awake) != 0x0000 {
		body.M_force.OperatorPlusInplace(force)
	}
}

func (body *Body) ApplyTorque(torque float64, wake bool) {
	if body.M_type != BodyType.DynamicBody {
		return
	}

	if wake && (body.M_flags&BodyFlags.Awake) == 0 {
		body.SetAwake(true)
	}

	// Don't accumulate a force if the body is sleeping
	if (body.M_flags & BodyFlags.Awake) != 0x0000 {
		body.M_torque += torque
	}
}

func (body *Body) ApplyLinearImpulse(impulse Vec2, point Vec2, wake bool) {
	if body.M_type != BodyType.DynamicBody {
		return
	}

	if wake && (body.M_flags&BodyFlags.Awake) == 0 {
		body.SetAwake(true)
	}

	// Don't accumulate velocity if the body is sleeping
	if (body.M_flags & BodyFlags.Awake) != 0x0000 {
		body.M_linearVelocity.OperatorPlusInplace(Vec2MulScalar(body.M_invMass, impulse))
		body.M_angularVelocity += body.M_invI * Vec2Cross(
			Vec2Sub(point, body.M_sweep.C),
			impulse,
		)
	}
}

func (body *Body) ApplyLinearImpulseToCenter(impulse Vec2, wake bool) {
	if body.M_type != BodyType.DynamicBody {
		return
	}

	if wake && (body.M_flags&BodyFlags.Awake) == 0 {
		body.SetAwake(true)
	}

	// Don't accumulate velocity if the body is sleeping
	if (body.M_flags & BodyFlags.Awake) != 0x0000 {
		body.M_linearVelocity.OperatorPlusInplace(Vec2MulScalar(body.M_invMass, impulse))
	}
}

func (body *Body) ApplyAngularImpulse(impulse float64, wake bool) {
	if body.M_type != BodyType.DynamicBody {
		return
	}

	if wake && (body.M_flags&BodyFlags.Awake) == 0 {
		body.SetAwake(true)
	}

	// Don't accumulate velocity if the body is sleeping
	if (body.M_flags & BodyFlags.Awake) != 0x0000 {
		body.M_angularVelocity += body.M_invI * impulse
	}
}

func (body *Body) SynchronizeTransform() {
	body.M_xf.Q.Set(body.M_sweep.A)
	body.M_xf.P = Vec2Sub(body.M_sweep.C, RotVec2Mul(body.M_xf.Q, body.M_sweep.LocalCenter))
}

func (body *Body) Advance(alpha float64) {
	// Advance to the new safe time. This doesn't sync the broad-phase.
	body.M_sweep.Advance(alpha)
	body.M_sweep.C = body.M_sweep.C0
	body.M_sweep.A = body.M_sweep.A0
	body.M_xf.Q.Set(body.M_sweep.A)
	body.M_xf.P = Vec2Sub(body.M_sweep.C, RotVec2Mul(body.M_xf.Q, body.M_sweep.LocalCenter))
}

func (body Body) GetWorld() *B2World {
	return body.M_world
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2Body.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func NewBody(bd *BodyDef, world *B2World) *Body {
	assert(bd.Position.IsValid())
	assert(bd.LinearVelocity.IsValid())
	assert(IsValid(bd.Angle))
	assert(IsValid(bd.AngularVelocity))
	assert(IsValid(bd.AngularDamping) && bd.AngularDamping >= 0.0)
	assert(IsValid(bd.LinearDamping) && bd.LinearDamping >= 0.0)

	body := &Body{}

	body.M_flags = 0

	if bd.Bullet {
		body.M_flags |= BodyFlags.Bullet
	}

	if bd.FixedRotation {
		body.M_flags |= BodyFlags.FixedRotation
	}

	if bd.AllowSleep {
		body.M_flags |= BodyFlags.AutoSleep
	}

	if bd.Awake {
		body.M_flags |= BodyFlags.Awake
	}

	if bd.Active {
		body.M_flags |= BodyFlags.Active
	}

	body.M_world = world

	body.M_xf.P = bd.Position
	body.M_xf.Q.Set(bd.Angle)

	body.M_sweep.LocalCenter.SetZero()
	body.M_sweep.C0 = body.M_xf.P
	body.M_sweep.C = body.M_xf.P
	body.M_sweep.A0 = bd.Angle
	body.M_sweep.A = bd.Angle
	body.M_sweep.Alpha0 = 0.0

	body.M_jointList = nil
	body.M_contactList = nil
	body.M_prev = nil
	body.M_next = nil

	body.M_linearVelocity = bd.LinearVelocity
	body.M_angularVelocity = bd.AngularVelocity

	body.M_linearDamping = bd.LinearDamping
	body.M_angularDamping = bd.AngularDamping
	body.M_gravityScale = bd.GravityScale

	body.M_force.SetZero()
	body.M_torque = 0.0

	body.M_sleepTime = 0.0

	body.M_type = bd.Type

	if body.M_type == BodyType.DynamicBody {
		body.M_mass = 1.0
		body.M_invMass = 1.0
	} else {
		body.M_mass = 0.0
		body.M_invMass = 0.0
	}

	body.M_I = 0.0
	body.M_invI = 0.0

	body.M_userData = bd.UserData

	body.M_fixtureList = nil
	body.M_fixtureCount = 0

	return body
}

func (body *Body) SetType(bodytype uint8) {
	assert(!body.M_world.IsLocked())
	if body.M_world.IsLocked() {
		return
	}

	if body.M_type == bodytype {
		return
	}

	body.M_type = bodytype

	body.ResetMassData()

	if body.M_type == BodyType.StaticBody {
		body.M_linearVelocity.SetZero()
		body.M_angularVelocity = 0.0
		body.M_sweep.A0 = body.M_sweep.A
		body.M_sweep.C0 = body.M_sweep.C
		body.SynchronizeFixtures()
	}

	body.SetAwake(true)

	body.M_force.SetZero()
	body.M_torque = 0.0

	// Delete the attached contacts.
	ce := body.M_contactList
	for ce != nil {
		ce0 := ce
		ce = ce.Next
		body.M_world.M_contactManager.Destroy(ce0.Contact)
	}

	body.M_contactList = nil

	// Touch the proxies so that new contacts will be created (when appropriate)
	broadPhase := body.M_world.M_contactManager.M_broadPhase
	for f := body.M_fixtureList; f != nil; f = f.M_next {
		proxyCount := f.M_proxyCount
		for i := 0; i < proxyCount; i++ {
			broadPhase.TouchProxy(f.M_proxies[i].ProxyId)
		}
	}
}

func (body *Body) CreateFixtureFromDef(def *B2FixtureDef) *B2Fixture {
	assert(!body.M_world.IsLocked())
	if body.M_world.IsLocked() {
		return nil
	}

	fixture := NewB2Fixture()
	fixture.Create(body, def)

	if (body.M_flags & BodyFlags.Active) != 0x0000 {
		broadPhase := &body.M_world.M_contactManager.M_broadPhase
		fixture.CreateProxies(broadPhase, body.M_xf)
	}

	fixture.M_next = body.M_fixtureList
	body.M_fixtureList = fixture
	body.M_fixtureCount++

	fixture.M_body = body

	// Adjust mass properties if needed.
	if fixture.M_density > 0.0 {
		body.ResetMassData()
	}

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	body.M_world.M_flags |= B2World_Flags.E_newFixture

	return fixture
}

func (body *Body) CreateFixture(shape B2ShapeInterface, density float64) *B2Fixture {
	def := MakeB2FixtureDef()
	def.Shape = shape
	def.Density = density

	return body.CreateFixtureFromDef(&def)
}

func (body *Body) DestroyFixture(fixture *B2Fixture) {
	if fixture == nil {
		return
	}

	assert(!body.M_world.IsLocked())
	if body.M_world.IsLocked() {
		return
	}

	assert(fixture.M_body == body)

	// Remove the fixture from this body's singly linked list.
	assert(body.M_fixtureCount > 0)
	node := &body.M_fixtureList
	found := false
	for *node != nil {
		if *node == fixture {
			*node = fixture.M_next
			found = true
			break
		}

		node = &(*node).M_next
	}

	// You tried to remove a shape that is not attached to this body.
	assert(found)

	// Destroy any contacts associated with the fixture.
	edge := body.M_contactList
	for edge != nil {
		c := edge.Contact
		edge = edge.Next

		fixtureA := c.GetFixtureA()
		fixtureB := c.GetFixtureB()

		if fixture == fixtureA || fixture == fixtureB {
			// This destroys the contact and removes it from
			// this body's contact list.
			body.M_world.M_contactManager.Destroy(c)
		}
	}

	if (body.M_flags & BodyFlags.Active) != 0x0000 {
		broadPhase := &body.M_world.M_contactManager.M_broadPhase
		fixture.DestroyProxies(broadPhase)
	}

	fixture.M_body = nil
	fixture.M_next = nil
	fixture.Destroy()

	body.M_fixtureCount--

	// Reset the mass data.
	body.ResetMassData()
}

func (body *Body) ResetMassData() {
	// Compute mass data from shapes. Each shape has its own density.
	body.M_mass = 0.0
	body.M_invMass = 0.0
	body.M_I = 0.0
	body.M_invI = 0.0
	body.M_sweep.LocalCenter.SetZero()

	// Static and kinematic bodies have zero mass.
	if body.M_type == BodyType.StaticBody || body.M_type == BodyType.KinematicBody {
		body.M_sweep.C0 = body.M_xf.P
		body.M_sweep.C = body.M_xf.P
		body.M_sweep.A0 = body.M_sweep.A
		return
	}

	assert(body.M_type == BodyType.DynamicBody)

	// Accumulate mass over all fixtures.
	localCenter := MakeVec2(0, 0)
	for f := body.M_fixtureList; f != nil; f = f.M_next {
		if f.M_density == 0.0 {
			continue
		}

		massData := f.GetMassData()
		body.M_mass += massData.Mass
		localCenter.OperatorPlusInplace(Vec2MulScalar(massData.Mass, massData.Center))
		body.M_I += massData.I
	}

	// Compute center of mass.
	if body.M_mass > 0.0 {
		body.M_invMass = 1.0 / body.M_mass
		localCenter.OperatorScalarMulInplace(body.M_invMass)
	} else {
		// Force all dynamic bodies to have a positive mass.
		body.M_mass = 1.0
		body.M_invMass = 1.0
	}

	if body.M_I > 0.0 && (body.M_flags&BodyFlags.FixedRotation) == 0 {
		// Center the inertia about the center of mass.
		body.M_I -= body.M_mass * Vec2Dot(localCenter, localCenter)
		assert(body.M_I > 0.0)
		body.M_invI = 1.0 / body.M_I

	} else {
		body.M_I = 0.0
		body.M_invI = 0.0
	}

	// Move center of mass.
	oldCenter := body.M_sweep.C
	body.M_sweep.LocalCenter = localCenter
	body.M_sweep.C0 = TransformVec2Mul(body.M_xf, body.M_sweep.LocalCenter)
	body.M_sweep.C = TransformVec2Mul(body.M_xf, body.M_sweep.LocalCenter)

	// Update center of mass velocity.
	body.M_linearVelocity.OperatorPlusInplace(Vec2CrossScalarVector(
		body.M_angularVelocity,
		Vec2Sub(body.M_sweep.C, oldCenter),
	))
}

func (body *Body) SetMassData(massData *B2MassData) {
	assert(!body.M_world.IsLocked())
	if body.M_world.IsLocked() {
		return
	}

	if body.M_type != BodyType.DynamicBody {
		return
	}

	body.M_invMass = 0.0
	body.M_I = 0.0
	body.M_invI = 0.0

	body.M_mass = massData.Mass
	if body.M_mass <= 0.0 {
		body.M_mass = 1.0
	}

	body.M_invMass = 1.0 / body.M_mass

	if massData.I > 0.0 && (body.M_flags&BodyFlags.FixedRotation) == 0 {
		body.M_I = massData.I - body.M_mass*Vec2Dot(massData.Center, massData.Center)
		assert(body.M_I > 0.0)
		body.M_invI = 1.0 / body.M_I
	}

	// Move center of mass.
	oldCenter := body.M_sweep.C
	body.M_sweep.LocalCenter = massData.Center
	body.M_sweep.C0 = TransformVec2Mul(body.M_xf, body.M_sweep.LocalCenter)
	body.M_sweep.C = TransformVec2Mul(body.M_xf, body.M_sweep.LocalCenter)

	// Update center of mass velocity.
	body.M_linearVelocity.OperatorPlusInplace(
		Vec2CrossScalarVector(
			body.M_angularVelocity,
			Vec2Sub(body.M_sweep.C, oldCenter),
		),
	)
}

func (body Body) ShouldCollide(other *Body) bool {
	// At least one body should be dynamic.
	if body.M_type != BodyType.DynamicBody && other.M_type != BodyType.DynamicBody {
		return false
	}

	// Does a joint prevent collision?
	for jn := body.M_jointList; jn != nil; jn = jn.Next {
		if jn.Other == other {
			if !jn.Joint.IsCollideConnected() {
				return false
			}
		}
	}

	return true
}

func (body *Body) SetTransform(position Vec2, angle float64) {
	assert(!body.M_world.IsLocked())

	if body.M_world.IsLocked() {
		return
	}

	body.M_xf.Q.Set(angle)
	body.M_xf.P = position

	body.M_sweep.C = TransformVec2Mul(body.M_xf, body.M_sweep.LocalCenter)
	body.M_sweep.A = angle

	body.M_sweep.C0 = body.M_sweep.C
	body.M_sweep.A0 = angle

	broadPhase := &body.M_world.M_contactManager.M_broadPhase
	for f := body.M_fixtureList; f != nil; f = f.M_next {
		f.Synchronize(broadPhase, body.M_xf, body.M_xf)
	}
}

func (body *Body) SynchronizeFixtures() {
	xf1 := MakeTransform()
	xf1.Q.Set(body.M_sweep.A0)
	xf1.P = Vec2Sub(body.M_sweep.C0, RotVec2Mul(xf1.Q, body.M_sweep.LocalCenter))

	broadPhase := &body.M_world.M_contactManager.M_broadPhase
	for f := body.M_fixtureList; f != nil; f = f.M_next {
		f.Synchronize(broadPhase, xf1, body.M_xf)
	}
}

func (body *Body) SetActive(flag bool) {
	assert(!body.M_world.IsLocked())

	if flag == body.IsActive() {
		return
	}

	if flag {
		body.M_flags |= BodyFlags.Active

		// Create all proxies.
		broadPhase := &body.M_world.M_contactManager.M_broadPhase
		for f := body.M_fixtureList; f != nil; f = f.M_next {
			f.CreateProxies(broadPhase, body.M_xf)
		}

		// Contacts are created the next time step.
	} else {
		body.M_flags &= ^BodyFlags.Active

		// Destroy all proxies.
		broadPhase := &body.M_world.M_contactManager.M_broadPhase
		for f := body.M_fixtureList; f != nil; f = f.M_next {
			f.DestroyProxies(broadPhase)
		}

		// Destroy the attached contacts.
		ce := body.M_contactList
		for ce != nil {
			ce0 := ce
			ce = ce.Next
			body.M_world.M_contactManager.Destroy(ce0.Contact)
		}

		body.M_contactList = nil
	}
}

func (body *Body) SetFixedRotation(flag bool) {
	status := (body.M_flags & BodyFlags.FixedRotation) == BodyFlags.FixedRotation

	if status == flag {
		return
	}

	if flag {
		body.M_flags |= BodyFlags.FixedRotation
	} else {
		body.M_flags &= ^BodyFlags.FixedRotation
	}

	body.M_angularVelocity = 0.0

	body.ResetMassData()
}

func (body *Body) Dump() {
	bodyIndex := body.M_islandIndex

	fmt.Print("{\n")
	fmt.Print("  BodyDef bd;\n")
	fmt.Printf("  bd.type = b2BodyType(%d);\n", body.M_type)
	fmt.Printf("  bd.position.Set(%.15f, %.15f);\n", body.M_xf.P.X, body.M_xf.P.Y)
	fmt.Printf("  bd.angle = %.15f;\n", body.M_sweep.A)
	fmt.Printf("  bd.linearVelocity.Set(%.15f, %.15f);\n", body.M_linearVelocity.X, body.M_linearVelocity.Y)
	fmt.Printf("  bd.angularVelocity = %.15f;\n", body.M_angularVelocity)
	fmt.Printf("  bd.linearDamping = %.15f;\n", body.M_linearDamping)
	fmt.Printf("  bd.angularDamping = %.15f;\n", body.M_angularDamping)
	fmt.Printf("  bd.allowSleep = bool(%d);\n", body.M_flags&BodyFlags.AutoSleep)
	fmt.Printf("  bd.awake = bool(%d);\n", body.M_flags&BodyFlags.Awake)
	fmt.Printf("  bd.fixedRotation = bool(%d);\n", body.M_flags&BodyFlags.FixedRotation)
	fmt.Printf("  bd.bullet = bool(%d);\n", body.M_flags&BodyFlags.Bullet)
	fmt.Printf("  bd.active = bool(%d);\n", body.M_flags&BodyFlags.Active)
	fmt.Printf("  bd.gravityScale = %.15f;\n", body.M_gravityScale)
	fmt.Printf("  bodies[%d] = body.M_world.CreateBody(&bd);\n", body.M_islandIndex)
	fmt.Print("\n")
	for f := body.M_fixtureList; f != nil; f = f.M_next {
		fmt.Print("  {\n")
		f.Dump(bodyIndex)
		fmt.Print("  }\n")
	}
	fmt.Print("}\n")
}
