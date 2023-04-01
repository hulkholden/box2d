package box2d

import (
	"math"
)

// Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
// For example, anything slides on ice.
func MixFriction(friction1, friction2 float64) float64 {
	return math.Sqrt(friction1 * friction2)
}

// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
// For example, a superball bounces on anything.
func MixRestitution(restitution1, restitution2 float64) float64 {
	if restitution1 > restitution2 {
		return restitution1
	}

	return restitution2
}

type (
	ContactCreateFcn  func(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) ContactInterface // returned contact should be a pointer
	ContactDestroyFcn func(contact ContactInterface)                                                      // contact should be a pointer
)

type ContactRegister struct {
	CreateFcn  ContactCreateFcn
	DestroyFcn ContactDestroyFcn
	Primary    bool
}

// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// nodes, one for each attached body.
type ContactEdge struct {
	Other   *Body            ///< provides quick access to the other body attached.
	Contact ContactInterface ///< the contact
	Prev    *ContactEdge     ///< the previous contact edge in the body's contact list
	Next    *ContactEdge     ///< the next contact edge in the body's contact list
}

func NewContactEdge() *ContactEdge {
	return &ContactEdge{}
}

var ContactFlags = struct {
	Island    uint32 // Used when crawling contact graph when forming islands.
	Touching  uint32 // Set when the shapes are touching.
	Enabled   uint32 // This contact can be disabled (by user)
	Filter    uint32 // This contact needs filtering because a fixture filter was changed.
	BulletHit uint32 // This bullet contact had a TOI event
	TOI       uint32 // This contact has a valid TOI in m_toi
}{
	Island:    0x0001,
	Touching:  0x0002,
	Enabled:   0x0004,
	Filter:    0x0008,
	BulletHit: 0x0010,
	TOI:       0x0020,
}

// The class manages contact between two shapes. A contact exists for each overlapping
// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
// that has no contact points.
var (
	s_registers   [][]ContactRegister
	s_initialized = false
)

type ContactInterface interface {
	GetFlags() uint32
	SetFlags(flags uint32)

	GetPrev() ContactInterface
	SetPrev(prev ContactInterface)

	GetNext() ContactInterface
	SetNext(prev ContactInterface)

	GetNodeA() *ContactEdge
	SetNodeA(node *ContactEdge)

	GetNodeB() *ContactEdge
	SetNodeB(node *ContactEdge)

	GetFixtureA() *Fixture
	SetFixtureA(fixture *Fixture)

	GetFixtureB() *Fixture
	SetFixtureB(fixture *Fixture)

	GetChildIndexA() int
	SetChildIndexA(index int)

	GetChildIndexB() int
	SetChildIndexB(index int)

	GetManifold() *B2Manifold
	SetManifold(manifold *B2Manifold)

	GetTOICount() int
	SetTOICount(toiCount int)

	GetTOI() float64
	SetTOI(toiCount float64)

	GetFriction() float64
	SetFriction(friction float64)
	ResetFriction()

	GetRestitution() float64
	SetRestitution(restitution float64)
	ResetRestitution()

	GetTangentSpeed() float64
	SetTangentSpeed(tangentSpeed float64)

	IsTouching() bool
	IsEnabled() bool
	SetEnabled(bool)

	Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform)

	FlagForFiltering()

	GetWorldManifold(worldManifold *B2WorldManifold)
}

type Contact struct {
	M_flags uint32

	// World pool and list pointers.
	M_prev ContactInterface // should be backed by a pointer
	M_next ContactInterface // should be backed by a pointer

	// Nodes for connecting bodies.
	M_nodeA *ContactEdge
	M_nodeB *ContactEdge

	M_fixtureA *Fixture
	M_fixtureB *Fixture

	M_indexA int
	M_indexB int

	M_manifold *B2Manifold

	M_toiCount     int
	M_toi          float64
	M_friction     float64
	M_restitution  float64
	M_tangentSpeed float64
}

func (contact Contact) GetFlags() uint32 {
	return contact.M_flags
}

func (contact *Contact) SetFlags(flags uint32) {
	contact.M_flags = flags
}

func (contact Contact) GetPrev() ContactInterface {
	return contact.M_prev
}

func (contact *Contact) SetPrev(prev ContactInterface) {
	contact.M_prev = prev
}

func (contact Contact) GetNext() ContactInterface {
	return contact.M_next
}

func (contact *Contact) SetNext(next ContactInterface) {
	contact.M_next = next
}

func (contact Contact) GetNodeA() *ContactEdge {
	return contact.M_nodeA
}

func (contact *Contact) SetNodeA(node *ContactEdge) {
	contact.M_nodeA = node
}

func (contact Contact) GetNodeB() *ContactEdge {
	return contact.M_nodeB
}

func (contact *Contact) SetNodeB(node *ContactEdge) {
	contact.M_nodeB = node
}

func (contact Contact) GetFixtureA() *Fixture {
	return contact.M_fixtureA
}

func (contact *Contact) SetFixtureA(fixture *Fixture) {
	contact.M_fixtureA = fixture
}

func (contact Contact) GetFixtureB() *Fixture {
	return contact.M_fixtureB
}

func (contact *Contact) SetFixtureB(fixture *Fixture) {
	contact.M_fixtureB = fixture
}

func (contact Contact) GetChildIndexA() int {
	return contact.M_indexA
}

func (contact *Contact) SetChildIndexA(index int) {
	contact.M_indexA = index
}

func (contact Contact) GetChildIndexB() int {
	return contact.M_indexB
}

func (contact *Contact) SetChildIndexB(index int) {
	contact.M_indexB = index
}

func (contact Contact) GetManifold() *B2Manifold {
	return contact.M_manifold
}

func (contact *Contact) SetManifold(manifold *B2Manifold) {
	contact.M_manifold = manifold
}

func (contact Contact) GetTOICount() int {
	return contact.M_toiCount
}

func (contact *Contact) SetTOICount(toiCount int) {
	contact.M_toiCount = toiCount
}

func (contact Contact) GetTOI() float64 {
	return contact.M_toi
}

func (contact *Contact) SetTOI(toi float64) {
	contact.M_toi = toi
}

func (contact Contact) GetFriction() float64 {
	return contact.M_friction
}

func (contact *Contact) SetFriction(friction float64) {
	contact.M_friction = friction
}

func (contact *Contact) ResetFriction() {
	contact.M_friction = MixFriction(contact.M_fixtureA.M_friction, contact.M_fixtureB.M_friction)
}

func (contact Contact) GetRestitution() float64 {
	return contact.M_restitution
}

func (contact *Contact) SetRestitution(restitution float64) {
	contact.M_restitution = restitution
}

func (contact *Contact) ResetRestitution() {
	contact.M_restitution = MixRestitution(contact.M_fixtureA.M_restitution, contact.M_fixtureB.M_restitution)
}

func (contact Contact) GetTangentSpeed() float64 {
	return contact.M_tangentSpeed
}

func (contact *Contact) SetTangentSpeed(speed float64) {
	contact.M_tangentSpeed = speed
}

func (contact Contact) GetWorldManifold(worldManifold *B2WorldManifold) {
	bodyA := contact.M_fixtureA.GetBody()
	bodyB := contact.M_fixtureB.GetBody()
	shapeA := contact.M_fixtureA.GetShape()
	shapeB := contact.M_fixtureB.GetShape()

	worldManifold.Initialize(contact.M_manifold, bodyA.GetTransform(), shapeA.GetRadius(), bodyB.GetTransform(), shapeB.GetRadius())
}

func (contact *Contact) SetEnabled(flag bool) {
	if flag {
		contact.M_flags |= ContactFlags.Enabled
	} else {
		contact.M_flags &= ^ContactFlags.Enabled
	}
}

func (contact Contact) IsEnabled() bool {
	return (contact.M_flags & ContactFlags.Enabled) == ContactFlags.Enabled
}

func (contact Contact) IsTouching() bool {
	return (contact.M_flags & ContactFlags.Touching) == ContactFlags.Touching
}

func (contact *Contact) FlagForFiltering() {
	contact.M_flags |= ContactFlags.Filter
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Contact.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func ContactInitializeRegisters() {
	s_registers = make([][]ContactRegister, ShapeType.Count)
	for i := 0; i < int(ShapeType.Count); i++ {
		s_registers[i] = make([]ContactRegister, ShapeType.Count)
	}

	AddType(CircleContact_Create, CircleContact_Destroy, ShapeType.Circle, ShapeType.Circle)
	AddType(PolygonAndCircleContact_Create, PolygonAndCircleContact_Destroy, ShapeType.Polygon, ShapeType.Circle)
	AddType(PolygonContact_Create, PolygonContact_Destroy, ShapeType.Polygon, ShapeType.Polygon)
	AddType(EdgeAndCircleContact_Create, EdgeAndCircleContact_Destroy, ShapeType.Edge, ShapeType.Circle)
	AddType(EdgeAndPolygonContact_Create, EdgeAndPolygonContact_Destroy, ShapeType.Edge, ShapeType.Polygon)
	AddType(ChainAndCircleContact_Create, ChainAndCircleContact_Destroy, ShapeType.Chain, ShapeType.Circle)
	AddType(ChainAndPolygonContact_Create, ChainAndPolygonContact_Destroy, ShapeType.Chain, ShapeType.Polygon)
}

func AddType(createFcn ContactCreateFcn, destroyFcn ContactDestroyFcn, type1 uint8, type2 uint8) {
	assert(type1 < ShapeType.Count)
	assert(type2 < ShapeType.Count)

	s_registers[type1][type2].CreateFcn = createFcn
	s_registers[type1][type2].DestroyFcn = destroyFcn
	s_registers[type1][type2].Primary = true

	if type1 != type2 {
		s_registers[type2][type1].CreateFcn = createFcn
		s_registers[type2][type1].DestroyFcn = destroyFcn
		s_registers[type2][type1].Primary = false
	}
}

func ContactFactory(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) ContactInterface { // returned contact should be a pointer
	if !s_initialized {
		ContactInitializeRegisters()
		s_initialized = true
	}

	type1 := fixtureA.GetType()
	type2 := fixtureB.GetType()

	assert(type1 < ShapeType.Count)
	assert(type2 < ShapeType.Count)

	createFcn := s_registers[type1][type2].CreateFcn
	if createFcn != nil {
		if s_registers[type1][type2].Primary {
			return createFcn(fixtureA, indexA, fixtureB, indexB)
		} else {
			return createFcn(fixtureB, indexB, fixtureA, indexA)
		}
	}

	return nil
}

func ContactDestroy(contact ContactInterface) {
	assert(s_initialized)

	fixtureA := contact.GetFixtureA()
	fixtureB := contact.GetFixtureB()

	if contact.GetManifold().PointCount > 0 && !fixtureA.IsSensor() && !fixtureB.IsSensor() {
		fixtureA.GetBody().SetAwake(true)
		fixtureB.GetBody().SetAwake(true)
	}

	typeA := fixtureA.GetType()
	typeB := fixtureB.GetType()

	assert(typeA < ShapeType.Count)
	assert(typeB < ShapeType.Count)

	destroyFcn := s_registers[typeA][typeB].DestroyFcn
	destroyFcn(contact)
}

func MakeContact(fA *Fixture, indexA int, fB *Fixture, indexB int) Contact {
	contact := Contact{}
	contact.M_flags = ContactFlags.Enabled

	contact.M_fixtureA = fA
	contact.M_fixtureB = fB

	contact.M_indexA = indexA
	contact.M_indexB = indexB

	contact.M_manifold = NewB2Manifold()
	contact.M_manifold.PointCount = 0

	contact.M_prev = nil
	contact.M_next = nil

	contact.M_nodeA = NewContactEdge()

	contact.M_nodeA.Contact = nil
	contact.M_nodeA.Prev = nil
	contact.M_nodeA.Next = nil
	contact.M_nodeA.Other = nil

	contact.M_nodeB = NewContactEdge()

	contact.M_nodeB.Contact = nil
	contact.M_nodeB.Prev = nil
	contact.M_nodeB.Next = nil
	contact.M_nodeB.Other = nil

	contact.M_toiCount = 0

	contact.M_friction = MixFriction(contact.M_fixtureA.M_friction, contact.M_fixtureB.M_friction)
	contact.M_restitution = MixRestitution(contact.M_fixtureA.M_restitution, contact.M_fixtureB.M_restitution)

	contact.M_tangentSpeed = 0.0

	return contact
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
func ContactUpdate(contact ContactInterface, listener B2ContactListenerInterface) {
	oldManifold := *contact.GetManifold()

	// Re-enable this contact.
	contact.SetFlags(contact.GetFlags() | ContactFlags.Enabled)

	touching := false
	wasTouching := (contact.GetFlags() & ContactFlags.Touching) == ContactFlags.Touching

	sensorA := contact.GetFixtureA().IsSensor()
	sensorB := contact.GetFixtureB().IsSensor()
	sensor := sensorA || sensorB

	bodyA := contact.GetFixtureA().GetBody()
	bodyB := contact.GetFixtureB().GetBody()
	xfA := bodyA.GetTransform()
	xfB := bodyB.GetTransform()

	// Is this contact a sensor?
	if sensor {
		shapeA := contact.GetFixtureA().GetShape()
		shapeB := contact.GetFixtureB().GetShape()
		touching = B2TestOverlapShapes(shapeA, contact.GetChildIndexA(), shapeB, contact.GetChildIndexB(), xfA, xfB)

		// Sensors don't generate manifolds.
		contact.GetManifold().PointCount = 0
	} else {
		// *Contact is extended by specialized contact structs and mentionned by ContactInterface but not implemented on specialized structs
		// Thus when
		// spew.Dump("AVANT", contact.GetManifold())
		contact.Evaluate(contact.GetManifold(), xfA, xfB) // should be evaluated on specialisations of contact (like CircleContact)
		// spew.Dump("APRES", contact.GetManifold())
		touching = contact.GetManifold().PointCount > 0

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for i := 0; i < contact.GetManifold().PointCount; i++ {
			mp2 := &contact.GetManifold().Points[i]
			mp2.NormalImpulse = 0.0
			mp2.TangentImpulse = 0.0
			id2 := mp2.Id

			for j := 0; j < oldManifold.PointCount; j++ {
				mp1 := &oldManifold.Points[j]

				if mp1.Id.Key() == id2.Key() {
					mp2.NormalImpulse = mp1.NormalImpulse
					mp2.TangentImpulse = mp1.TangentImpulse
					break
				}
			}
		}

		if touching != wasTouching {
			bodyA.SetAwake(true)
			bodyB.SetAwake(true)
		}
	}

	if touching {
		contact.SetFlags(contact.GetFlags() | ContactFlags.Touching)
	} else {
		contact.SetFlags(contact.GetFlags() & ^ContactFlags.Touching)
	}

	if !wasTouching && touching && listener != nil {
		listener.BeginContact(contact)
	}

	if wasTouching && !touching && listener != nil {
		listener.EndContact(contact)
	}

	if !sensor && touching && listener != nil {
		listener.PreSolve(contact, oldManifold)
	}
}
