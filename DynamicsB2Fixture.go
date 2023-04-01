package box2d

import (
	"fmt"
)

// This holds contact filtering data.
type B2Filter struct {
	/// The collision category bits. Normally you would just set one bit.
	CategoryBits uint16

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	MaskBits uint16

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). Zero means no collision group. Non-zero group
	/// filtering always wins against the mask bits.
	GroupIndex int16
}

func MakeB2Filter() B2Filter {
	return B2Filter{
		CategoryBits: 0x0001,
		MaskBits:     0xFFFF,
		GroupIndex:   0,
	}
}

// A fixture definition is used to create a fixture. This class defines an
// abstract fixture definition. You can reuse fixture definitions safely.
type FixtureDef struct {
	/// The shape, this must be set. The shape will be cloned, so you
	/// can create the shape on the stack.
	Shape B2ShapeInterface

	/// Use this to store application specific fixture data.
	UserData interface{}

	/// The friction coefficient, usually in the range [0,1].
	Friction float64

	/// The restitution (elasticity) usually in the range [0,1].
	Restitution float64

	/// The density, usually in kg/m^2.
	Density float64

	/// A sensor shape collects contact information but never generates a collision
	/// response.
	IsSensor bool

	/// Contact filtering data.
	Filter B2Filter
}

// The constructor sets the default fixture definition values.
func MakeFixtureDef() FixtureDef {
	return FixtureDef{
		Friction: 0.2,
		Filter:   MakeB2Filter(),
	}
}

// This proxy is used internally to connect fixtures to the broad-phase.
type FixtureProxy struct {
	Aabb       B2AABB
	Fixture    *Fixture
	ChildIndex int
	ProxyId    int
}

// A fixture is used to attach a shape to a body for collision detection. A fixture
// inherits its transform from its parent. Fixtures hold additional non-geometric data
// such as friction, collision filters, etc.
// Fixtures are created via b2Body::CreateFixture.
// @warning you cannot reuse fixtures.
type Fixture struct {
	M_density float64

	M_next *Fixture
	M_body *Body

	M_shape B2ShapeInterface

	M_friction    float64
	M_restitution float64

	M_proxies    []FixtureProxy
	M_proxyCount int

	M_filter B2Filter

	M_isSensor bool

	M_userData interface{}
}

func MakeFixture() Fixture {
	return Fixture{
		M_filter: MakeB2Filter(),
	}
}

func NewFixture() *Fixture {
	res := MakeFixture()
	return &res
}

func (fix Fixture) GetType() uint8 {
	return fix.M_shape.GetType()
}

func (fix Fixture) GetShape() B2ShapeInterface {
	return fix.M_shape
}

func (fix Fixture) IsSensor() bool {
	return fix.M_isSensor
}

func (fix Fixture) GetFilterData() B2Filter {
	return fix.M_filter
}

func (fix Fixture) GetUserData() interface{} {
	return fix.M_userData
}

func (fix *Fixture) SetUserData(data interface{}) {
	fix.M_userData = data
}

func (fix Fixture) GetBody() *Body {
	return fix.M_body
}

func (fix Fixture) GetNext() *Fixture {
	return fix.M_next
}

func (fix *Fixture) SetDensity(density float64) {
	assert(IsValid(density) && density >= 0.0)
	fix.M_density = density
}

func (fix Fixture) GetDensity() float64 {
	return fix.M_density
}

func (fix Fixture) GetFriction() float64 {
	return fix.M_friction
}

func (fix *Fixture) SetFriction(friction float64) {
	fix.M_friction = friction
}

func (fix Fixture) GetRestitution() float64 {
	return fix.M_restitution
}

func (fix *Fixture) SetRestitution(restitution float64) {
	fix.M_restitution = restitution
}

func (fix Fixture) TestPoint(p Vec2) bool {
	return fix.M_shape.TestPoint(fix.M_body.GetTransform(), p)
}

func (fix Fixture) RayCast(output *B2RayCastOutput, input B2RayCastInput, childIndex int) bool {
	return fix.M_shape.RayCast(output, input, fix.M_body.GetTransform(), childIndex)
}

func (fix Fixture) GetMassData() B2MassData {
	return fix.M_shape.ComputeMass(fix.M_density)
}

func (fix Fixture) GetAABB(childIndex int) B2AABB {
	assert(0 <= childIndex && childIndex < fix.M_proxyCount)
	return fix.M_proxies[childIndex].Aabb
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Fixture.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func (fix *Fixture) Create(body *Body, def *FixtureDef) {
	fix.M_userData = def.UserData
	fix.M_friction = def.Friction
	fix.M_restitution = def.Restitution

	fix.M_body = body
	fix.M_next = nil

	fix.M_filter = def.Filter

	fix.M_isSensor = def.IsSensor

	fix.M_shape = def.Shape.Clone()

	// Reserve proxy space
	childCount := fix.M_shape.GetChildCount()
	fix.M_proxies = make([]FixtureProxy, childCount)

	for i := 0; i < childCount; i++ {
		fix.M_proxies[i].Fixture = nil
		fix.M_proxies[i].ProxyId = E_nullProxy
	}
	fix.M_proxyCount = 0

	fix.M_density = def.Density
}

func (fix *Fixture) Destroy() {
	// The proxies must be destroyed before calling this.
	assert(fix.M_proxyCount == 0)

	// Free the proxy array.
	fix.M_proxies = nil

	// Free the child shape.
	switch fix.M_shape.GetType() {
	case B2Shape_Type.E_circle:
		s := fix.M_shape.(*CircleShape)
		s.Destroy()

	case B2Shape_Type.E_edge:
		s := fix.M_shape.(*B2EdgeShape)
		s.Destroy()

	case B2Shape_Type.E_polygon:
		s := fix.M_shape.(*B2PolygonShape)
		s.Destroy()

	case B2Shape_Type.E_chain:
		s := fix.M_shape.(*ChainShape)
		s.Destroy()

	default:
		assert(false)
	}

	fix.M_shape = nil
}

func (fix *Fixture) CreateProxies(broadPhase *B2BroadPhase, xf Transform) {
	assert(fix.M_proxyCount == 0)

	// Create proxies in the broad-phase.
	fix.M_proxyCount = fix.M_shape.GetChildCount()

	for i := 0; i < fix.M_proxyCount; i++ {
		proxy := &fix.M_proxies[i]
		proxy.Aabb = fix.M_shape.ComputeAABB(xf, i)
		proxy.ProxyId = broadPhase.CreateProxy(proxy.Aabb, proxy)
		proxy.Fixture = fix
		proxy.ChildIndex = i
	}
}

func (fix *Fixture) DestroyProxies(broadPhase *B2BroadPhase) {
	// Destroy proxies in the broad-phase.
	for i := 0; i < fix.M_proxyCount; i++ {
		proxy := &fix.M_proxies[i]
		broadPhase.DestroyProxy(proxy.ProxyId)
		proxy.ProxyId = E_nullProxy
	}

	fix.M_proxyCount = 0
}

func (fix *Fixture) Synchronize(broadPhase *B2BroadPhase, transform1 Transform, transform2 Transform) {
	if fix.M_proxyCount == 0 {
		return
	}

	for i := 0; i < fix.M_proxyCount; i++ {

		proxy := &fix.M_proxies[i]

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		aabb1 := fix.M_shape.ComputeAABB(transform1, proxy.ChildIndex)
		aabb2 := fix.M_shape.ComputeAABB(transform2, proxy.ChildIndex)

		proxy.Aabb.CombineTwoInPlace(aabb1, aabb2)

		displacement := Vec2Sub(transform2.P, transform1.P)

		broadPhase.MoveProxy(proxy.ProxyId, proxy.Aabb, displacement)
	}
}

func (fix *Fixture) SetFilterData(filter B2Filter) {
	fix.M_filter = filter
	fix.Refilter()
}

func (fix *Fixture) Refilter() {
	if fix.M_body == nil {
		return
	}

	// Flag associated contacts for filtering.
	edge := fix.M_body.GetContactList()
	for edge != nil {
		contact := edge.Contact
		fixtureA := contact.GetFixtureA()
		fixtureB := contact.GetFixtureB()
		if fixtureA == fix || fixtureB == fix {
			contact.FlagForFiltering()
		}

		edge = edge.Next
	}

	world := fix.M_body.GetWorld()

	if world == nil {
		return
	}

	// Touch each proxy so that new pairs may be created
	broadPhase := &world.M_contactManager.M_broadPhase
	for i := 0; i < fix.M_proxyCount; i++ {
		broadPhase.TouchProxy(fix.M_proxies[i].ProxyId)
	}
}

func (fix *Fixture) SetSensor(sensor bool) {
	if sensor != fix.M_isSensor {
		fix.M_body.SetAwake(true)
		fix.M_isSensor = sensor
	}
}

func (fix *Fixture) Dump(bodyIndex int) {
	fmt.Printf("    b2FixtureDef fd;\n")
	fmt.Printf("    fd.friction = %.15f;\n", fix.M_friction)
	fmt.Printf("    fd.restitution = %.15f;\n", fix.M_restitution)
	fmt.Printf("    fd.density = %.15f;\n", fix.M_density)
	fmt.Printf("    fd.isSensor = bool(%v);\n", fix.M_isSensor)
	fmt.Printf("    fd.filter.categoryBits = uint16(%d);\n", fix.M_filter.CategoryBits)
	fmt.Printf("    fd.filter.maskBits = uint16(%d);\n", fix.M_filter.MaskBits)
	fmt.Printf("    fd.filter.groupIndex = int16(%d);\n", fix.M_filter.GroupIndex)

	switch fix.M_shape.GetType() {
	case B2Shape_Type.E_circle:
		s := fix.M_shape.(*CircleShape)
		fmt.Printf("    b2CircleShape shape;\n")
		fmt.Printf("    shape.m_radius = %.15f;\n", s.M_radius)
		fmt.Printf("    shape.m_p.Set(%.15f, %.15f);\n", s.M_p.X, s.M_p.Y)

	case B2Shape_Type.E_edge:
		s := fix.M_shape.(*B2EdgeShape)
		fmt.Printf("    b2EdgeShape shape;\n")
		fmt.Printf("    shape.m_radius = %.15f;\n", s.M_radius)
		fmt.Printf("    shape.m_vertex0.Set(%.15f, %.15f);\n", s.M_vertex0.X, s.M_vertex0.Y)
		fmt.Printf("    shape.m_vertex1.Set(%.15f, %.15f);\n", s.M_vertex1.X, s.M_vertex1.Y)
		fmt.Printf("    shape.m_vertex2.Set(%.15f, %.15f);\n", s.M_vertex2.X, s.M_vertex2.Y)
		fmt.Printf("    shape.m_vertex3.Set(%.15f, %.15f);\n", s.M_vertex3.X, s.M_vertex3.Y)
		fmt.Printf("    shape.m_hasVertex0 = bool(%v);\n", s.M_hasVertex0)
		fmt.Printf("    shape.m_hasVertex3 = bool(%v);\n", s.M_hasVertex3)

	case B2Shape_Type.E_polygon:
		s := fix.M_shape.(*B2PolygonShape)
		fmt.Printf("    b2PolygonShape shape;\n")
		fmt.Printf("    b2Vec2 vs[%d];\n", maxPolygonVertices)
		for i := 0; i < s.M_count; i++ {
			fmt.Printf("    vs[%d].Set(%.15f, %.15f);\n", i, s.M_vertices[i].X, s.M_vertices[i].Y)
		}
		fmt.Printf("    shape.Set(vs, %d);\n", s.M_count)

	case B2Shape_Type.E_chain:
		s := fix.M_shape.(*ChainShape)
		fmt.Printf("    b2ChainShape shape;\n")
		fmt.Printf("    b2Vec2 vs[%d];\n", s.M_count)
		for i := 0; i < s.M_count; i++ {
			fmt.Printf("    vs[%d].Set(%.15f, %.15f);\n", i, s.M_vertices[i].X, s.M_vertices[i].Y)
		}
		fmt.Printf("    shape.CreateChain(vs, %d);\n", s.M_count)
		fmt.Printf("    shape.m_prevVertex.Set(%.15f, %.15f);\n", s.M_prevVertex.X, s.M_prevVertex.Y)
		fmt.Printf("    shape.m_nextVertex.Set(%.15f, %.15f);\n", s.M_nextVertex.X, s.M_nextVertex.Y)
		fmt.Printf("    shape.m_hasPrevVertex = bool(%v);\n", s.M_hasPrevVertex)
		fmt.Printf("    shape.m_hasNextVertex = bool(%v);\n", s.M_hasNextVertex)

	default:
		return
	}

	fmt.Print("\n")
	fmt.Print("    fd.shape = &shape;\n")
	fmt.Print("\n")
	fmt.Printf("    bodies[%d].CreateFixture(&fd);\n", bodyIndex)
}
