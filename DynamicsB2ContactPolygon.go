package box2d

type PolygonContact struct {
	Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ContactPolygon.go
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func PolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == B2Shape_Type.E_polygon)
	assert(fixtureB.GetType() == B2Shape_Type.E_polygon)
	res := &PolygonContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func PolygonContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *PolygonContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	B2CollidePolygons(
		manifold,
		contact.GetFixtureA().GetShape().(*PolygonShape), xfA,
		contact.GetFixtureB().GetShape().(*PolygonShape), xfB,
	)
}
