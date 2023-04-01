package box2d

type B2PolygonContact struct {
	Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ContactPolygon.go
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func B2PolygonContact_Create(fixtureA *B2Fixture, indexA int, fixtureB *B2Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == B2Shape_Type.E_polygon)
	assert(fixtureB.GetType() == B2Shape_Type.E_polygon)
	res := &B2PolygonContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func B2PolygonContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *B2PolygonContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	B2CollidePolygons(
		manifold,
		contact.GetFixtureA().GetShape().(*B2PolygonShape), xfA,
		contact.GetFixtureB().GetShape().(*B2PolygonShape), xfB,
	)
}
