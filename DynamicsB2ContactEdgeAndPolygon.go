package box2d

type B2EdgeAndPolygonContact struct {
	B2Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ContactEdgeAndPolygon.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func B2EdgeAndPolygonContact_Create(fixtureA *B2Fixture, indexA int, fixtureB *B2Fixture, indexB int) B2ContactInterface {
	assert(fixtureA.GetType() == B2Shape_Type.E_edge)
	assert(fixtureB.GetType() == B2Shape_Type.E_polygon)
	res := &B2EdgeAndPolygonContact{
		B2Contact: MakeB2Contact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func B2EdgeAndPolygonContact_Destroy(contact B2ContactInterface) { // should be a pointer
}

func (contact *B2EdgeAndPolygonContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	B2CollideEdgeAndPolygon(
		manifold,
		contact.GetFixtureA().GetShape().(*B2EdgeShape), xfA,
		contact.GetFixtureB().GetShape().(*B2PolygonShape), xfB,
	)
}
