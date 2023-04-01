package box2d

type B2EdgeAndPolygonContact struct {
	Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ContactEdgeAndPolygon.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func B2EdgeAndPolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == B2Shape_Type.E_edge)
	assert(fixtureB.GetType() == B2Shape_Type.E_polygon)
	res := &B2EdgeAndPolygonContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func B2EdgeAndPolygonContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *B2EdgeAndPolygonContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	B2CollideEdgeAndPolygon(
		manifold,
		contact.GetFixtureA().GetShape().(*B2EdgeShape), xfA,
		contact.GetFixtureB().GetShape().(*B2PolygonShape), xfB,
	)
}
