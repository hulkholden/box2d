package box2d

type EdgeAndPolygonContact struct {
	Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ContactEdgeAndPolygon.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func EdgeAndPolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == B2Shape_Type.E_edge)
	assert(fixtureB.GetType() == B2Shape_Type.E_polygon)
	res := &EdgeAndPolygonContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func EdgeAndPolygonContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *EdgeAndPolygonContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	B2CollideEdgeAndPolygon(
		manifold,
		contact.GetFixtureA().GetShape().(*EdgeShape), xfA,
		contact.GetFixtureB().GetShape().(*PolygonShape), xfB,
	)
}
