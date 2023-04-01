package box2d

type B2ChainAndPolygonContact struct {
	Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ChainAndPolygonContact.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func B2ChainAndPolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == B2Shape_Type.E_chain)
	assert(fixtureB.GetType() == B2Shape_Type.E_polygon)
	res := &B2ChainAndPolygonContact{
		Contact: MakeContact(fixtureA, indexA, fixtureB, indexB),
	}

	return res
}

func B2ChainAndPolygonContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *B2ChainAndPolygonContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	chain := contact.GetFixtureA().GetShape().(*ChainShape)
	edge := MakeB2EdgeShape()
	chain.GetChildEdge(&edge, contact.M_indexA)
	B2CollideEdgeAndPolygon(manifold, &edge, xfA, contact.GetFixtureB().GetShape().(*B2PolygonShape), xfB)
}
