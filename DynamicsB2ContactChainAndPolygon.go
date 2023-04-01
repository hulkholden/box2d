package box2d

type ChainAndPolygonContact struct {
	Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ChainAndPolygonContact.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func ChainAndPolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == B2Shape_Type.E_chain)
	assert(fixtureB.GetType() == B2Shape_Type.E_polygon)
	res := &ChainAndPolygonContact{
		Contact: MakeContact(fixtureA, indexA, fixtureB, indexB),
	}

	return res
}

func ChainAndPolygonContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *ChainAndPolygonContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	chain := contact.GetFixtureA().GetShape().(*ChainShape)
	edge := MakeEdgeShape()
	chain.GetChildEdge(&edge, contact.M_indexA)
	B2CollideEdgeAndPolygon(manifold, &edge, xfA, contact.GetFixtureB().GetShape().(*PolygonShape), xfB)
}
