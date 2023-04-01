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
	assert(fixtureA.GetType() == ShapeType.Chain)
	assert(fixtureB.GetType() == ShapeType.Polygon)
	res := &ChainAndPolygonContact{
		Contact: MakeContact(fixtureA, indexA, fixtureB, indexB),
	}

	return res
}

func ChainAndPolygonContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *ChainAndPolygonContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	chain := contact.GetFixtureA().GetShape().(*ChainShape)
	edge := MakeEdgeShape()
	chain.GetChildEdge(&edge, contact.M_indexA)
	CollideEdgeAndPolygon(manifold, &edge, xfA, contact.GetFixtureB().GetShape().(*PolygonShape), xfB)
}
