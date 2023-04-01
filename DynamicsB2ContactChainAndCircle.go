package box2d

type ChainAndCircleContact struct {
	Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ChainAndCircleContact.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func ChainAndCircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == B2Shape_Type.E_chain)
	assert(fixtureB.GetType() == B2Shape_Type.E_circle)
	res := &ChainAndCircleContact{
		Contact: MakeContact(fixtureA, indexA, fixtureB, indexB),
	}

	return res
}

func ChainAndCircleContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *ChainAndCircleContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	chain := contact.GetFixtureA().GetShape().(*ChainShape)
	edge := MakeEdgeShape()
	chain.GetChildEdge(&edge, contact.M_indexA)
	CollideEdgeAndCircle(
		manifold,
		&edge, xfA,
		contact.GetFixtureB().GetShape().(*CircleShape), xfB,
	)
}
