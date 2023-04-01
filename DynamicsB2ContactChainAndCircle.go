package box2d

type B2ChainAndCircleContact struct {
	B2Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ChainAndCircleContact.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func B2ChainAndCircleContact_Create(fixtureA *B2Fixture, indexA int, fixtureB *B2Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == B2Shape_Type.E_chain)
	assert(fixtureB.GetType() == B2Shape_Type.E_circle)
	res := &B2ChainAndCircleContact{
		B2Contact: MakeB2Contact(fixtureA, indexA, fixtureB, indexB),
	}

	return res
}

func B2ChainAndCircleContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *B2ChainAndCircleContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	chain := contact.GetFixtureA().GetShape().(*B2ChainShape)
	edge := MakeB2EdgeShape()
	chain.GetChildEdge(&edge, contact.M_indexA)
	B2CollideEdgeAndCircle(
		manifold,
		&edge, xfA,
		contact.GetFixtureB().GetShape().(*B2CircleShape), xfB,
	)
}
