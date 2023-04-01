package box2d

type B2EdgeAndCircleContact struct {
	Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ContactEdgeAndCircle.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func B2EdgeAndCircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == B2Shape_Type.E_edge)
	assert(fixtureB.GetType() == B2Shape_Type.E_circle)
	res := &B2EdgeAndCircleContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func B2EdgeAndCircleContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *B2EdgeAndCircleContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	B2CollideEdgeAndCircle(
		manifold,
		contact.GetFixtureA().GetShape().(*EdgeShape), xfA,
		contact.GetFixtureB().GetShape().(*CircleShape), xfB,
	)
}
