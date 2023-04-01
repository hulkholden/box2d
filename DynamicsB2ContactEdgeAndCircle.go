package box2d

type EdgeAndCircleContact struct {
	Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ContactEdgeAndCircle.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func EdgeAndCircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == ShapeType.Edge)
	assert(fixtureB.GetType() == ShapeType.Circle)
	res := &EdgeAndCircleContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func EdgeAndCircleContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *EdgeAndCircleContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	CollideEdgeAndCircle(
		manifold,
		contact.GetFixtureA().GetShape().(*EdgeShape), xfA,
		contact.GetFixtureB().GetShape().(*CircleShape), xfB,
	)
}
