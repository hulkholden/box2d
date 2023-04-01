package box2d

type B2PolygonAndCircleContact struct {
	Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ContactPolygonAndCircle.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func B2PolygonAndCircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == B2Shape_Type.E_polygon)
	assert(fixtureB.GetType() == B2Shape_Type.E_circle)
	res := &B2PolygonAndCircleContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func B2PolygonAndCircleContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *B2PolygonAndCircleContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	B2CollidePolygonAndCircle(
		manifold,
		contact.GetFixtureA().GetShape().(*B2PolygonShape), xfA,
		contact.GetFixtureB().GetShape().(*B2CircleShape), xfB,
	)
}
