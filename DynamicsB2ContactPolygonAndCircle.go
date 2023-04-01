package box2d

type PolygonAndCircleContact struct {
	Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ContactPolygonAndCircle.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func PolygonAndCircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == ShapeType.Polygon)
	assert(fixtureB.GetType() == ShapeType.Circle)
	res := &PolygonAndCircleContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func PolygonAndCircleContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *PolygonAndCircleContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	B2CollidePolygonAndCircle(
		manifold,
		contact.GetFixtureA().GetShape().(*PolygonShape), xfA,
		contact.GetFixtureB().GetShape().(*CircleShape), xfB,
	)
}
