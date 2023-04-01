package box2d

type PolygonContact struct {
	Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ContactPolygon.go
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func PolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == ShapeType.Polygon)
	assert(fixtureB.GetType() == ShapeType.Polygon)
	res := &PolygonContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func PolygonContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *PolygonContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	CollidePolygons(
		manifold,
		contact.GetFixtureA().GetShape().(*PolygonShape), xfA,
		contact.GetFixtureB().GetShape().(*PolygonShape), xfB,
	)
}
