package box2d

type B2CircleContact struct {
	Contact
}

func B2CircleContact_Create(fixtureA *B2Fixture, indexA int, fixtureB *B2Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == B2Shape_Type.E_circle)
	assert(fixtureB.GetType() == B2Shape_Type.E_circle)
	res := &B2CircleContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func B2CircleContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *B2CircleContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	B2CollideCircles(
		manifold,
		contact.GetFixtureA().GetShape().(*B2CircleShape), xfA,
		contact.GetFixtureB().GetShape().(*B2CircleShape), xfB,
	)
}
