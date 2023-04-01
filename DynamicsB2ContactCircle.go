package box2d

type CircleContact struct {
	Contact
}

func CircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) ContactInterface {
	assert(fixtureA.GetType() == B2Shape_Type.E_circle)
	assert(fixtureB.GetType() == B2Shape_Type.E_circle)
	res := &CircleContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func CircleContact_Destroy(contact ContactInterface) { // should be a pointer
}

func (contact *CircleContact) Evaluate(manifold *B2Manifold, xfA Transform, xfB Transform) {
	B2CollideCircles(
		manifold,
		contact.GetFixtureA().GetShape().(*CircleShape), xfA,
		contact.GetFixtureB().GetShape().(*CircleShape), xfB,
	)
}
