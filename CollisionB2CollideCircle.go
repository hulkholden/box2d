package box2d

func CollideCircles(manifold *B2Manifold, circleA *CircleShape, xfA Transform, circleB *CircleShape, xfB Transform) {
	manifold.PointCount = 0

	pA := TransformVec2Mul(xfA, circleA.M_p)
	pB := TransformVec2Mul(xfB, circleB.M_p)

	d := Vec2Sub(pB, pA)
	distSqr := Vec2Dot(d, d)
	rA := circleA.M_radius
	rB := circleB.M_radius
	radius := rA + rB
	if distSqr > radius*radius {
		return
	}

	manifold.Type = ManifoldType.Circles
	manifold.LocalPoint = circleA.M_p
	manifold.LocalNormal.SetZero()
	manifold.PointCount = 1

	manifold.Points[0].LocalPoint = circleB.M_p
	manifold.Points[0].Id.SetKey(0)
}

func CollidePolygonAndCircle(manifold *B2Manifold, polygonA *PolygonShape, xfA Transform, circleB *CircleShape, xfB Transform) {
	manifold.PointCount = 0

	// Compute circle position in the frame of the polygon.
	c := TransformVec2Mul(xfB, circleB.M_p)
	cLocal := TransformVec2MulT(xfA, c)

	// Find the min separating edge.
	normalIndex := 0
	separation := -maxFloat
	radius := polygonA.M_radius + circleB.M_radius
	vertexCount := polygonA.M_count
	vertices := polygonA.M_vertices
	normals := polygonA.M_normals

	for i := 0; i < vertexCount; i++ {
		s := Vec2Dot(normals[i], Vec2Sub(cLocal, vertices[i]))

		if s > radius {
			// Early out.
			return
		}

		if s > separation {
			separation = s
			normalIndex = i
		}
	}

	// Vertices that subtend the incident face.
	vertIndex1 := normalIndex
	vertIndex2 := 0
	if vertIndex1+1 < vertexCount {
		vertIndex2 = vertIndex1 + 1
	}

	v1 := vertices[vertIndex1]
	v2 := vertices[vertIndex2]

	// If the center is inside the polygon ...
	if separation < epsilon {
		manifold.PointCount = 1
		manifold.Type = ManifoldType.FaceA
		manifold.LocalNormal = normals[normalIndex]
		manifold.LocalPoint = Vec2MulScalar(0.5, Vec2Add(v1, v2))
		manifold.Points[0].LocalPoint = circleB.M_p
		manifold.Points[0].Id.SetKey(0)
		return
	}

	// Compute barycentric coordinates
	u1 := Vec2Dot(Vec2Sub(cLocal, v1), Vec2Sub(v2, v1))
	u2 := Vec2Dot(Vec2Sub(cLocal, v2), Vec2Sub(v1, v2))
	if u1 <= 0.0 {
		if Vec2DistanceSquared(cLocal, v1) > radius*radius {
			return
		}

		manifold.PointCount = 1
		manifold.Type = ManifoldType.FaceA
		manifold.LocalNormal = Vec2Sub(cLocal, v1)
		manifold.LocalNormal.Normalize()
		manifold.LocalPoint = v1
		manifold.Points[0].LocalPoint = circleB.M_p
		manifold.Points[0].Id.SetKey(0)
	} else if u2 <= 0.0 {
		if Vec2DistanceSquared(cLocal, v2) > radius*radius {
			return
		}

		manifold.PointCount = 1
		manifold.Type = ManifoldType.FaceA
		manifold.LocalNormal = Vec2Sub(cLocal, v2)
		manifold.LocalNormal.Normalize()
		manifold.LocalPoint = v2
		manifold.Points[0].LocalPoint = circleB.M_p
		manifold.Points[0].Id.SetKey(0)
	} else {
		faceCenter := Vec2MulScalar(0.5, Vec2Add(v1, v2))
		s := Vec2Dot(Vec2Sub(cLocal, faceCenter), normals[vertIndex1])
		if s > radius {
			return
		}

		manifold.PointCount = 1
		manifold.Type = ManifoldType.FaceA
		manifold.LocalNormal = normals[vertIndex1]
		manifold.LocalPoint = faceCenter
		manifold.Points[0].LocalPoint = circleB.M_p
		manifold.Points[0].Id.SetKey(0)
	}
}
