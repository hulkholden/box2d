package box2d

// Find the max separation between poly1 and poly2 using edge normals from poly1.
func B2FindMaxSeparation(edgeIndex *int, poly1 *PolygonShape, xf1 Transform, poly2 *PolygonShape, xf2 Transform) float64 {
	count1 := poly1.M_count
	count2 := poly2.M_count
	n1s := poly1.M_normals
	v1s := poly1.M_vertices
	v2s := poly2.M_vertices

	xf := TransformMulT(xf2, xf1)

	bestIndex := 0
	maxSeparation := -maxFloat
	for i := 0; i < count1; i++ {
		// Get poly1 normal in frame2.
		n := RotVec2Mul(xf.Q, n1s[i])
		v1 := TransformVec2Mul(xf, v1s[i])

		// Find deepest point for normal i.
		si := maxFloat
		for j := 0; j < count2; j++ {
			sij := Vec2Dot(n, Vec2Sub(v2s[j], v1))
			if sij < si {
				si = sij
			}
		}

		if si > maxSeparation {
			maxSeparation = si
			bestIndex = i
		}
	}

	*edgeIndex = bestIndex
	return maxSeparation
}

func B2FindIncidentEdge(c []B2ClipVertex, poly1 *PolygonShape, xf1 Transform, edge1 int, poly2 *PolygonShape, xf2 Transform) {
	normals1 := poly1.M_normals

	count2 := poly2.M_count
	vertices2 := poly2.M_vertices
	normals2 := poly2.M_normals

	assert(0 <= edge1 && edge1 < poly1.M_count)

	// Get the normal of the reference edge in poly2's frame.
	normal1 := RotVec2MulT(xf2.Q, RotVec2Mul(xf1.Q, normals1[edge1]))

	// Find the incident edge on poly2.
	index := 0
	minDot := maxFloat
	for i := 0; i < count2; i++ {
		dot := Vec2Dot(normal1, normals2[i])
		if dot < minDot {
			minDot = dot
			index = i
		}
	}

	// Build the clip vertices for the incident edge.
	i1 := index
	i2 := 0
	if i1+1 < count2 {
		i2 = i1 + 1
	}

	c[0].V = TransformVec2Mul(xf2, vertices2[i1])
	c[0].Id.IndexA = uint8(edge1)
	c[0].Id.IndexB = uint8(i1)
	c[0].Id.TypeA = ContactFeatureType.Face
	c[0].Id.TypeB = ContactFeatureType.Vertex

	c[1].V = TransformVec2Mul(xf2, vertices2[i2])
	c[1].Id.IndexA = uint8(edge1)
	c[1].Id.IndexB = uint8(i2)
	c[1].Id.TypeA = ContactFeatureType.Face
	c[1].Id.TypeB = ContactFeatureType.Vertex
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
func B2CollidePolygons(manifold *B2Manifold, polyA *PolygonShape, xfA Transform, polyB *PolygonShape, xfB Transform) {
	manifold.PointCount = 0
	totalRadius := polyA.M_radius + polyB.M_radius

	edgeA := 0
	separationA := B2FindMaxSeparation(&edgeA, polyA, xfA, polyB, xfB)
	if separationA > totalRadius {
		return
	}

	edgeB := 0
	separationB := B2FindMaxSeparation(&edgeB, polyB, xfB, polyA, xfA)
	if separationB > totalRadius {
		return
	}

	var poly1 *PolygonShape // reference polygon
	var poly2 *PolygonShape // incident polygon

	var xf1, xf2 Transform

	edge1 := 0 // reference edge
	var flip uint8
	k_tol := 0.1 * linearSlop

	if separationB > separationA+k_tol {
		poly1 = polyB
		poly2 = polyA
		xf1 = xfB
		xf2 = xfA
		edge1 = edgeB
		manifold.Type = ManifoldType.FaceB
		flip = 1
	} else {
		poly1 = polyA
		poly2 = polyB
		xf1 = xfA
		xf2 = xfB
		edge1 = edgeA
		manifold.Type = ManifoldType.FaceA
		flip = 0
	}

	incidentEdge := make([]B2ClipVertex, 2)
	B2FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2)

	count1 := poly1.M_count
	vertices1 := poly1.M_vertices

	iv1 := edge1
	iv2 := 0
	if edge1+1 < count1 {
		iv2 = edge1 + 1
	}

	v11 := vertices1[iv1]
	v12 := vertices1[iv2]

	localTangent := Vec2Sub(v12, v11)
	localTangent.Normalize()

	localNormal := Vec2CrossVectorScalar(localTangent, 1.0)
	planePoint := Vec2MulScalar(0.5, Vec2Add(v11, v12))

	tangent := RotVec2Mul(xf1.Q, localTangent)
	normal := Vec2CrossVectorScalar(tangent, 1.0)

	v11 = TransformVec2Mul(xf1, v11)
	v12 = TransformVec2Mul(xf1, v12)

	// Face offset.
	frontOffset := Vec2Dot(normal, v11)

	// Side offsets, extended by polytope skin thickness.
	sideOffset1 := -Vec2Dot(tangent, v11) + totalRadius
	sideOffset2 := Vec2Dot(tangent, v12) + totalRadius

	// Clip incident edge against extruded edge1 side edges.
	clipPoints1 := make([]B2ClipVertex, 2)
	clipPoints2 := make([]B2ClipVertex, 2)
	np := 0

	// Clip to box side 1
	np = ClipSegmentToLine(clipPoints1, incidentEdge, tangent.OperatorNegate(), sideOffset1, iv1)

	if np < 2 {
		return
	}

	// Clip to negative box side 1
	np = ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2)

	if np < 2 {
		return
	}

	// Now clipPoints2 contains the clipped points.
	manifold.LocalNormal = localNormal
	manifold.LocalPoint = planePoint

	pointCount := 0
	for i := 0; i < maxManifoldPoints; i++ {
		separation := Vec2Dot(normal, clipPoints2[i].V) - frontOffset

		if separation <= totalRadius {
			cp := &manifold.Points[pointCount]
			cp.LocalPoint = TransformVec2MulT(xf2, clipPoints2[i].V)
			cp.Id = clipPoints2[i].Id
			if flip != 0 {
				// Swap features
				cf := cp.Id
				cp.Id.IndexA = cf.IndexB
				cp.Id.IndexB = cf.IndexA
				cp.Id.TypeA = cf.TypeB
				cp.Id.TypeB = cf.TypeA
			}
			pointCount++
		}
	}

	manifold.PointCount = pointCount
}
