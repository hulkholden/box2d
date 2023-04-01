package box2d

/// A chain shape is a free form sequence of line segments.
/// The chain has two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
/// Since there may be many vertices, they are allocated using b2Alloc.
/// Connectivity information is used to create smooth collisions.
/// WARNING: The chain will not collide properly if there are self-intersections.

// A circle shape.
type ChainShape struct {
	B2Shape

	/// The vertices. Owned by this class.
	M_vertices []Vec2

	/// The vertex count.
	M_count int

	M_prevVertex    Vec2
	M_nextVertex    Vec2
	M_hasPrevVertex bool
	M_hasNextVertex bool
}

func MakeChainShape() ChainShape {
	return ChainShape{
		B2Shape: B2Shape{
			M_type:   ShapeType.Chain,
			M_radius: polygonRadius,
		},
		M_vertices:      nil,
		M_count:         0,
		M_hasPrevVertex: false,
		M_hasNextVertex: false,
	}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// ChainShape.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func (chain *ChainShape) Destroy() {
	chain.Clear()
}

func (chain *ChainShape) Clear() {
	chain.M_vertices = nil
	chain.M_count = 0
}

func (chain *ChainShape) CreateLoop(vertices []Vec2, count int) {
	assert(chain.M_vertices == nil && chain.M_count == 0)
	assert(count >= 3)
	if count < 3 {
		return
	}

	for i := 1; i < count; i++ {
		v1 := vertices[i-1]
		v2 := vertices[i]
		// If the code crashes here, it means your vertices are too close together.
		assert(Vec2DistanceSquared(v1, v2) > linearSlop*linearSlop)
	}

	chain.M_count = count + 1
	chain.M_vertices = make([]Vec2, chain.M_count)
	copy(chain.M_vertices, vertices)

	chain.M_vertices[count] = chain.M_vertices[0]
	chain.M_prevVertex = chain.M_vertices[chain.M_count-2]
	chain.M_nextVertex = chain.M_vertices[1]
	chain.M_hasPrevVertex = true
	chain.M_hasNextVertex = true
}

func (chain *ChainShape) CreateChain(vertices []Vec2, count int) {
	assert(chain.M_vertices == nil && chain.M_count == 0)
	assert(count >= 2)
	for i := 1; i < count; i++ {
		// If the code crashes here, it means your vertices are too close together.
		assert(Vec2DistanceSquared(vertices[i-1], vertices[i]) > linearSlop*linearSlop)
	}

	chain.M_count = count
	chain.M_vertices = make([]Vec2, count)
	copy(chain.M_vertices, vertices)

	chain.M_hasPrevVertex = false
	chain.M_hasNextVertex = false

	chain.M_prevVertex.SetZero()
	chain.M_nextVertex.SetZero()
}

func (chain *ChainShape) SetPrevVertex(prevVertex Vec2) {
	chain.M_prevVertex = prevVertex
	chain.M_hasPrevVertex = true
}

func (chain *ChainShape) SetNextVertex(nextVertex Vec2) {
	chain.M_nextVertex = nextVertex
	chain.M_hasNextVertex = true
}

func (chain ChainShape) Clone() ShapeInterface {
	clone := MakeChainShape()
	clone.CreateChain(chain.M_vertices, chain.M_count)
	clone.M_prevVertex = chain.M_prevVertex
	clone.M_nextVertex = chain.M_nextVertex
	clone.M_hasPrevVertex = chain.M_hasPrevVertex
	clone.M_hasNextVertex = chain.M_hasNextVertex

	return &clone
}

func (chain ChainShape) GetChildCount() int {
	// edge count = vertex count - 1
	return chain.M_count - 1
}

func (chain ChainShape) GetChildEdge(edge *EdgeShape, index int) {
	assert(0 <= index && index < chain.M_count-1)

	edge.M_type = ShapeType.Edge
	edge.M_radius = chain.M_radius

	edge.M_vertex1 = chain.M_vertices[index+0]
	edge.M_vertex2 = chain.M_vertices[index+1]

	if index > 0 {
		edge.M_vertex0 = chain.M_vertices[index-1]
		edge.M_hasVertex0 = true
	} else {
		edge.M_vertex0 = chain.M_prevVertex
		edge.M_hasVertex0 = chain.M_hasPrevVertex
	}

	if index < chain.M_count-2 {
		edge.M_vertex3 = chain.M_vertices[index+2]
		edge.M_hasVertex3 = true
	} else {
		edge.M_vertex3 = chain.M_nextVertex
		edge.M_hasVertex3 = chain.M_hasNextVertex
	}
}

func (chain ChainShape) TestPoint(xf Transform, p Vec2) bool {
	return false
}

func (chain ChainShape) RayCast(output *B2RayCastOutput, input B2RayCastInput, xf Transform, childIndex int) bool {
	assert(childIndex < chain.M_count)

	edgeShape := MakeEdgeShape()

	i1 := childIndex
	i2 := childIndex + 1
	if i2 == chain.M_count {
		i2 = 0
	}

	edgeShape.M_vertex1 = chain.M_vertices[i1]
	edgeShape.M_vertex2 = chain.M_vertices[i2]

	return edgeShape.RayCast(output, input, xf, 0)
}

func (chain ChainShape) ComputeAABB(xf Transform, childIndex int) B2AABB {
	assert(childIndex < chain.M_count)

	i1 := childIndex
	i2 := childIndex + 1
	if i2 == chain.M_count {
		i2 = 0
	}

	v1 := TransformVec2Mul(xf, chain.M_vertices[i1])
	v2 := TransformVec2Mul(xf, chain.M_vertices[i2])

	return MakeB2AABB(Vec2Min(v1, v2), Vec2Max(v1, v2))
}

func (chain ChainShape) ComputeMass(density float64) MassData {
	massData := MakeMassData()
	massData.Mass = 0.0
	massData.Center.SetZero()
	massData.I = 0.0
	return massData
}
