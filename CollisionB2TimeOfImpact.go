package box2d

import (
	"math"
)

// Input parameters for b2TimeOfImpact
type TOIInput struct {
	ProxyA DistanceProxy
	ProxyB DistanceProxy
	SweepA Sweep
	SweepB Sweep
	TMax   float64 // defines sweep interval [0, tMax]
}

func MakeTOIInput() TOIInput {
	return TOIInput{}
}

// Output parameters for b2TimeOfImpact.

var TOIOutputState = struct {
	Unknown    uint8
	Failed     uint8
	Overlapped uint8
	Touching   uint8
	Separated  uint8
}{
	Unknown:    1,
	Failed:     2,
	Overlapped: 3,
	Touching:   4,
	Separated:  5,
}

type TOIOutput struct {
	State uint8
	T     float64
}

func MakeTOIOutput() TOIOutput {
	return TOIOutput{}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2TimeOfImpact.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

var (
	toiTime, toiMaxTime             float64
	toiCalls, toiIters, toiMaxIters int
	toiRootIters, toiMaxRootIters   int
)

var SeparationFunctionType = struct {
	Points uint8
	FaceA  uint8
	FaceB  uint8
}{
	Points: 0,
	FaceA:  1,
	FaceB:  2,
}

type SeparationFunction struct {
	M_proxyA           *DistanceProxy
	M_proxyB           *DistanceProxy
	M_sweepA, M_sweepB Sweep
	M_type             uint8
	M_localPoint       Vec2
	M_axis             Vec2
}

// TODO_ERIN might not need to return the separation
func (sepfunc *SeparationFunction) Initialize(cache *SimplexCache, proxyA *DistanceProxy, sweepA Sweep, proxyB *DistanceProxy, sweepB Sweep, t1 float64) float64 {
	sepfunc.M_proxyA = proxyA
	sepfunc.M_proxyB = proxyB
	count := cache.Count
	assert(0 < count && count < 3)

	sepfunc.M_sweepA = sweepA
	sepfunc.M_sweepB = sweepB

	xfA := MakeTransform()
	xfB := MakeTransform()
	sepfunc.M_sweepA.GetTransform(&xfA, t1)
	sepfunc.M_sweepB.GetTransform(&xfB, t1)

	if count == 1 {
		sepfunc.M_type = SeparationFunctionType.Points
		localPointA := sepfunc.M_proxyA.GetVertex(cache.IndexA[0])
		localPointB := sepfunc.M_proxyB.GetVertex(cache.IndexB[0])
		pointA := TransformVec2Mul(xfA, localPointA)
		pointB := TransformVec2Mul(xfB, localPointB)
		sepfunc.M_axis = Vec2Sub(pointB, pointA)
		s := sepfunc.M_axis.Normalize()
		return s
	} else if cache.IndexA[0] == cache.IndexA[1] {
		// Two points on B and one on A.
		sepfunc.M_type = SeparationFunctionType.FaceB
		localPointB1 := proxyB.GetVertex(cache.IndexB[0])
		localPointB2 := proxyB.GetVertex(cache.IndexB[1])

		sepfunc.M_axis = Vec2CrossVectorScalar(
			Vec2Sub(localPointB2, localPointB1),
			1.0,
		)

		sepfunc.M_axis.Normalize()
		normal := RotVec2Mul(xfB.Q, sepfunc.M_axis)

		sepfunc.M_localPoint = Vec2MulScalar(0.5, Vec2Add(localPointB1, localPointB2))
		pointB := TransformVec2Mul(xfB, sepfunc.M_localPoint)

		localPointA := proxyA.GetVertex(cache.IndexA[0])
		pointA := TransformVec2Mul(xfA, localPointA)

		s := Vec2Dot(Vec2Sub(pointA, pointB), normal)
		if s < 0.0 {
			sepfunc.M_axis = sepfunc.M_axis.OperatorNegate()
			s = -s
		}

		return s
	} else {
		// Two points on A and one or two points on B.
		sepfunc.M_type = SeparationFunctionType.FaceA
		localPointA1 := sepfunc.M_proxyA.GetVertex(cache.IndexA[0])
		localPointA2 := sepfunc.M_proxyA.GetVertex(cache.IndexA[1])

		sepfunc.M_axis = Vec2CrossVectorScalar(Vec2Sub(localPointA2, localPointA1), 1.0)
		sepfunc.M_axis.Normalize()
		normal := RotVec2Mul(xfA.Q, sepfunc.M_axis)

		sepfunc.M_localPoint = Vec2MulScalar(0.5, Vec2Add(localPointA1, localPointA2))
		pointA := TransformVec2Mul(xfA, sepfunc.M_localPoint)

		localPointB := sepfunc.M_proxyB.GetVertex(cache.IndexB[0])
		pointB := TransformVec2Mul(xfB, localPointB)

		s := Vec2Dot(Vec2Sub(pointB, pointA), normal)
		if s < 0.0 {
			sepfunc.M_axis = sepfunc.M_axis.OperatorNegate()
			s = -s
		}

		return s
	}
}

func (sepfunc *SeparationFunction) FindMinSeparation(indexA *int, indexB *int, t float64) float64 {
	xfA := MakeTransform()
	xfB := MakeTransform()

	sepfunc.M_sweepA.GetTransform(&xfA, t)
	sepfunc.M_sweepB.GetTransform(&xfB, t)

	switch sepfunc.M_type {
	case SeparationFunctionType.Points:
		{
			axisA := RotVec2MulT(xfA.Q, sepfunc.M_axis)
			axisB := RotVec2MulT(xfB.Q, sepfunc.M_axis.OperatorNegate())

			*indexA = sepfunc.M_proxyA.GetSupport(axisA)
			*indexB = sepfunc.M_proxyB.GetSupport(axisB)

			localPointA := sepfunc.M_proxyA.GetVertex(*indexA)
			localPointB := sepfunc.M_proxyB.GetVertex(*indexB)

			pointA := TransformVec2Mul(xfA, localPointA)
			pointB := TransformVec2Mul(xfB, localPointB)

			separation := Vec2Dot(Vec2Sub(pointB, pointA), sepfunc.M_axis)
			return separation
		}

	case SeparationFunctionType.FaceA:
		{
			normal := RotVec2Mul(xfA.Q, sepfunc.M_axis)
			pointA := TransformVec2Mul(xfA, sepfunc.M_localPoint)

			axisB := RotVec2MulT(xfB.Q, normal.OperatorNegate())

			*indexA = -1
			*indexB = sepfunc.M_proxyB.GetSupport(axisB)

			localPointB := sepfunc.M_proxyB.GetVertex(*indexB)
			pointB := TransformVec2Mul(xfB, localPointB)

			separation := Vec2Dot(Vec2Sub(pointB, pointA), normal)
			return separation
		}

	case SeparationFunctionType.FaceB:
		{
			normal := RotVec2Mul(xfB.Q, sepfunc.M_axis)
			pointB := TransformVec2Mul(xfB, sepfunc.M_localPoint)

			axisA := RotVec2MulT(xfA.Q, normal.OperatorNegate())

			*indexB = -1
			*indexA = sepfunc.M_proxyA.GetSupport(axisA)

			localPointA := sepfunc.M_proxyA.GetVertex(*indexA)
			pointA := TransformVec2Mul(xfA, localPointA)

			separation := Vec2Dot(Vec2Sub(pointA, pointB), normal)
			return separation
		}

	default:
		assert(false)
		*indexA = -1
		*indexB = -1
		return 0.0
	}
}

func (sepfunc *SeparationFunction) Evaluate(indexA int, indexB int, t float64) float64 {
	xfA := MakeTransform()
	xfB := MakeTransform()

	sepfunc.M_sweepA.GetTransform(&xfA, t)
	sepfunc.M_sweepB.GetTransform(&xfB, t)

	switch sepfunc.M_type {
	case SeparationFunctionType.Points:
		{
			localPointA := sepfunc.M_proxyA.GetVertex(indexA)
			localPointB := sepfunc.M_proxyB.GetVertex(indexB)

			pointA := TransformVec2Mul(xfA, localPointA)
			pointB := TransformVec2Mul(xfB, localPointB)
			separation := Vec2Dot(Vec2Sub(pointB, pointA), sepfunc.M_axis)

			return separation
		}

	case SeparationFunctionType.FaceA:
		{
			normal := RotVec2Mul(xfA.Q, sepfunc.M_axis)
			pointA := TransformVec2Mul(xfA, sepfunc.M_localPoint)

			localPointB := sepfunc.M_proxyB.GetVertex(indexB)
			pointB := TransformVec2Mul(xfB, localPointB)

			separation := Vec2Dot(Vec2Sub(pointB, pointA), normal)
			return separation
		}

	case SeparationFunctionType.FaceB:
		{
			normal := RotVec2Mul(xfB.Q, sepfunc.M_axis)
			pointB := TransformVec2Mul(xfB, sepfunc.M_localPoint)

			localPointA := sepfunc.M_proxyA.GetVertex(indexA)
			pointA := TransformVec2Mul(xfA, localPointA)

			separation := Vec2Dot(Vec2Sub(pointA, pointB), normal)
			return separation
		}

	default:
		assert(false)
		return 0.0
	}
}

// Compute the upper bound on time before two shapes penetrate. Time is represented as
// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
// non-tunneling collision. If you change the time interval, you should call this function
// again.
// Note: use Distance to compute the contact point and normal at the time of impact.
// CCD via the local separating axis method. This seeks progression
// by computing the largest time at which separation is maintained.
func TimeOfImpact(output *TOIOutput, input *TOIInput) {
	timer := MakeTimer()

	toiCalls++

	output.State = TOIOutputState.Unknown
	output.T = input.TMax

	proxyA := &input.ProxyA
	proxyB := &input.ProxyB

	sweepA := input.SweepA
	sweepB := input.SweepB

	// Large rotations can make the root finder fail, so we normalize the
	// sweep angles.
	sweepA.Normalize()
	sweepB.Normalize()

	tMax := input.TMax

	totalRadius := proxyA.M_radius + proxyB.M_radius
	target := math.Max(linearSlop, totalRadius-3.0*linearSlop)
	tolerance := 0.25 * linearSlop
	assert(target > tolerance)

	t1 := 0.0
	k_maxIterations := 20 // TODO_ERIN b2Settings
	iter := 0

	// Prepare input for distance query.
	cache := MakeSimplexCache()
	cache.Count = 0
	distanceInput := MakeDistanceInput()
	distanceInput.ProxyA = input.ProxyA
	distanceInput.ProxyB = input.ProxyB
	distanceInput.UseRadii = false

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for {

		xfA := MakeTransform()
		xfB := MakeTransform()

		sweepA.GetTransform(&xfA, t1)
		sweepB.GetTransform(&xfB, t1)

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		distanceInput.TransformA = xfA
		distanceInput.TransformB = xfB
		distanceOutput := MakeDistanceOutput()
		Distance(&distanceOutput, &cache, &distanceInput)

		// If the shapes are overlapped, we give up on continuous collision.
		if distanceOutput.Distance <= 0.0 {
			// Failure!
			output.State = TOIOutputState.Overlapped
			output.T = 0.0
			break
		}

		if distanceOutput.Distance < target+tolerance {
			// Victory!
			output.State = TOIOutputState.Touching
			output.T = t1
			break
		}

		// Initialize the separating axis.
		var fcn SeparationFunction
		fcn.Initialize(&cache, proxyA, sweepA, proxyB, sweepB, t1)

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		done := false
		t2 := tMax
		pushBackIter := 0
		for {
			// Find the deepest point at t2. Store the witness point indices.
			var indexA, indexB int
			s2 := fcn.FindMinSeparation(&indexA, &indexB, t2)

			// Is the final configuration separated?
			if s2 > target+tolerance {
				// Victory!
				output.State = TOIOutputState.Separated
				output.T = tMax
				done = true
				break
			}

			// Has the separation reached tolerance?
			if s2 > target-tolerance {
				// Advance the sweeps
				t1 = t2
				break
			}

			// Compute the initial separation of the witness points.
			s1 := fcn.Evaluate(indexA, indexB, t1)

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if s1 < target-tolerance {
				output.State = TOIOutputState.Failed
				output.T = t1
				done = true
				break
			}

			// Check for touching
			if s1 <= target+tolerance {
				// Victory! t1 should hold the TOI (could be 0.0).
				output.State = TOIOutputState.Touching
				output.T = t1
				done = true
				break
			}

			// Compute 1D root of: f(x) - target = 0
			rootIterCount := 0
			a1 := t1
			a2 := t2

			for {
				// Use a mix of the secant rule and bisection.
				t := 0.0

				if (rootIterCount & 1) != 0x0000 {
					// Secant rule to improve convergence.
					t = a1 + (target-s1)*(a2-a1)/(s2-s1)
				} else {
					// Bisection to guarantee progress.
					t = 0.5 * (a1 + a2)
				}

				rootIterCount++
				toiRootIters++

				s := fcn.Evaluate(indexA, indexB, t)

				if math.Abs(s-target) < tolerance {
					// t2 holds a tentative value for t1
					t2 = t
					break
				}

				// Ensure we continue to bracket the root.
				if s > target {
					a1 = t
					s1 = s
				} else {
					a2 = t
					s2 = s
				}

				if rootIterCount == 50 {
					break
				}
			}

			toiMaxRootIters = MaxInt(toiMaxRootIters, rootIterCount)

			pushBackIter++

			if pushBackIter == maxPolygonVertices {
				break
			}
		}

		iter++
		toiIters++

		if done {
			break
		}

		if iter == k_maxIterations {
			// Root finder got stuck. Semi-victory.
			output.State = TOIOutputState.Failed
			output.T = t1
			break
		}
	}

	toiMaxIters = MaxInt(toiMaxIters, iter)

	time := timer.GetMilliseconds()
	toiMaxTime = math.Max(toiMaxTime, time)
	toiTime += time
}
