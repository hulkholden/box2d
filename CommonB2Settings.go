package box2d

import "math"

// @port(OK)
const B2DEBUG = false

// @port(OK)
func B2Assert(a bool) {
	if !a {
		panic("B2Assert")
	}
}

const (
	maxFloat = math.MaxFloat64
	epsilon  = math.SmallestNonzeroFloat64
	Pi       = math.Pi
)

/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///

// Collision

// The maximum number of contact points between two convex shapes. Do
// not change this value.
const maxManifoldPoints = 2

// The maximum number of vertices on a convex polygon. You cannot increase
// this too much because b2BlockAllocator has a maximum object size.
const maxPolygonVertices = 8

// This is used to fatten AABBs in the dynamic tree. This allows proxies
// to move by a small amount without triggering a tree adjustment.
// This is in meters.
const aabbExtension = 0.1

// This is used to fatten AABBs in the dynamic tree. This is used to predict
// the future position based on the current displacement.
// This is a dimensionless multiplier.
const aabbMultiplier = 2.0

// A small length used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant.
const linearSlop = 0.005

// A small angle used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant.
const angularSlop = (2.0 / 180.0 * Pi)

// The radius of the polygon/edge shape skin. This should not be modified. Making
// this smaller means polygons will have an insufficient buffer for continuous collision.
// Making it larger may create artifacts for vertex collision.
const polygonRadius = (2.0 * linearSlop)

// Maximum number of sub-steps per contact in continuous physics simulation.
const maxSubSteps = 8

// Dynamics

// Maximum number of contacts to be handled to solve a TOI impact.
const maxTOIContacts = 32

// A velocity threshold for elastic collisions. Any collision with a relative linear
// velocity below this threshold will be treated as inelastic.
const velocityThreshold = 1.0

// The maximum linear position correction used when solving constraints. This helps to
// prevent overshoot.
const maxLinearCorrection = 0.2

// The maximum angular position correction used when solving constraints. This helps to
// prevent overshoot.
const maxAngularCorrection = (8.0 / 180.0 * Pi)

// The maximum linear velocity of a body. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this.
const (
	maxTranslation        = 2.0
	maxTranslationSquared = (maxTranslation * maxTranslation)
)

// The maximum angular velocity of a body. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this.
const (
	maxRotation        = (0.5 * Pi)
	maxRotationSquared = (maxRotation * maxRotation)
)

// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
// that overlap is removed in one time step. However using values close to 1 often lead
// to overshoot.
const (
	baumgarte   = 0.2
	toiBaugarte = 0.75
)

// Sleep

// The time that a body must be still before it will go to sleep.
const timeToSleep = 0.5

// A body cannot sleep if its linear velocity is above this tolerance.
const linearSleepTolerance = 0.01

// A body cannot sleep if its angular velocity is above this tolerance.
const angularSleepTolerance = (2.0 / 180.0 * Pi)
