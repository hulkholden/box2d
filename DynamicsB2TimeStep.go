package box2d

// Profiling data. Times are in milliseconds.
type Profile struct {
	Step          float64
	Collide       float64
	Solve         float64
	SolveInit     float64
	SolveVelocity float64
	SolvePosition float64
	Broadphase    float64
	SolveTOI      float64
}

func MakeProfile() Profile {
	return Profile{}
}

// This is an internal structure.
type TimeStep struct {
	Dt                 float64 // time step
	Inv_dt             float64 // inverse time step (0 if dt == 0).
	DtRatio            float64 // dt * inv_dt0
	VelocityIterations int
	PositionIterations int
	WarmStarting       bool
}

func MakeTimeStep() TimeStep {
	return TimeStep{}
}

// This is an internal structure.
type Position struct {
	C Vec2
	A float64
}

// This is an internal structure.
type Velocity struct {
	V Vec2
	W float64
}

// Solver Data
type SolverData struct {
	Step       TimeStep
	Positions  []Position
	Velocities []Velocity
}

func MakeSolverData() SolverData {
	return SolverData{}
}
