package box2d

import (
	"math"
)

// This is an internal class.
type Island struct {
	M_listener ContactListenerInterface

	M_bodies   []*Body
	M_contacts []ContactInterface // has to be backed by pointers
	M_joints   []JointInterface   // has to be backed by pointers

	M_positions  []Position
	M_velocities []Velocity

	M_bodyCount    int
	M_jointCount   int
	M_contactCount int

	M_bodyCapacity    int
	M_contactCapacity int
	M_jointCapacity   int
}

func (island *Island) Clear() {
	island.M_bodyCount = 0
	island.M_contactCount = 0
	island.M_jointCount = 0
}

func (island *Island) AddBody(body *Body) {
	assert(island.M_bodyCount < island.M_bodyCapacity)
	body.M_islandIndex = island.M_bodyCount
	island.M_bodies[island.M_bodyCount] = body
	island.M_bodyCount++
}

func (island *Island) AddContact(contact ContactInterface) { // contact has to be a pointer
	assert(island.M_contactCount < island.M_contactCapacity)
	island.M_contacts[island.M_contactCount] = contact
	island.M_contactCount++
}

func (island *Island) Add(joint JointInterface) { // joint has to be a pointer
	assert(island.M_jointCount < island.M_jointCapacity)
	island.M_joints[island.M_jointCount] = joint
	island.M_jointCount++
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Island.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than b2_linearSlop.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/

/*
Cache Performance

The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The constraint structures are iterated over linearly, which leads
to few cache misses.

The bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the constraint
impulses and the bodies velocities/positions. The impulses are held inside the
constraint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.
*/

/*
2D Rotation

R = [cos(theta) -sin(theta)]
    [sin(theta) cos(theta) ]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
    [q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize.

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
*/

func MakeIsland(bodyCapacity int, contactCapacity int, jointCapacity int, listener ContactListenerInterface) Island {
	island := Island{}

	island.M_bodyCapacity = bodyCapacity
	island.M_contactCapacity = contactCapacity
	island.M_jointCapacity = jointCapacity
	island.M_bodyCount = 0
	island.M_contactCount = 0
	island.M_jointCount = 0

	island.M_listener = listener

	island.M_bodies = make([]*Body, bodyCapacity)
	island.M_contacts = make([]ContactInterface, contactCapacity)
	island.M_joints = make([]JointInterface, jointCapacity)

	island.M_velocities = make([]Velocity, bodyCapacity)
	island.M_positions = make([]Position, bodyCapacity)

	return island
}

func (island *Island) Destroy() {
}

func (island *Island) Solve(profile *Profile, step TimeStep, gravity Vec2, allowSleep bool) {
	timer := MakeTimer()

	h := step.Dt

	// Integrate velocities and apply damping. Initialize the body state.
	for i := 0; i < island.M_bodyCount; i++ {
		b := island.M_bodies[i]

		c := b.M_sweep.C
		a := b.M_sweep.A
		v := b.M_linearVelocity
		w := b.M_angularVelocity

		// Store positions for continuous collision.
		b.M_sweep.C0 = b.M_sweep.C
		b.M_sweep.A0 = b.M_sweep.A

		if b.M_type == BodyType.DynamicBody {

			// Integrate velocities.
			v.OperatorPlusInplace(
				Vec2MulScalar(
					h,
					Vec2Add(
						Vec2MulScalar(b.M_gravityScale, gravity),
						Vec2MulScalar(b.M_invMass, b.M_force),
					),
				),
			)
			w += h * b.M_invI * b.M_torque

			// Apply damping.
			// ODE: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Pade approximation:
			// v2 = v1 * 1 / (1 + c * dt)
			v.OperatorScalarMulInplace(1.0 / (1.0 + h*b.M_linearDamping))
			w *= 1.0 / (1.0 + h*b.M_angularDamping)
		}

		island.M_positions[i].C = c
		island.M_positions[i].A = a
		island.M_velocities[i].V = v
		island.M_velocities[i].W = w
	}

	timer.Reset()

	// Solver data
	solverData := MakeSolverData()
	solverData.Step = step
	solverData.Positions = island.M_positions
	solverData.Velocities = island.M_velocities

	// Initialize velocity constraints.
	contactSolverDef := MakeContactSolverDef()
	contactSolverDef.Step = step
	contactSolverDef.Contacts = island.M_contacts
	contactSolverDef.Count = island.M_contactCount
	contactSolverDef.Positions = island.M_positions
	contactSolverDef.Velocities = island.M_velocities

	contactSolver := MakeContactSolver(&contactSolverDef)
	contactSolver.InitializeVelocityConstraints()

	if step.WarmStarting {
		contactSolver.WarmStart()
	}

	for i := 0; i < island.M_jointCount; i++ {
		island.M_joints[i].InitVelocityConstraints(solverData)
	}

	profile.SolveInit = timer.GetMilliseconds()

	// Solve velocity constraints
	timer.Reset()
	for i := 0; i < step.VelocityIterations; i++ {

		for j := 0; j < island.M_jointCount; j++ {
			island.M_joints[j].SolveVelocityConstraints(solverData)
		}

		contactSolver.SolveVelocityConstraints()
	}

	// Store impulses for warm starting
	contactSolver.StoreImpulses()
	profile.SolveVelocity = timer.GetMilliseconds()

	// Integrate positions
	for i := 0; i < island.M_bodyCount; i++ {
		c := island.M_positions[i].C
		a := island.M_positions[i].A
		v := island.M_velocities[i].V
		w := island.M_velocities[i].W

		// Check for large velocities
		translation := Vec2MulScalar(h, v)
		if Vec2Dot(translation, translation) > maxTranslationSquared {
			ratio := maxTranslation / translation.Length()
			v.OperatorScalarMulInplace(ratio)
		}

		rotation := h * w
		if rotation*rotation > maxRotationSquared {
			ratio := maxRotation / math.Abs(rotation)
			w *= ratio
		}

		// Integrate
		c.OperatorPlusInplace(Vec2MulScalar(h, v))
		a += h * w

		island.M_positions[i].C = c
		island.M_positions[i].A = a
		island.M_velocities[i].V = v
		island.M_velocities[i].W = w
	}

	// Solve position constraints
	timer.Reset()
	positionSolved := false
	for i := 0; i < step.PositionIterations; i++ {
		contactsOkay := contactSolver.SolvePositionConstraints()

		jointsOkay := true
		for j := 0; j < island.M_jointCount; j++ {
			jointOkay := island.M_joints[j].SolvePositionConstraints(solverData)
			jointsOkay = jointsOkay && jointOkay
		}

		if contactsOkay && jointsOkay {
			// Exit early if the position errors are small.
			positionSolved = true
			break
		}
	}

	// Copy state buffers back to the bodies
	for i := 0; i < island.M_bodyCount; i++ {
		body := island.M_bodies[i]
		body.M_sweep.C = island.M_positions[i].C
		body.M_sweep.A = island.M_positions[i].A
		body.M_linearVelocity = island.M_velocities[i].V
		body.M_angularVelocity = island.M_velocities[i].W
		body.SynchronizeTransform()
	}

	profile.SolvePosition = timer.GetMilliseconds()

	island.Report(contactSolver.M_velocityConstraints)

	if allowSleep {
		minSleepTime := maxFloat

		linTolSqr := linearSleepTolerance * linearSleepTolerance
		angTolSqr := angularSleepTolerance * angularSleepTolerance

		for i := 0; i < island.M_bodyCount; i++ {
			b := island.M_bodies[i]
			if b.GetType() == BodyType.StaticBody {
				continue
			}

			if (b.M_flags&BodyFlags.AutoSleep) == 0 || b.M_angularVelocity*b.M_angularVelocity > angTolSqr || Vec2Dot(b.M_linearVelocity, b.M_linearVelocity) > linTolSqr {
				b.M_sleepTime = 0.0
				minSleepTime = 0.0
			} else {
				b.M_sleepTime += h
				minSleepTime = math.Min(minSleepTime, b.M_sleepTime)
			}
		}

		if minSleepTime >= timeToSleep && positionSolved {
			for i := 0; i < island.M_bodyCount; i++ {
				b := island.M_bodies[i]
				b.SetAwake(false)
			}
		}
	}
}

func (island *Island) SolveTOI(subStep TimeStep, toiIndexA int, toiIndexB int) {
	assert(toiIndexA < island.M_bodyCount)
	assert(toiIndexB < island.M_bodyCount)

	// Initialize the body state.
	for i := 0; i < island.M_bodyCount; i++ {
		b := island.M_bodies[i]
		island.M_positions[i].C = b.M_sweep.C
		island.M_positions[i].A = b.M_sweep.A
		island.M_velocities[i].V = b.M_linearVelocity
		island.M_velocities[i].W = b.M_angularVelocity
	}

	contactSolverDef := MakeContactSolverDef()

	contactSolverDef.Contacts = island.M_contacts
	contactSolverDef.Count = island.M_contactCount
	contactSolverDef.Step = subStep
	contactSolverDef.Positions = island.M_positions
	contactSolverDef.Velocities = island.M_velocities
	contactSolver := MakeContactSolver(&contactSolverDef)

	// Solve position constraints.
	for i := 0; i < subStep.PositionIterations; i++ {
		contactsOkay := contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB)
		if contactsOkay {
			break
		}
	}

	// Leap of faith to new safe state.
	island.M_bodies[toiIndexA].M_sweep.C0 = island.M_positions[toiIndexA].C
	island.M_bodies[toiIndexA].M_sweep.A0 = island.M_positions[toiIndexA].A
	island.M_bodies[toiIndexB].M_sweep.C0 = island.M_positions[toiIndexB].C
	island.M_bodies[toiIndexB].M_sweep.A0 = island.M_positions[toiIndexB].A

	// No warm starting is needed for TOI events because warm
	// starting impulses were applied in the discrete solver.
	contactSolver.InitializeVelocityConstraints()

	// Solve velocity constraints.
	for i := 0; i < subStep.VelocityIterations; i++ {
		contactSolver.SolveVelocityConstraints()
	}

	// Don't store the TOI contact forces for warm starting
	// because they can be quite large.

	h := subStep.Dt

	// Integrate positions
	for i := 0; i < island.M_bodyCount; i++ {
		c := island.M_positions[i].C
		a := island.M_positions[i].A
		v := island.M_velocities[i].V
		w := island.M_velocities[i].W

		// Check for large velocities
		translation := Vec2MulScalar(h, v)
		if Vec2Dot(translation, translation) > maxTranslationSquared {
			ratio := maxTranslation / translation.Length()
			v.OperatorScalarMulInplace(ratio)
		}

		rotation := h * w
		if rotation*rotation > maxRotationSquared {
			ratio := maxRotation / math.Abs(rotation)
			w *= ratio
		}

		// Integrate
		c.OperatorPlusInplace(Vec2MulScalar(h, v))
		a += h * w

		island.M_positions[i].C = c
		island.M_positions[i].A = a
		island.M_velocities[i].V = v
		island.M_velocities[i].W = w

		// Sync bodies
		body := island.M_bodies[i]
		body.M_sweep.C = c
		body.M_sweep.A = a
		body.M_linearVelocity = v
		body.M_angularVelocity = w
		body.SynchronizeTransform()
	}

	island.Report(contactSolver.M_velocityConstraints)
}

func (island *Island) Report(constraints []ContactVelocityConstraint) {
	if island.M_listener == nil {
		return
	}

	for i := 0; i < island.M_contactCount; i++ {
		c := island.M_contacts[i]

		vc := constraints[i]

		impulse := MakeContactImpulse()
		impulse.Count = vc.PointCount

		for j := 0; j < vc.PointCount; j++ {
			impulse.NormalImpulses[j] = vc.Points[j].NormalImpulse
			impulse.TangentImpulses[j] = vc.Points[j].TangentImpulse
		}

		island.M_listener.PostSolve(c, &impulse)
	}
}
