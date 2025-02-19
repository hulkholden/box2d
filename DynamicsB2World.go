package box2d

import (
	"fmt"
	"math"
)

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.

var WorldFlags = struct {
	NewFixture  int
	Locked      int
	ClearForces int
}{
	NewFixture:  0x0001,
	Locked:      0x0002,
	ClearForces: 0x0004,
}

// The world class manages all physics entities, dynamic simulation,
// and asynchronous queries. The world also contains efficient memory
// management facilities.
type World struct {
	M_flags int

	M_contactManager ContactManager

	M_bodyList  *Body          // linked list
	M_jointList JointInterface // has to be backed by pointer

	M_bodyCount  int
	M_jointCount int

	M_gravity    Vec2
	M_allowSleep bool

	M_destructionListener DestructionListenerInterface
	G_debugDraw           Draw

	// This is used to compute the time step ratio to
	// support a variable time step.
	M_inv_dt0 float64

	// These are for debugging the solver.
	M_warmStarting      bool
	M_continuousPhysics bool
	M_subStepping       bool

	M_stepComplete bool

	M_profile Profile
}

func (world World) GetBodyList() *Body {
	return world.M_bodyList
}

func (world World) GetJointList() JointInterface { // returns a pointer
	return world.M_jointList
}

func (world World) GetContactList() ContactInterface { // returns a pointer
	return world.M_contactManager.M_contactList
}

func (world World) GetBodyCount() int {
	return world.M_bodyCount
}

func (world World) GetJointCount() int {
	return world.M_jointCount
}

func (world World) GetContactCount() int {
	return world.M_contactManager.M_contactCount
}

func (world *World) SetGravity(gravity Vec2) {
	world.M_gravity = gravity
}

func (world World) GetGravity() Vec2 {
	return world.M_gravity
}

func (world World) IsLocked() bool {
	return (world.M_flags & WorldFlags.Locked) == WorldFlags.Locked
}

func (world *World) SetAutoClearForces(flag bool) {
	if flag {
		world.M_flags |= WorldFlags.ClearForces
	} else {
		world.M_flags &= ^WorldFlags.ClearForces
	}
}

// Get the flag that controls automatic clearing of forces after each time step.
func (world World) GetAutoClearForces() bool {
	return (world.M_flags & WorldFlags.ClearForces) == WorldFlags.ClearForces
}

func (world World) GetContactManager() ContactManager {
	return world.M_contactManager
}

func (world World) GetProfile() Profile {
	return world.M_profile
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// World.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func MakeWorld(gravity Vec2) World {
	world := World{}

	world.M_destructionListener = nil
	world.G_debugDraw = nil

	world.M_bodyList = nil
	world.M_jointList = nil

	world.M_bodyCount = 0
	world.M_jointCount = 0

	world.M_warmStarting = true
	world.M_continuousPhysics = true
	world.M_subStepping = false

	world.M_stepComplete = true

	world.M_allowSleep = true
	world.M_gravity = gravity

	world.M_flags = WorldFlags.ClearForces

	world.M_inv_dt0 = 0.0

	world.M_contactManager = MakeContactManager()

	return world
}

func (world *World) Destroy() {
	// Some shapes allocate using b2Alloc.
	b := world.M_bodyList
	for b != nil {
		bNext := b.M_next

		f := b.M_fixtureList
		for f != nil {
			fNext := f.M_next
			f.M_proxyCount = 0
			f.Destroy()
			f = fNext
		}

		b = bNext
	}
}

func (world *World) SetDestructionListener(listener DestructionListenerInterface) {
	world.M_destructionListener = listener
}

func (world *World) SetContactFilter(filter ContactFilterInterface) {
	world.M_contactManager.M_contactFilter = filter
}

func (world *World) SetContactListener(listener ContactListenerInterface) {
	world.M_contactManager.M_contactListener = listener
}

func (world *World) SetDebugDraw(debugDraw Draw) {
	world.G_debugDraw = debugDraw
}

func (world *World) CreateBody(def *BodyDef) *Body {
	assert(!world.IsLocked())

	if world.IsLocked() {
		return nil
	}

	b := NewBody(def, world)

	// Add to world doubly linked list.
	b.M_prev = nil
	b.M_next = world.M_bodyList
	if world.M_bodyList != nil {
		world.M_bodyList.M_prev = b
	}
	world.M_bodyList = b
	world.M_bodyCount++

	return b
}

func (world *World) DestroyBody(b *Body) {
	assert(world.M_bodyCount > 0)
	assert(!world.IsLocked())

	if world.IsLocked() {
		return
	}

	// Delete the attached joints.
	je := b.M_jointList
	for je != nil {
		je0 := je
		je = je.Next

		if world.M_destructionListener != nil {
			world.M_destructionListener.SayGoodbyeToJoint(je0.Joint)
		}

		world.DestroyJoint(je0.Joint)

		b.M_jointList = je
	}
	b.M_jointList = nil

	// Delete the attached contacts.
	ce := b.M_contactList
	for ce != nil {
		ce0 := ce
		ce = ce.Next
		world.M_contactManager.Destroy(ce0.Contact)
	}
	b.M_contactList = nil

	// Delete the attached fixtures. This destroys broad-phase proxies.
	f := b.M_fixtureList
	for f != nil {
		f0 := f
		f = f.M_next

		if world.M_destructionListener != nil {
			world.M_destructionListener.SayGoodbyeToFixture(f0)
		}

		f0.DestroyProxies(&world.M_contactManager.M_broadPhase)
		f0.Destroy()

		b.M_fixtureList = f
		b.M_fixtureCount -= 1
	}

	b.M_fixtureList = nil
	b.M_fixtureCount = 0

	// Remove world body list.
	if b.M_prev != nil {
		b.M_prev.M_next = b.M_next
	}

	if b.M_next != nil {
		b.M_next.M_prev = b.M_prev
	}

	if b == world.M_bodyList {
		world.M_bodyList = b.M_next
	}

	world.M_bodyCount--
}

func (world *World) CreateJoint(def JointDefInterface) JointInterface {
	assert(!world.IsLocked())
	if world.IsLocked() {
		return nil
	}

	j := JointCreate(def)

	// Connect to the world list.
	j.SetPrev(nil)
	j.SetNext(world.M_jointList)
	if world.M_jointList != nil {
		world.M_jointList.SetPrev(j)
	}
	world.M_jointList = j
	world.M_jointCount++

	// Connect to the bodies' doubly linked lists.
	j.GetEdgeA().Joint = j
	j.GetEdgeA().Other = j.GetBodyB()
	j.GetEdgeA().Prev = nil
	j.GetEdgeA().Next = j.GetBodyA().M_jointList
	if j.GetBodyA().M_jointList != nil {
		j.GetBodyA().M_jointList.Prev = j.GetEdgeA()
	}

	j.GetBodyA().M_jointList = j.GetEdgeA()

	j.GetEdgeB().Joint = j
	j.GetEdgeB().Other = j.GetBodyA()
	j.GetEdgeB().Prev = nil
	j.GetEdgeB().Next = j.GetBodyB().M_jointList
	if j.GetBodyB().M_jointList != nil {
		j.GetBodyB().M_jointList.Prev = j.GetEdgeB()
	}
	j.GetBodyB().M_jointList = j.GetEdgeB()

	bodyA := def.GetBodyA()
	bodyB := def.GetBodyB()

	// If the joint prevents collisions, then flag any contacts for filtering.
	if !def.IsCollideConnected() {
		edge := bodyB.GetContactList()
		for edge != nil {
			if edge.Other == bodyA {
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge.Contact.FlagForFiltering()
			}

			edge = edge.Next
		}
	}

	// Note: creating a joint doesn't wake the bodies.

	return j
}

func (world *World) DestroyJoint(j JointInterface) { // j backed by pointer
	assert(!world.IsLocked())
	if world.IsLocked() {
		return
	}

	collideConnected := j.IsCollideConnected()

	// Remove from the doubly linked list.
	if j.GetPrev() != nil {
		j.GetPrev().SetNext(j.GetNext())
	}

	if j.GetNext() != nil {
		j.GetNext().SetPrev(j.GetPrev())
	}

	if j == world.M_jointList {
		world.M_jointList = j.GetNext()
	}

	// Disconnect from island graph.
	bodyA := j.GetBodyA()
	bodyB := j.GetBodyB()

	// Wake up connected bodies.
	bodyA.SetAwake(true)
	bodyB.SetAwake(true)

	// Remove from body 1.
	if j.GetEdgeA().Prev != nil {
		j.GetEdgeA().Prev.Next = j.GetEdgeA().Next
	}

	if j.GetEdgeA().Next != nil {
		j.GetEdgeA().Next.Prev = j.GetEdgeA().Prev
	}

	if j.GetEdgeA() == bodyA.M_jointList {
		bodyA.M_jointList = j.GetEdgeA().Next
	}

	j.GetEdgeA().Prev = nil
	j.GetEdgeA().Next = nil

	// Remove from body 2
	if j.GetEdgeB().Prev != nil {
		j.GetEdgeB().Prev.Next = j.GetEdgeB().Next
	}

	if j.GetEdgeB().Next != nil {
		j.GetEdgeB().Next.Prev = j.GetEdgeB().Prev
	}

	if j.GetEdgeB() == bodyB.M_jointList {
		bodyB.M_jointList = j.GetEdgeB().Next
	}

	j.GetEdgeB().Prev = nil
	j.GetEdgeB().Next = nil

	JointDestroy(j)

	assert(world.M_jointCount > 0)
	world.M_jointCount--

	// If the joint prevents collisions, then flag any contacts for filtering.
	if !collideConnected {
		edge := bodyB.GetContactList()
		for edge != nil {
			if edge.Other == bodyA {
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge.Contact.FlagForFiltering()
			}

			edge = edge.Next
		}
	}
}

func (world *World) SetAllowSleeping(flag bool) {
	if flag == world.M_allowSleep {
		return
	}

	world.M_allowSleep = flag
	if !world.M_allowSleep {
		for b := world.M_bodyList; b != nil; b = b.M_next {
			b.SetAwake(true)
		}
	}
}

// Find islands, integrate and solve constraints, solve position constraints
func (world *World) Solve(step TimeStep) {
	world.M_profile.SolveInit = 0.0
	world.M_profile.SolveVelocity = 0.0
	world.M_profile.SolvePosition = 0.0

	// Size the island for the worst case.
	island := MakeIsland(
		world.M_bodyCount,
		world.M_contactManager.M_contactCount,
		world.M_jointCount,
		world.M_contactManager.M_contactListener,
	)

	// Clear all the island flags.
	for b := world.M_bodyList; b != nil; b = b.M_next {
		b.M_flags &= ^BodyFlags.Island
	}
	for c := world.M_contactManager.M_contactList; c != nil; c = c.GetNext() {
		c.SetFlags(c.GetFlags() & ^BodyFlags.Island)
	}

	for j := world.M_jointList; j != nil; j = j.GetNext() {
		j.SetIslandFlag(false)
	}

	// Build and simulate all awake islands.
	stackSize := world.M_bodyCount
	stack := make([]*Body, stackSize)

	for seed := world.M_bodyList; seed != nil; seed = seed.M_next {
		if (seed.M_flags & BodyFlags.Island) != 0x0000 {
			continue
		}

		if !seed.IsAwake() || !seed.IsActive() {
			continue
		}

		// The seed can be dynamic or kinematic.
		if seed.GetType() == BodyType.StaticBody {
			continue
		}

		// Reset island and stack.
		island.Clear()
		stackCount := 0
		stack[stackCount] = seed
		stackCount++
		seed.M_flags |= BodyFlags.Island

		// Perform a depth first search (DFS) on the constraint graph.
		for stackCount > 0 {
			// Grab the next body off the stack and add it to the island.
			stackCount--
			b := stack[stackCount]
			assert(b.IsActive())
			island.AddBody(b)

			// Make sure the body is awake (without resetting sleep timer).
			b.M_flags |= BodyFlags.Awake

			// To keep islands as small as possible, we don't
			// propagate islands across static bodies.
			if b.GetType() == BodyType.StaticBody {
				continue
			}

			// Search all contacts connected to this body.
			for ce := b.M_contactList; ce != nil; ce = ce.Next {
				contact := ce.Contact

				// Has this contact already been added to an island?
				if (contact.GetFlags() & BodyFlags.Island) != 0x0000 {
					continue
				}

				// Is this contact solid and touching?
				if !contact.IsEnabled() || !contact.IsTouching() {
					continue
				}

				// Skip sensors.
				sensorA := contact.GetFixtureA().M_isSensor
				sensorB := contact.GetFixtureB().M_isSensor

				if sensorA || sensorB {
					continue
				}

				island.AddContact(contact)
				contact.SetFlags(contact.GetFlags() | BodyFlags.Island)

				other := ce.Other

				// Was the other body already added to this island?
				if (other.M_flags & BodyFlags.Island) != 0x0000 {
					continue
				}

				assert(stackCount < stackSize)
				stack[stackCount] = other
				stackCount++
				other.M_flags |= BodyFlags.Island
			}

			// Search all joints connect to this body.
			for je := b.M_jointList; je != nil; je = je.Next {

				if je.Joint.GetIslandFlag() {
					continue
				}

				other := je.Other

				// Don't simulate joints connected to inactive bodies.
				if !other.IsActive() {
					continue
				}

				island.Add(je.Joint)
				je.Joint.SetIslandFlag(true)

				if other.M_flags&BodyFlags.Island != 0x0000 {
					continue
				}

				assert(stackCount < stackSize)
				stack[stackCount] = other
				stackCount++
				other.M_flags |= BodyFlags.Island
			}
		}

		profile := MakeProfile()
		island.Solve(&profile, step, world.M_gravity, world.M_allowSleep)
		world.M_profile.SolveInit += profile.SolveInit
		world.M_profile.SolveVelocity += profile.SolveVelocity
		world.M_profile.SolvePosition += profile.SolvePosition

		// Post solve cleanup.
		for i := 0; i < island.M_bodyCount; i++ {
			// Allow static bodies to participate in other islands.
			b := island.M_bodies[i]
			if b.GetType() == BodyType.StaticBody {
				b.M_flags &= ^BodyFlags.Island
			}
		}
	}

	stack = nil

	{
		timer := MakeTimer()

		// Synchronize fixtures, check for out of range bodies.
		for b := world.M_bodyList; b != nil; b = b.GetNext() {
			// If a body was not in an island then it did not move.
			if (b.M_flags & BodyFlags.Island) == 0 {
				continue
			}

			if b.GetType() == BodyType.StaticBody {
				continue
			}

			// Update fixtures (for broad-phase).
			b.SynchronizeFixtures()
		}

		// Look for new contacts.
		world.M_contactManager.FindNewContacts()
		world.M_profile.Broadphase = timer.GetMilliseconds()
	}
}

// Find TOI contacts and solve them.
func (world *World) SolveTOI(step TimeStep) {
	island := MakeIsland(2*maxTOIContacts, maxTOIContacts, 0, world.M_contactManager.M_contactListener)

	if world.M_stepComplete {
		for b := world.M_bodyList; b != nil; b = b.M_next {
			b.M_flags &= ^BodyFlags.Island
			b.M_sweep.Alpha0 = 0.0
		}

		for c := world.M_contactManager.M_contactList; c != nil; c = c.GetNext() {
			// Invalidate TOI
			c.SetFlags(c.GetFlags() & ^(ContactFlags.TOI | ContactFlags.Island))
			c.SetTOICount(0)
			c.SetTOI(1.0)
		}
	}

	// Find TOI events and solve them.
	for {
		// Find the first TOI.
		var minContact ContactInterface = nil // has to be a pointer
		minAlpha := 1.0

		for c := world.M_contactManager.M_contactList; c != nil; c = c.GetNext() {

			// Is this contact disabled?
			if !c.IsEnabled() {
				continue
			}

			// Prevent excessive sub-stepping.
			if c.GetTOICount() > maxSubSteps {
				continue
			}

			alpha := 1.0
			if (c.GetFlags() & ContactFlags.TOI) != 0x0000 {
				// This contact has a valid cached TOI.
				alpha = c.GetTOI()
			} else {
				fA := c.GetFixtureA()
				fB := c.GetFixtureB()

				// Is there a sensor?
				if fA.IsSensor() || fB.IsSensor() {
					continue
				}

				bA := fA.GetBody()
				bB := fB.GetBody()

				typeA := bA.M_type
				typeB := bB.M_type
				assert(typeA == BodyType.DynamicBody || typeB == BodyType.DynamicBody)

				activeA := bA.IsAwake() && typeA != BodyType.StaticBody
				activeB := bB.IsAwake() && typeB != BodyType.StaticBody

				// Is at least one body active (awake and dynamic or kinematic)?
				if !activeA && !activeB {
					continue
				}

				collideA := bA.IsBullet() || typeA != BodyType.DynamicBody
				collideB := bB.IsBullet() || typeB != BodyType.DynamicBody

				// Are these two non-bullet dynamic bodies?
				if !collideA && !collideB {
					continue
				}

				// Compute the TOI for this contact.
				// Put the sweeps onto the same time interval.
				alpha0 := bA.M_sweep.Alpha0

				if bA.M_sweep.Alpha0 < bB.M_sweep.Alpha0 {
					alpha0 = bB.M_sweep.Alpha0
					bA.M_sweep.Advance(alpha0)
				} else if bB.M_sweep.Alpha0 < bA.M_sweep.Alpha0 {
					alpha0 = bA.M_sweep.Alpha0
					bB.M_sweep.Advance(alpha0)
				}

				assert(alpha0 < 1.0)

				indexA := c.GetChildIndexA()
				indexB := c.GetChildIndexB()

				// Compute the time of impact in interval [0, minTOI]
				input := MakeTOIInput()
				input.ProxyA.Set(fA.GetShape(), indexA)
				input.ProxyB.Set(fB.GetShape(), indexB)
				input.SweepA = bA.M_sweep
				input.SweepB = bB.M_sweep
				input.TMax = 1.0

				output := MakeTOIOutput()
				TimeOfImpact(&output, &input)

				// Beta is the fraction of the remaining portion of the .
				beta := output.T
				if output.State == TOIOutputState.Touching {
					alpha = math.Min(alpha0+(1.0-alpha0)*beta, 1.0)
				} else {
					alpha = 1.0
				}

				c.SetTOI(alpha)
				c.SetFlags(c.GetFlags() | ContactFlags.TOI)
			}

			if alpha < minAlpha {
				// This is the minimum TOI found so far.
				minContact = c
				minAlpha = alpha
			}
		}

		if minContact == nil || 1.0-10.0*epsilon < minAlpha {
			// No more TOI events. Done!
			world.M_stepComplete = true
			break
		}

		// Advance the bodies to the TOI.
		fA := minContact.GetFixtureA()
		fB := minContact.GetFixtureB()
		bA := fA.GetBody()
		bB := fB.GetBody()

		backup1 := bA.M_sweep
		backup2 := bB.M_sweep

		bA.Advance(minAlpha)
		bB.Advance(minAlpha)

		// The TOI contact likely has some new contact points.
		ContactUpdate(minContact, world.M_contactManager.M_contactListener)
		minContact.SetFlags(minContact.GetFlags() & ^ContactFlags.TOI)
		minContact.SetTOICount(minContact.GetTOICount() + 1)

		// Is the contact solid?
		if !minContact.IsEnabled() || !minContact.IsTouching() {
			// Restore the sweeps.
			minContact.SetEnabled(false)
			bA.M_sweep = backup1
			bB.M_sweep = backup2
			bA.SynchronizeTransform()
			bB.SynchronizeTransform()
			continue
		}

		bA.SetAwake(true)
		bB.SetAwake(true)

		// Build the island
		island.Clear()
		island.AddBody(bA)
		island.AddBody(bB)
		island.AddContact(minContact)

		bA.M_flags |= BodyFlags.Island
		bB.M_flags |= BodyFlags.Island
		minContact.SetFlags(minContact.GetFlags() | ContactFlags.Island)

		// Get contacts on bodyA and bodyB.
		bodies := [2]*Body{bA, bB}

		for i := 0; i < 2; i++ {
			body := bodies[i]
			if body.M_type == BodyType.DynamicBody {
				for ce := body.M_contactList; ce != nil; ce = ce.Next {
					if island.M_bodyCount == island.M_bodyCapacity {
						break
					}

					if island.M_contactCount == island.M_contactCapacity {
						break
					}

					contact := ce.Contact

					// Has this contact already been added to the island?
					if (contact.GetFlags() & ContactFlags.Island) != 0x0000 {
						continue
					}

					// Only add static, kinematic, or bullet bodies.
					other := ce.Other
					if other.M_type == BodyType.DynamicBody && !body.IsBullet() && !other.IsBullet() {
						continue
					}

					// Skip sensors.
					sensorA := contact.GetFixtureA().M_isSensor
					sensorB := contact.GetFixtureB().M_isSensor
					if sensorA || sensorB {
						continue
					}

					// Tentatively advance the body to the TOI.
					backup := other.M_sweep
					if (other.M_flags & BodyFlags.Island) == 0 {
						other.Advance(minAlpha)
					}

					// Update the contact points
					ContactUpdate(contact, world.M_contactManager.M_contactListener)

					// Was the contact disabled by the user?
					if !contact.IsEnabled() {
						other.M_sweep = backup
						other.SynchronizeTransform()
						continue
					}

					// Are there contact points?
					if !contact.IsTouching() {
						other.M_sweep = backup
						other.SynchronizeTransform()
						continue
					}

					// Add the contact to the island
					contact.SetFlags(contact.GetFlags() | ContactFlags.Island)
					island.AddContact(contact)

					// Has the other body already been added to the island?
					if (other.M_flags & BodyFlags.Island) != 0x0000 {
						continue
					}

					// Add the other body to the island.
					other.M_flags |= BodyFlags.Island

					if other.M_type != BodyType.StaticBody {
						other.SetAwake(true)
					}

					island.AddBody(other)
				}
			}
		}

		subStep := MakeTimeStep()
		subStep.Dt = (1.0 - minAlpha) * step.Dt
		subStep.Inv_dt = 1.0 / subStep.Dt
		subStep.DtRatio = 1.0
		subStep.PositionIterations = 20
		subStep.VelocityIterations = step.VelocityIterations
		subStep.WarmStarting = false
		island.SolveTOI(subStep, bA.M_islandIndex, bB.M_islandIndex)

		// Reset island flags and synchronize broad-phase proxies.
		for i := 0; i < island.M_bodyCount; i++ {
			body := island.M_bodies[i]
			body.M_flags &= ^BodyFlags.Island

			if body.M_type != BodyType.DynamicBody {
				continue
			}

			body.SynchronizeFixtures()

			// Invalidate all contact TOIs on this displaced body.
			for ce := body.M_contactList; ce != nil; ce = ce.Next {
				ce.Contact.SetFlags(ce.Contact.GetFlags() & ^(ContactFlags.TOI | ContactFlags.Island))
			}
		}

		// Commit fixture proxy movements to the broad-phase so that new contacts are created.
		// Also, some contacts can be destroyed.
		world.M_contactManager.FindNewContacts()

		if world.M_subStepping {
			world.M_stepComplete = false
			break
		}
	}
}

func (world *World) Step(dt float64, velocityIterations int, positionIterations int) {
	stepTimer := MakeTimer()

	// If new fixtures were added, we need to find the new contacts.
	if (world.M_flags & WorldFlags.NewFixture) != 0x0000 {
		world.M_contactManager.FindNewContacts()
		world.M_flags &= ^WorldFlags.NewFixture
	}

	world.M_flags |= WorldFlags.Locked

	step := MakeTimeStep()
	step.Dt = dt
	step.VelocityIterations = velocityIterations
	step.PositionIterations = positionIterations
	if dt > 0.0 {
		step.Inv_dt = 1.0 / dt
	} else {
		step.Inv_dt = 0.0
	}

	step.DtRatio = world.M_inv_dt0 * dt

	step.WarmStarting = world.M_warmStarting

	// Update contacts. This is where some contacts are destroyed.
	{
		timer := MakeTimer()
		world.M_contactManager.Collide()
		world.M_profile.Collide = timer.GetMilliseconds()
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if world.M_stepComplete && step.Dt > 0.0 {
		timer := MakeTimer()
		world.Solve(step)
		world.M_profile.Solve = timer.GetMilliseconds()
	}

	// Handle TOI events.
	if world.M_continuousPhysics && step.Dt > 0.0 {
		timer := MakeTimer()
		world.SolveTOI(step)
		world.M_profile.SolveTOI = timer.GetMilliseconds()
	}

	if step.Dt > 0.0 {
		world.M_inv_dt0 = step.Inv_dt
	}

	if (world.M_flags & WorldFlags.ClearForces) != 0x0000 {
		world.ClearForces()
	}

	world.M_flags &= ^WorldFlags.Locked

	world.M_profile.Step = stepTimer.GetMilliseconds()
}

func (world *World) ClearForces() {
	for body := world.M_bodyList; body != nil; body = body.GetNext() {
		body.M_force.SetZero()
		body.M_torque = 0.0
	}
}

type WorldQueryWrapper struct {
	BroadPhase *BroadPhase
	Callback   BroadPhaseQueryCallback
}

func MakeWorldQueryWrapper() WorldQueryWrapper {
	return WorldQueryWrapper{}
}

func (query *WorldQueryWrapper) QueryCallback(proxyId int) bool {
	proxy := query.BroadPhase.GetUserData(proxyId).(*FixtureProxy)
	return query.Callback(proxy.Fixture)
}

func (world *World) QueryAABB(callback BroadPhaseQueryCallback, aabb AABB) {
	wrapper := MakeWorldQueryWrapper()
	wrapper.BroadPhase = &world.M_contactManager.M_broadPhase
	wrapper.Callback = callback
	world.M_contactManager.M_broadPhase.Query(wrapper.QueryCallback, aabb)
}

func (world *World) RayCast(callback RaycastCallback, point1 Vec2, point2 Vec2) {
	// TreeRayCastCallback
	wrapper := func(input RayCastInput, nodeId int) float64 {
		userData := world.M_contactManager.M_broadPhase.GetUserData(nodeId)
		proxy := userData.(*FixtureProxy)
		fixture := proxy.Fixture
		index := proxy.ChildIndex
		output := MakeRayCastOutput()
		hit := fixture.RayCast(&output, input, index)

		if hit {
			fraction := output.Fraction
			point := Vec2Add(Vec2MulScalar((1.0-fraction), input.P1), Vec2MulScalar(fraction, input.P2))
			return callback(fixture, point, output.Normal, fraction)
		}

		return input.MaxFraction
	}

	input := MakeRayCastInput()
	input.MaxFraction = 1.0
	input.P1 = point1
	input.P2 = point2
	world.M_contactManager.M_broadPhase.RayCast(wrapper, input)
}

func (world *World) DrawShape(fixture *Fixture, xf Transform, color Color) {
	switch fixture.GetType() {
	case ShapeType.Circle:
		circle := fixture.GetShape().(*CircleShape)

		center := TransformVec2Mul(xf, circle.M_p)
		radius := circle.M_radius
		axis := RotVec2Mul(xf.Q, MakeVec2(1.0, 0.0))

		world.G_debugDraw.DrawSolidCircle(center, radius, axis, color)

	case ShapeType.Edge:
		edge := fixture.GetShape().(*EdgeShape)
		v1 := TransformVec2Mul(xf, edge.M_vertex1)
		v2 := TransformVec2Mul(xf, edge.M_vertex2)
		world.G_debugDraw.DrawSegment(v1, v2, color)

	case ShapeType.Chain:
		chain := fixture.GetShape().(*ChainShape)
		count := chain.M_count
		vertices := chain.M_vertices

		ghostColor := MakeColorRGBA(0.75*color.R, 0.75*color.G, 0.75*color.B, color.A)

		v1 := TransformVec2Mul(xf, vertices[0])
		world.G_debugDraw.DrawPoint(v1, 4.0, color)

		if chain.M_hasPrevVertex {
			vp := TransformVec2Mul(xf, chain.M_prevVertex)
			world.G_debugDraw.DrawSegment(vp, v1, ghostColor)
			world.G_debugDraw.DrawCircle(vp, 0.1, ghostColor)
		}

		for i := 1; i < count; i++ {
			v2 := TransformVec2Mul(xf, vertices[i])
			world.G_debugDraw.DrawSegment(v1, v2, color)
			world.G_debugDraw.DrawPoint(v2, 4.0, color)
			v1 = v2
		}

		if chain.M_hasNextVertex {
			vn := TransformVec2Mul(xf, chain.M_nextVertex)
			world.G_debugDraw.DrawSegment(v1, vn, ghostColor)
			world.G_debugDraw.DrawCircle(vn, 0.1, ghostColor)
		}

	case ShapeType.Polygon:
		poly := fixture.GetShape().(*PolygonShape)
		vertexCount := poly.M_count
		assert(vertexCount <= maxPolygonVertices)
		var vertices [maxPolygonVertices]Vec2

		for i := 0; i < vertexCount; i++ {
			vertices[i] = TransformVec2Mul(xf, poly.M_vertices[i])
		}

		world.G_debugDraw.DrawSolidPolygon(vertices[:vertexCount], color)

	default:
	}
}

/*
// TODO: Figure out how to cast Joint to the derived type.
func (world *World) DrawJoint(joint *Joint) {
	bodyA := joint.GetBodyA()
	bodyB := joint.GetBodyB()
	xf1 := bodyA.GetTransform()
	xf2 := bodyB.GetTransform()
	x1 := xf1.P
	x2 := xf2.P

	color := MakeColorRGB(0.5, 0.8, 0.8)

	switch joint.GetType() {
	case JointType.Distance:
		distance := joint.(*DistanceJoint)
		p1 := distance.GetAnchorA()
		p2 := distance.GetAnchorB()

		world.G_debugDraw.DrawSegment(p1, p2, color)

	case JointType.Pulley:

		pulley := joint.(*PulleyJoint)
		p1 := pulley.GetAnchorA()
		p2 := pulley.GetAnchorB()

		s1 := pulley.GetGroundAnchorA()
		s2 := pulley.GetGroundAnchorB()
		world.G_debugDraw.DrawSegment(s1, p1, color)
		world.G_debugDraw.DrawSegment(s2, p2, color)
		world.G_debugDraw.DrawSegment(s1, s2, color)

	case JointType.Mouse:
		// Don't draw this

	default:
		world.G_debugDraw.DrawSegment(x1, p1, color)
		world.G_debugDraw.DrawSegment(p1, p2, color)
		world.G_debugDraw.DrawSegment(x2, p2, color)
	}
}
*/

func (world *World) DrawDebugData() {
	if world.G_debugDraw == nil {
		return
	}

	flags := world.G_debugDraw.GetFlags()

	if (flags & DrawFlags.Shape) != 0 {
		for b := world.M_bodyList; b != nil; b = b.GetNext() {
			xf := b.GetTransform()
			for f := b.GetFixtureList(); f != nil; f = f.GetNext() {
				if !b.IsActive() {
					world.DrawShape(f, xf, MakeColorRGB(0.5, 0.5, 0.3))
				} else if b.GetType() == BodyType.StaticBody {
					world.DrawShape(f, xf, MakeColorRGB(0.5, 0.9, 0.5))
				} else if b.GetType() == BodyType.KinematicBody {
					world.DrawShape(f, xf, MakeColorRGB(0.5, 0.5, 0.9))
				} else if !b.IsAwake() {
					world.DrawShape(f, xf, MakeColorRGB(0.6, 0.6, 0.6))
				} else {
					world.DrawShape(f, xf, MakeColorRGB(0.9, 0.7, 0.7))
				}
			}
		}
	}

	if (flags & DrawFlags.Joint) != 0 {
		for j := world.M_jointList; j != nil; j = j.GetNext() {
			// world.DrawJoint(j)
		}
	}

	if (flags & DrawFlags.Pair) != 0 {
		// color := MakeColorRGB(0.3, 0.9, 0.9)
		for c := world.M_contactManager.M_contactList; c != nil; c = c.GetNext() {
			// fixtureA := c.GetFixtureA()
			// fixtureB := c.GetFixtureB()

			// cA := fixtureA.GetAABB().GetCenter()
			// cB := fixtureB.GetAABB().GetCenter()

			// world.G_debugDraw.DrawSegment(cA, cB, color)
		}
	}

	if (flags & DrawFlags.AABB) != 0 {
		color := MakeColorRGB(0.9, 0.3, 0.9)
		bp := &world.M_contactManager.M_broadPhase

		for b := world.M_bodyList; b != nil; b = b.GetNext() {
			if !b.IsActive() {
				continue
			}

			for f := b.GetFixtureList(); f != nil; f = f.GetNext() {
				for i := 0; i < f.M_proxyCount; i++ {
					proxy := f.M_proxies[i]
					aabb := bp.GetFatAABB(proxy.ProxyId)
					var vs [4]Vec2
					vs[0].Set(aabb.LowerBound.X, aabb.LowerBound.Y)
					vs[1].Set(aabb.UpperBound.X, aabb.LowerBound.Y)
					vs[2].Set(aabb.UpperBound.X, aabb.UpperBound.Y)
					vs[3].Set(aabb.LowerBound.X, aabb.UpperBound.Y)

					world.G_debugDraw.DrawPolygon(vs[:4], color)
				}
			}
		}
	}

	if (flags & DrawFlags.CenterOfMass) != 0 {
		for b := world.M_bodyList; b != nil; b = b.GetNext() {
			xf := b.GetTransform()
			xf.P = b.GetWorldCenter()
			world.G_debugDraw.DrawTransform(xf)
		}
	}
}

func (world World) GetProxyCount() int {
	return world.M_contactManager.M_broadPhase.GetProxyCount()
}

func (world World) GetTreeHeight() int {
	return world.M_contactManager.M_broadPhase.GetTreeHeight()
}

func (world World) GetTreeBalance() int {
	return world.M_contactManager.M_broadPhase.GetTreeBalance()
}

func (world World) GetTreeQuality() float64 {
	return world.M_contactManager.M_broadPhase.GetTreeQuality()
}

func (world *World) ShiftOrigin(newOrigin Vec2) {
	assert((world.M_flags & WorldFlags.Locked) == 0)
	if (world.M_flags & WorldFlags.Locked) == WorldFlags.Locked {
		return
	}

	for b := world.M_bodyList; b != nil; b = b.M_next {
		b.M_xf.P.OperatorMinusInplace(newOrigin)
		b.M_sweep.C0.OperatorMinusInplace(newOrigin)
		b.M_sweep.C.OperatorMinusInplace(newOrigin)
	}

	for j := world.M_jointList; j != nil; j = j.GetNext() {
		j.ShiftOrigin(newOrigin)
	}

	world.M_contactManager.M_broadPhase.ShiftOrigin(newOrigin)
}

func (world *World) Dump() {
	if (world.M_flags & WorldFlags.Locked) == WorldFlags.Locked {
		return
	}

	fmt.Printf("var g = Vec2{%.15f, %.15f}\n", world.M_gravity.X, world.M_gravity.Y)
	fmt.Print("m_world.SetGravity(g);\n")

	fmt.Printf("bodies := make([]Body, %d)\n", world.M_bodyCount)
	fmt.Printf("joints := make([]Joint, %d)\n", world.M_jointCount)

	i := 0
	for b := world.M_bodyList; b != nil; b = b.M_next {
		b.M_islandIndex = i
		b.Dump()
		i++
	}

	i = 0
	for j := world.M_jointList; j != nil; j = j.GetNext() {
		j.SetIndex(i)
		i++
	}

	// First pass on joints, skip gear joints.
	for j := world.M_jointList; j != nil; j = j.GetNext() {
		if j.GetType() == JointType.Gear {
			continue
		}

		fmt.Print("{\n")
		j.Dump()
		fmt.Print("}\n")
	}

	// Second pass on joints, only gear joints.
	for j := world.M_jointList; j != nil; j = j.GetNext() {
		if j.GetType() != JointType.Gear {
			continue
		}

		fmt.Print("{\n")
		j.Dump()
		fmt.Print("}\n")
	}
}
