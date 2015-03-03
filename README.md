
Collision Detection
	Local vs. Global
	Jacobian, PGS
	LCP
	Narrowphase not necessary (used for mesh objects w/ multiple primitives)
	> Storage
		- R-Trees?  K-d Trees?
	> Broadphase
		- Bounding Boxes & Sweep and Prune?  --- is this necessary for non-AABB boxes?
	> Narrowphase
		- GJK or SAT?
	> Contact Points & Normals
	> Vertex/Face, Edge-Edge?

Collision Resolution
	Avoiding large mass ratios
	Only resolves contacts found from collision detection
	CCD vs. DCD  -- CCD uses continuous collision query (raycast in direction that the object moved)
	> LCP PGS (Sequential Impulse)
	> Restitution
	> Warm starting
		- Iterative solver requires initial guess for lambda; store lambda from previous time step and use that as initial guess for next time step
	> Slop
		- Allow some penetration to avoid overshooting impulse
	> Caching?
		- Contact caching: cache contacts at end of timestep; on next timestep find all new contacts, and if a cached contact is found then use the lambda calculated from that cache
	> Sleeping/Freezing
	> Accumulated impulses + Clamping
		- For each constraint keep track of the total impulse applied (accumulation); clamp this total impulse to not be negative
		- Hence, corrective impulses can be negative but total impulse will be positive (avoids jiterring)



Questions:
	> What is the process? (predict collisions and THEN resolve each individual contact?  for i=0..maxIters { for c=0...nCollisions { resole(); } } ??)
	> How does iterative solver find corrective impulse, and how does it add impulses to total impulse? (also, what is a positive/negative impulse in this case, for clamping)
	> What is lambda in the iterative solver (used in warm starting)
	> How to find contact points & normals
	> Is sleeping/freezing useful?
	> Collision detection: finds predicted collisions?
