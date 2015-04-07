
A realtime physics engine. This is a rigid body simulator. Collision detection is using GJK for testing and EPA for reporting contact points, while collision resolution uses PGS Sequential Impulses. Bodies are split into islands before the resolution phase, and guendelman shock propagation is applied to improve stable stacking. 

![StableStack](screenshots/stable-stack.gif)

Collision Detection
-------------

##Broadphase

I'm using the sweep and prune method quickly and efficiently determine determine potential contacts between AABB's of bodies. Sweep and Prune makes great use of temporal coherence since bodies don't move much between frames, they can be stored in arrays for each axis and compared with bodies in the same region of the array.

##Narrowphase

Contacts from the broadphase are passed into the narrowphase for accurate testing of contact. I've adopted the commonly used GJK and EPA algorithms because of their efficiency and versatility in accurate collision detection.

##GJK

GJK is an efficient and versatile intersection testing algorithm. This algorithm is widely adopted in many games because of its efficiency and scalability to handle many shapes. GJK only works with convex polyhedrons, so more complicated bodies must be split into multiple shapes. GJK also will only tell you if the bodies are colliding, but not where.

##EPA

If a collision is detected from GJK, the terminated simplex from GJK is passed to EPA to be blown up inside of the minkowski configuration between both bodies. From this we can find two points of intersection (one for each body), a normal axis, and a contact depth. Sadly EPA only reports the deepest point of contact, so I've implemented contact caching between bodies in order to maintain the contact manifold between two bodies.

Collision Resolution
-------------


##Sequential Impulses

	TODO: Explanation/Descriptions




Fun
-------

| | | |
|---|---|---|
| ![Baumgarte](screenshots/baumgarte.gif) | ![Funnel](screenshots/funnel.gif) | ![Ball](screenshots/ballrolling.gif) |
