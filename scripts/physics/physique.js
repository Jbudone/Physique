define(['physics/collision/narrowphase', 'physics/collision/island', 'physics/collision/broadphase', 'physics/memstore'], function(Narrowphase, Island, Broadphase, MemStore){

	Keys.add('BODY_CUBE');
	Keys.add('BODY_SPHERE');
	Keys.add('BODY_FLOOR');


	var Physique = function(){

		this.bodies = {};
		var physique = this;
		var dt = 0;

		this.world = {
			gravity: -10.0,
			scaleTime: 0.001,// 0.0002,//0.001,

			slop: -0.0001,
			maxDepth: 4.0, // before falling through
			damping: 0.98,//0.95, // FIXME: needed for box stacking?
			warmth: 0.9,
			baumgarte: 0.2, // FIXME: helps with faster penetration solving
			restitution: 0.0, // FIXME: found as 0.25 in other places ... NOTE: one source suggests that this is negative!
			friction: 0.5, // sqrt( mu1 * mu2 )  FIXME: 0.1 for better box stacking stability
			minVel: 0.000,
			minToSleep: 0.1,//0.004,
			minIterationsBeforeSleep: 2,//2,

			velocityIterations: 5,

			enableIslands: false,

			maxContactsInManifold: 4,
			runOnce: null
		};

		var TEST_ScaleT1 = 1,
			TEST_ScaleT2 = -1,
			TEST_T1Factor = 1,
			TEST_T2Factor = 1,
			TEST_DenomFactor1 = 1,
			TEST_DenomFactor2 = 1,
			TEST_NormTangent = true,
			resetDeltas = true;

		var mass = 1.0,//0.60,
			inertia = (1/3) * mass;

		/*****************************************************************
		 *****************************************************************
		 						Broadphase Collision                         
		 */

		/*****************************************************************
		 *****************************************************************
		 						Narrowphase Collision                         
		 */


		/*****************************************************************
		 *****************************************************************
		 						Collision Resolution
		 */

		var oldHits = null;

		var Constraint = function(){

			this.J = new Array(12);
			this.lagrange = 0;
			this.D = null;
			this.B = new Array(12);
			this.JdotV = 0;

		};

		var Contact = function(contact, manifold){

			this.bodyA = contact.bodyA;
			this.bodyB = contact.bodyB;

			this.manifold = manifold;

			this.normal = contact.normal;
			this.vertexA = contact.originA;
			this.vertexB = contact.originB;
			this.localA = contact.localA;
			this.localB = contact.localB;
			this.depth = contact.depth;

			// Find a tangential basis: http://box2d.org/2014/02/computing-a-basis/
			if (Math.abs(this.normal.x) >= 0.57735) {
				this.tangent1 = new THREE.Vector3( this.normal.y, -this.normal.x, 0.0 );
			} else {
				this.tangent1 = new THREE.Vector3( 0.0, this.normal.z, -this.normal.y );
			}
			this.tangent1.normalize();
			this.tangent2 = new THREE.Vector3();
			this.tangent2.copy(this.normal).cross(this.tangent1);

			this.impulse = 0;
			this.impulseT1 = 0;
			this.impulseT2 = 0;
			this.appliedImpulse =0;
			this.appliedPushImpulse = 0;

			this.rows = [];

			this.isPointInContact = function(point, body){

				for (var i=0; i<body.geometry.faces.length; ++i) {
					var face = body.geometry.faces[i],
						pointNorm = body.geometry.vertices[face.a].clone().applyQuaternion(body.quaternion).add(body.position).sub(point);
					if (face.normal.clone().applyQuaternion(body.quaternion).dot(pointNorm) < 0) {
						debugger;
						return false;
					}
				}
				return true;
			};

			this.remove = function(){

			};

			this.addConstraint = function(){
				this.rows.push(new Constraint());
				var row = this.rows[this.rows.length-1];
				this.setJacobian(row);
			};

			this.setJacobian = function(row){

				if (this.bodyA && this.bodyA.invMass !== 0) {
					row.J[0] = this.normal.x;
					row.J[1] = this.normal.y;
					row.J[2] = this.normal.z;

					var tangent = this.vertexA.clone().sub(this.bodyA.position).multiplyScalar(TEST_ScaleT1).cross( this.normal ); // FIXME: WHAT IS THIS?!

					if (TEST_NormTangent) {
						tangent.normalize();
					}

					row.J[3] = tangent.x;
					row.J[4] = tangent.y;
					row.J[5] = tangent.z;
				} else {
					row.J[0] = 0; row.J[1] = 0; row.J[2] = 0;
					row.J[3] = 0; row.J[4] = 0; row.J[5] = 0;
				}

				if (this.bodyB && this.bodyB.invMass !== 0) {
					row.J[6] = -this.normal.x;
					row.J[7] = -this.normal.y;
					row.J[8] = -this.normal.z;

					var tangent = this.vertexB.clone().sub(this.bodyB.position).multiplyScalar(TEST_ScaleT2).cross( this.normal ); // FIXME: WHAT IS THIS?!

					if (TEST_NormTangent) {
						tangent.normalize();
					}
					row.J[9]  = tangent.x;
					row.J[10] = tangent.y;
					row.J[11] = tangent.z;
				} else {
					row.J[6] = 0; row.J[7] = 0; row.J[8] = 0;
					row.J[9] = 0; row.J[10] = 0; row.J[11] = 0;
				}
			};

			this.addConstraint();

			this.update = function(newInfo){

				if (newInfo !== undefined) {
					this.normal = contact.normal;
					this.vertexA = contact.originA;
					this.vertexB = contact.originB;
					if (Math.abs(this.normal.x) >= 0.57735) {
						this.tangent1 = new THREE.Vector3( this.normal.y, -this.normal.x, 0.0 );
					} else {
						this.tangent1 = new THREE.Vector3( 0.0, this.normal.z, -this.normal.y );
					}
					this.tangent1.normalize();
					this.tangent2 = new THREE.Vector3();
					this.tangent2.copy(this.normal).cross(this.tangent1);

					// if (TEST_NormTangent) {
					// 	this.tangent.normalize();
					// 	this.tangent2.normalize();
					// }
					this.depth = contact.depth;
					this.setJacobian(this.rows[0]);
				}


				this.JDiagABInv = 0;
				var denom1 = 0, denom2 = 0;
				if (this.bodyA && this.bodyA.invMass !== 0) {
					// denom1 = this.bodyA.invMass + this.normal.dot( this.tangent.clone().multiplyScalar(this.bodyA.invInertiaTensor).cross( this.vertexA.clone().sub(this.bodyA.position) ) );
					denom1 = this.bodyA.invMass + this.normal.dot( this.tangent1.clone().multiplyScalar(TEST_DenomFactor1*this.bodyA.invInertiaTensor).cross( this.vertexA.clone().sub(this.bodyA.position) ) );
				}

				if (this.bodyB && this.bodyB.invMass !== 0) {
					denom2 = this.bodyB.invMass - this.normal.dot( this.tangent2.clone().multiplyScalar(TEST_DenomFactor2*this.bodyB.invInertiaTensor).cross( this.vertexB.clone().sub(this.bodyB.position) ) );
				}

				var relaxation = 1.0;
				this.JDiagABInv = relaxation / (denom1 + denom2);








				for (var i=0; i<this.rows.length; ++i) {
					var row = this.rows[i];

					// Compute B  (J*M^-1)
					if (this.bodyA && this.bodyA.invMass !== 0) {
						row.B[0] = this.bodyA.invMass * row.J[0];
						row.B[1] = this.bodyA.invMass * row.J[1];
						row.B[2] = this.bodyA.invMass * row.J[2];

						row.B[3] = this.bodyA.invInertiaTensor * row.J[3]; // FIXME: THIS IS TOTALLY WRONG FOR INERTIA
						row.B[4] = this.bodyA.invInertiaTensor * row.J[4];
						row.B[5] = this.bodyA.invInertiaTensor * row.J[5];
					} else {
						row.B[0] = 0; row.B[1] = 0; row.B[2] = 0;
						row.B[3] = 0; row.B[4] = 0; row.B[5] = 0;
					}

					if (this.bodyB && this.bodyB.invMass !== 0) {
						row.B[6] = this.bodyB.invMass * row.J[6];
						row.B[7] = this.bodyB.invMass * row.J[7];
						row.B[8] = this.bodyB.invMass * row.J[8];

						row.B[9]  = this.bodyB.invInertiaTensor * row.J[9]; // FIXME: THIS IS TOTALLY WRONG FOR INERTIA
						row.B[10] = this.bodyB.invInertiaTensor * row.J[10];
						row.B[11] = this.bodyB.invInertiaTensor * row.J[11];
					} else {
						row.B[6] = 0; row.B[7] = 0; row.B[8] = 0;
						row.B[9] = 0; row.B[10] = 0; row.B[11] = 0;
					}


					// Compute D  ...?
					row.D = 0;
					for (var ji=0; ji<12; ++ji){
						row.D += row.B[ji] * row.J[ji];
					}


					row.JdotV = 0;
					var idt = 1/dt;
					if (this.bodyA && this.bodyA.invMass !== 0) {
						row.JdotV += row.J[0] * idt * this.bodyA.velocity.x;
						row.JdotV += row.J[1] * idt * this.bodyA.velocity.y;
						row.JdotV += row.J[2] * idt * this.bodyA.velocity.z;

						row.JdotV += row.J[3] * idt * this.bodyA.angularVelocity.x;
						row.JdotV += row.J[4] * idt * this.bodyA.angularVelocity.y;
						row.JdotV += row.J[5] * idt * this.bodyA.angularVelocity.z;
					}

					if (this.bodyB && this.bodyB.invMass !== 0) {
						row.JdotV += row.J[6]  * idt * this.bodyB.velocity.x;
						row.JdotV += row.J[7]  * idt * this.bodyB.velocity.y;
						row.JdotV += row.J[8]  * idt * this.bodyB.velocity.z;

						row.JdotV += row.J[9]  * idt * this.bodyB.angularVelocity.x;
						row.JdotV += row.J[10] * idt * this.bodyB.angularVelocity.y;
						row.JdotV += row.J[11] * idt * this.bodyB.angularVelocity.z;
					}

					var velNormal = this.bodyA.angularVelocity.clone().cross( this.vertexA ).add( this.bodyA.position ).dot( this.normal );
					velNormal 	 -= this.bodyB.angularVelocity.clone().cross( this.vertexB ).add( this.bodyB.position ).dot( this.normal );
					var bias = velNormal * 0.1;
					row.JdotV -= (bias * idt);
				}
			};
		};

		var islands = {};

		var Manifold = function(bodyA, bodyB, uid){

			this.bodyA = bodyA;
			this.bodyB = bodyB;
			this.contacts = {};
			this.length = 0;

			this.island = null;
			this.indexInIsland = null;
			this.shockLevel = null;
			this.uid = uid;

			this.hashContactVerts = function(contact){
				return Math.max(contact.originA.i, contact.originB.i) * 10000 + Math.min(contact.originA.i, contact.originB.i);
			};

			this.updateContact = function(contact){

				if (contact.depth < physique.world.slop) return; // FIXME: slop
				contact.depth -= physique.world.slop;
				var hash = this.hashContactVerts(contact);
				contact.vHash = hash;
				if (this.contacts.hasOwnProperty(hash)) {
					this.contacts[hash].update(contact);
					physique.onUpdateContact(contact.hash + hash, contact.originA);
					physique.onUpdateContact(contact.hash + hash + 'B', contact.originB);
				} else {
					this.addContact(hash, contact);
				}
				
			};

			this.addContact = function(hash, contact){

				// for (var contactID in this.contacts) {
				// 	var _contact = this.contacts[contactID];
				// 	if (_contact.vertexA.i == contact.originA.i) {
				// 	// if (_contact.vertexA.clone().sub(contact.originA).lengthSq() < 0.02) {
				// 		physique.onRemoveContact(this.contacts[contactID].hash + this.contacts[contactID].vHash);
				// 		this.contacts[contactID].remove();
				// 		delete this.contacts[contactID];
				// 		--this.length;
				// 	}
				// }

				if (this.length >= physique.world.maxContactsInManifold) {
					// Replace weakest contact with this one
					var weakestContactDepth = -99999,
						weakestContactI = null;
					for (var contactID in this.contacts) {
						if (this.contacts[contactID].depth > weakestContactDepth) {
							weakestContactDepth = this.contacts[contactID].depth;
							weakestContact = contactID;
						}
					}

					// physique.onRemoveContact(this.contacts[contactID].uid, this.contacts[contactID].vertexA);
					// physique.onRemoveContact(this.contacts[contactID].uid, this.contacts[contactID].vertexB);
					physique.onRemoveContact(this.contacts[contactID].hash + this.contacts[contactID].vHash);
					physique.onRemoveContact(this.contacts[contactID].hash + this.contacts[contactID].vHash + 'B');
					this.contacts[contactID].remove();
					delete this.contacts[contactID];

					--this.length;
				}

				++this.length;
				physique.onNewContact(contact.hash + hash, contact.originA);
				physique.onNewContact(contact.hash + hash + 'B', contact.originB);
				var _contact = new Contact(contact, this);
				_contact.hash = contact.hash;
				_contact.vHash = hash;
				this.contacts[hash] = _contact;
			};

			this.update = function(){
				// TODO: check position of contacts (model space) to see if they're far enough away and no
				// longer in contact:
				// https://github.com/chandlerprall/GoblinPhysics/blob/master/src/classes/ContactManifold.js
				// :186

				for (var contactID in this.contacts) {
					var contact = this.contacts[contactID];

					// var worldA = contact.vertexA.clone().applyQuaternion(contact.bodyA.quaternion).add(contact.bodyA.position),
					// 	worldB = contact.vertexB.clone().applyQuaternion(contact.bodyB.quaternion).add(contact.bodyB.position),
					// 	diff   = worldA.sub(worldB);

					// var newVertexA = contact.bodyA.geometry.vertices[ contact.vertexA.i ].clone().applyQuaternion( contact.bodyA.quaternion ).add( contact.bodyA.position );
					// var diff = newVertexA.clone().sub( contact.vertexA );

					// var diff = contact.vertexB.clone().sub( contact.vertexA.clone().sub( contact.normal.clone().multiplyScalar( contact.depth ) ) );

					// var diff = contact.vertexA.clone().sub( contact.vertexB );

					// var normalLine = new THREE.Line3( contact.vertexB, (new THREE.Vector3()).copy(contact.vertexB).add(contact.normal.clone().multiplyScalar(contact.depth)) );
					// var normalLine = new THREE.Line3( contact.vertexA.clone().add(contact.normal.clone().multiplyScalar(contact.depth)), contact.vertexA.clone() );

					// var diff = normalLine.closestPointToPoint( newVertexA ).sub( contact.vertexA );
					// if (diff.lengthSq() > contact.depth * contact.depth + 0.0001) {



					var newContactA = contact.localA.clone().applyQuaternion(contact.bodyA.quaternion).add(contact.bodyA.position),
						newContactB = contact.localB.clone().applyQuaternion(contact.bodyB.quaternion).add(contact.bodyB.position),
						newDistance = newContactA.clone().sub(newContactB).dot(contact.normal);

					// var newDistance = contact.bodyA.geometry.vertices[contact.vertexA.iA].clone().applyQuaternion(contact.bodyA.quaternion).add(contact.bodyA.position).sub( contact.bodyB.geometry.vertices[contact.vertexB.iB].clone().applyQuaternion(contact.bodyB.quaternion).add(contact.bodyB.position) ).dot( contact.normal );
					if (newDistance >= -physique.world.slop || newDistance < -physique.world.maxDepth) {
						physique.onRemoveContact(contact.hash + contact.vHash);
						physique.onRemoveContact(contact.hash + contact.vHash + 'B');
						contact.remove();
						delete this.contacts[contactID];
						--this.length;
						continue;
					}


					// Contact points are too far away along the plane orthogonal to the normal axis
					var diffBetweenPoints = newContactB.clone().sub( newContactA.clone().add( contact.normal.clone().multiplyScalar(-newDistance) ) );
					if (diffBetweenPoints.lengthSq() > 0.09) {
						physique.onRemoveContact(contact.hash + contact.vHash);
						physique.onRemoveContact(contact.hash + contact.vHash + 'B');
						contact.remove();
						delete this.contacts[contactID];
						--this.length;
						continue;
					}



					newContactA.i = contact.vertexA.i;
					newContactA.iA = contact.vertexA.iA;
					newContactB.i = contact.vertexB.i;
					newContactB.iB = contact.vertexB.iB;

					contact.vertexA = newContactA;
					contact.vertexB = newContactB;

					// if (!contact.isPointInContact(newVertexA, contact.bodyB)) {
					// 	physique.onRemoveContact(contact.hash + contact.vHash);
					// 	contact.remove();
					// 	delete this.contacts[contactID];
					// 	--this.length;
					// 	continue;
					// }

					contact.depth = -newDistance + physique.world.slop;
					physique.onUpdateContact(contact.hash + contact.vHash, contact.vertexA);
					physique.onUpdateContact(contact.hash + contact.vHash + 'B', contact.vertexB);

					contact.update();
				}
			};
		};


		var contactManifolds = {};
		this.updateContact = function(contact){
			if (!contactManifolds.hasOwnProperty(contact.hash)) {

				var manifold = null;
				if (this.world.useIslands) {
					// Find island of bodies
					var island = null;
					if (!contact.bodyA.island || !contact.bodyB.island) {
						if (!contact.bodyA.island && !contact.bodyB.island) {
							// Neither bodies belong to an island yet
						} else if (!contact.bodyA.island) {
							contact.bodyA.island = contact.bodyB.island;
						} else {
							contact.bodyB.island = contact.bodyA.island;
						}
						island = contact.bodyA.island;
					} else if (contact.bodyA.island == contact.bodyB.island) {
						island = contact.bodyA.island;
					} else {
						// Both bodies belong to different islands..need to merge the islands
						island = contact.bodyA.island.mergeWith( contact.bodyB.island );
					}

					if (!island) {
						island = new Island(contact.hash);
						islands[island.uid] = island;
					}

					var manifold = new Manifold(contact.bodyA, contact.bodyB, contact.hash);
					contactManifolds[contact.hash] = manifold;
					island.addManifold(manifold);

				} else {
					var manifold = new Manifold(contact.bodyA, contact.bodyB, contact.hash);
					contactManifolds[contact.hash] = manifold;
				}

				manifold.bodyA.addManifold(manifold);
				manifold.bodyB.addManifold(manifold);
			}

			contactManifolds[contact.hash].updateContact(contact);
		};

		this.removeManifold = function(hash){
			if (contactManifolds[hash]) {
				var manifold = contactManifolds[hash];
				manifold.bodyA.removeManifold(manifold);
				manifold.bodyB.removeManifold(manifold);
				if (this.world.useIslands && manifold.island) {
					manifold.island.removeManifold(manifold);
					if (manifold.island.count == 0) {
						delete islands[manifold.island.uid];
					}
				}

				for (var contactID in manifold.contacts) {
					var contact = manifold.contacts[contactID];
					physique.onRemoveContact(contact.hash + contact.vHash);
					physique.onRemoveContact(contact.hash + contact.vHash + 'B');
				}
				delete contactManifolds[hash];
			}
		};

		this.updateManifolds = function(){

			for (var manifoldID in contactManifolds) {
				var manifold = contactManifolds[manifoldID];
				manifold.update();

				if (_.isEmpty(manifold.contacts)) {
					if (this.world.useIslands && manifold.island) manifold.island.removeManifold(manifold);
					delete contactManifolds[manifoldID];
				}
			}
		};

		this.solveConstraints = function(){

			for (var manifoldID in contactManifolds) {

				var manifold = contactManifolds[manifoldID];
				for (var contactID in manifold.contacts) {

					var contact = manifold.contacts[contactID];

					if ((contact.bodyA.asleep || contact.bodyA.static) && (contact.bodyB.asleep || contact.bodyB.static)) {
						continue;
					}
					// contact.appliedImpulse = 0;

					// ReactPhysics way
					// Warmstart
					contact.impulse *= physique.world.warmth;
					var rA = contact.vertexA.clone().sub(contact.bodyA.position),
						rB = contact.vertexB.clone().sub(contact.bodyB.position);
					contact.bodyA.velocity.add( contact.normal.clone().multiplyScalar(contact.bodyA.invMass * contact.impulse) );
					contact.bodyA.angularVelocity.add( rA.clone().cross(contact.normal).multiplyScalar(contact.bodyA.invInertiaTensor * contact.impulse) );

					contact.bodyB.velocity.sub( contact.normal.clone().multiplyScalar(contact.bodyB.invMass * contact.impulse) );
					contact.bodyB.angularVelocity.sub( rB.clone().cross(contact.normal).multiplyScalar(contact.bodyB.invInertiaTensor * contact.impulse) );

					// Friction 1 Warmstart
					contact.impulseT1 *= physique.world.warmth;
					contact.bodyA.velocity.add( contact.tangent1.clone().multiplyScalar(-contact.bodyA.invMass * contact.impulseT1) );
					contact.bodyA.angularVelocity.add( rA.clone().cross(contact.tangent1).multiplyScalar(-contact.bodyA.invInertiaTensor * contact.impulseT1) );

					contact.bodyB.velocity.add( contact.tangent1.clone().multiplyScalar(contact.bodyB.invMass * contact.impulseT1) );
					contact.bodyB.angularVelocity.add( rB.clone().cross(contact.tangent1).multiplyScalar(contact.bodyB.invInertiaTensor * contact.impulseT1) );


					// Friction 2 Warmstart
					contact.impulseT2 *= physique.world.warmth;
					contact.bodyA.velocity.add( contact.tangent2.clone().multiplyScalar(-contact.bodyA.invMass * contact.impulseT2) );
					contact.bodyA.angularVelocity.add( rA.clone().cross(contact.tangent2).multiplyScalar(-contact.bodyA.invInertiaTensor * contact.impulseT2) );

					contact.bodyB.velocity.add( contact.tangent2.clone().multiplyScalar(contact.bodyB.invMass * contact.impulseT2) );
					contact.bodyB.angularVelocity.add( rB.clone().cross(contact.tangent2).multiplyScalar(contact.bodyB.invInertiaTensor * contact.impulseT2) );




					contact.appliedPushImpulse = 0;
					// contact.impulse = 0;
				}
			}

			for (var uid in physique.bodies) {
				var body = physique.bodies[uid];

				if (resetDeltas) {
					body.deltaV.multiplyScalar(0);
					body.deltaW.multiplyScalar(0);
				}
			}



			if (this.world.useIslands) {

			var TEST_ENABLE_SHOCK_REWEIGHT = true;

					for (var iteration=0; iteration<physique.world.velocityIterations; ++iteration) {
					for (var islandID in islands) {
						var island = islands[islandID];
						for (var shockLevel=0; shockLevel<island.manifolds.length; ++shockLevel) {
						
							if (TEST_ENABLE_SHOCK_REWEIGHT) {
								if (shockLevel>0) {
									for (var mI=0; mI<island.manifolds[shockLevel-1].length; ++mI) {
										var manifold = island.manifolds[shockLevel-1][mI];

										if (!manifold.bodyA.static && !manifold.bodyA.hasOwnProperty('storedInvMass')) {
											manifold.bodyA.storedInvMass = manifold.bodyA.invMass;
											manifold.bodyA.storedInvInertiaTensor = manifold.bodyA.invInertiaTensor;
											manifold.bodyA.invMass = 0;
											manifold.bodyA.invInertiaTensor = 0;
										}

										if (!manifold.bodyB.static && !manifold.bodyB.hasOwnProperty('storedInvMass')) {
											manifold.bodyB.storedInvMass = manifold.bodyB.invMass;
											manifold.bodyB.storedInvInertiaTensor = manifold.bodyB.invInertiaTensor;

											manifold.bodyB.invMass = 0;
											manifold.bodyB.invInertiaTensor = 0;
										}
									}
								}
							}

							for (var manifoldI=0; manifoldI<island.manifolds[shockLevel].length; ++manifoldI) {
								var manifold = island.manifolds[shockLevel][manifoldI];
					// for (var manifoldID in contactManifolds) {

					// 	var manifold = contactManifolds[manifoldID];
						for (var contactID in manifold.contacts) {

							var contact = manifold.contacts[contactID];


							this.solveContact(contact);


							}
						}
					}
					if (TEST_ENABLE_SHOCK_REWEIGHT) {
						for (var shockLevel=0; shockLevel<island.manifolds.length; ++shockLevel) {
						
							for (var mI=0; mI<island.manifolds[shockLevel].length; ++mI) {
								var manifold = island.manifolds[shockLevel][mI];
								if (manifold.bodyA.hasOwnProperty('storedInvMass')) {
									manifold.bodyA.invMass = manifold.bodyA.storedInvMass;
									manifold.bodyA.invInertiaTensor = manifold.bodyA.storedInvInertiaTensor;
									delete manifold.bodyA.storedInvMass;
									delete manifold.bodyA.storedInvInertiaTensor;
								}
								if (manifold.bodyB.hasOwnProperty('storedInvMass')) {
									manifold.bodyB.invMass = manifold.bodyB.storedInvMass;
									manifold.bodyB.invInertiaTensor = manifold.bodyB.storedInvInertiaTensor;
									delete manifold.bodyB.storedInvMass;
									delete manifold.bodyB.storedInvInertiaTensor;
								}
							}
						}
					}
						} } // For shock propagation

			} else {

				for (var iteration=0; iteration<physique.world.velocityIterations; ++iteration) {
					for (var manifoldID in contactManifolds) {

						var manifold = contactManifolds[manifoldID];
						for (var contactID in manifold.contacts) {

							var contact = manifold.contacts[contactID];
							this.solveContact(contact);
						}
					}
				}

			}

		};


		this.solveContact = function(contact){
						var row = contact.rows[0], // contact constraint
							JdotV = 0,
							delta = 0;




						var rA = contact.vertexA.clone().sub(contact.bodyA.position),
							rB = contact.vertexB.clone().sub(contact.bodyB.position),
							dV = 0,
							deltaVDotN = 0,
							JV = 0,
							MA = 0,
							MB = 0,
							deltaLambda = 0,
							lambdaTemp = 0;




						// ReactPhysics3D way
						// FIXME: may need to reverse normal (they use triangle.normal)
						// FIXME: rA = pA - a, rB = pB - b ... they use 2 contact points.. transform contact
						// FIXME: double check effective mass: http://danielchappuis.ch/download/ConstraintsDerivationRigidBody3D.pdf  
						// point into A or B space (probably rotate/add position). 
						// 	pA = triangle->computeClosestPointOfObject(suppPointsA)
						// 	pB = body2Tobody1.getInverse() * triangle->computeClosestPointOfObject(suppPointsB)
						var rA = contact.vertexA.clone().sub(contact.bodyA.position),
							rB = contact.vertexB.clone().sub(contact.bodyB.position),
							dV = contact.bodyB.velocity.clone().add( contact.bodyB.angularVelocity.clone().cross(rB) );
						dV.sub(  contact.bodyA.velocity.clone().add( contact.bodyA.angularVelocity.clone().cross(rA) ) );

						var deltaVDotN = -dV.dot(contact.normal),
							JV = deltaVDotN;

						var b = (contact.depth - physique.world.slop) * (-physique.world.baumgarte * (dt * 200.0)) - JV * physique.world.restitution; // TODO: restitution

						// var MA = new THREE.Vector3(), MB = new THREE.Vector3();
						var MA = 0, MB = 0;
						if (contact.bodyA.invMass !== 0) {
							// MA = contact.bodyA.invMass + contact.bodyA.invInertiaTensor * (rA.clone().cross(contact.normal.clone().negate()).cross(rA)).dot(contact.normal.clone().negate());
							MA = contact.bodyA.invMass + (rA.clone().multiplyScalar(contact.bodyA.invInertiaTensor).cross(contact.normal.clone().negate()).cross(rA)).dot(contact.normal.clone().negate());
							// MA = contact.bodyA.invMass + (contact.vertexA.clone().cross(contact.normal.clone().negate()).multiplyScalar(contact.bodyA.invInertiaTensor)).cross(contact.vertexA).dot(contact.normal.clone().negate());
							// MA = contact.bodyA.invMass + contact.bodyA.invInertiaTensor * (contact.vertexA.clone().cross(contact.normal.clone().negate())).cross(contact.vertexA).dot(contact.normal.clone().negate());
							// var mass = contact.bodyA.invMass,
							// 	iner = contact.bodyA.invInertiaTensor,
							// 	norm = contact.normal.clone().negate(),
							// 	tang = rA.clone().cross(norm);
							// // MA = norm.clone().multiplyScalar(mass);
							// // MA.add( rA.clone().cross(norm).multiplyScalar(iner).cross(rA) );
							// MA = mass * Math.pow(norm.x, 2) + mass * Math.pow(norm.y, 2) + mass * Math.pow(norm.z, 2); // FIXME: == mass  (since norm.dot(norm) == 1)
							// MA += iner * Math.pow(tang.x, 2) + iner * Math.pow(tang.y, 2) + iner * Math.pow(tang.z, 2);
						}
						if (contact.bodyB.invMass !== 0) {
							// MB = contact.bodyB.invMass + contact.bodyB.invInertiaTensor * (rB.clone().cross(contact.normal.clone().negate()).cross(rB)).dot(contact.normal.clone().negate());
							MB = contact.bodyB.invMass + (rB.clone().multiplyScalar(contact.bodyB.invInertiaTensor).cross(contact.normal.clone().negate()).cross(rB)).dot(contact.normal.clone().negate());
							// MB = contact.bodyB.invMass + (contact.vertexB.clone().cross(contact.normal.clone().negate()).multiplyScalar(contact.bodyB.invInertiaTensor)).cross(contact.vertexB).dot(contact.normal.clone().negate());
							// MB = contact.bodyB.invMass + contact.bodyB.invInertiaTensor * (contact.vertexB.clone().cross(contact.normal.clone().negate())).cross(contact.vertexB).dot(contact.normal.clone().negate());
							// var mass = contact.bodyB.invMass,
							// 	iner = contact.bodyB.invInertiaTensor,
							// 	norm = contact.normal.clone().negate(),
							// 	tang = rB.clone().cross(norm);
							// // MB = norm.clone().multiplyScalar(-mass);
							// // MB.sub( rB.clone().cross(norm.clone().negate()).multiplyScalar(iner).cross(rB) );
							// MB = mass * Math.pow(norm.x, 2) + mass * Math.pow(norm.y, 2) + mass * Math.pow(norm.z, 2);
							// MB += iner * Math.pow(tang.x, 2) + iner * Math.pow(tang.y, 2) + iner * Math.pow(tang.z, 2);
						}
						// var Meffective = MA.add(MB).dot(contact.normal.clone().negate());// 1 / (MA + MB);
						var Meffective =  (MA + MB);
						var deltaLambda = -(JV + b) / Meffective,
							lambdaTemp = contact.impulse;

						contact.impulse = Math.max(contact.impulse + deltaLambda, 0.0);
						deltaLambda = contact.impulse - lambdaTemp;



						contact.bodyA.velocity.add( contact.normal.clone().multiplyScalar(contact.bodyA.invMass * deltaLambda) );
						contact.bodyA.angularVelocity.add( rA.clone().cross(contact.normal).multiplyScalar(contact.bodyA.invInertiaTensor * deltaLambda) );

						contact.bodyB.velocity.sub( contact.normal.clone().multiplyScalar(contact.bodyB.invMass * deltaLambda) );
						contact.bodyB.angularVelocity.sub( rB.clone().cross(contact.normal).multiplyScalar(contact.bodyB.invInertiaTensor * deltaLambda) );

						// Friction 1
						dV = contact.bodyB.velocity.clone().add( contact.bodyB.angularVelocity.clone().cross(rB) );
						dV.sub(  contact.bodyA.velocity.clone().add( contact.bodyA.angularVelocity.clone().cross(rA) ) );

						deltaVDotN = dV.dot(contact.tangent1);
						JV = deltaVDotN;

						var friction1Mass = 0.0,
							friction2Mass = 0.0;
						MA = 0;
						if (contact.bodyA.invMass !== 0) {
							var mass = contact.bodyA.invMass,
								iner = contact.bodyA.invInertiaTensor,
								tang = contact.tangent1.clone(),
								rv   = rA.clone().cross(tang);
							MA = mass * Math.pow(tang.x, 2) + mass * Math.pow(tang.y, 2) + mass * Math.pow(tang.z, 2);
							MA += iner * Math.pow(rv.x, 2) + iner * Math.pow(rv.y, 2) + iner * Math.pow(rv.z, 2);
						}

						MB = 0;
						if (contact.bodyB.invMass !== 0) {
							var mass = contact.bodyB.invMass,
								iner = contact.bodyB.invInertiaTensor,
								tang = contact.tangent1.clone(),
								rv   = rB.clone().cross(tang);
							MA = mass * Math.pow(tang.x, 2) + mass * Math.pow(tang.y, 2) + mass * Math.pow(tang.z, 2);
							MA += iner * Math.pow(rv.x, 2) + iner * Math.pow(rv.y, 2) + iner * Math.pow(rv.z, 2);
						}

						deltaLambda = -JV / (MA + MB);
						lambdaTemp = contact.impulseT1;
						var maxImpulse = physique.world.friction * contact.impulse; 
						contact.impulseT1 = Math.max(-maxImpulse, Math.min(maxImpulse, lambdaTemp + deltaLambda));
						deltaLambda = contact.impulseT1 - lambdaTemp;

						contact.bodyA.velocity.add( contact.tangent1.clone().multiplyScalar(-contact.bodyA.invMass * deltaLambda) );
						contact.bodyA.angularVelocity.add( rA.clone().cross(contact.tangent1).multiplyScalar(-contact.bodyA.invInertiaTensor * deltaLambda) );

						contact.bodyB.velocity.add( contact.tangent1.clone().multiplyScalar(contact.bodyB.invMass * deltaLambda) );
						contact.bodyB.angularVelocity.add( rB.clone().cross(contact.tangent1).multiplyScalar(contact.bodyB.invInertiaTensor * deltaLambda) );


						// Friction 2
						dV = contact.bodyB.velocity.clone().add( contact.bodyB.angularVelocity.clone().cross(rB) );
						dV.sub(  contact.bodyA.velocity.clone().add( contact.bodyA.angularVelocity.clone().cross(rA) ) );

						deltaVDotN = dV.dot(contact.tangent2);
						JV = deltaVDotN;

						var friction1Mass = 0.0,
							friction2Mass = 0.0;
						MA = 0;
						if (contact.bodyA.invMass !== 0) {
							var mass = contact.bodyA.invMass,
								iner = contact.bodyA.invInertiaTensor,
								tang = contact.tangent2.clone(),
								rv   = rA.clone().cross(tang);
							MA = mass * Math.pow(tang.x, 2) + mass * Math.pow(tang.y, 2) + mass * Math.pow(tang.z, 2);
							MA += iner * Math.pow(rv.x, 2) + iner * Math.pow(rv.y, 2) + iner * Math.pow(rv.z, 2);
						}

						MB = 0;
						if (contact.bodyB.invMass !== 0) {
							var mass = contact.bodyB.invMass,
								iner = contact.bodyB.invInertiaTensor,
								tang = contact.tangent2.clone(),
								rv   = rB.clone().cross(tang);
							MA = mass * Math.pow(tang.x, 2) + mass * Math.pow(tang.y, 2) + mass * Math.pow(tang.z, 2);
							MA += iner * Math.pow(rv.x, 2) + iner * Math.pow(rv.y, 2) + iner * Math.pow(rv.z, 2);
						}

						deltaLambda = -JV / (MA + MB);
						lambdaTemp = contact.impulseT2;
						var maxImpulse = physique.world.friction * contact.impulse;
						contact.impulseT2 = Math.max(-maxImpulse, Math.min(maxImpulse, lambdaTemp + deltaLambda));
						deltaLambda = contact.impulseT2 - lambdaTemp;

						contact.bodyA.velocity.add( contact.tangent2.clone().multiplyScalar(-contact.bodyA.invMass * deltaLambda) );
						contact.bodyA.angularVelocity.add( rA.clone().cross(contact.tangent2).multiplyScalar(-contact.bodyA.invInertiaTensor * deltaLambda) );

						contact.bodyB.velocity.add( contact.tangent2.clone().multiplyScalar(contact.bodyB.invMass * deltaLambda) );
						contact.bodyB.angularVelocity.add( rB.clone().cross(contact.tangent2).multiplyScalar(contact.bodyB.invInertiaTensor * deltaLambda) );

		};



		/*****************************************************************
		 *****************************************************************
		 							Dynamics
		 */

		this.removeBody = function(mesh){
			var body = this.bodies[mesh.uid];

			for (var manifoldID in body.manifolds) {
				var manifold = body.manifolds[manifoldID];
				if (manifold.island) manifold.island.removeManifold(manifold);

				for (var contactID in manifold.contacts) {
					var contact = manifold.contacts[contactID];
					physique.onRemoveContact(contact.hash + contact.vHash);
					physique.onRemoveContact(contact.hash + contact.vHash + 'B');
					delete manifold.contacts[contactID];
				}

				delete manifold.bodyA.manifolds[manifoldID];
				delete manifold.bodyB.manifolds[manifoldID];
				delete contactManifolds[manifoldID];
			}

			Broadphase.removeBodyInBroadphase(body);
			delete this.bodies[mesh.uid];
		};

		this.addBody = function(mesh){
			if (this.bodies.hasOwnProperty(mesh.uid)) {
				throw new Error("Body uid ("+mesh.uid+") already exists");
			}

			this.bodies[mesh.uid] = mesh.body;

			var bodyType = mesh.settings.body;
			if (bodyType === BODY_CUBE) {

				mesh.body.invMass = 1 / (mass);
				mesh.body.invInertiaTensor = 1 / (inertia);
				mesh.body.static = false;

			} else if (bodyType === BODY_SPHERE) {

				var _inertia = 2/5 * mass * Math.pow(mesh.body.radius,2);
				mesh.body.invMass = 1 / (mass);
				mesh.body.invInertiaTensor = 1 / (_inertia);
				mesh.body.static = false;

			} else if (bodyType === BODY_FLOOR) {

				mesh.body.invMass = 0.0;
				mesh.body.invInertiaTensor = 0.0;
				mesh.body.static = true;

			} else {
				throw new Error("Adding undefined body: "+ bodyType);
			}

			mesh.body.bodyType = bodyType;
			mesh.body.velocity = new THREE.Vector3();
			mesh.body.angularVelocity = new THREE.Vector3();
			mesh.body.impulse = [0,0,0,0,0,0];
			mesh.body.asleep = false;
			mesh.body.manifolds = {};
			mesh.body.island = null;
			mesh.body.wantToSleep = 0;

			mesh.body.addManifold = function(manifold){
				this.manifolds[manifold.uid] = manifold;

				if (physique.world.useIslands) {
					if (manifold.island) {
						if (this.island != manifold.island) {
							if (!this.island) {
								this.island = manifold.island;
							} else {
								// We currently belong to two islands, need to merge the two islands..
								var newIsland = this.island.mergeWith( manifold.island );
								this.island = newIsland;
							}
						}
					} else {
						debugger; // This should never occur!
					}
				}
			};

			mesh.body.removeManifold = function(manifold){
				delete this.manifolds[manifold.uid];
			};

			mesh.body.deltaV = new THREE.Vector3(0,0,0);
			mesh.body.deltaW = new THREE.Vector3(0,0,0);
			mesh.body.newVelocity = new THREE.Vector3(0,0,0);
			mesh.body.newAngularVelocity = new THREE.Vector3(0,0,0);
			mesh.body.applyImpulse = function(linear, angular, impulseMagnitude){

				this.deltaV.add( linear.clone().multiplyScalar(impulseMagnitude) );
				this.deltaW.add( angular.clone().multiplyScalar(impulseMagnitude) );

				// this.velocity.add( linear.clone().multiplyScalar(impulseMagnitude) );
				// this.angularVelocity.add( angular.clone().multiplyScalar(impulseMagnitude) );
			};


			/*
			mesh.body.updateRotation = function(dt){


				// Update rotation
				var q = new THREE.Quaternion(
					this.angularVelocity.x * dt,
					this.angularVelocity.y * dt,
					this.angularVelocity.z * dt,
					0
				);

				q.multiply( this.quaternion );

				var half_dt = 0.5;
				this.quaternion.x += half_dt * q.x;
				this.quaternion.y += half_dt * q.y;
				this.quaternion.z += half_dt * q.z;
				this.quaternion.w += half_dt * q.w;
				this.quaternion.normalize();

			};
		*/

			Broadphase.addBodyIntoBroadphase(mesh.body);
		};

		this.onDebugHelperArrow = new Function();
		this.onNewContact = new Function();
		this.onUpdateContact = new Function();
		this.onRemoveContact = new Function();
		this.step = function(delta){

			dt = delta * this.world.scaleTime;
			if (dt == 0) return;
			if (this.world.runOnce === false) return;
			if (this.world.runOnce === true) this.world.runOnce = false;
			var angularScale = new THREE.Vector3(1, 1, 1);
			for (var uid in this.bodies) {
				var body = this.bodies[uid];

				if (!body.static && !body.asleep) {

					body.storedState = {
						position: body.position.clone()
					};


					var scale = 1.0;
					body.position.x += scale * body.velocity.x * dt;
					body.position.y += scale * body.velocity.y * dt;
					body.position.z += scale * body.velocity.z * dt;

					var v = (new THREE.Vector3(body.angularVelocity.x * angularScale.x, body.angularVelocity.y * angularScale.y, body.angularVelocity.z * angularScale.z)).multiplyScalar(dt).multiplyScalar(scale);
					// 	e = (new THREE.Euler()).setFromVector3(v),
					// 	q = (new THREE.Quaternion()).setFromEuler(e);
					// body.quaternion.multiply(q);
					// body.rotation.add(v);
					var q = new THREE.Quaternion(body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z, 0);
					q.multiply(body.quaternion);
					body.quaternion.x += q.x * 0.5 * dt;
					body.quaternion.y += q.y * 0.5 * dt;
					body.quaternion.z += q.z * 0.5 * dt;
					body.quaternion.w += q.w * 0.5 * dt;
					body.quaternion.normalize();

					body.velocity.y += (this.world.gravity / body.invMass * dt);

					Broadphase.updateAABB(body);
					Broadphase.updateBodyInBroadphase(body);
				} else if (body.userMoved) {
					body.userMoved = false;
					body.asleep = false;
					Broadphase.updateAABB(body);
					Broadphase.updateBodyInBroadphase(body);
				}
			}


			this.updateManifolds(); // NOTE: need to update manifolds BEFORE updating particular contacts below
			var hits = Broadphase.sweepAndPrune();
			for (var hitHash in hits) {

				var hit = hits[hitHash],
					bodyA = hit.bodyA,
					bodyB = hit.bodyB,
					contact = null;

				if ((bodyA.asleep || bodyA.static) && (bodyB.asleep || bodyB.static)) {
					continue;
				}
				contact = Narrowphase.findContact(bodyA, bodyB);
				// contact = this.SAT(bodyA, bodyB);
				if (!contact) {
					delete hits[hitHash];
					continue;
				}

				if (bodyA.asleep) {
					bodyA.asleep = false;
					if (bodyA.material.hasOwnProperty('storedColor')) {
						bodyA.material.color.r = bodyA.material.storedColor.r;
						bodyA.material.color.g = bodyA.material.storedColor.g;
						bodyA.material.color.b = bodyA.material.storedColor.b;
						delete bodyA.material.storedColor;
					}
				}

				if (bodyB.asleep) {
					bodyB.asleep = false;
					if (bodyB.material.hasOwnProperty('storedColor')) {
						bodyB.material.color.r = bodyB.material.storedColor.r;
						bodyB.material.color.g = bodyB.material.storedColor.g;
						bodyB.material.color.b = bodyB.material.storedColor.b;
						delete bodyB.material.storedColor;
					}
				}

				contact.hash = hitHash;
				// contact.normal = contact.originB.clone().sub(contact.originA).normalize();
				// contact.normal = contact.originA.clone().sub(contact.bodyA.position).normalize();
				this.updateContact(contact);

				// var contactDir = contact.end.clone().sub(contact.origin).normalize(); // FIXME: using correct dir?
				var contactDir = contact.normal;
				// this.onDebugHelperArrow(hitHash, contactDir, contact.originA, contact.depth); //contact.end);
				this.onDebugHelperArrow(hitHash, contactDir, contact.originA, contact.depth); //contact.end);

				// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				// FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME
				// Fix collision resolution!
				//
				// http://allenchou.net/2013/12/game-physics-constraints-sequential-impulse/
				// C': JV + b = 0
				// V is the column vector: [Va; Wa; Vb; Wb]   (V velocity, W angular velocity) -- 12x1 vector
				// J (Jacobian): [Jva^T  Jwa^T  Jvb^T  Jwb^T]
				//
				// Jva: 3x1 vector of "coefficents of linear combinations of components of velocity vector"
				//
				//	Solution ΔV
				// J(V+ΔV) + b = 0
				// ΔV = M¯¹J^Tλ = M¯¹J^T(-JV - b)/(JM¯¹J^T)
				// var restitution = 18.2;
				// if (contact.depth < 0.001) contact.depth = 0;
				// bodyA.velocity.add(contactDir.clone().multiplyScalar(-1 * contact.depth * restitution));
				// bodyB.velocity.add(contactDir.clone().multiplyScalar(contact.depth * restitution));
				// FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME
				// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

				if (!bodyA.material.hasOwnProperty('storedColor') && !bodyA.static) {
					bodyA.material.storedColor = _.clone(bodyA.material.color);
					bodyA.material.color.r = bodyA.uid % 2 == 0 ? 1.0 : 0.0;
					bodyA.material.color.g = bodyA.uid % 2 == 0 ? 0.0 : 1.0;
					bodyA.material.color.b = 0.0;
				}

				if (!bodyB.material.hasOwnProperty('storedColor') && !bodyB.static) {
					bodyB.material.storedColor = _.clone(bodyB.material.color);
					bodyB.material.color.r = bodyB.uid % 2 == 0 ? 1.0 : 0.0;
					bodyB.material.color.g = bodyB.uid % 2 == 0 ? 0.0 : 1.0;
					bodyB.material.color.b = 0.0;
				}
			}

			if (oldHits) {
				for (var oldHitHash in oldHits) {
					if (!hits.hasOwnProperty(oldHitHash)) {
						this.removeManifold(oldHitHash);
						var oldHit = oldHits[oldHitHash],
							bodyA = oldHit.bodyA,
							bodyB = oldHit.bodyB;
						if (bodyA.material.hasOwnProperty('storedColor')) {
							bodyA.material.color.r = bodyA.material.storedColor.r;
							bodyA.material.color.g = bodyA.material.storedColor.g;
							bodyA.material.color.b = bodyA.material.storedColor.b;
							delete bodyA.material.storedColor;
						}

						if (bodyB.material.hasOwnProperty('storedColor')) {
							bodyB.material.color.r = bodyB.material.storedColor.r;
							bodyB.material.color.g = bodyB.material.storedColor.g;
							bodyB.material.color.b = bodyB.material.storedColor.b;
							delete bodyB.material.storedColor;
						}
					}
				}
			}
			oldHits = hits;

			this.solveConstraints();

			// Integrate changes
			for (var uid in this.bodies) {
				var body = this.bodies[uid];

				// if (body.hasOwnProperty('storedState')) {
				// 	body.position.copy(body.storedState.position);
				// 	delete body.storedState;
				// }

				if (!body.static && !body.asleep) {

					body.velocity.add(body.deltaV);
					body.angularVelocity.add(body.deltaW);

					/*
					var scale = 1.0;
					body.position.x += scale * body.velocity.x * dt;
					body.position.y += scale * body.velocity.y * dt;
					body.position.z += scale * body.velocity.z * dt;

					var v = (new THREE.Vector3(body.angularVelocity.x * angularScale.x, body.angularVelocity.y * angularScale.y, body.angularVelocity.z * angularScale.z)).multiplyScalar(dt).multiplyScalar(scale),
						e = (new THREE.Euler()).setFromVector3(v),
						q = (new THREE.Quaternion()).setFromEuler(e);
					body.quaternion.multiply(q);
					// body.rotation.x += v.x;
					// body.rotation.y += v.y;
					// body.rotation.z += v.z;
					*/

					// body.updateRotation(dt);
					// body.angularVelocity.multiplyScalar(0);

					var damping = physique.world.damping;
					body.velocity.multiplyScalar(damping);
					body.angularVelocity.multiplyScalar(damping);

					var epsV = this.world.minVel;
					if (Math.abs(body.velocity.x) < epsV) body.velocity.x = 0;
					if (Math.abs(body.velocity.y) < epsV) body.velocity.y = 0;
					if (Math.abs(body.velocity.z) < epsV) body.velocity.z = 0;
					if (Math.abs(body.angularVelocity.x) < epsV) body.angularVelocity.x = 0;
					if (Math.abs(body.angularVelocity.y) < epsV) body.angularVelocity.y = 0;
					if (Math.abs(body.angularVelocity.z) < epsV) body.angularVelocity.z = 0;

					var epsVSleep = this.world.minToSleep;
					if (Math.abs(body.velocity.x) < epsVSleep &&
						Math.abs(body.velocity.y) < epsVSleep &&
						Math.abs(body.velocity.z) < epsVSleep &&
						Math.abs(body.angularVelocity.x) < epsVSleep &&
						Math.abs(body.angularVelocity.y) < epsVSleep &&
						Math.abs(body.angularVelocity.z) < epsVSleep) {
							++body.wantToSleep;

							if (body.wantToSleep >= this.world.minIterationsBeforeSleep) {
								body.wantToSleep = 0;
								body.asleep = true;

								if (!body.static) {
									body.material.color.r = body.uid % 2 == 0 ? 0.2 : 0.4;
									body.material.color.g = body.uid % 2 == 0 ? 0.4 : 0.2;
									body.material.color.b = 0.2;
								}
							}
						}


					// body.deltaV.multiplyScalar(0);
					// body.deltaW.multiplyScalar(0);
					// body.deltaV.multiplyScalar(damping);
					// body.deltaW.multiplyScalar(damping);

					Broadphase.updateAABB(body);
					Broadphase.updateBodyInBroadphase(body);
				}
			}
		};
	};

	return Physique;
});
