define(['physics/collision/narrowphase', 'physics/collision/island', 'physics/collision/broadphase', 'physics/memstore', 'physics/body'], function(Narrowphase, Island, Broadphase, MemStore, Body){

	Keys.add('BODY_CUBE');
	Keys.add('BODY_SPHERE');
	Keys.add('BODY_TETRAHEDRON');
	Keys.add('BODY_OCTAHEDRON');
	Keys.add('BODY_FLOOR');


	var Physique = function(){

		this.bodies = {};
		var physique = this;
		var dt = 0;

		this.world = {
			gravity: -10.0,
			scaleTime: 0.001,//0.001,// 0.0002,//0.001,

			slop: 0.01,//0.001,//-0.0001,
			maxDepth: 4.0,//0.3*0.3,//4.0, // FIXME: seems like this should be a high value, but some places (bullet) use persistentContactDistanceThreshold here
			minDepth: 0,//-0.009, // FIXME: seems like this should be 0, why do some places (react) use a negative value here??
			damping: 1.0,//0.95, // FIXME: needed for box stacking? .. NOTE: any dampening is making the ball rolling problem fail!
			warmth: 0.9, // NOTE: can't use 1.0, otherwise box stack will sway.. must use a fractional value
			baumgarte: 0.2, // FIXME: helps with faster penetration solving
			restitution: 0.0, // FIXME: found as 0.25 in other places ... NOTE: one source suggests that this is negative!
			friction: 0.3, // sqrt( mu1 * mu2 )  FIXME: 0.1 for better box stacking stability
			minVel: 0.00,//0.06,
			minToSleep: 0.06,//0.01,//0.004,
			minWakeDepth: 0.1, // NOTE: 0.04 too small
			minIterationsBeforeSleep: 8*60,//20, // NOTE: 20 minimum for ball rolling problem

			velocityIterations: 10,

			solveWorstContactsFirst: true,
			useIslands: false,
			useIslandsDEBUG: false,
			onlyRemoveOnePointPerStep: true,

			persistentContactDistanceThreshold: 0.3,
			closestContactDistanceThreshold: 0.001, // FIXME: find a good number..
			contactsByFeatures: false,

			maxContactsInManifold: 4,
			runOnce: null
		};


		/*****************************************************************
		 *****************************************************************
		 						Collision Resolution
		 */

		var oldHits = null;

		// FIXME: REMOVE THIS
		var badVec = function(v){ return (isNaN(v.x) || isNaN(v.y) || isNaN(v.z) ||
											v.x == Infinity || v.y == Infinity || v.z == Infinity ||
											v.x == -Infinity || v.y == -Infinity || v.z == -Infinity ||
											v.x >= 1e10 || v.y >= 1e10 || v.z >= 1e10 ||
											v.x <= -1e10 || v.y <= -1e10 || v.z <= -1e10); };
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

			this.localA = contact.localA;
			this.localB = contact.localB;
			this.refPoint = contact.refPoint; // midpoint between vertexA and vertexB



			this.impulse = 0;
			this.impulseT1 = 0;
			this.impulseT2 = 0;
			this.appliedImpulse =0;


			this.rAT1 = null;
			this.rAT1rA = null;
			this.rBT1 = null;
			this.rBT1rB = null;
			this.rAT2 = null;
			this.rAT2rA = null;
			this.rBT2 = null;
			this.rBT2rB = null;
			this.rAN  = null;
			this.rANrA  = null;
			this.rBN  = null;
			this.rBNrB  = null;

			this.MAT1 = null;
			this.MBT1 = null;
			this.MAT2 = null;
			this.MBT2 = null;
			this.MAN = null;
			this.MBN = null;

			this.remove = function(){
				// this.rAT1.delete(); this.rAT1rA.delete();
				// this.rBT1.delete(); this.rBT1rB.delete();

				// this.rAT2.delete(); this.rAT2rA.delete();
				// this.rBT2.delete(); this.rBT2rB.delete();

				// this.rAN.delete(); this.rANrA.delete();
				// this.rBN.delete(); this.rBNrB.delete();
			};

			this.update = function(newInfo){

				if (newInfo !== undefined) {
					this.normal = contact.normal;
					this.vertexA = contact.originA;
					this.vertexB = contact.originB;
					this.rA = contact.originA.clone().sub(contact.bodyA.position);
					this.rB = contact.originB.clone().sub(contact.bodyB.position);
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


					this.rAT1 = MemStore.Vector3( this.rA ).cross( this.tangent1 );
					this.rBT1 = MemStore.Vector3( this.rB ).cross( this.tangent1 );
					this.rAT1rA = MemStore.Vector3( this.rAT1 ).cross( this.rA );
					this.rBT1rB = MemStore.Vector3( this.rBT1 ).cross( this.rB );

					this.rAT2 = MemStore.Vector3( this.rA ).cross( this.tangent2 );
					this.rBT2 = MemStore.Vector3( this.rB ).cross( this.tangent2 );
					this.rAT2rA = MemStore.Vector3( this.rAT2 ).cross( this.rA );
					this.rBT2rB = MemStore.Vector3( this.rBT2 ).cross( this.rB );

					if (badVec(this.rAT1) || badVec(this.rBT1) ||
						badVec(this.rAT2) || badVec(this.rBT2) ||
						badVec(this.rAT1rA) || badVec(this.rBT1rB) ||
						badVec(this.rAT2rA) || badVec(this.rBT2rB)) debugger;

					this.rAN = MemStore.Vector3( this.rA ).cross( this.normal );
					this.rBN = MemStore.Vector3( this.rB ).cross( this.normal );
					this.rANrA = MemStore.Vector3( this.rAN ).cross( this.rA );
					this.rBNrB = MemStore.Vector3( this.rBN ).cross( this.rB );

					this.MAT1 = contact.bodyA.invMass + contact.bodyA.invInertiaTensor * this.rAT1rA.dot(this.tangent1);
					this.MBT1 = contact.bodyB.invMass + contact.bodyB.invInertiaTensor * this.rBT1rB.dot(this.tangent1);

					this.MAT2 = contact.bodyA.invMass + contact.bodyA.invInertiaTensor * this.rAT2rA.dot(this.tangent2);
					this.MBT2 = contact.bodyB.invMass + contact.bodyB.invInertiaTensor * this.rBT2rB.dot(this.tangent2);

					this.MAN = contact.bodyA.invMass + contact.bodyA.invInertiaTensor * this.rANrA.dot(this.normal);
					this.MBN = contact.bodyB.invMass + contact.bodyB.invInertiaTensor * this.rBNrB.dot(this.normal);
				}

				if (this.depth > physique.world.minDepthToWakeUp) {
					this.bodyA.wantToSleep = 0;
					this.bodyB.wantToSleep = 0;
				}
			};

			this.update(contact);
		};

		var islands = {};

		var Manifold = function(bodyA, bodyB, uid){

			this.bodyA = bodyA;
			this.bodyB = bodyB;
			this.contacts = [];
			this.length = 0;

			this.island = null;
			this.indexInIsland = null;
			this.shockLevel = null;
			this.uid = uid;

			if (physique.world.contactsByFeatures) {
				this.contactsByFeatures = {};
			}

			// FIXME: improve this
			this.hashContactVerts = function(contact){
				return Math.max(contact.originA.i, contact.originB.i) * 10000 + '' + Math.min(contact.originA.i, contact.originB.i);
			};

			this.updateContact = function(contact){

				if (contact.depth < 0.0) return;
				// contact.depth -= physique.world.slop;

				var hash = this.hashContactVerts(contact);
				contact.vHash = hash;

				var closestDist = Math.pow(physique.world.closestContactDistanceThreshold, 2),
					closestI = null;

				if (physique.world.contactsByFeatures) {

					if (this.contactsByFeatures.hasOwnProperty(hash)) {
						this.contactsByFeatures[hash].update(contact);
						physique.onUpdateContact(contact.hash + contact.vHash, contact.originA);
						physique.onUpdateContact(contact.hash + contact.vHash + 'B', contact.originB);
					} else {
						this.addContact(hash, contact);
					}

				} else {
					
					// Find matching contact by distance
					for (var i=0; i<this.contacts.length; ++i) {
						// var dist = this.contacts[i].refPoint.distanceToSquared(contact.refPoint);
						var dist = this.contacts[i].localA.distanceToSquared(contact.localA);

						if (dist < closestDist) {
							// Same point ??
							// if (closestDist > dist) debugger; // Probably need to decrease dist if this hits more than 1 point
							closestDist = dist;
							closestI = i;
						}
					}

					if (closestI != null) {
						this.contacts[closestI].update(contact);
					} else {
						this.addContact(hash, contact);
					}
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
					for (var i=0; i<this.contacts.length; ++i) {
						if (this.contacts[i].depth > weakestContactDepth) {
							weakestContactDepth = this.contacts[i].depth;
							weakestContactI = i;
						}
					}
					// for (var contactID in this.contacts) {
					// 	if (this.contacts[contactID].depth > weakestContactDepth) {
					// 		weakestContactDepth = this.contacts[contactID].depth;
					// 		weakestContact = contactID;
					// 	}
					// }

					// physique.onRemoveContact(this.contacts[contactID].hash + this.contacts[contactID].vHash);
					// physique.onRemoveContact(this.contacts[contactID].hash + this.contacts[contactID].vHash + 'B');
					physique.onRemoveContact(this.contacts[weakestContactI].hash + this.contacts[weakestContactI].vHash);
					physique.onRemoveContact(this.contacts[weakestContactI].hash + this.contacts[weakestContactI].vHash + 'B');

					if (physique.world.contactsByFeatures) {
						delete this.contactsByFeatures[this.contacts[weakestContactI].vHash];
					}
					this.contacts[weakestContactI].remove();
					this.contacts.splice(weakestContactI, 1);

					--this.length;
				}

				++this.length;
				// physique.onNewContact(contact.hash + hash, contact.originA);
				// physique.onNewContact(contact.hash + hash + 'B', contact.originB);
				physique.onNewContact(contact.hash + hash, contact.originA);
				physique.onNewContact(contact.hash + hash + 'B', contact.originB);
				var _contact = new Contact(contact, this);
				_contact.hash = contact.hash;
				_contact.vHash = hash;
				this.contacts.push( _contact );

				if (physique.world.contactsByFeatures) {
					this.contactsByFeatures[hash] = _contact;
				}
			};

			this.update = function(){

				var removedCount = 0;
				for (var i=0; i<this.contacts.length; ++i) {
				// for (var contactID in this.contacts) {
					var contact = this.contacts[i];

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

					if (removedCount == 0 || !physique.world.onlyRemoveOnePointPerStep) {
						// var newDistance = contact.bodyA.geometry.vertices[contact.vertexA.iA].clone().applyQuaternion(contact.bodyA.quaternion).add(contact.bodyA.position).sub( contact.bodyB.geometry.vertices[contact.vertexB.iB].clone().applyQuaternion(contact.bodyB.quaternion).add(contact.bodyB.position) ).dot( contact.normal );
						if (newDistance < physique.world.minDepth || newDistance > physique.world.maxDepth) {
							++removedCount;
							// physique.onRemoveContact(contact.hash + contact.vHash);
							// physique.onRemoveContact(contact.hash + contact.vHash + 'B');
							physique.onRemoveContact(contact.hash + contact.vHash);
							physique.onRemoveContact(contact.hash + contact.vHash + 'B');
							contact.remove();

							if (physique.world.contactsByFeatures) {
								delete this.contactsByFeatures[contact.vHash];
							}
							this.contacts.splice(i, 1);
							--this.length;
							continue;
						}


						// Contact points are too far away along the plane orthogonal to the normal axis
						var diffBetweenPoints = newContactB.clone().sub( newContactA.clone().add( contact.normal.clone().multiplyScalar(newDistance) ) );
						if (diffBetweenPoints.lengthSq() > (physique.world.persistentContactDistanceThreshold*physique.world.persistentContactDistanceThreshold)) {
							++removedCount;
							// physique.onRemoveContact(contact.hash + contact.vHash);
							// physique.onRemoveContact(contact.hash + contact.vHash + 'B');
							physique.onRemoveContact(contact.hash + contact.vHash);
							physique.onRemoveContact(contact.hash + contact.vHash + 'B');
							contact.remove();

							if (physique.world.contactsByFeatures) {
								delete this.contactsByFeatures[contact.vHash];
							}
							this.contacts.splice(i, 1);
							--this.length;
							continue;
						}

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

					contact.depth = newDistance;
					// physique.onUpdateContact(contact.hash + contact.vHash, contact.vertexA);
					// physique.onUpdateContact(contact.hash + contact.vHash + 'B', contact.vertexB);
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
						island = new Island();
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
					var disconnected = manifold.island.removeManifold(manifold);
					if (manifold.island.count == 0) {
						delete islands[manifold.island.uid];
					}

					if (disconnected) {

						for (var i=0; i<disconnected.length; ++i) {

							var manifold = disconnected[i],
								island = null;
							if (!manifold.bodyA.island || !manifold.bodyB.island) {
								if (!manifold.bodyA.island && !manifold.bodyB.island) {
									// Neither bodies belong to an island yet
								} else if (!manifold.bodyA.island) {
									manifold.bodyA.island = manifold.bodyB.island;
								} else {
									manifold.bodyB.island = manifold.bodyA.island;
								}
								island = manifold.bodyA.island;
							} else if (manifold.bodyA.island == manifold.bodyB.island) {
								island = manifold.bodyA.island;
							} else {
								// Both bodies belong to different islands..need to merge the islands
								island = manifold.bodyA.island.mergeWith( manifold.bodyB.island );
							}

							if (!island) {
								island = new Island();
								islands[island.uid] = island;
							}
							island.addManifold(manifold);

						}

					}
				}

				for (var i=0; i<manifold.contacts.length; ++i) {
				// for (var contactID in manifold.contacts) {
					var contact = manifold.contacts[i];
					// physique.onRemoveContact(contact.hash + contact.vHash);
					// physique.onRemoveContact(contact.hash + contact.vHash + 'B');
					physique.onRemoveContact(contact.hash + contact.vHash);
					physique.onRemoveContact(contact.hash + contact.vHash + 'B');
					contact.remove();
				}
				delete contactManifolds[hash];
			}
		};

		this.updateManifolds = function(){

			for (var manifoldID in contactManifolds) {
				var manifold = contactManifolds[manifoldID];
				manifold.update();

				if (_.isEmpty(manifold.contacts)) {
					if (this.world.useIslands && manifold.island) {
						var disconnected = manifold.island.removeManifold(manifold);

						if (manifold.island && manifold.island.count === 0) {
							delete islands[manifold.island.uid];
						}

						if (disconnected) {

							for (var i=0; i<disconnected.length; ++i) {

								var manifold = disconnected[i],
									island = null;
								if (!manifold.bodyA.island || !manifold.bodyB.island) {
									if (!manifold.bodyA.island && !manifold.bodyB.island) {
										// Neither bodies belong to an island yet
									} else if (!manifold.bodyA.island) {
										manifold.bodyA.island = manifold.bodyB.island;
									} else {
										manifold.bodyB.island = manifold.bodyA.island;
									}
									island = manifold.bodyA.island;
								} else if (manifold.bodyA.island == manifold.bodyB.island) {
									island = manifold.bodyA.island;
								} else {
									// Both bodies belong to different islands..need to merge the islands
									island = manifold.bodyA.island.mergeWith( manifold.bodyB.island );
								}

								if (!island) {
									island = new Island();
									islands[island.uid] = island;
								}
								island.addManifold(manifold);

							}
						}
					}
					delete contactManifolds[manifoldID];
				}
			}
		};

		this.solveConstraints = function(){

			for (var manifoldID in contactManifolds) {

				var manifold = contactManifolds[manifoldID];
				for (var i=0; i<manifold.contacts.length; ++i) {

					var contact = manifold.contacts[i];

					if ((contact.bodyA.asleep || contact.bodyA.static) && (contact.bodyB.asleep || contact.bodyB.static)) {
						continue;
					}

					// Warmstart
					contact.impulse *= physique.world.warmth;
					var rA = contact.vertexA.clone().sub(contact.bodyA.position),
						rB = contact.vertexB.clone().sub(contact.bodyB.position);
					contact.bodyA.velocity.sub( contact.normal.clone().multiplyScalar(contact.bodyA.invMass * contact.impulse) );
					contact.bodyA.angularVelocity.sub( rA.clone().cross(contact.normal).multiplyScalar(contact.bodyA.invInertiaTensor * contact.impulse) );

					contact.bodyB.velocity.add( contact.normal.clone().multiplyScalar(contact.bodyB.invMass * contact.impulse) );
					contact.bodyB.angularVelocity.add( rB.clone().cross(contact.normal).multiplyScalar(contact.bodyB.invInertiaTensor * contact.impulse) );

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



					if (isNaN(contact.bodyA.velocity.x)) debugger;
					if (isNaN(contact.bodyB.velocity.x)) debugger;
					if (isNaN(contact.bodyA.angularVelocity.x)) debugger;
					if (isNaN(contact.bodyB.angularVelocity.x)) debugger;
				}
			}

			for (var uid in physique.bodies) {
				var body = physique.bodies[uid];
			}


			var island_count = 0;
			var Island2 = function(){
				this.uid = (++island_count);
				this.manifolds = [];

				this.addManifold = function(manifold, shockLevel){
					if (!this.manifolds[shockLevel]) this.manifolds[shockLevel] = [];
					this.manifolds[shockLevel].push(manifold);
					manifold.shockLevel = shockLevel;
					manifold.indexInIsland = this.manifolds[shockLevel].length - 1;
					manifold.island = this;
					manifold.bodyA.island = this;
					manifold.bodyB.island = this;
				};
			};

			var TEST_UNOPTIMIZED_ISLANDS = this.world.useIslandsDEBUG;
			if (this.world.useIslands || TEST_UNOPTIMIZED_ISLANDS) {

				if (TEST_UNOPTIMIZED_ISLANDS) {

					Profiler.profile("islandCreation");

					islands = {};


					// - Islands: list all unchecked manifolds, then starting from static objects loop through all neighbours and set shock propagation level if not set yet OR if new shock < this shock. Each traversal results in a new island. Any leftover objects belong to a separate island (no shock level)
					
					var manifoldsToAdd = [],
						staticManifolds = [],
						uncheckedManifolds = {},
						staticManifoldIDs = {};
					for (var manifoldID in contactManifolds) {
						var manifold = contactManifolds[manifoldID];

						if (manifold.bodyA.static || manifold.bodyB.static) {
							staticManifolds.push( manifold );
							staticManifoldIDs[manifoldID] = manifold;
						} else {
							manifoldsToAdd.push( manifold );
							uncheckedManifolds[manifoldID] = manifold;
						}

						manifold.shockLevel = null;
						manifold.island = null;
						manifold.bodyA.island = null;
						manifold.bodyB.island = null;
					}



					var addNeighbour = function(island, manifold, shockLevel){

						if (manifold.island) {
							debugger; // this shouldn't occur!!
							if (manifold.island != island) {
								// Merge these two islands
								island.mergeWith(manifold.island);
								delete islands[manifold.island.uid];
								return true;
							}

							if (manifold.shockLevel > shockLevel) {
								manifold.shockLevel = shockLevel;
								island.updateManifolds();
								return true;
							}
						} else {
							// Add manifold to this island
							island.addManifold(manifold, shockLevel);
							delete uncheckedManifolds[manifold.uid];
							return true;
						}

						return false;
					};

					var adjustNeighbour = function(manifold, shockLevel){
						
						manifold.shockLevel = shockLevel;
						manifold.island.manifolds[shockLevel].splice(manifold.indexInIsland, 1);
						for (var i=manifold.indexInIsland; i<manifold.island.manifolds[shockLevel].length; ++i) {
							--manifold.island.manifolds[shockLevel][i].indexInIsland;
						}

						var shockLevel = manifold.shockLevel + 1;
						for (var manifoldID in manifold.bodyA.manifolds) {
							var _manifold = manifold.bodyA.manifolds[manifoldID];
							if (_manifold == manifold) continue;
							if (staticManifoldIDs.hasOwnProperty(manifoldID)) continue;
							if (!_manifold.island) {
								// TODO: look into why this might be occuring (although rare, should it ever
								// occur?)
								addNeighbour(manifold.island, _manifold, shockLevel);
							} else if (_manifold.shockLevel > shockLevel) {
								adjustNeighbour(_manifold, shockLevel);
							}
						}

						for (var manifoldID in manifold.bodyB.manifolds) {
							var _manifold = manifold.bodyB.manifolds[manifoldID];
							if (_manifold == manifold) continue;
							if (staticManifoldIDs.hasOwnProperty(manifoldID)) continue;
							if (!_manifold.island) {
								// TODO: look into why this might be occuring (although rare, should it ever
								// occur?)
								addNeighbour(manifold.island, _manifold, shockLevel);
							} else if (_manifold.shockLevel > shockLevel) {
								adjustNeighbour(_manifold, shockLevel);
							}
						}
					};

					var addNeighbours = function(manifold){

						var shockLevel = manifold.shockLevel + 1;
						for (var manifoldID in manifold.bodyA.manifolds) {
							var _manifold = manifold.bodyA.manifolds[manifoldID];
							if (_manifold == manifold) continue;
							if (staticManifoldIDs.hasOwnProperty(manifoldID)) continue;
							if (_manifold.island && _manifold.shockLevel > shockLevel) {
								adjustNeighbour(_manifold, shockLevel);
							} else if (uncheckedManifolds.hasOwnProperty(manifoldID)) {
								addNeighbour(manifold.island, _manifold, shockLevel);
								addNeighbours(_manifold);
							}
						}

						for (var manifoldID in manifold.bodyB.manifolds) {
							var _manifold = manifold.bodyB.manifolds[manifoldID];
							if (_manifold == manifold) continue;
							if (staticManifoldIDs.hasOwnProperty(manifoldID)) continue;
							if (_manifold.island && _manifold.shockLevel > shockLevel) {
								adjustNeighbour(_manifold, shockLevel);
							} else if (uncheckedManifolds.hasOwnProperty(manifoldID)) {
								addNeighbour(manifold.island, _manifold, shockLevel);
								addNeighbours(_manifold);
							}
						}
					};

					for (var i=0; i<staticManifolds.length; ++i) {
						var manifold = staticManifolds[i];

						if (!manifold.island) {
							// Need to make an island for this manifold

							var island = null;
							if (!manifold.bodyA.island || !manifold.bodyB.island) {
								if (!manifold.bodyA.island && !manifold.bodyB.island) {
									// Neither bodies belong to an island yet
								} else if (!manifold.bodyA.island) {
									manifold.bodyA.island = manifold.bodyB.island;
								} else {
									manifold.bodyB.island = manifold.bodyA.island;
								}
								island = manifold.bodyA.island;
							} else if (manifold.bodyA.island == manifold.bodyB.island) {
								island = manifold.bodyA.island;
							} else {
								// Both bodies belong to different islands..need to merge the islands
								// Likely that both bodies are static
								// island = manifold.bodyA.island.mergeWith( manifold.bodyB.island );

								// Merge islands.. NOTE: shouldn't need to recheck shock levels
								//
								// TODO: check shock levels just in case
								island = manifold.bodyA.island;
								for (var shockLevel in manifold.bodyB.island) {
									var shock = manifold.bodyB.island.manifolds[shock];
									if (!shock) continue;
									for (var mI=0; mI < shock.length; ++mI) {
										var _manifold = shock[mI];
										island.addManifold(_manifold, shockLevel);
									}
								}
								delete islands[manifold.bodyB.island.uid];

							}

							if (!island) {
								var island = new Island2();
								islands[island.uid] = island;
								manifold.island = island;
								manifold.bodyA.island = island;
								manifold.bodyB.island = island;
							}
							
							island.addManifold(manifold, 0);
						}

						addNeighbours(manifold);
					}

					if (!_.isEmpty(uncheckedManifolds)) {
						var island = new Island2();
						islands[island.uid] = island;

						for (var manifoldID in uncheckedManifolds) {
							island.addManifold(uncheckedManifolds[manifoldID]);
						}
					}




					Profiler.profileEnd("islandCreation");
				}

			var TEST_ENABLE_SHOCK_REWEIGHT = false;

					for (var iteration=0; iteration<physique.world.velocityIterations; ++iteration) {
					for (var islandID in islands) {
						var island = islands[islandID];
						for (var shockLevel=0; shockLevel<island.manifolds.length; ++shockLevel) {
						
							if (TEST_ENABLE_SHOCK_REWEIGHT) {
								if (shockLevel>0) {
									if (!island.manifolds[shockLevel-1]) continue;
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

							if (!island.manifolds[shockLevel]) continue;

							for (var manifoldI=0; manifoldI<island.manifolds[shockLevel].length; ++manifoldI) {
								var manifold = island.manifolds[shockLevel][manifoldI];
							// for (var manifoldID in contactManifolds) {

								// 	var manifold = contactManifolds[manifoldID];

								if (physique.world.solveWorstContactsFirst) {
									var contacts = [];
									for (var i=0; i<manifold.contacts.length; ++i) {
									// for (var contactID in manifold.contacts) {
										var contact = manifold.contacts[i];
										var ith = contacts.length;
										for (var j=0; j<contacts.length; ++j) {
											if (contact.depth < contacts[j].depth) {
												ith = j;
												break;
											}
										}
										contacts.splice(ith, 0, contact);
									}

									for (var i=contacts.length-1; i>=0; --i) {
										var contact = contacts[i];
										this.solveContact(contact);
									}
								} else {
									for (var i=0; i<manifold.contacts.length; ++i) {
									// for (var contactID in manifold.contacts) {

										var contact = manifold.contacts[i];
										this.solveContact(contact);
									}
								}
							}
					}
					if (TEST_ENABLE_SHOCK_REWEIGHT) {
						for (var shockLevel=0; shockLevel<island.manifolds.length; ++shockLevel) {
						
							if (!island.manifolds[shockLevel]) continue;
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

						if (physique.world.solveWorstContactsFirst) {
							var contacts = [];
							for (var i=0; i<manifold.contacts.length; ++i) {
							// for (var contactID in manifold.contacts) {
								var contact = manifold.contacts[i];
								var ith = contacts.length;
								for (var j=0; j<contacts.length; ++j) {
									if (contact.depth < contacts[j].depth) {
										ith = j;
										break;
									}
								}
								contacts.splice(ith, 0, contact);
							}

							for (var i=contacts.length-1; i>=0; --i) {
								var contact = contacts[i];
								this.solveContact(contact);
							}
						} else {
							for (var i=0; i<manifold.contacts.length; ++i) {
							// for (var contactID in manifold.contacts) {

								var contact = manifold.contacts[i];
								this.solveContact(contact);
							}
						}
					}
				}

			}

		};


		this.solveContact = function(contact){

			if ((contact.bodyA.static || contact.bodyA.asleep) &&
				(contact.bodyB.static || contact.bodyB.asleep)) return;

						var rA = contact.rA,
							rB = contact.rB,
							dV = 0,
							JdotV = 0,
							delta = 0,
							deltaVDotN = 0,
							JV = 0,
							MA = 0,
							MB = 0,
							deltaLambda = 0,
							lambdaTemp = 0;




					if (badVec(contact.bodyA.velocity) || badVec(contact.bodyA.angularVelocity) ||
						badVec(contact.bodyB.velocity) || badVec(contact.bodyB.angularVelocity)) debugger;
					if (isNaN(contact.bodyA.velocity.x)) debugger;
					if (isNaN(contact.bodyB.velocity.x)) debugger;
					if (isNaN(contact.bodyA.angularVelocity.x)) debugger;
					if (isNaN(contact.bodyB.angularVelocity.x)) debugger;

						// Friction 1
						var dVB = MemStore.Vector3( contact.bodyB.angularVelocity ).cross(rB).add( contact.bodyB.velocity ),
							dVA = MemStore.Vector3( contact.bodyA.angularVelocity ).cross(rA).add( contact.bodyA.velocity );
						dV = dVB.sub(dVA);

					if (badVec(dVB) || badVec(dVA)) debugger;
					if (isNaN(contact.bodyA.velocity.x)) debugger;
					if (isNaN(contact.bodyB.velocity.x)) debugger;
					if (isNaN(contact.bodyA.angularVelocity.x)) debugger;
					if (isNaN(contact.bodyB.angularVelocity.x)) debugger;
						dVA.delete();

					if (isNaN(contact.bodyA.velocity.x)) debugger;
					if (isNaN(contact.bodyB.velocity.x)) debugger;
					if (isNaN(contact.bodyA.angularVelocity.x)) debugger;
					if (isNaN(contact.bodyB.angularVelocity.x)) debugger;
						// dV = contact.bodyB.velocity.clone().add( contact.bodyB.angularVelocity.clone().cross(rB) );
						// dV.sub(  contact.bodyA.velocity.clone().add( contact.bodyA.angularVelocity.clone().cross(rA) ) );

						deltaVDotN = dV.dot(contact.tangent1);
						JV = deltaVDotN;

					if (isNaN(contact.bodyA.velocity.x)) debugger;
					if (isNaN(contact.bodyB.velocity.x)) debugger;
					if (isNaN(contact.bodyA.angularVelocity.x)) debugger;
					if (isNaN(contact.bodyB.angularVelocity.x)) debugger;

						deltaLambda = -JV / (contact.MAT1 + contact.MBT1);
						lambdaTemp = contact.impulseT1;
						var maxImpulse = physique.world.friction * contact.impulse; 
						contact.impulseT1 = Math.max(-maxImpulse, Math.min(maxImpulse, lambdaTemp + deltaLambda));
						deltaLambda = contact.impulseT1 - lambdaTemp;

					if (isNaN(contact.bodyA.velocity.x)) debugger;
					if (isNaN(contact.bodyB.velocity.x)) debugger;
					if (isNaN(contact.bodyA.angularVelocity.x)) debugger;
					if (isNaN(contact.bodyB.angularVelocity.x)) debugger;

						var tAM = MemStore.Vector3( contact.tangent1 ).multiplyScalar(-contact.bodyA.invMass * deltaLambda),
							tBM = MemStore.Vector3( contact.tangent1 ).multiplyScalar(contact.bodyB.invMass * deltaLambda);

					if (isNaN(contact.bodyA.velocity.x)) debugger;
					if (isNaN(contact.bodyB.velocity.x)) debugger;
					if (isNaN(contact.bodyA.angularVelocity.x)) debugger;
					if (isNaN(contact.bodyB.angularVelocity.x)) debugger;

						contact.bodyA.velocity.add( tAM );
						contact.bodyA.angularVelocity.add( contact.rAT1.multiplyScalar(-contact.bodyA.invInertiaTensor * deltaLambda) );

					if (badVec(contact.bodyA.velocity) || badVec(contact.bodyA.angularVelocity) ||
						badVec(contact.bodyB.velocity) || badVec(contact.bodyB.angularVelocity)) debugger;
					if (isNaN(contact.bodyA.velocity.x)) debugger;
					if (isNaN(contact.bodyB.velocity.x)) debugger;
					if (isNaN(contact.bodyA.angularVelocity.x)) debugger;
					if (isNaN(contact.bodyB.angularVelocity.x)) debugger;

						contact.bodyB.velocity.add( tBM );
						contact.bodyB.angularVelocity.add( contact.rBT1.multiplyScalar(contact.bodyB.invInertiaTensor * deltaLambda) );

					if (badVec(contact.bodyA.velocity) || badVec(contact.bodyA.angularVelocity) ||
						badVec(contact.bodyB.velocity) || badVec(contact.bodyB.angularVelocity)) debugger;
					if (isNaN(contact.bodyA.velocity.x)) debugger;
					if (isNaN(contact.bodyB.velocity.x)) debugger;
					if (isNaN(contact.bodyA.angularVelocity.x)) debugger;
					if (isNaN(contact.bodyB.angularVelocity.x)) debugger;

						tAM.delete();
						tBM.delete();


						// Friction 2
						// FIXME: do we need to update dV after friction 1 ???
						dV = contact.bodyB.velocity.clone().add( contact.bodyB.angularVelocity.clone().cross(rB) );
						dV.sub(  contact.bodyA.velocity.clone().add( contact.bodyA.angularVelocity.clone().cross(rA) ) );

						deltaVDotN = dV.dot(contact.tangent2);
						JV = deltaVDotN;

						deltaLambda = -JV / (contact.MAT2 + contact.MBT2);
						lambdaTemp = contact.impulseT2;
						var maxImpulse = physique.world.friction * contact.impulse;
						contact.impulseT2 = Math.max(-maxImpulse, Math.min(maxImpulse, lambdaTemp + deltaLambda));
						deltaLambda = contact.impulseT2 - lambdaTemp;

						tAM = MemStore.Vector3( contact.tangent2 ).multiplyScalar(-contact.bodyA.invMass * deltaLambda);
						tBM = MemStore.Vector3( contact.tangent2 ).multiplyScalar(contact.bodyB.invMass * deltaLambda);

						contact.bodyA.velocity.add( tAM );
						contact.bodyA.angularVelocity.add( contact.rAT2.multiplyScalar(-contact.bodyA.invInertiaTensor * deltaLambda) );

						contact.bodyB.velocity.add( tBM );
						contact.bodyB.angularVelocity.add( contact.rBT2.multiplyScalar(contact.bodyB.invInertiaTensor * deltaLambda) );

					if (badVec(contact.bodyA.velocity) || badVec(contact.bodyA.angularVelocity) ||
						badVec(contact.bodyB.velocity) || badVec(contact.bodyB.angularVelocity)) debugger;
					if (isNaN(contact.bodyA.velocity.x)) debugger;
					if (isNaN(contact.bodyB.velocity.x)) debugger;
					if (isNaN(contact.bodyA.angularVelocity.x)) debugger;
					if (isNaN(contact.bodyB.angularVelocity.x)) debugger;

						tAM.delete();
						tBM.delete();


						// FIXME: recalculate dV for normal ??
						var dV = contact.bodyB.velocity.clone().add( contact.bodyB.angularVelocity.clone().cross(rB) );
						dV.sub(  contact.bodyA.velocity.clone().add( contact.bodyA.angularVelocity.clone().cross(rA) ) );

						var deltaVDotN = dV.dot(contact.normal),
							JV = deltaVDotN;

						var invdt = dt * (1/physique.world.scaleTime);
						var b = Math.max(0.0, (contact.depth - physique.world.slop)) * (physique.world.baumgarte * invdt) - JV * physique.world.restitution; // TODO: restitution

						var Meffective = 1/(contact.MAN + contact.MBN);
						var deltaLambda = (b - JV) * Meffective,
							lambdaTemp = contact.impulse;

						contact.impulse = Math.max(contact.impulse + deltaLambda, 0.0);
						deltaLambda = contact.impulse - lambdaTemp;

						var nAM = MemStore.Vector3( contact.normal ).multiplyScalar(contact.bodyA.invMass * deltaLambda),
							nBM = MemStore.Vector3( contact.normal ).multiplyScalar(contact.bodyB.invMass * deltaLambda);


						contact.bodyA.velocity.sub( nAM );
						contact.bodyA.angularVelocity.sub( contact.rAN.multiplyScalar(contact.bodyA.invInertiaTensor * deltaLambda) );

						contact.bodyB.velocity.add( nBM );
						contact.bodyB.angularVelocity.add( contact.rBN.multiplyScalar(contact.bodyB.invInertiaTensor * deltaLambda) );

						nAM.delete();
						nBM.delete();

						dV.delete();


					if (badVec(contact.bodyA.velocity) || badVec(contact.bodyA.angularVelocity) ||
						badVec(contact.bodyB.velocity) || badVec(contact.bodyB.angularVelocity)) debugger;
					if (isNaN(contact.bodyA.velocity.x)) debugger;
					if (isNaN(contact.bodyB.velocity.x)) debugger;
					if (isNaN(contact.bodyA.angularVelocity.x)) debugger;
					if (isNaN(contact.bodyB.angularVelocity.x)) debugger;
		};



		/*****************************************************************
		 *****************************************************************
		 							Dynamics
		 */

		this.removeBody = function(mesh){
			var body = this.bodies[mesh.uid];

			for (var manifoldID in body.manifolds) {
				var manifold = body.manifolds[manifoldID];
				if (physique.world.useIslands && manifold.island) manifold.island.removeManifold(manifold);

				for (var i=0; i<manifold.contacts.length; ++i) {
					var contact = manifold.contacts[i];
					physique.onRemoveContact(contact.hash + contact.vHash);
					physique.onRemoveContact(contact.hash + contact.vHash + 'B');
					manifold.contacts.splice(i, 1);
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

			var body = new Body(mesh.body, mesh.settings);
			this.bodies[mesh.uid] = body;



			
			body.addManifold = function(manifold){
				this.manifolds[manifold.uid] = manifold;
			};

			body.removeManifold = function(manifold){
				delete this.manifolds[manifold.uid];
			};

			body.updateInBroadphase = function(){
				Broadphase.updateAABB(this);
			};

			body.updateInBroadphase();
			body.mesh.updateInBroadphase = body.updateInBroadphase;
			Broadphase.addBodyIntoBroadphase(body);
		};

		this.reset = function(){
			oldHits = {};
			this.bodies = {};
			Broadphase.reset();
		};

		this.onDebugHelperArrow = new Function();
		this.onNewContact = new Function();
		this.onUpdateContact = new Function();
		this.onRemoveContact = new Function();
		this.step = function(delta){

			dt = delta * this.world.scaleTime;
			if (dt == 0) return;
			var idt = 1/dt;
			if (this.world.runOnce === false) return;
			if (this.world.runOnce === true) this.world.runOnce = false;
			var angularScale = new THREE.Vector3(1, 1, 1);
			for (var uid in this.bodies) {
				var body = this.bodies[uid];

				if (!body.static && !body.asleep) {

					body.storedState = {
						position: body.position.clone()
					};

					body.velocity.y += (this.world.gravity * dt);

					Broadphase.updateAABB(body.mesh);
					// Broadphase.updateBodyInBroadphase(body);
				} else if (body.userMoved) {
					body.userMoved = false;

					if (body.asleep) {
						body.asleep = false;
						body.wantToSleep = 0;
						body.invMass = body.storedInvMass;
						body.invInertiaTensor = body.storedInvInertiaTensor;
						body.updateState();
					}
					Broadphase.updateAABB(body.mesh);
					// Broadphase.updateBodyInBroadphase(body);
				} else {

					// FIXME: this shouldn't be necessary, but checking just in case
					Broadphase.updateAABB(body.mesh);
					// Broadphase.updateBodyInBroadphase(body);
				}
			}


			Profiler.profile('maintainManifolds');
			this.updateManifolds(); // NOTE: need to update manifolds BEFORE updating particular contacts below
			Profiler.profileEnd('maintainManifolds');

			Profiler.profile('Broadphase');
			var hits = Broadphase.sweepAndPrune();
			Profiler.profileEnd('Broadphase');

			Profiler.profile('Narrowphase');
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

				var minDepthToWakeUp = (contactManifolds.hasOwnProperty(hitHash) ? this.world.minWakeDepth : 0.0); // Only consider waking up the body IF this is a new contact pair OR if the penetration depth is big enough
				if (bodyA.asleep && contact.depth > minDepthToWakeUp) {
					bodyA.asleep = false;
					bodyA.wantToSleep = 0;
					bodyA.invMass = bodyA.storedInvMass;
					bodyA.invInertiaTensor = bodyA.storedInvInertiaTensor;
				}

				if (bodyB.asleep && contact.depth > minDepthToWakeUp) {
					bodyB.asleep = false;
					bodyB.wantToSleep = 0;
					bodyB.invMass = bodyB.storedInvMass;
					bodyB.invInertiaTensor = bodyB.storedInvInertiaTensor;
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

				bodyA.updateState();
				bodyB.updateState();
			}
			Profiler.profileEnd('Narrowphase');

			if (oldHits) {
				for (var oldHitHash in oldHits) {
					if (!hits.hasOwnProperty(oldHitHash)) {
						this.removeManifold(oldHitHash);
						var oldHit = oldHits[oldHitHash],
							bodyA = oldHit.bodyA,
							bodyB = oldHit.bodyB;

						if (bodyA.asleep) {
							bodyA.asleep = false;
							bodyA.wantToSleep = 0;
							bodyA.invMass = bodyA.storedInvMass;
							bodyA.invInertiaTensor = bodyA.storedInvInertiaTensor;
						}
						bodyA.updateState();

						if (bodyB.asleep) {
							bodyB.asleep = false;
							bodyB.wantToSleep = 0;
							bodyB.invMass          = bodyB.storedInvMass;
							bodyB.invInertiaTensor = bodyB.storedInvInertiaTensor;
						}
						bodyB.updateState();
					}
				}
			}
			oldHits = hits;

			Profiler.profile('PGS');
			this.solveConstraints();
			Profiler.profileEnd('PGS');

			// Integrate changes
			for (var uid in this.bodies) {
				var body = this.bodies[uid];

				// if (body.hasOwnProperty('storedState')) {
				// 	body.position.copy(body.storedState.position);
				// 	delete body.storedState;
				// }

				if (!body.static && !body.asleep) {

					var scale = 1.0;
					body.position.x += scale * body.velocity.x * dt;
					body.position.y += scale * body.velocity.y * dt;
					body.position.z += scale * body.velocity.z * dt;

					var v = (new THREE.Vector3(body.angularVelocity.x * angularScale.x, body.angularVelocity.y * angularScale.y, body.angularVelocity.z * angularScale.z)).multiplyScalar(dt).multiplyScalar(scale);
					var q = new THREE.Quaternion(body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z, 0);
					q.multiply(body.quaternion);
					body.quaternion.x += q.x * 0.5 * dt;
					body.quaternion.y += q.y * 0.5 * dt;
					body.quaternion.z += q.z * 0.5 * dt;
					body.quaternion.w += q.w * 0.5 * dt;
					body.quaternion.normalize();


					// body.updateRotation(dt);
					// body.angularVelocity.multiplyScalar(0);

					var damping = physique.world.damping;
					if (damping != 1) {
						// damping *= dt;
						body.velocity.multiplyScalar(damping);
						body.angularVelocity.multiplyScalar(damping);
					}

					// var epsV = this.world.minVel;
					// if (Math.abs(body.velocity.x) < epsV) body.velocity.x = 0;
					// if (Math.abs(body.velocity.y) < epsV) body.velocity.y = 0;
					// if (Math.abs(body.velocity.z) < epsV) body.velocity.z = 0;
					// if (Math.abs(body.angularVelocity.x) < epsV) body.angularVelocity.x = 0;
					// if (Math.abs(body.angularVelocity.y) < epsV) body.angularVelocity.y = 0;
					// if (Math.abs(body.angularVelocity.z) < epsV) body.angularVelocity.z = 0;
					var epsV = this.world.minVel,
						v = body.velocity.lengthSq(),
						av = body.angularVelocity.lengthSq();
					if (v < epsV*epsV) {
						body.velocity.multiplyScalar(0.0);
					}
					if (av < epsV*epsV) {
						body.angularVelocity.multiplyScalar(0.0);
					}

					var epsVSleep = this.world.minToSleep;
					if (v < epsVSleep*epsVSleep && av < epsVSleep*epsVSleep) {
					// if (Math.abs(body.velocity.x) < epsVSleep &&
					// 	Math.abs(body.velocity.y) < epsVSleep &&
					// 	Math.abs(body.velocity.z) < epsVSleep &&
					// 	Math.abs(body.angularVelocity.x) < epsVSleep &&
					// 	Math.abs(body.angularVelocity.y) < epsVSleep &&
					// 	Math.abs(body.angularVelocity.z) < epsVSleep) {
							++body.wantToSleep;

							if (body.wantToSleep >= this.world.minIterationsBeforeSleep) {
								body.wantToSleep = 0;
								body.asleep = true;
								body.storedInvMass = body.invMass;
								body.storedInvInertiaTensor = body.invInertiaTensor;
								body.invMass = 0;
								body.invInertiaTensor = 0;
								body.velocity.multiplyScalar(0.0);
								body.angularVelocity.multiplyScalar(0.0);

								body.updateState();
							}
						}


					Broadphase.updateAABB(body.mesh);
					// Broadphase.updateBodyInBroadphase(body);
				}
			}
		};
	};

	return Physique;
});
