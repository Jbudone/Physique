define(function(){

	Keys.add('BODY_CUBE');
	Keys.add('BODY_FLOOR');


	var Physique = function(){

		this.bodies = {};
		var physique = this;
		var dt = 0;

		this.world = {
			gravity: -9.8,
			scaleTime: 0.00001,

			slop: 0,//0.001,
			maxDepth: 4.0, // before falling through
			damping: 1.00,

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

		var mass = 2,
			inertia = (1/3) * mass;

		/*****************************************************************
		 *****************************************************************
		 						Broadphase Collision                         
		 */

		// Update axis-aligned bounding box of a body
		//
		// AABB's are used in the sweep & prune method for broadphase collision. Collision uses a predictive
		// approach, to detect potential collisions. Hence, AABB's need to be updated at the beginning of each
		// timestep with its predicted displacement. 
		//
		// @displacement: { dx, dy, .. }
		this.updateAABB = function(body, displacement){

			if (!body.hasOwnProperty('AABB')) {
			} else {
				body.AABB.update();
			}

			// FIXME: apply displacement to AABB
		};

		this.sweepAndPrune = function(){

			var hits = {},
				hashContact = function(bodyA, bodyB){
					var min = Math.min(bodyA.uid, bodyB.uid),
						max = Math.max(bodyA.uid, bodyB.uid);
					return min * 100000 + max;
				};

			for (var i=0; i<this.axisLineX.length; ++i) {
				var bodyA = this.axisLineX[i];

				for (var j=0; j<i; ++j) {
					var bodyB = this.axisLineX[j];

					if (bodyA.AABB.box.min.x <= bodyB.AABB.box.max.x) {
						var hash = hashContact(bodyA, bodyB);
						if (!hits.hasOwnProperty(hash)) {
							hits[hash] = {
								bodyA: bodyA,
								bodyB: bodyB,
								x: true
							};
						}
					}
				}
			}

			for (var i=0; i<this.axisLineZ.length; ++i) {
				var bodyA = this.axisLineZ[i];

				for (var j=0; j<i; ++j) {
					var bodyB = this.axisLineZ[j];

					if (bodyA.AABB.box.min.z <= bodyB.AABB.box.max.z) {
						var hash = hashContact(bodyA, bodyB);
						if (hits.hasOwnProperty(hash)) {
							hits[hash].z = true;
						}
					}
				}
			}

			for (var hitHash in hits) {
				var hit = hits[hitHash];

				if (!hit.z || !this.checkAABBCollision(hit.bodyA, hit.bodyB)) {
					delete hits[hitHash];
				}
			}

			return hits;
		};

		this.axisLineX = [];
		this.axisLineZ = [];

		this.checkAABBCollision = function(bodyA, bodyB){

			var boundsA = bodyA.AABB.box,
				boundsB = bodyB.AABB.box;
			if (((boundsA.min.x >= boundsB.min.x && boundsB.max.x >= boundsA.min.x) ||
				 (boundsA.min.x <= boundsB.min.x && boundsA.max.x >= boundsB.min.x) ||
				 (boundsA.min.x >= boundsB.min.x && boundsA.max.x <= boundsB.max.x))
					&&
				((boundsA.min.y >= boundsB.min.y && boundsB.max.y >= boundsA.min.y) ||
				 (boundsA.min.y <= boundsB.min.y && boundsA.max.y >= boundsB.min.y) ||
				 (boundsA.min.y >= boundsB.min.y && boundsA.max.y <= boundsB.max.y))
					&&
				((boundsA.min.z >= boundsB.min.z && boundsB.max.z >= boundsA.min.z) ||
				 (boundsA.min.z <= boundsB.min.z && boundsA.max.z >= boundsB.min.z) ||
				 (boundsA.min.z >= boundsB.min.z && boundsA.max.z <= boundsB.max.z))) {

				 return true;
			 }
			
			return false;
		};

		this.addBodyIntoBroadphase = function(body){
			
			var xIndex = 0;
			for (xIndex=0; xIndex<this.axisLineX.length; ++xIndex) {
				var xBody = this.axisLineX[xIndex];
				if (xBody.AABB.box.min.x >= body.AABB.box.min.x) break;
			}
			this.axisLineX.splice(xIndex, 0, body);
			
			var zIndex = 0;
			for (zIndex=0; zIndex<this.axisLineZ.length; ++zIndex) {
				var zBody = this.axisLineZ[zIndex];
				if (zBody.AABB.box.min.z >= body.AABB.box.min.z) break;
			}
			this.axisLineZ.splice(zIndex, 0, body);
		};

		this.updateBodyInBroadphase = function(body){

			// Find my position
			var xIndex = 0,
				zIndex = 0;
			for (var i=0; i<this.axisLineX.length; ++i) {
				if (this.axisLineX[i] == body) {
					xIndex = i;
					break;
				}
			}
			for (var i=0; i<this.axisLineZ.length; ++i) {
				if (this.axisLineZ[i] == body) {
					zIndex = i;
					break;
				}
			}
			this.axisLineX.splice(xIndex, 1);
			this.axisLineZ.splice(zIndex, 1);

			// Find my new position
			var xIndex = 0;
			for (xIndex=0; xIndex<this.axisLineX.length; ++xIndex) {
				var xBody = this.axisLineX[xIndex];
				if (xBody.AABB.box.min.x > body.AABB.box.min.x) break;
			}
			this.axisLineX.splice(xIndex, 0, body);
			
			var zIndex = 0;
			for (zIndex=0; zIndex<this.axisLineZ.length; ++zIndex) {
				var zBody = this.axisLineZ[zIndex];
				if (zBody.AABB.box.min.z > body.AABB.box.min.z) break;
			}
			this.axisLineZ.splice(zIndex, 0, body);
		};

		/*****************************************************************
		 *****************************************************************
		 						Narrowphase Collision                         
		 */


		this.SAT = function(bodyA, bodyB){

			var axes = {},
				hashAxis = function(v){
					return parseInt(v.x * 100 * 100 * 100 + v.y * 100 * 100 + v.z * 100); // TODO: better hashing
				};

			// Get all axes
			for (var i=0; i<bodyA.geometry.faces.length; ++i) {
				var axis = bodyA.geometry.faces[i].normal.clone().applyQuaternion(bodyA.quaternion).normalize(),
					hash = hashAxis(axis),
					negHash = hashAxis(axis.clone().negate());
				if (!axes.hasOwnProperty(hash) && !axes.hasOwnProperty(negHash)) {
					axes[hash] = axis;
				}
			}
			for (var i=0; i<bodyB.geometry.faces.length; ++i) {
				var axis = bodyB.geometry.faces[i].normal.clone().applyQuaternion(bodyB.quaternion).normalize(),
					hash = hashAxis(axis),
					negHash = hashAxis(axis.clone().negate());
				if (!axes.hasOwnProperty(hash) && !axes.hasOwnProperty(negHash)) {
					axes[hash] = axis;
				}
			}

			var crossAxisList = [];
			for (var hashA in axes) {
				for (var hashB in axes) {
					if (hashA == hashB) continue;

					var hash = hashA + '' + hashB,
						cross = axes[hashA].clone().cross(axes[hashB]);
					crossAxisList.push({hash: hash, axis: cross});
				}
			}
			for (var i=0; i<crossAxisList.length; ++i) {
				var axis = crossAxisList[i];
				axes[axis.hash] = axis.axis;
			}

			// Project each vertex onto each axis
			var deepestPenetrationV = null,
				deepestPenetration = 9999999,
				originPoint = null,
				endPoint = null;
			for (var hash in axes) {
				var axis = axes[hash],
					bounds = {
						minA: null,
						maxA: null,
						minB: null,
						maxB: null,

						minAV: null,
						maxAV: null,
						minBV: null,
						maxBV: null
					};

				for (var i=0; i<bodyA.geometry.vertices.length; ++i) {
					var vertex = bodyA.geometry.vertices[i].clone().applyQuaternion(bodyA.quaternion).add(bodyA.position),
						proj = vertex.dot(axis);

					if (bounds.minA === null || bounds.minA > proj) {
						bounds.minA = proj;
						bounds.minAV = vertex;
					}
					if (bounds.maxA === null || bounds.maxA < proj) {
						bounds.maxA = proj;
						bounds.maxAV = vertex;
					}
				}


				for (var i=0; i<bodyB.geometry.vertices.length; ++i) {
					var vertex = bodyB.geometry.vertices[i].clone().applyQuaternion(bodyB.quaternion).add(bodyB.position),
						proj = vertex.dot(axis);

					if (bounds.minB === null || bounds.minB > proj) {
						bounds.minB = proj;
						bounds.minBV = vertex;
					}
					if (bounds.maxB === null || bounds.maxB < proj) {
						bounds.maxB = proj;
						bounds.maxBV = vertex;
					}
				}

				if (bounds.minA <= bounds.minB && bounds.maxA >= bounds.minB) {
					var penetrationDepth = bounds.maxA - bounds.minB;
					if (penetrationDepth < deepestPenetration) {
						deepestPenetration = penetrationDepth;
						deepestPenetrationV = axis.clone();

						originPoint = bounds.minBV;
						endPoint = bounds.maxAV;
					}
				} else if (bounds.minB <= bounds.minA && bounds.maxB >= bounds.minA) {
					var penetrationDepth = bounds.maxB - bounds.minA;
					if (penetrationDepth < deepestPenetration) {
						deepestPenetration = penetrationDepth;
						deepestPenetrationV = axis.clone();

						originPoint = bounds.minAV;
						endPoint = bounds.maxBV;
					}
				} else {
					return null;
				}
			}

			if (!deepestPenetrationV) {
				return null;
			}

			return {
				normal: deepestPenetrationV,
				depth: deepestPenetration,

				origin: originPoint,
				end: endPoint
			}
		};



		this.checkGJK = function(bodyA, bodyB){

			var d = new THREE.Vector3(1, 0, 0),
				p = this.supportGJK(bodyA, bodyB, d),
				simplex = [p];

			d = (new THREE.Vector3()).sub(p);
			var iFroze = 100;
			while (true) {
				p = this.supportGJK(bodyA, bodyB, d);

				if (p.dot(d) < 0) {
					return false;
				}

				simplex.push(p);
				d = this.doSimplex(simplex);
				if (d === true) {

					// EPA (Expanding Polytope Algorithm)
					//

					var polytope = {
							faces: [],
							vertices: []
						},
						a = simplex[3],
						b = simplex[2],
						AB = simplex[2].clone().sub(a),
						AC = simplex[1].clone().sub(a),
						AD = simplex[0].clone().sub(a),
						BD = simplex[0].clone().sub(b),
						BC = simplex[1].clone().sub(b),
						ABC = AB.clone().cross(AC),
						ACD = AC.clone().cross(AD),
						ADB = AD.clone().cross(AB),
						BDC = BD.clone().cross(BC);

					polytope.vertices = simplex;
					polytope.faces.push(new THREE.Face3(3, 2, 1, ABC));
					polytope.faces[polytope.faces.length-1].normal.normalize();
					polytope.faces.push(new THREE.Face3(3, 1, 0, ACD));
					polytope.faces[polytope.faces.length-1].normal.normalize();
					polytope.faces.push(new THREE.Face3(3, 0, 2, ADB));
					polytope.faces[polytope.faces.length-1].normal.normalize();
					polytope.faces.push(new THREE.Face3(2, 0, 1, BDC)); // FIXME: is this necessary ??
					polytope.faces[polytope.faces.length-1].normal.normalize();

						// FIXME FIXME FIXME FIXME FIXME FIXME!
					var faceNormalIsGood = function(face){

						if (face.normal.dot(polytope.vertices[face.a]) <= 0 ||
							face.normal.dot(polytope.vertices[face.b]) <= 0 ||
							face.normal.dot(polytope.vertices[face.c]) <= 0) {

								face.normal.negate();
								return false;
						}
						return true;
					};
					
					var darn=0;
					if (!faceNormalIsGood(polytope.faces[0])) ++darn; //debugger;
					if (!faceNormalIsGood(polytope.faces[1])) ++darn; //debugger;
					if (!faceNormalIsGood(polytope.faces[2])) ++darn; //debugger;
					if (!faceNormalIsGood(polytope.faces[3])) ++darn; //debugger;
					var _pHashTable = {};
					for (var i=0; i<polytope.vertices.length; ++i) {
						var p = polytope.vertices[i],
							_pHash = parseInt(p.x * 1000*1000*1000 + p.y * 1000*1000 + p.z * 1000);
						_pHashTable[_pHash] = p;
					}
						// FIXME FIXME FIXME FIXME FIXME FIXME!
					while(true) {
						var nearestFace = this.getNearestFaceGJK(polytope);

						var p = this.supportGJK(bodyA, bodyB, nearestFace.face.normal);

						if ((p.dot(nearestFace.face.normal) - nearestFace.distance) < 0.0001) {

							// Barycentric coordinates
							var a = polytope.vertices[nearestFace.face.a],
								b = polytope.vertices[nearestFace.face.b],
								c = polytope.vertices[nearestFace.face.c],
								_p = nearestFace.face.normal.clone().multiplyScalar(nearestFace.distance),
								AB = b.clone().sub(a),
								AC = c.clone().sub(a),
								BC = c.clone().sub(b),
								m = AB.clone().cross(AC),
								nu = null,
								nv = null,
								ood = null,
								x = Math.abs(m.x),
								y = Math.abs(m.y),
								z = Math.abs(m.z);

							/*
							// TODO: referenced this from pg. 52 of Collision Detection book...figure it out!
							var TriArea2D = function(x1, y1, x2, y2, x3, y3){
								return (x1-x2)*(y2-y3) - (x2-x3)*(y1-y2);
							};

							if (x >= y && x >= d) {
								nu = TriArea2D(p.y, p.z, b.y, b.z, c.y, c.z);
								nv = TriArea2D(p.y, p.z, c.y, c.z, a.y, a.z);
								ood = 1.0 / m.x;
							} else if (y >= x && y >= z) {
								nu = TriArea2D(p.x, p.z, b.x, b.z, c.x, c.z);
								nv = TriArea2D(p.x, p.z, c.x, c.z, a.x, a.z);
								ood = 1.0 / -m.y;
							} else {
								nu = TriArea2D(p.x, p.y, b.x, b.y, c.x, c.y);
								nv = TriArea2D(p.x, p.y, c.x, c.y, a.x, a.y);
								ood = 1.0 / m.z;
							}

							var u = nu*ood,
								v = nv*ood,
								w = 1.0 - u - v;
							*/


							// FIXME FIXME FIXME FIXME FIXME FIXME!
							// TODO: referenced this from http://hacktank.net/blog/?p=119 ... figure it out!!
							var v0 = AB,
								v1 = AC,
								v2 = _p.clone().sub(a),
								d00 = v0.dot(v0),
								d01 = v0.dot(v1),
								d11 = v1.dot(v1),
								d20 = a.dot(v0),//v2.dot(v0), // FIXME: a vs. v2
								d21 = a.dot(v1),//v2.dot(v1), // FIXME: a vs. v2
								denom = d00 * d11 - d01 * d01;

							var lambda1 = d21 * d01 - d20 * d11,
								lambda2 = d20 * d01 - d21 * d00,
								closestPoint = a.clone().add( b.clone().multiplyScalar( (1.0 / denom) * lambda1 ) ).add( c.clone().multiplyScalar( (1.0 / denom) * lambda2 ) ),
								distSquare = closestPoint.dot(closestPoint);
							// var v = (d11 * d20 - d01 * d21) / denom,
							// 	w = (d00 * d21 - d01 * d20) / denom,
							// 	u = 1.0 - v - w;
							// FIXME FIXME FIXME FIXME FIXME FIXME!

							/*
							var _a = a, // 0
								_b = b, // 1
								_c = c, // 2
								_p0 = _b.sup_a.clone().sub(_p).cross( _c.sup_a.clone().sub(_p) ).length(),
								_p1 = _c.sup_a.clone().sub(_p).cross( _a.sup_a.clone().sub(_p) ).length(),
								_p2 = _a.sup_a.clone().sub(_p).cross( _b.sup_a.clone().sub(_p) ).length(),
								sum = _p0+_p1+_p2;
							_p0 /= sum;
							_p1 /= sum;
							_p2 /= sum;
							*/



							// var contact = a.sup_a.clone().applyQuaternion(bodyA.quaternion).add(bodyA.position).multiplyScalar(u).add( b.sup_a.clone().applyQuaternion(bodyA.quaternion).add(bodyA.position).multiplyScalar(v) ).add( c.sup_a.clone().applyQuaternion(bodyA.quaternion).add(bodyA.position).multiplyScalar(w) );
							// var contactA = a.sup_a.multiplyScalar(u).add( b.sup_a.multiplyScalar(v) ).add( c.sup_a.multiplyScalar(w) ),
							// 	contactB = a.sup_b.multiplyScalar(u).add( b.sup_b.multiplyScalar(v) ).add( c.sup_b.multiplyScalar(w) );


							// var contactA = a.sup_a.clone().add( b.sup_a.clone().sub(a.sup_a.clone()).multiplyScalar( (1.0/denom) * lambda1 ) ).add( c.sup_a.clone().sub(a.sup_a.clone()).multiplyScalar( (1.0/denom) * lambda2 ) ),
							// 	contactB = a.sup_b.clone().add( b.sup_b.clone().sub(a.sup_b.clone()).multiplyScalar( (1.0/denom) * lambda1 ) ).add( c.sup_b.clone().sub(a.sup_b.clone()).multiplyScalar( (1.0/denom) * lambda2 ) );
							// contactB = contactA.clone().add( nearestFace.face.normal.clone().multiplyScalar( nearestFace.distance ) );
							var aA = bodyA.geometry.vertices[a.sup_a.i].clone(),
								bA = bodyA.geometry.vertices[b.sup_a.i].clone(),
								cA = bodyA.geometry.vertices[c.sup_a.i].clone(),
								aB = bodyB.geometry.vertices[a.sup_b.i].clone(),
								bB = bodyB.geometry.vertices[b.sup_b.i].clone(),
								cB = bodyB.geometry.vertices[c.sup_b.i].clone(),
								invD = 1.0 / denom,
								localA = aA.clone().add( bA.sub(aA).multiplyScalar( invD * lambda1 ) ).add( cA.sub(aA).multiplyScalar( invD * lambda2 ) ),
								localB = aB.clone().add( bB.sub(aB).multiplyScalar( invD * lambda1 ) ).add( cB.sub(aB).multiplyScalar( invD * lambda2 ) ),
								contactA = localA.clone().applyQuaternion( bodyA.quaternion ).add( bodyA.position ),
								contactB = localB.clone().applyQuaternion( bodyB.quaternion ).add( bodyB.position );
							// var contactA = a.sup_a.clone().add( b.sup_a.clone().sub(a.sup_a.clone()).multiplyScalar( (1.0/denom) * lambda1 ) ).add( c.sup_a.clone().sub(a.sup_a.clone()).multiplyScalar( (1.0/denom) * lambda2 ) ),
							// 	contactB = a.sup_b.clone().add( b.sup_b.clone().sub(a.sup_b.clone()).multiplyScalar( (1.0/denom) * lambda1 ) ).add( c.sup_b.clone().sub(a.sup_b.clone()).multiplyScalar( (1.0/denom) * lambda2 ) );
							// contactB = contactA.clone().add( nearestFace.face.normal.clone().multiplyScalar( nearestFace.distance ) );
							// FIXME: fix index/hash
							contactA.i = parseInt(a.sup_a.i * 400 + '' + b.sup_a.i * 20 + '' + c.sup_a.i);
							contactB.i = parseInt(a.sup_b.i * 400 + '' + b.sup_b.i * 20 + '' + c.sup_b.i);

							// contact.applyQuaternion(bodyA.quaternion);
							// contact.add(bodyA.position);
							// contact.sub( new THREE.Vector3(0.5, 0.5, 0.5) );
							
							// FIXME FIXME FIXME FIXME FIXME FIXME!
							if (nearestFace.face.normal.x == 0 &&
								nearestFace.face.normal.y == 0 &&
								nearestFace.face.normal.z == 0) {
									debugger;
								} else if (nearestFace.distance == 0) {
									debugger;
								}
							// FIXME FIXME FIXME FIXME FIXME FIXME!

							var contact = contactA.clone().add(contactB).multiplyScalar(0.5);
							contact.iA = contactA.i;
							contact.iB = contactB.i;

							contactA.iA = contactA.i;
							contactB.iB = contactB.i;

							var norm = nearestFace.face.normal.negate();
							// if (norm.dot(bodyA.position.clone().sub(bodyB.position)) < 0) {
							// 	debugger;
							// 	norm.negate();
							// }


							// var localA = bodyA.geometry.vertices[a.sup_a.i].clone().multiplyScalar(u).add( bodyA.geometry.vertices[b.sup_a.i].clone().multiplyScalar(v) ).add( bodyA.geometry.vertices[c.sup_a.i].clone().multiplyScalar(w) ),
							// 	localB = bodyB.geometry.vertices[a.sup_b.i].clone().multiplyScalar(u).add( bodyB.geometry.vertices[b.sup_b.i].clone().multiplyScalar(v) ).add( bodyB.geometry.vertices[c.sup_b.i].clone().multiplyScalar(w) );


							// var localA = contactA.clone().sub( bodyA.position ).applyQuaternion( bodyA.quaternion.clone().inverse() ),
							// 	localB = contactB.clone().sub( bodyB.position ).applyQuaternion( bodyB.quaternion.clone().inverse() );
							return {
								originA: contactA,
								originB: contactB,

								localA: localA,
								localB: localB,

								normal: norm,
								depth: nearestFace.distance,

								bodyA: bodyA,
								bodyB: bodyB
							};
						} else {

						// FIXME FIXME FIXME FIXME FIXME FIXME!
						var _pHash = parseInt(p.x * 1000*1000*1000 + p.y * 1000*1000 + p.z * 1000);
						if (_pHashTable.hasOwnProperty(_pHash)) {
							return false;
						}
						_pHashTable[_pHash] = p;
						// FIXME FIXME FIXME FIXME FIXME FIXME!

							// Remove this triangle and all triangles facing same direction
							//
							// We're removing any other triangles which can "see" the support point. Do this
							// by taking the normal of the other triangle and the direction of (p-v), then
							// seeing if the dot product between the two vectors is positive. V can be any
							// vertex on the triangle

							// FIXME FIXME FIXME FIXME FIXME FIXME
							if (polytope.faces.length > 100) {
								debugger;
								return false;
							}
							// FIXME FIXME FIXME FIXME FIXME FIXME
							var i,
								badTriangles = [],
								consideredEdges = {},
								badEdges = [],
								emptyEdges = [];
							for (i=0; i<polytope.faces.length; ++i) {
								if (polytope.faces[i] == nearestFace.face) {
									badTriangles.push({
										face: nearestFace.face,
										i: i
									});
								// } else if (nearestFace.face.normal.dot(p.clone().sub(polytope.vertices[polytope.faces[i].a])) > 0) {
								} else if (polytope.faces[i].normal.dot(p.clone().sub(polytope.vertices[polytope.faces[i].a])) > 0) {
									badTriangles.push({
										face: polytope.faces[i],
										i: i
									});
								}
							}

							var hashEdge = function(a, b){
								return Math.max(a,b) * 1000 + Math.min(a,b);
								// return ((a+1)*100+(b+1)) + ((b+1)*100+(a+1));
							};

							// Remove all bad triangles, and keep track of removed/stored edges
							// NOTE: need to traverse in descending order since we're splicing based off the
							// stored index of each triangle
							// 
							// When a triangle is removed, its edges may or may not be removed too. Each edge
							// on the removed triangle is stored in consideredEdges; then if the same edge is
							// found again from another triangle being removed, the edge is removed. This is
							// because an edge is only shared by two triangles (never 1, and never more than
							// 2). 
							for (i=badTriangles.length-1; i>=0; --i){
								var badTri = badTriangles[i],
									face = badTri.face;
								polytope.faces.splice(badTri.i, 1);

								var eAB = hashEdge(face.a, face.b),
									eAC = hashEdge(face.a, face.c),
									eBC = hashEdge(face.b, face.c);
								if (!consideredEdges.hasOwnProperty(eAB)) consideredEdges[eAB] = {count:0, a: face.a, b: face.b};
								if (!consideredEdges.hasOwnProperty(eAC)) consideredEdges[eAC] = {count:0, a: face.a, b: face.c};
								if (!consideredEdges.hasOwnProperty(eBC)) consideredEdges[eBC] = {count:0, a: face.b, b: face.c};
								++consideredEdges[eAB].count;
								++consideredEdges[eAC].count;
								++consideredEdges[eBC].count;

								if (consideredEdges[eAB].count == 2) badEdges.push({a: face.a, b: face.b, hash:eAB});
								if (consideredEdges[eAC].count == 2) badEdges.push({a: face.a, b: face.c, hash:eAC});
								if (consideredEdges[eBC].count == 2) badEdges.push({a: face.b, b: face.c, hash:eBC});
							}

							// Find the lone-edges which need to be filled
							for (var eHash in consideredEdges) {
								var edge = consideredEdges[eHash];
								if (edge.count < 2) {
									emptyEdges.push(edge);
								}
							}

							polytope.vertices.push(p);
							for (i=0; i<emptyEdges.length; ++i) {
								var edge = emptyEdges[i],
									a = polytope.vertices[edge.a],
									b = polytope.vertices[edge.b],
									_i = polytope.vertices.length-1,
									_v = polytope.vertices[_i],
									AB = b.clone().sub(a),
									AV = _v.clone().sub(a),
									BV = _v.clone().sub(b),
									ABV = AB.clone().cross(AV);
								if (ABV == 0) debugger; // FIXME FIXME FIXME FIXME FIXME FIXME!
								polytope.faces.push(new THREE.Face3(edge.a, edge.b, _i, ABV));
								polytope.faces[polytope.faces.length-1].normal.normalize();

								// FIXME FIXME FIXME FIXME FIXME FIXME!
								// TODO: this is a terrible way of fixing triangles!
								if (!faceNormalIsGood(polytope.faces[polytope.faces.length-1])) {
									// debugger;
									polytope.faces[polytope.faces.length-1].normal = AV.clone().cross(AB).normalize();
								}
								// FIXME FIXME FIXME FIXME FIXME FIXME!

								var _new_face_test = polytope.faces[polytope.faces.length-1];
								// FIXME FIXME FIXME FIXME FIXME FIXME!
								if (_new_face_test.normal.x == 0 &&
									_new_face_test.normal.y == 0 &&
									_new_face_test.normal.z == 0) {
										debugger;
									}
								// FIXME FIXME FIXME FIXME FIXME FIXME!

								/*
								var a = simplex[nearestFace.a],
									b = simplex[nearestFace.b],
									c = simplex[nearestFace.c],
									_i = polytope.vertices.length-1,
									_v = polytope.vertices[_i],
									AB = b.clone().sub(a),
									AC = c.clone().sub(a),
									BC = c.clone().sub(b),
									AV = _v.clone().sub(a),
									BV = _v.clone().sub(b),
									AVB = AV.clone().cross(AB),
									ACV = AC.clone().cross(AV),
									BCV = BC.clone().cross(BV);
								polytope.faces.push(new THREE.Face3(nearestFace.a, _i, nearestFace.b, AVB));
								polytope.faces.push(new THREE.Face3(nearestFace.a, nearestFace.c, _i, ACV));
								polytope.faces.push(new THREE.Face3(nearestFace.b, nearestFace.c, _i, BCV));
								*/
							}
						}
					};

					return true;
				}

				if (--iFroze <= 0) return false;
			}
		};

		this.doSimplex = function(simplex){

			var a = simplex[simplex.length - 1].clone(),
				ao = a.clone().negate();

			a.sup_a = simplex[simplex.length - 1].sup_a;
			a.sup_b = simplex[simplex.length - 1].sup_b;
			if (simplex.length === 2) {

				// Dealing with a line
				//
				//	You can imagine the line AB being held by 2 planes. AB is the normal to both planes, and
				//	each plane as a point on it (A/B for their associated plane). The origin belongs in one of
				//	3 possible locations:
				//		1) The origin is between the two planes
				//		2) The origin is on the other side of the A-plane
				//		3*) The origin is on the other side of the B-plane
				//
				//	Since A is the most recently added point to the simplex, then we know that we've picked A
				//	based off of the direction towards the origin. In other words, we started at B and then
				//	choose A such that A should be in the direction of the origin. So we can rule out
				//	possibility 3 (origin on the other side of B-plane) since we already know the origin is in
				//	the direction of A.
				//
				//
				//         |            |
				//         |     [1]    |
				//         |            |
				//  [3]  B <------------X A  [2]
				//         |            |
				//         |     [1]    |
				//         |            |
				//
				//
				// 	We can take the dot product between AB and AO to find the case here. If the dot product is
				// 	positive, then O is between the two planes, otherwise O is on the other side of the
				// 	A-plane.

				var AB = simplex[0].clone().sub(a),
					AO = a.clone().negate(),
					dot = AB.dot(AO),
					d = null;

				if (dot > 0) {
					d = AB.clone().cross(AO).cross(AB);
				} else {
					simplex.shift(); // Remove B (A is the new B)
					d = AO;
				}

				if (!d) debugger;
				return d;

			} else if (simplex.length === 3) {

				// Dealing with a triangle
				//
				// The triangle case is a little more complicated, with 8 different cases (5 of which are
				// possibilities). First its important to note that the triangle is a planar primitive in 3d
				// space, so this triangle has a 3d normal specified by ABC. Since A is the newest added
				// point, then we can throw away cases 6-8 as possibilites for the origin.
				// 		1) The origin is on the other side of AC
				// 		2) The origin is in the triangular region (positive side of the normal)
				// 		3) The origin is in the triangular region (negative side of the normal)
				// 		4) The origin is on the other side of AB
				// 		5) The origin is on the A side
				//
				// 	Since the triangle formed by ABC is a 3d planar primtive, it has a front and back side
				// 	associated with it. The front side (2) is the positive normal, whereas the back side (3)
				// 	is the negative normal.
				// 	NOTE: ABC (triangle normal) is pointing downwards
				//
				//
				//             *
				//            *
				//   [6]    *
				//          *          [1]          *
				//      C  *                       *
				//  *******X<--___                *
				//         *     ---___          *
				//         *           ---__    *
				//         *  [2]          -- *
				//    [7]  *                  X   A   [5]
				//         *       [3]   __-- *
				//         *         ___--      *
				//         *   ___---            *
				//  *******X<--                   *
				//      B  *                       *
				//          *         [4]           *
				//   [8]     *
				//            *
				//             *
				//
				//
				//	This is again solved by going through each possible region and comparing the dot product
				//	of the line with AO to determine if the origin is within that region.

				var AB = simplex[1].clone().sub(a),
					AC = simplex[0].clone().sub(a),
					AO = a.clone().negate(),
					ABC = AB.clone().cross(AC),
					d = null;

				// Are we in either [1] or [5] ?
				//	NOTE: ABCxAC = -ACxABC
				if (ABC.clone().cross(AC).dot(AO) > 0) {
					if (AC.dot(AO) > 0) {
						// We're in [1]
						simplex.splice(1, 1); // Use AC line instead
						d = AC.clone().cross(AO).cross(AC);
						if (!d) debugger;
					} else {
						// We're in [5]
						// FIXME: we should ONLY be in [5] right? why check [4]?
						if (AB.dot(AO) > 0) {
							simplex.splice(0, 1);
							d = AB.cross(AO).cross(AB);
							if (!d) debugger;
						} else {
							simplex[0] = simplex[2];
							simplex.splice(1,2);
							d = AO;
							if (!d) debugger;
						}
					}
				} else {
					// Are we in either [4] or [5] ?
					if (AB.clone().cross(ABC).dot(AO) > 0) {
						if (AB.dot(AO) > 0) {
							simplex.splice(0, 1);
							d = AB.cross(AO).cross(AB);
							if (!d) debugger;
						} else {
							simplex[0] = simplex[2];
							simplex.splice(1,2);
							d = AO;
							if (!d) debugger;
						}
					} else {
						// We're in either [2] or [3]
						if (ABC.dot(AO) > 0) {
							// Along the normal
							d = ABC;
							if (!d) debugger;
						} else {
							// Along the negative normal
							var _B = simplex[1];
							simplex[1] = simplex[0];
							simplex[0] = _B;
							d = ABC.negate();
							if (!d) debugger;
						}
					}
				}

				if (!d) debugger;
				return d;

			} else if (simplex.length === 4) {



				var b  = simplex[2],
					c  = simplex[1],
					d  = simplex[0],
					AB = simplex[2].clone().sub(a),
					AC = simplex[1].clone().sub(a),
					AD = simplex[0].clone().sub(a),
					AO = a.clone().negate(),
					ABC = AB.clone().cross(AC),
					ACD = AC.clone().cross(AD),
					ADB = AD.clone().cross(AB),
					D = null;

				// Find the face (normal direction)
				if (ABC.dot(AO) > 0) {
					simplex[0] = c;
					simplex[1] = b;
					simplex[2] = a;
					simplex.splice(3,1);
					D = ABC;
				} else if (ACD.dot(AO) > 0) {
					simplex[0] = d;
					simplex[1] = c;
					simplex[2] = a;
					simplex.splice(3,1);
					D = ACD;
				} else if (ADB.dot(AO) > 0) {
					simplex[0] = d;
					simplex[1] = b;
					simplex[2] = a;
					simplex.splice(3,1);
					D = ADB;
				} else {
					return true;
				}


				c = simplex[0];
				b = simplex[1];
				a = simplex[2];
				AB = b.clone().sub(a);
				AC = c.clone().sub(a);

				if (AB.clone().cross(D).dot(AO) > 0) {
					D = AB.clone().cross(AO).cross(AB);
					simplex[0] = b;
					simplex[1] = a;
					simplex.splice(2,1);
					return D;
				} else if (D.clone().cross(AC).dot(AO) > 0) {
					D = AC.clone().cross(AO).cross(AC);
					simplex[0] = c;
					simplex[1] = a;
					simplex.splice(2,1);
					return D;
				}

				return D;
			}
		};

		this.getNearestFaceGJK = function(polytope){
			// TODO: cache dot products of faces

			var nearestFace = null,
				nearestFaceD = 99999999;
			for (var i=0; i<polytope.faces.length; ++i) {
				var face = polytope.faces[i],
					dA = Math.abs(polytope.vertices[face.a].dot(face.normal)),
					dB = Math.abs(polytope.vertices[face.b].dot(face.normal)),
					dC = Math.abs(polytope.vertices[face.c].dot(face.normal));
				if (dA < nearestFaceD) {
					nearestFace = face;
					nearestFaceD = dA;
				}
				if (dB < nearestFaceD) {
					nearestFace = face;
					nearestFaceD = dB;
				}
				if (dC < nearestFaceD) {
					nearestFace = face;
					nearestFaceD = dC;
				}
			}

			return {
				face: nearestFace,
				distance: nearestFaceD
			}
		};

		this.supportGJK = function(bodyA, bodyB, direction){

			// TODO: transform direction vector into bodyA/bodyB (rotation) for optimization
			// TODO: initially create a neighbouring of vertices in each body; then use hill-climbing
			// 			technique to find support vertex
			// TODO: cache previous support points, then use hill-climbing (from each support point) to reach
			// 			vertex faster


			/////////////////////////////////////////////////////////////////
			// FIXME FIXME FIXME FIXME FIXME FIXME FIXME
			var supportCube = function(body, direction){

				var dir = direction.clone().normalize(),
					maxDot = -1,
					maxV = null;

				// dir.applyQuaternion(body.quaternion);
				for (var i=0; i<body.geometry.vertices.length; ++i) {
					var v = body.geometry.vertices[i].clone().applyQuaternion(body.quaternion),
						d = v.dot(dir);
					if (d > maxDot) {
						maxDot = d;
						maxV = v;
						v.i = i;
					}
				}

				if (maxV === null) {
					throw new Error("Couldn't find support vertex");
				}

				maxV.add(body.position);
				return maxV;
			};
			// FIXME FIXME FIXME FIXME FIXME FIXME FIXME
			/////////////////////////////////////////////////////////////////


			var p1 = supportCube(bodyA, direction),
				p2 = supportCube(bodyB, direction.clone().negate());
				
			var p = p1.clone().sub(p2);
			p.sup_a = p1;
			p.sup_b = p2;
			return p;
		};

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

		var FIXMEBlock = true;
		var Contact = function(contact){

			this.bodyA = contact.bodyA;
			this.bodyB = contact.bodyB;

			this.normal = contact.normal;
			this.vertexA = contact.originA;
			this.vertexB = contact.originB;
			this.localA = contact.localA;
			this.localB = contact.localB;
			this.depth = contact.depth;

			this.tangent = this.vertexA.clone().sub(this.bodyA.position).multiplyScalar(TEST_ScaleT1).cross( this.normal ); // FIXME: using correct normal?
			this.tangent2 = this.vertexB.clone().sub(this.bodyB.position).multiplyScalar(TEST_ScaleT2).cross( this.normal );// FIXME: using correct normal

			if (TEST_NormTangent) {
				this.tangent.normalize();
				this.tangent2.normalize();
			}

			this.impulse = 0;
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
					this.tangent = this.vertexA.clone().sub(this.bodyA.position).multiplyScalar(TEST_ScaleT1).cross( this.normal ); // FIXME: using correct normal?
					this.tangent2 = this.vertexB.clone().sub(this.bodyB.position).multiplyScalar(TEST_ScaleT2).cross( this.normal );// FIXME: using correct normal

					if (TEST_NormTangent) {
						this.tangent.normalize();
						this.tangent2.normalize();
					}
					this.depth = contact.depth;
					this.setJacobian(this.rows[0]);
				}


				this.JDiagABInv = 0;
				var denom1 = 0, denom2 = 0;
				if (this.bodyA && this.bodyA.invMass !== 0) {
					// denom1 = this.bodyA.invMass + this.normal.dot( this.tangent.clone().multiplyScalar(this.bodyA.invInertiaTensor).cross( this.vertexA.clone().sub(this.bodyA.position) ) );
					denom1 = this.bodyA.invMass + this.normal.dot( this.tangent.clone().multiplyScalar(TEST_DenomFactor1*this.bodyA.invInertiaTensor).cross( this.vertexA.clone().sub(this.bodyA.position) ) );
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

		var Manifold = function(bodyA, bodyB){

			this.bodyA = bodyA;
			this.bodyB = bodyB;
			this.contacts = {};
			this.length = 0;

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
				var _contact = new Contact(contact);
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
				contactManifolds[contact.hash] = new Manifold(contact.bodyA, contact.bodyB);
			}

			contactManifolds[contact.hash].updateContact(contact);
		};

		this.removeManifold = function(hash){
			if (contactManifolds[hash]) {
				var manifold = contactManifolds[hash];
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
					delete contactManifolds[manifoldID];
				}
			}
		};

		this.solveConstraints = function(){

			for (var manifoldID in contactManifolds) {

				var manifold = contactManifolds[manifoldID];
				for (var contactID in manifold.contacts) {

					var contact = manifold.contacts[contactID];
					// contact.appliedImpulse = 0;

					// ReactPhysics way
					// Warmstart
					contact.impulse *= 0.9;
					var rA = contact.vertexA.clone().sub(contact.bodyA.position),
						rB = contact.vertexB.clone().sub(contact.bodyB.position);
					contact.bodyA.velocity.add( contact.normal.clone().multiplyScalar(contact.bodyA.invMass * contact.impulse) );
					contact.bodyA.angularVelocity.add( rA.clone().cross(contact.normal).multiplyScalar(contact.bodyA.invInertiaTensor * contact.impulse) );

					contact.bodyB.velocity.sub( contact.normal.clone().multiplyScalar(contact.bodyB.invMass * contact.impulse) );
					contact.bodyB.angularVelocity.sub( rB.clone().cross(contact.normal).multiplyScalar(contact.bodyB.invInertiaTensor * contact.impulse) );


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

			if (false) {
// 
// 			// Penetration Solver
// 			for (var manifoldID in contactManifolds) {
// 
// 				var manifold = contactManifolds[manifoldID];
// 				for (var contactID in manifold.contacts) {
// 
// 					// FIXME: reset appliedImpulse ??
// 					var contact = manifold.contacts[contactID],
// 						row = contact.rows[0];
// 
// 
// 					// contact.appliedImpulse = 0;
// 
// 
// 					for (var iteration=0; iteration<1; ++iteration) {
// 
// 						var delta = contact.depth - contact.appliedPushImpulse, // FIXME: appliedPushImpulse
// 							deltaV1Dotn = contact.normal.dot( contact.bodyA.velocity ) + TEST_T1Factor * contact.tangent.dot( contact.bodyA.angularVelocity ),
// 							deltaV2Dotn = contact.normal.clone().negate().dot( contact.bodyB.velocity ) + TEST_T2Factor * contact.tangent2.dot( contact.bodyB.angularVelocity );
// 
// 						delta -= deltaV1Dotn * contact.JDiagABInv;
// 						delta -= deltaV2Dotn * contact.JDiagABInv;
// 
// 						var sum = contact.appliedPushImpulse + delta; // FIXME: appliedPushImpulse
// 						if (sum < 0) {
// 							delta = -contact.appliedPushImpulse; // FIXME: appliedPushImpulse
// 							contact.appliedPushImpulse = 0; // FIXME: appliedPushImpulse
// 							break;
// 						} else {
// 							contact.appliedPushImpulse = sum; // FIXME: appliedPushImpulse
// 						}
// 
// 						contact.bodyA.velocity.add( contact.normal.clone().multiplyScalar(contact.bodyA.invMass * delta) );
// 
// 
// 						// var v = contact.tangent.clone().multiplyScalar(-contact.bodyA.invInertiaTensor * delta),
// 						// 	e = (new THREE.Euler()).setFromVector3(v),
// 						// 	q = (new THREE.Quaternion()).setFromEuler(e);
// 						// contact.bodyA.angularVelocity.applyQuaternion(q);
// 						contact.bodyA.angularVelocity.add( contact.tangent.clone().multiplyScalar(contact.bodyA.invInertiaTensor * delta) );
// 
// 						contact.bodyB.velocity.add( contact.normal.clone().negate().multiplyScalar(contact.bodyB.invMass * delta) );
// 						// v = contact.tangent2.clone().multiplyScalar(contact.bodyB.invInertiaTensor * delta);
// 						// e = (new THREE.Euler()).setFromVector3(v);
// 						// q = (new THREE.Quaternion()).setFromEuler(e);
// 						// contact.bodyB.angularVelocity.applyQuaternion(q);
// 						contact.bodyB.angularVelocity.add( contact.tangent2.clone().multiplyScalar(contact.bodyB.invInertiaTensor * delta) );
// 
// 						/*
// 						// Box2D way
// 						var rA = contact.vertexA.clone().sub(contact.bodyA.position),
// 							rB = contact.vertexB.clone().sub(contact.bodyB.position),
// 							C = 0.2 * contact.depth,
// 							rnA = rA.clone().cross(contact.normal),
// 							rnB = rB.clone().cross(contact.normal),
// 							K = contact.bodyA.invMass + contact.bodyB.invMass + contact.bodyA.invInertiaTensor * rnA.lengthSq() + contact.bodyB.invInertiaTensor * rnB.lengthSq(),
// 							impulse = K > 0 ? -C/K : 0,
// 							P = contact.normal.clone().multiplyScalar(impulse);
// 
// 						contact.bodyA.position.sub( P.clone().multiplyScalar(contact.bodyA.invMass) );
// 						contact.bodyA.rotation.sub( rA.clone().cross(P).multiplyScalar(contact.bodyA.invInertiaTensor) );
// 
// 						contact.bodyB.position.add( P.clone().multiplyScalar(contact.bodyB.invMass) );
// 						contact.bodyB.rotation.add( rB.clone().cross(P).multiplyScalar(contact.bodyB.invInertiaTensor) );
// 						*/
// 
// 
// 					}
// 
// 
// 
// 					// FIXME: warmstarting!?
// 					contact.appliedImpulse *= 0.85;
// 
// 					contact.bodyA.applyImpulse( contact.normal.clone().multiplyScalar(contact.bodyA.invMass), contact.tangent.clone().multiplyScalar(-contact.bodyA.invInertiaTensor), contact.appliedImpulse );
// 					contact.bodyB.applyImpulse( contact.normal.clone().multiplyScalar(contact.bodyB.invMass), contact.tangent2.clone().multiplyScalar(contact.bodyB.invInertiaTensor), contact.appliedImpulse );
// 
// 
// 
// 
// 
// 					// contact.appliedImpulse = 0;
// 					// contact.appliedPushImpulse = 0;
// 
// 					// var rel_vel = 0.0;
// 
// 					// if (contact.bodyA && contact.bodyA.invMass !== 0) {
// 					// 	// rel_vel += contact.bodyA.velocity.clone().add( contact.bodyA.angularVelocity.clone().cross( contact.vertexA.clone().sub(contact.bodyA.position) ) ).dot( contact.normal );
// 					// 	rel_vel += contact.normal.dot(contact.bodyA.velocity) + contact.tangent.dot(contact.bodyA.angularVelocity);
// 					// }
// 
// 					// if (contact.bodyB && contact.bodyB.invMass !== 0) {
// 					// 	// rel_vel += contact.bodyB.velocity.clone().add( contact.bodyB.angularVelocity.clone().cross( contact.vertexB.clone().sub(contact.bodyB.position) ) ).dot( contact.normal.clone().negate() );
// 					// 	rel_vel += contact.normal.clone().negate().dot(contact.bodyB.velocity) - contact.tangent2.dot(contact.bodyB.angularVelocity);
// 					// }
// 
// 
// 					var baumgarte = -0.2, // FIXME: wth to do with this?!
// 						restitution = 0.00; // FIXME: wth to do with this?!
// 
// 					// var positionalError = 0.0,
// 					// 	velocityError = restitution * -rel_vel - rel_vel * physique.world.damping;
// 
// 					// positionalError =  (contact.depth + physique.world.slop) * baumgarte / dt;
// 
// 					// var penetrationImpulse = positionalError * contact.JDiagABInv,
// 					// 	velocityImpulse = velocityError * contact.JDiagABInv;
// 
// 					// contact.rhs = penetrationImpulse + velocityImpulse; // FIXME: bullet way
// 					// contact.rhs = (contact.normal.clone().add(contact.bodyA.angularVelocity.clone().cross(contact.vertexA)).dot(contact.vertexA)) * restitution; // FIXME: goblin way
// 					var JDotV = 0;
// 					JDotV += row.J[0] * contact.bodyA.velocity.x;
// 					JDotV += row.J[1] * contact.bodyA.velocity.y;
// 					JDotV += row.J[2] * contact.bodyA.velocity.z;
// 					JDotV += row.J[3] * contact.bodyA.angularVelocity.x;
// 					JDotV += row.J[4] * contact.bodyA.angularVelocity.y;
// 					JDotV += row.J[5] * contact.bodyA.angularVelocity.z;
// 
// 					JDotV += row.J[6]  * contact.bodyB.velocity.x;
// 					JDotV += row.J[7]  * contact.bodyB.velocity.y;
// 					JDotV += row.J[8]  * contact.bodyB.velocity.z;
// 					JDotV += row.J[9]  * contact.bodyB.angularVelocity.x;
// 					JDotV += row.J[10] * contact.bodyB.angularVelocity.y;
// 					JDotV += row.J[11] * contact.bodyB.angularVelocity.z;
// 					var idt = 1/dt;
// 					var C = JDotV,
// 						JDotVTerm = 0,
// 						Fext = new THREE.Vector3(0, -9.8, 0),
// 						Text = new THREE.Vector3(0, 0, 0);
// 					JDotVTerm += row.J[0] * (contact.bodyA.velocity.x / idt + contact.bodyA.invMass * Fext.x);
// 					JDotVTerm += row.J[1] * (contact.bodyA.velocity.y / idt + contact.bodyA.invMass * Fext.y);
// 					JDotVTerm += row.J[2] * (contact.bodyA.velocity.z / idt + contact.bodyA.invMass * Fext.z);
// 					JDotVTerm += row.J[3] * (contact.bodyA.angularVelocity.x / idt + contact.bodyA.invMass * Text.x);
// 					JDotVTerm += row.J[4] * (contact.bodyA.angularVelocity.y / idt + contact.bodyA.invMass * Text.y);
// 					JDotVTerm += row.J[5] * (contact.bodyA.angularVelocity.z / idt + contact.bodyA.invMass * Text.z);
// 
// 					JDotVTerm += row.J[6]  * (contact.bodyB.velocity.x / idt + contact.bodyB.invMass * Fext.x);
// 					JDotVTerm += row.J[7]  * (contact.bodyB.velocity.y / idt + contact.bodyB.invMass * Fext.y);
// 					JDotVTerm += row.J[8]  * (contact.bodyB.velocity.z / idt + contact.bodyB.invMass * Fext.z);
// 					JDotVTerm += row.J[9]  * (contact.bodyB.angularVelocity.x / idt + contact.bodyB.invMass * Text.x);
// 					JDotVTerm += row.J[10] * (contact.bodyB.angularVelocity.y / idt + contact.bodyB.invMass * Text.y);
// 					JDotVTerm += row.J[11] * (contact.bodyB.angularVelocity.z / idt + contact.bodyB.invMass * Text.z);
// 
// 
// 					contact.rhs = baumgarte*C/idt - restitution*JDotVTerm; // FIXME: Erin Catto GDC2005 way
// 
// 				}
// 			}
// 
			}


			var collisionResIterations = 10;
					for (var iteration=0; iteration<collisionResIterations; ++iteration) {
			for (var manifoldID in contactManifolds) {

				var manifold = contactManifolds[manifoldID];
				for (var contactID in manifold.contacts) {

					var contact = manifold.contacts[contactID];


						var row = contact.rows[0], // contact constraint
							JdotV = 0,
							delta = 0;


						/* NOTE: bullet way
						delta = contact.rhs - contact.appliedImpulse;

						// FIXME: tangent contactOnNormalB? reset applied impulse? reset velocities? JDiagABInv, invInertiaTensor

						var dV1Dotn = contact.normal.dot(contact.bodyA.deltaV) + contact.tangent.dot(contact.bodyA.deltaW),
							dV2Dotn = contact.normal.clone().negate().dot(contact.bodyB.deltaV) + contact.tangent2.dot(contact.bodyB.deltaW);

						delta += -(dV1Dotn * contact.JDiagABInv + dV2Dotn * contact.JDiagABInv);
						*/

						/*
						// Erin Catto GDC-2005 way
						// a = B*lambda
						var d = row.D,
							JDotA = 0,
							a = [],
							JV = 0;

						// for (var ji=0; ji<12; ++ji) {
						// 	a[ji] = row.B[ji] * contact.appliedImpulse;
						// 	JDotA += row.J[ji] * a[ji];
						// }
						// delta = (contact.rhs - JDotA) / d;
						JV += contact.normal.dot(contact.bodyA.velocity) + contact.tangent.dot(contact.bodyA.angularVelocity);
						JV -= contact.normal.dot(contact.bodyB.velocity) - contact.tangent2.dot(contact.bodyB.angularVelocity);
						delta = (contact.rhs - JV) / d;






						var sum = contact.appliedImpulse + delta;
						if (sum < 0) {
							delta = -contact.appliedImpulse;
							sum = 0;
							contact.appliedImpulse = 0;
						} else {
							contact.appliedImpulse = sum;
						}

						// if (Math.abs(delta) < 0.0001) break; // Breaking impulse

						contact.bodyA.applyImpulse(contact.normal.clone().multiplyScalar(contact.bodyA.invMass), contact.tangent.clone().multiplyScalar(contact.bodyA.invInertiaTensor), delta);
						contact.bodyB.applyImpulse(contact.normal.clone().multiplyScalar(-contact.bodyB.invMass), contact.tangent2.clone().multiplyScalar(contact.bodyB.invInertiaTensor), delta);
						*/


						// ReactPhysics3D way
						// FIXME: may need to reverse normal (they use triangle.normal)
						// FIXME: rA = pA - a, rB = pB - b ... they use 2 contact points.. transform contact
						// point into A or B space (probably rotate/add position). 
						// 	pA = triangle->computeClosestPointOfObject(suppPointsA)
						// 	pB = body2Tobody1.getInverse() * triangle->computeClosestPointOfObject(suppPointsB)
						var rA = contact.vertexA.clone().sub(contact.bodyA.position),
							rB = contact.vertexB.clone().sub(contact.bodyB.position),
							dV = contact.bodyB.velocity.clone().add( contact.bodyB.angularVelocity.clone().cross(rB) );
						dV.sub(  contact.bodyA.velocity.clone().add( contact.bodyA.angularVelocity.clone().cross(rA) ) );

						var deltaVDotN = -dV.dot(contact.normal),
							JV = deltaVDotN;

						var b = (contact.depth - physique.world.slop) * (-0.4)// + JV * 0.25; // TODO: restitution

						// var MA = new THREE.Vector3(), MB = new THREE.Vector3();
						var MA = 0, MB = 0;
						if (contact.bodyA.invMass !== 0) {
							// MA = contact.bodyA.invMass + contact.bodyA.invInertiaTensor * (rA.clone().cross(contact.normal.clone().negate())).dot(contact.normal.clone().negate());
							// MA = contact.bodyA.invMass + (contact.vertexA.clone().cross(contact.normal.clone().negate()).multiplyScalar(contact.bodyA.invInertiaTensor)).cross(contact.vertexA).dot(contact.normal.clone().negate());
							// MA = contact.bodyA.invMass + contact.bodyA.invInertiaTensor * (contact.vertexA.clone().cross(contact.normal.clone().negate())).cross(contact.vertexA).dot(contact.normal.clone().negate());
							var mass = contact.bodyA.invMass,
								iner = contact.bodyA.invInertiaTensor,
								norm = contact.normal.clone().negate(),
								tang = rA.clone().cross(norm);
							// MA = norm.clone().multiplyScalar(mass);
							// MA.add( rA.clone().cross(norm).multiplyScalar(iner).cross(rA) );
							MA = mass * Math.pow(norm.x, 2) + mass * Math.pow(norm.y, 2) + mass * Math.pow(norm.z, 2);
							MA += iner * Math.pow(tang.x, 2) + iner * Math.pow(tang.y, 2) + iner * Math.pow(tang.z, 2);
						}
						if (contact.bodyB.invMass !== 0) {
							// MB = contact.bodyB.invMass + contact.bodyB.invInertiaTensor * (rB.clone().cross(contact.normal.clone().negate())).dot(contact.normal.clone().negate());
							// MB = contact.bodyB.invMass + (contact.vertexB.clone().cross(contact.normal.clone().negate()).multiplyScalar(contact.bodyB.invInertiaTensor)).cross(contact.vertexB).dot(contact.normal.clone().negate());
							// MB = contact.bodyB.invMass + contact.bodyB.invInertiaTensor * (contact.vertexB.clone().cross(contact.normal.clone().negate())).cross(contact.vertexB).dot(contact.normal.clone().negate());
							var mass = contact.bodyB.invMass,
								iner = contact.bodyB.invInertiaTensor,
								norm = contact.normal.clone().negate(),
								tang = rB.clone().cross(norm);
							// MB = norm.clone().multiplyScalar(-mass);
							// MB.sub( rB.clone().cross(norm.clone().negate()).multiplyScalar(iner).cross(rB) );
							MB = mass * Math.pow(norm.x, 2) + mass * Math.pow(norm.y, 2) + mass * Math.pow(norm.z, 2);
							MB += iner * Math.pow(tang.x, 2) + iner * Math.pow(tang.y, 2) + iner * Math.pow(tang.z, 2);
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




					}
				}
			}

		};



		/*****************************************************************
		 *****************************************************************
		 							Dynamics
		 */

		this.addBody = function(mesh){
			if (this.bodies.hasOwnProperty(mesh.uid)) {
				throw new Error("Body uid ("+mesh.uid+") already exists");
			}

			this.bodies[mesh.uid] = mesh.body;

			var bodyType = mesh.settings.body;
			if (bodyType === BODY_CUBE) {

				mesh.body.invMass = 1 / (mass);
				mesh.body.invInertiaTensor = 1 / (inertia);
				mesh.body.velocity = new THREE.Vector3();
				mesh.body.angularVelocity = new THREE.Vector3();
				mesh.body.impulse = [0,0,0,0,0,0]
				mesh.body.active = true;

			} else if (bodyType === BODY_FLOOR) {

				mesh.body.invMass = 0.0;
				mesh.body.invInertiaTensor = 0.0;
				mesh.body.velocity = new THREE.Vector3();
				mesh.body.angularVelocity = new THREE.Vector3();
				mesh.body.impulse = [0,0,0,0,0,0]
				mesh.body.active = false;

			} else {
				throw new Error("Adding undefined body: "+ bodyType);
			}

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

			this.addBodyIntoBroadphase(mesh.body);
		};

		this.checkCollision = function(bodyA, bodyB) {
			var posA = bodyA.position,
				posB = bodyB.position,
				dimA = bodyA.dimensions,
				dimB = bodyB.dimensions;

			var boundsA = {
					left:    posA.x - (dimA.width / 2.0),
					right:   posA.x + (dimA.width / 2.0),
					top:     posA.y + (dimA.height / 2.0),
					bottom:  posA.y - (dimA.height / 2.0),
					near:    posA.z - (dimA.depth / 2.0),
					far:     posA.z + (dimA.depth / 2.0)
				}, boundsB = {
					left:    posB.x - (dimB.width / 2.0),
					right:   posB.x + (dimB.width / 2.0),
					top:     posB.y + (dimB.height / 2.0),
					bottom:  posB.y - (dimB.height / 2.0),
					near:    posB.z - (dimB.depth / 2.0),
					far:     posB.z + (dimB.depth / 2.0)
				};

			if (((boundsA.left > boundsB.left && boundsB.right < boundsA.right) ||
				 (boundsA.left < boundsB.left && boundsA.right > boundsB.left) ||
				 (boundsA.left > boundsB.left && boundsA.right < boundsB.right))
					&&
				((boundsA.bottom > boundsB.bottom && boundsB.top < boundsA.top) ||
				 (boundsA.bottom < boundsB.bottom && boundsA.top > boundsB.bottom) ||
				 (boundsA.bottom > boundsB.bottom && boundsA.top < boundsB.top))
					&&
				((boundsA.near > boundsB.near && boundsB.far < boundsA.far) ||
				 (boundsA.near < boundsB.near && boundsA.far > boundsB.near)) ||
				 (boundsA.near > boundsB.near && boundsA.far < boundsB.far)) {

					 return true;
				 }
			
			return false;
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

				if (body.active) {

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

					this.updateAABB(body);
					this.updateBodyInBroadphase(body);
				} else if (body.userMoved) {
					body.userMoved = false;
					this.updateAABB(body);
					this.updateBodyInBroadphase(body);
				}
			}


			this.updateManifolds(); // NOTE: need to update manifolds BEFORE updating particular contacts below
			var hits = this.sweepAndPrune();
			for (var hitHash in hits) {

				var hit = hits[hitHash],
					bodyA = hit.bodyA,
					bodyB = hit.bodyB,
					contact = null;

				contact = this.checkGJK(bodyA, bodyB);
				// contact = this.SAT(bodyA, bodyB);
				if (!contact) {
					delete hits[hitHash];
					continue;
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

				if (!bodyA.material.hasOwnProperty('storedColor') && bodyA.active) {
					bodyA.material.storedColor = _.clone(bodyA.material.color);
					bodyA.material.color.r = 1.0;
					bodyA.material.color.g = 0.0;
					bodyA.material.color.b = 0.0;
				}

				if (!bodyB.material.hasOwnProperty('storedColor') && bodyB.active) {
					bodyB.material.storedColor = _.clone(bodyB.material.color);
					bodyB.material.color.r = 1.0;
					bodyB.material.color.g = 0.0;
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

				if (body.active) {

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

					// var epsV = 0;
					// if (Math.abs(body.velocity.x) < epsV) body.velocity.x = 0;
					// if (Math.abs(body.velocity.y) < epsV) body.velocity.y = 0;
					// if (Math.abs(body.velocity.z) < epsV) body.velocity.z = 0;
					// if (Math.abs(body.angularVelocity.x) < epsV) body.angularVelocity.x = 0;
					// if (Math.abs(body.angularVelocity.y) < epsV) body.angularVelocity.y = 0;
					// if (Math.abs(body.angularVelocity.z) < epsV) body.angularVelocity.z = 0;

					// body.deltaV.multiplyScalar(0);
					// body.deltaW.multiplyScalar(0);
					// body.deltaV.multiplyScalar(damping);
					// body.deltaW.multiplyScalar(damping);

					this.updateAABB(body);
					this.updateBodyInBroadphase(body);
				}
			}
		};
	};

	return Physique;
});
