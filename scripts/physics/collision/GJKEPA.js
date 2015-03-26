define(['physics/memstore'], function(MemStore){

	/* TODO
	 * 	- memstore
	 * 	- adjust iFroze
	 */

	var GJKEPA = function(){

		var maximumIterationsOfGJK = 0;
		this.checkGJK = function(bodyA, bodyB){

			var d = MemStore.Vector3(1, 0, 0),
				p = this.supportGJK(bodyA, bodyB, d),
				simplex = [p];

			d = (MemStore.Vector3()).sub(p);
			var iFroze = 50;
			while (true) {
				p = this.supportGJK(bodyA, bodyB, d);

				if (p.dot(d) < 0) {
					return false;
				}

				simplex.push(p);
				d = this.doSimplex(simplex);
				if (d === true) {

					if ((50 - iFroze) > maximumIterationsOfGJK) {
						maximumIterationsOfGJK = 50 - iFroze;
						console.log("Maximum iterations for GJK: "+maximumIterationsOfGJK);
					}

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
							// var aA = bodyA.geometry.vertices[a.sup_a.i].clone(),
							// 	bA = bodyA.geometry.vertices[b.sup_a.i].clone(),
							// 	cA = bodyA.geometry.vertices[c.sup_a.i].clone(),
							// 	aB = bodyB.geometry.vertices[a.sup_b.i].clone(),
							// 	bB = bodyB.geometry.vertices[b.sup_b.i].clone(),
							// 	cB = bodyB.geometry.vertices[c.sup_b.i].clone(),
							var bodyAInvQ = bodyA.quaternion.clone().inverse(),
								bodyBInvQ = bodyB.quaternion.clone().inverse();
							var aA = a.sup_a.sub(bodyA.position).applyQuaternion(bodyAInvQ),
								bA = b.sup_a.sub(bodyA.position).applyQuaternion(bodyAInvQ),
								cA = c.sup_a.sub(bodyA.position).applyQuaternion(bodyAInvQ),
								aB = a.sup_b.sub(bodyB.position).applyQuaternion(bodyBInvQ),
								bB = b.sup_b.sub(bodyB.position).applyQuaternion(bodyBInvQ),
								cB = c.sup_b.sub(bodyB.position).applyQuaternion(bodyBInvQ),
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
							// contact.sub( MemStore.Vector3(0.5, 0.5, 0.5) );
							
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

			var a = simplex[simplex.length - 1].clone();

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

				var AB = MemStore.Vector3( simplex[0] ).sub(a),
					AO = MemStore.Vector3( a ).negate(),
					dot = AB.dot(AO),
					d = null;

				if (dot > 0) {
					d = MemStore.Vector3( AB ).cross(AO).cross(AB);
					AB.free();
					AO.free();
				} else {
					simplex.shift(); // Remove B (A is the new B)
					d = AO;
				}

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

				var AB = MemStore.Vector3( simplex[1] ).sub(a),
					AC = MemStore.Vector3( simplex[0] ).sub(a),
					AO = MemStore.Vector3( a ).negate(),
					ABC = MemStore.Vector3( AB ).cross(AC),
					d = null,
					regionIndicator = MemStore.Vector3( ABC ).cross(AC);

				// Are we in either [1] or [5] ?
				//	NOTE: ABCxAC = -ACxABC
				if (regionIndicator.dot(AO) > 0) {
					if (AC.dot(AO) > 0) {
						// We're in [1]
						simplex.splice(1, 1); // Use AC line instead
						d = MemStore.Vector3( AC ).cross(AO).cross(AC);
					} else {
						// We're in [5]
						// FIXME: we should ONLY be in [5] right? why check [4]?
						if (AB.dot(AO) > 0) {
							simplex.splice(0, 1);
							d = MemStore.Vector3( AB ).cross(AO).cross(AB);
						} else {
							simplex[0] = simplex[2];
							simplex.splice(1,2);
							d = MemStore.Vector3( AO );
						}
					}
				} else {
					// Are we in either [4] or [5] ?
					regionIndicator.free();
					regionIndicator = MemStore.Vector3( AB ).cross(ABC);
					if (regionIndicator.dot(AO) > 0) {
						if (AB.dot(AO) > 0) {
							simplex.splice(0, 1);
							d = MemStore.Vector3( AB ).cross(AO).cross(AB);
						} else {
							simplex[0] = simplex[2];
							simplex.splice(1,2);
							d = MemStore.Vector3( AO );
						}
					} else {
						// We're in either [2] or [3]
						if (ABC.dot(AO) > 0) {
							// Along the normal
							d = MemStore.Vector3( ABC );
						} else {
							// Along the negative normal
							var _B = simplex[1];
							simplex[1] = simplex[0];
							simplex[0] = _B;
							d = MemStore.Vector3( ABC ).negate();
						}
					}
				}

				AB.free();
				AC.free();
				AO.free();
				ABC.free();
				regionIndicator.free();
				return d;

			} else if (simplex.length === 4) {



				var b  = simplex[2],
					c  = simplex[1],
					d  = simplex[0],
					AB = MemStore.Vector3( simplex[2] ).sub(a),
					AC = MemStore.Vector3( simplex[1] ).sub(a),
					AD = MemStore.Vector3( simplex[0] ).sub(a),
					AO = MemStore.Vector3( a ).negate(),
					ABC = MemStore.Vector3( AB ).cross(AC),
					ACD = MemStore.Vector3( AC ).cross(AD),
					ADB = MemStore.Vector3( AD ).cross(AB),
					D = null,
					regionIndicator = null;

				// Find the face (normal direction)
				if (ABC.dot(AO) > 0) {
					simplex[0] = c;
					simplex[1] = b;
					simplex[2] = a;
					simplex.splice(3,1);
					D = MemStore.Vector3( ABC );
				} else if (ACD.dot(AO) > 0) {
					simplex[0] = d;
					simplex[1] = c;
					simplex[2] = a;
					simplex.splice(3,1);
					D = MemStore.Vector3( ACD );
				} else if (ADB.dot(AO) > 0) {
					simplex[0] = d;
					simplex[1] = b;
					simplex[2] = a;
					simplex.splice(3,1);
					D = MemStore.Vector3( ADB );
				} else {
					AB.free(); AC.free(); AD.free(); AO.free();
					ABC.free(); ACD.free(); ADB.free();
					return true;
				}


				AB.free(); AC.free(); AD.free();
				ABC.free(); ACD.free(); ADB.free();

				c = simplex[0];
				b = simplex[1];
				a = simplex[2];

				AB = MemStore.Vector3( b ).sub(a);
				AC = MemStore.Vector3( c ).sub(a);
				regionIndicator = MemStore.Vector3( AB ).cross(D);

				if (regionIndicator.dot(AO) > 0) {
					D = MemStore.Vector3( AB ).cross(AO).cross(AB);
					simplex[0] = b;
					simplex[1] = a;
					simplex.splice(2,1);
				} else {
					regionIndicator.free();
					regionIndicator = MemStore.Vector3( D ).cross(AC);
					if (regionIndicator.dot(AO) > 0) {
						D = MemStore.Vector3( AC ).cross(AO).cross(AC);
						simplex[0] = c;
						simplex[1] = a;
						simplex.splice(2,1);
					}
				}

				AB.free(); AC.free(); AO.free();
				regionIndicator.free();
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

				dir.applyQuaternion(body.quaternion.clone().inverse());
				for (var i=0; i<body.geometry.vertices.length; ++i) {
					var v = MemStore.Vector3( body.geometry.vertices[i] ),
						d = v.dot(dir);
					if (d > maxDot) {
						maxDot = d;

						if (maxV) maxV.free();
						maxV = v;
						v.i = i;
					}
				}

				if (maxV === null) {
					throw new Error("Couldn't find support vertex");
				}

				maxV.applyQuaternion(body.quaternion).add(body.position);
				return maxV;
			};
			// FIXME FIXME FIXME FIXME FIXME FIXME FIXME
			/////////////////////////////////////////////////////////////////

			var supportSphere = function(body, direction){

				var dir = direction.clone().normalize();
				dir.i = 0;
				var min = 8;
				var xish = parseInt(dir.x / 0.02) + 13,
					yish = parseInt(dir.y / 0.02) + 13,
					zish = parseInt(dir.z / 0.02) + 13;
				dir.i += xish;
				dir.i += yish * 27;
				dir.i += zish * 27 * 27;
				// dir.i += (dir.x < 0 ? 0 : 1);
				// dir.i += (dir.y < 0 ? 0 : 2);
				// dir.i += (dir.z < 0 ? 0 : 4);

				return dir.applyQuaternion(body.quaternion).multiplyScalar( body.radius * body.radius ).add(body.position);
			};

			var p1 = null,
				p2 = null;
			
			if (bodyA.bodyType === BODY_SPHERE) {
				// p1 = supportSphere(bodyA, direction);
				p1 = supportCube(bodyA, direction);
			} else {
				p1 = supportCube(bodyA, direction);
			}

			if (bodyB.bodyType === BODY_SPHERE) {
				// p2 = supportSphere(bodyB, direction.clone().negate());
				p2 = supportCube(bodyB, direction.clone().negate());
			} else {
				p2 = supportCube(bodyB, direction.clone().negate());
			}
				
			var p = p1.clone().sub(p2);
			p.sup_a = p1;
			p.sup_b = p2;
			return p;
		};

	};

	return (new GJKEPA());
});
