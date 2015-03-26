

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
