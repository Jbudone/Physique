define(function(){

	var Broadphase = function(){

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

		this.removeBodyInBroadphase = function(body){

			// Find my position
			// FIXME: for some reason this sometimes doesn't remove the body..
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

		};

		this.reset = function(){
			this.axisLineX = [];
			this.axisLineZ = [];
		};

	};

	return (new Broadphase());
});
