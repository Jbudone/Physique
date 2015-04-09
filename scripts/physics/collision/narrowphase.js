define(['physics/collision/GJKEPA', 'physics/collision/SphereToCube', 'physics/collision/SphereToSphere'], function(GJKEPA, SphereToCube, SphereToSphere){

	var Narrowphase = function(){

		this.findContact = function(bodyA, bodyB){

			// if (bodyA.bodyType === BODY_SPHERE && bodyB.bodyType === BODY_SPHERE) {
			// 	return SphereToSphere.findContact(bodyA, bodyB);
			// } else {
				return GJKEPA.checkGJK(bodyA, bodyB);
			// }
		};
	};

	return (new Narrowphase());
});
