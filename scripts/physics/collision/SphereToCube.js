define(function(){

	var SphereToCube = function(){

		this.findContact = function(bodyA, bodyB){

			if (bodyA.bodyType === BODY_CUBE) {
				return this.sphereToCube(bodyB, bodyA);
			} else {
				return this.sphereToCube(bodyA, bodyB);
			}
		};

		this.sphereToCube = function(sphere, cube){

			// TODO: find closest point on sphere to 
		};

	};

	return (new SphereToCube());
});
