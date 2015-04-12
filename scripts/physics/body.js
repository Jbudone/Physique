define(function(){

	var Settings = {

		colorCollisions: false,
		colorSleep: true
	};

	var Body = function(mesh, settings){

		var mass = settings.mass || 1.0;

		if (mesh.hasOwnProperty('radius')) this.radius = mesh.radius;
		this.mesh = mesh;
		this.position = mesh.position;
		this.rotation = mesh.rotation;
		this.quaternion = mesh.quaternion;
		this.geometry = mesh.geometry;
		this.AABB = mesh.AABB;
		this.material = mesh.material;
		this.uid = mesh.uid;

		mesh.body = this;

		var bodyType = settings.body;
		if (bodyType === BODY_CUBE) {

			var extent = 2; // FIXME: assuming a cube (same width/height/depth)
			// FIXME: giving cube more inertia by doubling its extent... this seems to help in box stacking
			var _inertia = 1/12 * mass * (2*extent*extent);
			this.invMass = 1 / (mass);
			this.invInertiaTensor = 1 / (_inertia);
			this.static = false;

		} else if (bodyType === BODY_SPHERE) {

			var _inertia = 2/5 * mass * Math.pow(this.radius,2);
			this.invMass = 1 / (mass);
			this.invInertiaTensor = 1 / (_inertia);
			this.static = false;

		} else if (bodyType === BODY_TETRAHEDRON) {

			var _inertia = 1/20 * mass * 2*Math.pow(this.radius,2);
			this.invMass = 1 / (mass);
			this.invInertiaTensor = 1 / (_inertia);
			this.static = false;

		} else if (bodyType === BODY_OCTAHEDRON) {

			var _inertia = 1/5 * mass * Math.pow(this.radius,2);
			this.invMass = 1 / (mass);
			this.invInertiaTensor = 1 / (_inertia);
			this.static = false;

		} else if (bodyType === BODY_FLOOR) {

			this.invMass = 0.0;
			this.invInertiaTensor = 0.0;
			this.static = true;

		} else {
			throw new Error("Adding undefined body: "+ bodyType);
		}

		this.bodyType = bodyType;
		this.velocity = new THREE.Vector3();
		this.angularVelocity = new THREE.Vector3();
		this.impulse = [0,0,0,0,0,0];

		this.asleep = false;
		if (settings.asleep) {
			this.storedInvMass = this.invMass;
			this.storedInvInertiaTensor = this.invInertiaTensor;
			this.invMass = 0;
			this.invInertiaTensor = 0;
			this.asleep = true;
		}
		this.manifolds = {};
		this.island = null;
		this.wantToSleep = 0;


		this.updateContacts = function(){
			for (var manifoldID in this.manifolds) {
				var manifold = this.manifolds[manifoldID];

				for (var contactID in manifold.contacts) {
					var contact = manifold.contacts[contactID];

					contact.update();
				}
			}
		};


		var state = 0,
			STATE_COLLIDING = 1<<1,
			STATE_ASLEEP = 1<<2;



		this.storedColor = mesh.material.color.clone();
		this.updateState = function(){

			if (this.static) {
				if (state) {
					this.material.color.copy( this.storedColor );
					state = 0;
				}

				return;
			}

			var oldState = state;

			state = 0;
			if (this.asleep && Settings.colorSleep) state |= STATE_ASLEEP;
			if (!_.isEmpty(this.manifolds) && Settings.colorCollisions) state |= STATE_COLLIDING;

			if (oldState != state) {
				// Change colours

				if (state == 0) {
					this.material.color.copy( this.storedColor );
				} else {

					if (state & STATE_ASLEEP) {
						this.material.color.r = 0.2;
						this.material.color.g = 0.2;
						this.material.color.b = 0.2;
					} else if (state & STATE_COLLIDING) {
						this.material.color.r = this.uid % 2 == 0 ? 0.2 : 1.0;
						this.material.color.g = this.uid % 2 == 0 ? 1.0 : 0.2;
						this.material.color.b = 0.2;
					}

				}
			}
		};
	};

	return Body;
});
