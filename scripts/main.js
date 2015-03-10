define(['input', 'scene', 'renderer', 'physique'], function(Input, Scene, Renderer, Physique){


	var Settings = {

			rendering: {
				camera: {
					fov: 75,
					aspect: (window.innerWidth / window.innerHeight),
					near: 0.1,
					far: 1000
				}, renderer: {
					dimensions: {
						width: window.innerWidth,
						height: window.innerHeight
					}
				}
			},

			isGoblin: false,
			twoObjects: true,
			moreObjects: 2
	};

	THREE.Euler.prototype.sub = function(vec){
		this.x -= vec.x;
		this.y -= vec.y;
		this.z -= vec.z;
	};

	THREE.Euler.prototype.add = function(vec){
		this.x += vec.x;
		this.y += vec.y;
		this.z += vec.z;
	};

	scene = new Scene();
	renderer = new Renderer({
		settings: Settings.rendering
	});

	if (Settings.isGoblin) {
		world = new Goblin.World( new Goblin.BasicBroadphase(), new Goblin.NarrowPhase(), new Goblin.IterativeSolver() );
	} else {
		physique = new Physique();
	}

	raycaster = new THREE.Raycaster();

	var update     = null,
		startup    = null,
		lastUpdate = null,
		activeMesh = null;

	scene.onAddedMesh = function(_mesh){
		var mesh = renderer.addMesh(_mesh.settings);
		_mesh.body = mesh;
		_mesh.body.uid = _mesh.uid;

		mesh.bodyType = BODY_CUBE;

		if (Settings.isGoblin) {
			if (_mesh.settings.body === BODY_FLOOR) {
				mesh.goblin = new Goblin.RigidBody( new Goblin.BoxShape( 20, 2.5, 20 ), 0 );
			} else {
				mesh.goblin = new Goblin.RigidBody( new Goblin.BoxShape( 0.5, 0.5, 0.5 ), 1 );
			}
			world.addRigidBody( mesh.goblin );
			mesh.goblin.position.x = mesh.position.x;
			mesh.goblin.position.y = mesh.position.y;
			mesh.goblin.position.z = mesh.position.z;
			mesh.goblin.rotation.x = mesh.rotation._x;
			mesh.goblin.rotation.y = mesh.rotation._y;
			mesh.goblin.rotation.z = mesh.rotation._z;
		} else {
			physique.addBody(_mesh);
		}

		$('#objects').append( $('<option/>').attr('value', _mesh.uid) );
	};


	if (!Settings.isGoblin) {

	var debugArrows = {};
	physique.onDebugHelperArrow = function(hitHash, dir, origin, depth){
		if (debugArrows.hasOwnProperty(hitHash)) {
			var mesh = debugArrows[hitHash].mesh;
			renderer.remove(mesh);
			mesh = renderer.addArrow(dir, origin, depth);
			debugArrows[hitHash] = {
				mesh: mesh
			};
		} else {
			var mesh = renderer.addArrow(dir, origin, depth);
			debugArrows[hitHash] = {
				mesh: mesh
			};
		}
	};

	var debugContacts = {};
	physique.onNewContact = function(hash, point){
		var Bpoint = (hash[hash.length-1]=='B');
		var mesh = renderer.addContact(point, Bpoint);
		debugContacts[hash] = {
			mesh: mesh
		};
	};

	physique.onUpdateContact = function(hash, point){
		var mesh = debugContacts[hash].mesh;
		mesh.position.copy(point);
	};

	physique.onRemoveContact = function(hash){
		var mesh = debugContacts[hash].mesh;
		renderer.remove(mesh);
		delete debugContacts[hash];
	};

	}

	$('#objectsInput').on('input', function(){
		var meshID = $(this).val();
		activeMesh = scene.meshes[meshID].body;
			$(this).blur();
			// $(this).focusout();
			// $(this).trigger('blur');
	});

	$('#rotX').on('change input', function(){
		if (activeMesh) {
			var rotation = parseInt($(this).val()),
				delta = rotation - parseInt($(this).data('oldVal'));

			delta /= 100;
			$(this).data('oldVal', rotation);
			activeMesh.rotateX(delta);
		}
	}).data('oldVal', 0);

	$('#rotY').on('change input', function(){
		if (activeMesh) {
			var rotation = parseInt($(this).val()),
				delta = rotation - parseInt($(this).data('oldVal'));

			delta /= 100;
			$(this).data('oldVal', rotation);
			activeMesh.rotateY(delta);
		}
	}).data('oldVal', 0);

	$('#rotZ').on('change input', function(){
		if (activeMesh) {
			var rotation = parseInt($(this).val()),
				delta = rotation - parseInt($(this).data('oldVal'));

			delta /= 100;
			$(this).data('oldVal', rotation);
			activeMesh.rotateZ(delta);
		}
	}).data('oldVal', 0);

	var isMoving = false,
		moveScale = 0.2,
		MOVE_UP = 1<<5,
		MOVE_DOWN = 1<<6,
		MOVE_LEFT = 1<<3,
		MOVE_RIGHT = 1<<4,
		MOVE_FORWARD = 1<<1,
		MOVE_BACKWARD = 1<<2;

	renderer.startup().then(function(renderObj){
		canvas = renderObj.domElement;
		document.body.appendChild( canvas );

		scene.addMesh({
			type: MESH_BOX,
			body: BODY_CUBE,
			position: new THREE.Vector3(0, 0, 0)
		});

		if (Settings.twoObjects) {
			scene.addMesh({
				type: MESH_BOX,
				body: BODY_CUBE,
				position: new THREE.Vector3(0, 1.5, 0)
			});

			if (Settings.moreObjects) {
				var offsetY = 1.5;
				for (var i=2; i<Settings.moreObjects; ++i) {
					offsetY += 1.5;
					scene.addMesh({
						type: MESH_BOX,
						body: BODY_CUBE,
						position: new THREE.Vector3(0, offsetY, 0)
					});
				}
			}
		}

		scene.addMesh({
			type: MESH_PLANE,
			body: BODY_FLOOR,
			position: new THREE.Vector3(0, -4, 10)
		});



		document.addEventListener('keydown', function KeyDownEvent(evt){
			
				   if (evt.keyCode === 73) {
				isMoving |= MOVE_FORWARD;
			} else if (evt.keyCode === 75) {
				isMoving |= MOVE_BACKWARD;
			} else if (evt.keyCode === 74) {
				isMoving |= MOVE_LEFT;
			} else if (evt.keyCode === 76) {
				isMoving |= MOVE_RIGHT;
			} else if (evt.keyCode === 85) {
				isMoving |= MOVE_UP;
			} else if (evt.keyCode === 79) {
				isMoving |= MOVE_DOWN;
			}
		});

		document.addEventListener('keyup', function KeyUpEvent(evt){
			
				   if (evt.keyCode === 73) {
				isMoving &= ~MOVE_FORWARD;
			} else if (evt.keyCode === 75) {
				isMoving &= ~MOVE_BACKWARD;
			} else if (evt.keyCode === 74) {
				isMoving &= ~MOVE_LEFT;
			} else if (evt.keyCode === 76) {
				isMoving &= ~MOVE_RIGHT;
			} else if (evt.keyCode === 85) {
				isMoving &= ~MOVE_UP;
			} else if (evt.keyCode === 79) {
				isMoving &= ~MOVE_DOWN;
			}
		});

		document.addEventListener('mouseup', function MouseUpEvent(evt){
			activeMesh = null;
		});

		$('#startPhysics').click(function(){
			physique.world.scaleTime = 0.001;
			return false;
		});

		$('#stopPhysics').click(function(){
			physique.world.scaleTime = 0.0;
			return false;
		});



		startup();
	}, function(error){
		console.error(error);
		console.error(error.stack);
	});

	var moveMesh = function(delta){

		var moveDir = new THREE.Vector3();
		if (isMoving & MOVE_FORWARD)  moveDir.z += 1;
		if (isMoving & MOVE_BACKWARD) moveDir.z -= 1;
		if (isMoving & MOVE_LEFT)     moveDir.x += 1;
		if (isMoving & MOVE_RIGHT)    moveDir.x -= 1;
		if (isMoving & MOVE_UP)       moveDir.y -= 1;
		if (isMoving & MOVE_DOWN)     moveDir.y += 1;


			var qX = new THREE.Quaternion(),
				qY = new THREE.Quaternion(),
				qZ = new THREE.Quaternion(),
				m = new THREE.Matrix4(),
				vY = new THREE.Vector3(0,1,0),
				vX = new THREE.Vector3(1,0,0),
				vZ = new THREE.Vector3(0,0,1);

			qX.setFromAxisAngle( vY, -renderer.camera.phi );
			vX.applyQuaternion(qX);
			qY.setFromAxisAngle( vX, -renderer.camera.theta );
			vY.multiply(qY);
			qZ.setFromAxisAngle( vZ, renderer.camera.lambda );
			qY.multiply(qX);
			qY.multiply(qZ);

			moveDir.applyQuaternion(qY);

		moveDir.multiplyScalar(-moveScale);
		activeMesh.position.add(moveDir);
	};

	update = function(){
		var now = (new Date()).getTime(),
			deltaTime = now - lastUpdate;

		requestAnimationFrame(update);
		deltaTime = Math.min(200, deltaTime);
		while (deltaTime > 0) {
			var delta = Math.min(50, deltaTime);

			isMoving = Input.UI.viewport.isMoving;
			if (isMoving && activeMesh) {
				moveMesh(delta);
			}

			if (Settings.isGoblin) {
				world.step(1/60);
				// Goblin sync
				for (var meshID in scene.meshes){
					var mesh = scene.meshes[meshID];
					mesh.body.position.copy(mesh.body.goblin.position);
					mesh.body.rotation.copy(new THREE.Euler(mesh.body.goblin.rotation.x, mesh.body.goblin.rotation.y, mesh.body.goblin.rotation.z));
				}
			} else {
				physique.step(delta);
			}
			renderer.render();
			Input.step(delta);
			deltaTime -= delta;


			if (raycaster.active && raycaster.hasUpdated) {
				raycaster.setFromCamera( raycaster.mouse, renderer.camera );
				var intersects = renderer.raycast(raycaster);
				if (intersects.length > 0) {
					for (var i=0; i<intersects.length; ++i) {
						var intersect = intersects[i];
						if (intersect.object.active === true) {
							activeMesh = intersect.object;
							activeMesh.distanceFromCamera = intersect.distance;
							raycaster.holdingOnto = activeMesh;
							activeMesh.velocity.multiplyScalar(0.0);
							activeMesh.angularVelocity.multiplyScalar(0.0);
							raycaster.holdingOnto.active = false;
							break;
						}
					}
				}
				raycaster.hasUpdated = false;
			} else if (raycaster.holdingOnto && raycaster.hasMoved) {
				var farY = 2.0 * renderer.camera.far * Math.tan(renderer.camera.fov * 0.5 * Math.PI / 180.0),
					farX = farY * renderer.camera.aspect,
					distance = raycaster.holdingOnto.distanceFromCamera,
					x = raycaster.hasMoved.dX,
					y = -raycaster.hasMoved.dY;
				var dX = distance / renderer.camera.far * farX * x * 120,
					dY = distance / renderer.camera.far * farY * y * 120;
				var vec = new THREE.Vector3(dX, dY, 0.0);
				// var vec = new THREE.Vector3(raycaster.hasMoved.dX, -raycaster.hasMoved.dY, 0.0);
				vec.applyQuaternion(renderer.camera.quaternion);
				raycaster.holdingOnto.position.add( vec.multiplyScalar( 0.006 * (window.innerWidth/window.innerHeight) ) );
				raycaster.hasMoved = null;
				raycaster.holdingOnto.userMoved = true;
			}
		}

		lastUpdate = now;
	};

	startup = function(){
		lastUpdate = (new Date()).getTime();
		update();

		Input.initialize( canvas, renderer.camera, raycaster );
	};

});
