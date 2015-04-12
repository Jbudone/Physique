define(['input', 'scene', 'renderer', 'physics/physique'], function(Input, Scene, Renderer, Physique){


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

			twoObjects: true,
			moreObjects: 8,
			offsetYOfObjects: 1.5,
			debugPoints: false,
			maxTime: 600,
			stepTime: 16 // 1/60th second * 1000ms/s = 16.6ms
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

	physique = new Physique();

	raycaster = new THREE.Raycaster();

	var update     = null,
		startup    = null,
		lastUpdate = null,
		activeMesh = null,
		runWorld   = false;

	scene.onAddedMesh = function(_mesh){
		var mesh = renderer.addMesh(_mesh.settings);
		_mesh.body = mesh;
		_mesh.body.uid = _mesh.uid;

		mesh.bodyType = BODY_CUBE;

		physique.addBody(_mesh);

		$('#objects').append( $('<option/>').attr('value', _mesh.uid) );
	};

	scene.onRemovedMesh = function(mesh){
		renderer.removeMesh(mesh);
		physique.removeBody(mesh);
	};


	if (Settings.debugPoints) {
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
			if (!debugContacts.hasOwnProperty(hash)) return;
			var mesh = debugContacts[hash].mesh;
			mesh.position.copy(point);
		};

		physique.onRemoveContact = function(hash){
			if (!debugContacts.hasOwnProperty(hash)) return;
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
		MOVE_TURN = 1<<8,
		MOVE_LEFT = 1<<3,
		MOVE_RIGHT = 1<<4,
		MOVE_FORWARD = 1<<1,
		MOVE_BACKWARD = 1<<2;

	renderer.startup().then(function(renderObj){
		canvas = renderObj.domElement;
		document.body.appendChild( canvas );

		var loadedScene = function(exampleScene){


			runWorld = false;
			var startScene = function(){

				runWorld = true;
				exampleScene.initialize(scene);

				scene.stop = exampleScene.stop.bind(exampleScene);
				scene.pause = exampleScene.pause.bind(exampleScene);
				scene.resume = exampleScene.resume.bind(exampleScene);
			};

			var resetScene = function(){

				scene.stop();
				scene.reset();
				physique.reset();

				for (var contactID in debugContacts) {
					var mesh = debugContacts[contactID].mesh;
					renderer.remove(mesh);
					delete debugContacts[contactID];
				}
			};

			scene.stop();
			resetScene();
			startScene();

		};

		startup();

		var loadScene = function(file){
			require(['../examples/'+file], loadedScene);
		};

		var addScene = function(name, title){
			var _exampleEl = $('<a/>').attr('href', '#').addClass('button-link').text(title).click(function(){
				loadScene(name);
				return false;
			});
			$('#examples').append( _exampleEl );
		};

		addScene('rolling-ball', "Ball Roll");
		addScene('box-stacking', "Box Stacking");
		addScene('funnel', "Funnel");
		addScene('rand-stack', "Random Stack");
		addScene('catapult', "Catapult");
		addScene('wall', "The Wall");


		// Load default scene
		loadScene('box-stacking');


	}, function(error){
		console.error(error);
		console.error(error.stack);
	});



	var shootCube = function(){

		var mesh = scene.addMesh({
			type: MESH_BOX,
			body: BODY_CUBE,
			position: new THREE.Vector3(renderer.camera.position.x, renderer.camera.position.y, renderer.camera.position.z),
			mass: 30.0
		});

		var shootMultiplier = 20.0;
		var moveDir2 = new THREE.Vector3(0,0,-shootMultiplier);

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

		moveDir2.applyQuaternion(qY);

		mesh.body.body.velocity.add(moveDir2);

	};

	var shootSphere = function(){

		var mesh = scene.addMesh({
			type: MESH_SPHERE,
			body: BODY_SPHERE,
			position: new THREE.Vector3(renderer.camera.position.x, renderer.camera.position.y, renderer.camera.position.z),
			mass: 30.0,
			dimensions: { radius: 2.0 }
		});

		var shootMultiplier = 30.0;
		var moveDir2 = new THREE.Vector3(0,0,-shootMultiplier);

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

		moveDir2.applyQuaternion(qY);

		mesh.body.body.velocity.add(moveDir2);

	};


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
		} else if (evt.keyCode === 32) {
			shootCube();
		} else if (evt.keyCode === 88) {
			shootSphere();
		} else if (evt.keyCode === 17 || evt.ctrlKey) {
			isMoving |= MOVE_TURN;
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
		} else if (evt.keyCode === 17 || evt.ctrlKey) {
			isMoving &= ~MOVE_TURN;
		}
	});

	document.addEventListener('mouseup', function MouseUpEvent(evt){
		activeMesh = null;
	});


	$('#startPhysics').click(function(){
		physique.world.scaleTime = $('#scaleTime').val();
		physique.world.runOnce = null;
		scene.resume();
		return false;
	});

	$('#stopPhysics').click(function(){
		physique.world.scaleTime = 0.0;
		physique.world.runOnce = null;
		scene.pause();
		return false;
	});

	$('#stepPhysics').click(function(){
		physique.world.scaleTime = $('#scaleTime').val();
		physique.world.runOnce = true;
		return false;
	});

	var addInteractiveSetting = function(el, propName){
		el.val( physique.world[propName] ).on('change input', function(){
			var val = $(this).val();
			if (isNaN(val) || val === "") {
				$(this).val( physique.world[propName] );
			} else {
				physique.world[propName] = $(this).val();
			}
		});
	};
	addInteractiveSetting( $('#scaleTime'), 'scaleTime' );
	addInteractiveSetting( $('#baumgarte'), 'baumgarte' );
	addInteractiveSetting( $('#damping'), 'damping' );
	addInteractiveSetting( $('#warmth'), 'warmth' );
	addInteractiveSetting( $('#restitution'), 'restitution' );
	addInteractiveSetting( $('#friction'), 'friction' );
	addInteractiveSetting( $('#slop'), 'slop' );
	addInteractiveSetting( $('#minvel'), 'minVel' );


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

		if (!runWorld) {
			lastUpdate = now;
			return;
		}

		var _deltaTime = deltaTime;
		deltaTime = Math.min(Settings.maxTime, deltaTime);

			isMoving = Input.UI.viewport.isMoving;
			if (isMoving && activeMesh) {
				moveMesh(delta);
			}


			// console.log("Stepping at: "+deltaTime);
		while (deltaTime > 0) {
			var delta = Math.min(Settings.stepTime, deltaTime);

			physique.step(delta);
			deltaTime -= delta;
		}

		delta = _deltaTime;
			renderer.render();
			Input.step(delta);


			if (raycaster.active && raycaster.hasUpdated) {
				raycaster.setFromCamera( raycaster.mouse, renderer.camera );
				var intersects = renderer.raycast(raycaster);
				if (intersects.length > 0) {
					for (var i=0; i<intersects.length; ++i) {
						var intersect = intersects[i];
						if (intersect.object.body.static === false) {
							if (activeMesh) {
								activeMesh.body.static = false;
								activeMesh.body.invMass = activeMesh.storedInvMass;
								activeMesh.body.invInertiaTensor = activeMesh.storedInvInertiaTensor;
								activeMesh.body.updateState();
							}

							activeMesh = intersect.object;
							activeMesh.distanceFromCamera = intersect.distance;
							raycaster.holdingOnto = activeMesh;
							activeMesh.body.velocity.multiplyScalar(0.0);
							activeMesh.body.angularVelocity.multiplyScalar(0.0);
							if (!activeMesh.body.asleep) {
								activeMesh.body.storedInvMass = activeMesh.body.invMass;
								activeMesh.body.storedInvInertiaTensor = activeMesh.body.invInertiaTensor;
								activeMesh.body.invMass = 0.0;
								activeMesh.body.invInertiaTensor = 0.0;
							}
							raycaster.holdingOnto.body.static = true;
							activeMesh.body.updateState();
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

				if (isMoving & MOVE_TURN) {
					
				} else {
					raycaster.holdingOnto.position.add( vec.multiplyScalar( 0.006 * (window.innerWidth/window.innerHeight) ) );
				}

				raycaster.hasMoved = null;
				raycaster.holdingOnto.userMoved = true;
			}

		lastUpdate = now;
	};

	startup = function(){
		lastUpdate = (new Date()).getTime();
		update();

		Input.initialize( canvas, renderer.camera, raycaster );
	};

});
