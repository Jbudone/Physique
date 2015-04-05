define(function(){

	var MOVE_FORWARD  = 1<<1,
		MOVE_BACKWARD = 1<<2,
		MOVE_LEFT     = 1<<3,
		MOVE_RIGHT    = 1<<4,
		MOVE_UP       = 1<<5,
		MOVE_DOWN     = 1<<6,
		MOVE_RUNNING  = 1<<7;
	var UI = {
		viewport: {
			mouse: {
				position: {x:0,y:0},
				buttons: 0,
				target: new THREE.Vector3(0,0,0),
				phi: 0.0,
				scaleY: 0.0005,
				scaleX: 0.001
			},
			move: new THREE.Vector3(0,0,0),
			isMoving: 0,
			scaleMove: 0.2
		},
		mouse: {
			isDown: false,
			position: {x:0, y:0}
		},
		lastUpdate: (new Date()).getTime(),
	};

	var camera = null,
		raycaster = null;


	var moveScale = 0.2,
		runScale  = 4.0;
	var step = function(){
		if (UI.viewport.isMoving) {

			// Which way are we moving?
			var move = UI.viewport.move,
				movement = UI.viewport.isMoving;
			move.x = 0; move.y = 0; move.z = 0;
			if ( movement & MOVE_FORWARD )  move.z += 1;
			if ( movement & MOVE_BACKWARD ) move.z -= 1;
			if ( movement & MOVE_LEFT )     move.x += 1;
			if ( movement & MOVE_RIGHT )    move.x -= 1;
			if ( movement & MOVE_UP )       move.y -= 1;
			if ( movement & MOVE_DOWN )     move.y += 1;

			// What direction are we looking?
			var qX = new THREE.Quaternion(),
				qY = new THREE.Quaternion(),
				qZ = new THREE.Quaternion(),
				m = new THREE.Matrix4(),
				vY = new THREE.Vector3(0,1,0),
				vX = new THREE.Vector3(1,0,0),
				vZ = new THREE.Vector3(0,0,1);

			qX.setFromAxisAngle( vY, -camera.phi );
			vX.applyQuaternion(qX);
			qY.setFromAxisAngle( vX, -camera.theta );
			vY.multiply(qY);
			qZ.setFromAxisAngle( vZ, camera.lambda );
			qY.multiply(qX);
			qY.multiply(qZ);
			camera.quaternion.copy(qY);

			// m.makeRotationFromQuaternion(qY);
			// m.getInverse(m);
			// move.applyMatrix4(m);
			move.applyQuaternion(qY);

			// Are we running?
			//move.x *= -1;
			if ( movement & MOVE_RUNNING ) move.multiplyScalar(-1.0*runScale*UI.viewport.scaleMove);
			else move.multiplyScalar(-1.0*moveScale*UI.viewport.scaleMove);

			// Apply the movement
			camera.position.add(move);

		}
	};

	var initialize = function(canvas, _camera, _raycaster){

		camera = _camera;
		raycaster = _raycaster;
		raycaster.active = false;
		raycaster.hasUpdated = false;
		raycaster.mouse = {x:0, y:0};


		canvas.addEventListener('mousedown', function MouseDownEvent(evt){

			var bounds  = canvas.getBoundingClientRect(),
				mouseY  = evt.clientY - bounds.top,
				mouseX  = evt.clientX - bounds.left;

			UI.viewport.mouse.position.x = mouseX;
			UI.viewport.mouse.position.y = mouseY;

			var MOUSE_LOOK = 4,
				MOUSE_MOVE = 1 | MOUSE_LOOK;
			UI.viewport.mouse.buttons |= (1<<evt.button);
			if ((UI.viewport.mouse.buttons & MOUSE_MOVE) === MOUSE_MOVE) {
				UI.viewport.isMoving |= MOVE_FORWARD;
			} else if ((UI.viewport.mouse.buttons & MOUSE_LOOK) === MOUSE_LOOK) {

			} else {
				// FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME
				raycaster.active = true;
				raycaster.hasUpdated = true;
				raycaster.mouse.x = ( evt.clientX / window.innerWidth ) * 2 - 1;
				raycaster.mouse.y = - ( evt.clientY / window.innerHeight ) * 2 + 1;
				// FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME
			}

			evt.preventDefault();
			return false;

		});
		canvas.oncontextmenu = function(){ return false; };

		canvas.addEventListener('mouseup', function MouseUpEvent(evt){

			var MOUSE_LOOK = 4,
				MOUSE_MOVE = 1 | MOUSE_LOOK;
			UI.viewport.mouse.buttons = UI.viewport.mouse.buttons & ~(1<<evt.button);
			if ((UI.viewport.mouse.buttons & MOUSE_MOVE) !== MOUSE_MOVE) {
				UI.viewport.isMoving &= ~MOVE_FORWARD;
			}
			
			if (!UI.viewport.mouse.buttons) {
				raycaster.active = false;
				
				if (raycaster.holdingOnto) {
					raycaster.holdingOnto.static = false;
					raycaster.holdingOnto.invMass = raycaster.holdingOnto.storedInvMass;
					raycaster.holdingOnto.invInertiaTensor = raycaster.holdingOnto.storedInvInertiaTensor;
					if (raycaster.holdingOnto.invMass == 0) debugger;
					raycaster.holdingOnto = null;
				}
			}

			evt.preventDefault();
			return false;

		});

		canvas.addEventListener('mousemove', function MouseMoveEvent(evt){

			var MOUSE_LOOK = 4;
			if (UI.viewport.mouse.buttons & MOUSE_LOOK) {

				var bounds  = canvas.getBoundingClientRect(),
					mouseY  = evt.clientY - bounds.top,
					mouseX  = evt.clientX - bounds.left,
					deltaY  = 1.7*2*(mouseY - UI.viewport.mouse.position.y),
					deltaX  = 1.7*(mouseX -   UI.viewport.mouse.position.x);



				var dPhi = camera.phi;
				camera.phi += deltaX*UI.viewport.mouse.scaleX;
				camera.phi = camera.phi % (Math.PI*2.0);
				dPhi = camera.phi - dPhi;

				var dTheta = camera.theta;
				camera.theta += deltaY*UI.viewport.mouse.scaleY;
				if (camera.theta > Math.PI) camera.theta = Math.PI;
				if (camera.theta < -Math.PI)camera.theta = -Math.PI;
				dTheta = camera.theta - dTheta;


				UI.viewport.mouse.position.x = mouseX;
				UI.viewport.mouse.position.y = mouseY;

				UI.viewport.isMoving = true;
			} else if (raycaster.active && raycaster.holdingOnto) {

				var bounds  = canvas.getBoundingClientRect(),
					mouseY  = evt.clientY - bounds.top,
					mouseX  = evt.clientX - bounds.left,
					deltaY  = (mouseY - UI.viewport.mouse.position.y) / window.innerHeight,
					deltaX  = (mouseX - UI.viewport.mouse.position.x) / window.innerWidth;
					// deltaY  = 1.7*2*(mouseY - UI.viewport.mouse.position.y),
					// deltaX  = 1.7*(mouseX -   UI.viewport.mouse.position.x);

				UI.viewport.mouse.position.x = mouseX;
				UI.viewport.mouse.position.y = mouseY;

				raycaster.hasMoved = {
					dX: deltaX,
					dY: deltaY
				};
			}

		});

		document.addEventListener('keydown', function KeyDownEvent(evt){
			
				   if (evt.keyCode === 87) {
				UI.viewport.isMoving |= MOVE_FORWARD;
			} else if (evt.keyCode === 83) {
				UI.viewport.isMoving |= MOVE_BACKWARD;
			} else if (evt.keyCode === 65) {
				UI.viewport.isMoving |= MOVE_LEFT;
			} else if (evt.keyCode === 68) {
				UI.viewport.isMoving |= MOVE_RIGHT;
			} else if (evt.keyCode === 16) {
				UI.viewport.isMoving |= MOVE_RUNNING;
			} else if (evt.keyCode === 81) {
				UI.viewport.isMoving |= MOVE_UP;
			} else if (evt.keyCode === 69) {
				UI.viewport.isMoving |= MOVE_DOWN;
			}
		});

		document.addEventListener('keyup', function KeyUpEvent(evt){
			
				   if (evt.keyCode === 87) {
				UI.viewport.isMoving &= ~MOVE_FORWARD;
			} else if (evt.keyCode === 83) {
				UI.viewport.isMoving &= ~MOVE_BACKWARD;
			} else if (evt.keyCode === 65) {
				UI.viewport.isMoving &= ~MOVE_LEFT;
			} else if (evt.keyCode === 68) {
				UI.viewport.isMoving &= ~MOVE_RIGHT;
			} else if (evt.keyCode === 16) {
				UI.viewport.isMoving &= ~MOVE_RUNNING;
			} else if (evt.keyCode === 81) {
				UI.viewport.isMoving &= ~MOVE_UP;
			} else if (evt.keyCode === 69) {
				UI.viewport.isMoving &= ~MOVE_DOWN;
			}
		});

	};

	return {
		initialize: initialize,
		step: step,
		UI: UI
	};
});
