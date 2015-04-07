define(function(){

	var Settings = {

		funnelPieces: 10,
		funnelTopLength: 8.0,
		steep: 0.15,
		width: 4.0,

		funnelPipe: true,
		funnelPipeLength: 2.0,
		funnelPiecesPipe: 10.0,
		funnelPipeOffset: -2.5,
		pipeWidth: 2.0,

		funnelOffset: 6.0,

		objOffset: 20.0,
	};

	var nextRun = null,
		running = false;

	var Scene = function(){

		this.initialize = function(scene){

			// Build funnel
			var xRot = Math.PI * Settings.steep,
				yOffset = Settings.funnelOffset + (Settings.funnelPipe ? Settings.funnelPipeLength : 0.0),
				yDelta = 2.0 / Settings.funnelPieces;
			for (var j=0; j<Settings.funnelPieces; ++j) {

				var yRot = Math.PI * yDelta * j,
					xPos = Math.sin(yRot) * Settings.width,
					zPos = Math.cos(yRot) * Settings.width,
					rot = new THREE.Vector3(0.0, yRot, 0.0);
				var m = scene.addMesh({
					type: MESH_BOX,
					body: BODY_FLOOR,
					position: new THREE.Vector3(xPos, yOffset, zPos),
					dimensions: new THREE.Vector3(4.0, Settings.funnelTopLength, 0.2),
					rotation: rot
				});
				m.body.rotateX(xRot);
				m.body.updateInBroadphase();
			}
			

			// Build the funnel pipe
			if (Settings.funnelPipe) {
				for (var j=0; j<Settings.funnelPiecesPipe; ++j) {

					var yRot = Math.PI * 0.2 * j,
						xPos = Math.sin(yRot) * Settings.pipeWidth,
						zPos = Math.cos(yRot) * Settings.pipeWidth;
					scene.addMesh({
						type: MESH_BOX,
						body: BODY_FLOOR,
						position: new THREE.Vector3(xPos, Settings.funnelOffset + Settings.funnelPipeOffset, zPos),
						dimensions: new THREE.Vector3(4.0, Settings.funnelPipeLength, 0.2),
						rotation: new THREE.Vector3(0.0, yRot, 0.0)
					});

				}
			}

			// Floor
			scene.addMesh({
				type: MESH_PLANE,
				body: BODY_FLOOR,
				position: new THREE.Vector3(0, -4, 10)
			});

			running = true;
			this.startDropping();
		};

		this.startDropping = function(){

			if (!running) return;
			this.spawnObject();
			nextRun = setTimeout(this.startDropping.bind(this), 100);
		};

		this.stop = function(){
			running = false;
			if (nextRun) clearTimeout(nextRun);
		};

		this.pause = function(){
			running = false;
			if (nextRun) clearTimeout(nextRun);
		};

		this.resume = function(){
			running = true;
			this.startDropping();
		};


		this.spawnObject = function(){

			var yPos = Settings.funnelOffset + (Settings.funnelPipe ? Settings.funnelPipeLength : 0.0) + Settings.objOffset,
				angle = Math.random() * (Math.PI * 2),
				xPos = Math.cos(angle) * (Settings.width * 0.5 + Settings.pipeWidth),
				zPos = Math.sin(angle) * (Settings.width * 0.5 + Settings.pipeWidth),
				// xPos = (2 * Math.random() - 1) * (1 * Settings.width) + Settings.pipeWidth,
				// zPos = (2 * Math.random() - 1) * (1 * Settings.width) + Settings.pipeWidth,
				bodyType = (Math.random() > 0.5 ? BODY_CUBE : BODY_SPHERE),
				meshType = (bodyType == BODY_CUBE ? MESH_BOX : MESH_SPHERE),
				scale = 1.0,
				dimensions = (bodyType == BODY_CUBE ? (new THREE.Vector3(1.0, 1.0, 1.0)).multiplyScalar(scale) : {radius: 0.75*scale });

			var m = scene.addMesh({
				type: meshType,
				body: bodyType,
				position: new THREE.Vector3(xPos, yPos, zPos),
				dimensions: dimensions,
				rotation: new THREE.Vector3(0.0, 0.0, 0.0)
			});

			m.body.velocity.y -= 0.0;

			setTimeout(function(){
				scene.removeMesh(m);
			}, 4000);
		};

	};


	return (new Scene());
});
