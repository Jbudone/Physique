define(function(){

	var Settings = {
		offsetYOfObjects: 1.001,
		numObjects: 8
	};

	var Scene = function(){

		this.initialize = function(scene){

			var offsetY = -0.9,
				scale = 1.0;

			scene.addMesh({
				type: MESH_BOX,
				body: BODY_CUBE,
				position: new THREE.Vector3(0, offsetY, 0)
			});

			for (var i=1; i<Settings.numObjects; ++i) {
				offsetY += Settings.offsetYOfObjects;

				var meshTypes = [MESH_BOX, MESH_SPHERE],
					bodyTypes = [BODY_CUBE, BODY_SPHERE],
					bodyMeshI = parseInt(Math.random() * meshTypes.length),
					bodyType = bodyTypes[bodyMeshI],
					meshType = meshTypes[bodyMeshI],
					dimensions = null;
				
				if (bodyType == BODY_CUBE) {
					dimensions = (new THREE.Vector3(1.0, 1.0, 1.0)).multiplyScalar(scale);
				} else if (bodyType == BODY_SPHERE) {
					dimensions = {radius: 0.50*scale };
				} else {
					dimensions = {radius: 1.0*scale };
				}
				scene.addMesh({
					type: meshType,
					body: bodyType,
					position: new THREE.Vector3(0, offsetY, 0),
					dimensions: dimensions
				});
			}

			scene.addMesh({
				type: MESH_PLANE,
				body: BODY_FLOOR,
				position: new THREE.Vector3(0, -4, 10)
			});

		};

		this.stop = function(){};
		this.pause = function(){};
		this.resume = function(){};
	};

	return (new Scene());
});
