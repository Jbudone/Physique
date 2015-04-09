define(function(){

	var Settings = {
		offsetYOfObjects: 1.0,
	};

	var Scene = function(){

		this.initialize = function(scene){

			scene.addMesh({
				type: MESH_BOX,
				body: BODY_CUBE,
				position: new THREE.Vector3(0, -1.05, 0),
				asleep: true
			});

			// board
			scene.addMesh({
				type: MESH_BOX,
				body: BODY_CUBE,
				position: new THREE.Vector3(1.0, -0.5, 0.0),
				dimensions: new THREE.Vector3(3.0, 0.1, 1.0),
				asleep: true
			});

			// pegs
			for (var x=-0.3; x<=0.3; x+=0.3) {
				for (var z=-0.3; z<=0.3; z+=0.3) {
					scene.addMesh({
						type: MESH_BOX,
						body: BODY_CUBE,
						position: new THREE.Vector3(x, 0.05, z),
						dimensions: new THREE.Vector3(0.2, 1.0, 0.2),
						asleep: true
					});
				}
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
