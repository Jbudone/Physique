define(function(){

	var Settings = {
		offsetYOfObjects: 1.0,//1.5,
		numObjects: 5
	};

	var Scene = function(){

		this.initialize = function(scene){

			var offsetY = -1.9;
			for (var i=0; i<Settings.numObjects; ++i) {
				offsetY += Settings.offsetYOfObjects;
				scene.addMesh({
					type: MESH_BOX,
					body: BODY_CUBE,
					position: new THREE.Vector3(0, offsetY, 0)
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
