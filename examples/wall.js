define(function(){

	var Settings = {
		offsetYOfObjects: 1.001,
		numObjects: 8
	};

	var Scene = function(){

		this.initialize = function(scene){

			var offsetY = -1.4,
				scale = 0.2,
				extent = 6,
				slit = 0.0001;


			for (var x=-extent; x<=extent; ++x) {
				for (var y=0; y<=2*extent; ++y) {

					scene.addMesh({
						type: MESH_BOX,
						body: BODY_CUBE,
						position: new THREE.Vector3(x*scale + Math.abs(x)*slit, offsetY + y*scale + Math.abs(y)*slit, 0),
						dimensions: new THREE.Vector3(scale, scale, scale),
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
