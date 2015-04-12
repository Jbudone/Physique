define(function(){

	var Settings = {
		offsetYOfObjects: 1.001,
		numObjects: 8
	};

	var Scene = function(){

		this.initialize = function(scene){

			var offsetY = -1.08,
				scale = 0.8,
				extent = 2,
				height = 8,
				slit = 0.0,//0.0001,
				walls = 1,
				spacing = 0.01;


			for (var wall=0; wall<walls; ++wall) {

				var z = wall * (scale + spacing + slit);
				for (var x=-extent; x<=extent; ++x) {
					for (var y=0; y<=height; ++y) {

						scene.addMesh({
							type: MESH_BOX,
							body: BODY_CUBE,
							position: new THREE.Vector3(x*scale + Math.abs(x)*slit, offsetY + y*scale + Math.abs(y)*slit, z),
							dimensions: new THREE.Vector3(scale, scale, scale),
							asleep: true
						});
					}
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

