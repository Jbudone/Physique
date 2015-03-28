define(function(){

	var Settings = { };

	var Scene = function(){

		this.initialize = function(scene){



			scene.addMesh({
				type: MESH_BOX,
				body: BODY_FLOOR,
				position: new THREE.Vector3(-45, 40, 2),
				dimensions: new THREE.Vector3(6, 0.2, 2),
				rotation: new THREE.Vector3(0, 0, Math.PI * 0.24)
			});


			scene.addMesh({
				type: MESH_BOX,
				body: BODY_FLOOR,
				position: new THREE.Vector3(-30, 18, 2),
				dimensions: new THREE.Vector3(60, 0.2, 2),
				rotation: new THREE.Vector3(0, 0, Math.PI * -0.2)
			});


			scene.addMesh({
				type: MESH_BOX,
				body: BODY_FLOOR,
				position: new THREE.Vector3(-4, 1, 2),
				dimensions: new THREE.Vector3(4, 0.2, 2),
				rotation: new THREE.Vector3(0, 0, Math.PI * 0.1)
			});



			// Ball
			scene.addMesh({
				type: MESH_SPHERE,
				body: BODY_SPHERE,
				position: new THREE.Vector3(-44, 48, 2)
			});


			// Floor
			scene.addMesh({
				type: MESH_PLANE,
				body: BODY_FLOOR,
				position: new THREE.Vector3(0, -4, 10)
			});

		};
	};

	return (new Scene());
});
