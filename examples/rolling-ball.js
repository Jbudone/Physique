define(function(){

	var Settings = {
		offsetYOfObjects: 1.5,
		twoObjects: true,
		moreObjects: 8
	};

	var Scene = function(){

		this.initialize = function(scene){

			scene.addMesh({
				type: MESH_SPHERE,
				body: BODY_SPHERE,
				position: new THREE.Vector3(0, 5, 2)
			});

			scene.addMesh({
				type: MESH_BOX,
				body: BODY_FLOOR,
				position: new THREE.Vector3(0, 0, 2),
				dimensions: new THREE.Vector3(4, 0.2, 2),
				rotation: new THREE.Vector3(0, 0, Math.PI * 0.3)
			});

			if (Settings.twoObjects) {
				scene.addMesh({
					type: MESH_BOX,
					body: BODY_CUBE,
					position: new THREE.Vector3(0, 1.5, 0)
				});

				if (Settings.moreObjects) {
					var offsetY = Settings.offsetYOfObjects;
					for (var i=2; i<Settings.moreObjects; ++i) {
						offsetY += Settings.offsetYOfObjects;
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

		};
	};

	return (new Scene());
});
