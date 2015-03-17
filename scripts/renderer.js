define(function(){

	Keys.add('MESH_BOX');
	Keys.add('MESH_PLANE');

	var Renderer = function(components){


		var settings = components.settings,
			scene    = null,
			camera   = null,
			renderer = null;

		var _me = this;
		var texturePack = {

			textureList: {
				stone: { file: 'stone.jpg' },
				grass: { file: 'grass.jpg', options: { repeat: true } }
			},

			textureMap: {
			}
		};
		texturePack.textureMap[MESH_BOX] = 'stone';
		texturePack.textureMap[MESH_PLANE] = 'grass';

		this.camera = camera;
		this.startup = function(){

			return new Promise(function(succeeded, failed){

				try {

					scene = new THREE.Scene();
					_me.scene = scene;
					camera = new THREE.PerspectiveCamera( settings.camera.fov, settings.camera.aspect, settings.camera.near, settings.camera.far );

					renderer = new THREE.WebGLRenderer();

					renderer.setSize( settings.renderer.dimensions.width, settings.renderer.dimensions.height );

					camera.position.z = 5;

					camera.phi      = 0.0;
					camera.theta    = 0.0;
					camera.lambda   = 0.0;

					camera.rotation.y = camera.phi;
					camera.rotation.z = camera.lambda;
					camera.rotation.x = camera.theta;

					this.camera = camera;


					// Load textures
					var loader = new THREE.TextureLoader();
					var texturesToLoad = [];
					for (var textureID in texturePack.textureList) {
						texturesToLoad.push({ id: textureID, url: texturePack.textureList[textureID].file, options: texturePack.textureList[textureID].options });
					}
					var loadNext = function(){

						if (texturesToLoad.length == 0) {
							return;
						}

						var textureToLoad = texturesToLoad.shift();
						loader.load('textures/'+textureToLoad.url,
								function(texture){
									texturePack.textureList[textureToLoad.id].texture = texture;

									if (_.isObject(textureToLoad.options)) {
										texture.wrapS = THREE.RepeatWrapping;
										texture.wrapT = THREE.RepeatWrapping;
										texture.anisotropy = 8;
										texture.repeat.set( 256, 256 );
										texture.update();
									}

									if (texturesToLoad.length === 0) {
										succeeded(renderer);
									} else {
										loadNext();
									}
								},
								function(xhr){
									console.log( (xhr.loaded / xhr.total * 100) + '% loaded' );
								},
								function(xhr){
									console.log( 'An error happened' );
									failed(xhr);
								});
					};

					loadNext();

					this.loadSkybox().then(function(){
						loadNext();
					}, function(err){
						failed(err);
					});

				} catch(e) {
					failed(e);
				}
			}.bind(this));
		};

		this.loadSkybox = function(){

			return new Promise(function(succeeded, failed){

				var urlPrefix = "textures/ThickCloudsWater/ThickCloudsWater";
				var urls = [ urlPrefix + "Left2048.png", urlPrefix + "Right2048.png",
					urlPrefix + "Up2048.png", urlPrefix + "Down2048.png",
					urlPrefix + "Front2048.png", urlPrefix + "Back2048.png" ];
				
				var textureCube = THREE.ImageUtils.loadTextureCube( urls, null,
						function(texture){

							console.log("loaded textures..");

							var shader = THREE.ShaderLib["cube"];
							var uniforms = THREE.UniformsUtils.clone( shader.uniforms );
							uniforms['tCube'].value = texture;   // textureCube has been init before
							var material = new THREE.ShaderMaterial({
								fragmentShader    : shader.fragmentShader,
								vertexShader  : shader.vertexShader,
								uniforms  : uniforms,

								depthWrite: false,
								side: THREE.BackSide
							});

							// build the skybox Mesh 
							var skyboxMesh    = new THREE.Mesh( new THREE.BoxGeometry( 1000, 1000, 1000, 1, 1, 1, null, true ), material );
							// add it to the scene
							scene.add( skyboxMesh );

							succeeded();
						}, function(err){
							failed(err);
						});

			});
		};

		this.addContact = function(point, Bpoint){

			var geometry = new THREE.SphereGeometry( 0.05, 32, 32 ),
				material = new THREE.MeshBasicMaterial( {color: (Bpoint ? 0x00ffff : 0xffff00)} ),
				sphere = new THREE.Mesh( geometry, material );
			sphere.position.copy(point);
			scene.add( sphere );

			return sphere;
		};

		this.addArrow = function(dir, origin, depth){
			
			var mesh = new THREE.ArrowHelper(dir, origin, depth, 0x00ff00);
			scene.add(mesh);
			return mesh;
		};

		this.remove = function(mesh){
			scene.remove(mesh);
		};

		this.raycast = function(raycaster){
			var intersects = raycaster.intersectObjects( scene.children );
			return intersects;
		};

		this.addMesh = function(_meshProps){
			if (!_.isObject(_meshProps)) _meshProps = {};

			var meshProps = _.defaults(_meshProps, {
								type: MESH_BOX,
								material: {
									color: 0x00ff00
								}
							}),
				geometry = null,
				material = null,
				mesh = null;

			if (meshProps.type === MESH_BOX) {

				geometry = new THREE.BoxGeometry( 1, 1, 1, 1, 1, 1 );
				// material = new THREE.MeshBasicMaterial( { color: 0x00ff00, transparent: true, opacity: 0.5 } );
				material = new THREE.MeshBasicMaterial( { map: texturePack.textureList[texturePack.textureMap[MESH_BOX]].texture } );
				mesh = new THREE.Mesh( geometry, material );

				mesh.dimensions = { height: 1, width: 1, depth: 1 };

				if (_meshProps.hasOwnProperty('position')) mesh.position.add(_meshProps.position);
				// if (Math.random() > 0.3) mesh.rotation.x = Math.PI * 0.25;
				// if (Math.random() > 0.3) mesh.rotation.y = -Math.PI * 0.25;
				// if (Math.random() > 0.3) mesh.rotation.z = -Math.PI * 0.25;

			} else if (meshProps.type === MESH_PLANE) {

				geometry = new THREE.BoxGeometry( 400, 5, 400, 1, 1, 1 );
				material = new THREE.MeshBasicMaterial( { map: texturePack.textureList[texturePack.textureMap[MESH_PLANE]].texture } );
				mesh = new THREE.Mesh( geometry, material );

				if (_meshProps.hasOwnProperty('position')) mesh.position.add(_meshProps.position);
				mesh.rotation.x = 0.0;
				mesh.position.x = 0.0;
				mesh.dimensions = { height: 0.1, width: 40, depth: 40 };

			} else {
				throw new Error("Adding undefined mesh: "+ meshProps.type);
			}

			scene.add( mesh );

			// Bounding box
			var bbox = new THREE.BoundingBoxHelper( mesh, 0xff0000 );
			bbox.update();
			// scene.add( bbox );
			mesh.AABB = bbox;

			return mesh;
		};

		this.render = function(){
			renderer.render( scene, camera );
		};

	};

	return Renderer;
});
