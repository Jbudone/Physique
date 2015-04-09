define(function(){

	Keys.add('MESH_BOX');
	Keys.add('MESH_SPHERE');
	Keys.add('MESH_PLANE');
	Keys.add('MESH_TETRAHEDRON');
	Keys.add('MESH_OCTAHEDRON');

	var Renderer = function(components){


		var settings = components.settings,
			scene    = null,
			camera   = null,
			renderer = null;

		var _me = this;
		var texturePack = {

			textureList: {
				stone: { file: 'stone.jpg', loading:[] },
				grass: { file: 'grass.jpg', options: { repeat: true }, loading:[] }
			},

			textureMap: {
			}
		};
		texturePack.textureMap[MESH_BOX] = 'stone';
		texturePack.textureMap[MESH_SPHERE] = 'stone';
		texturePack.textureMap[MESH_TETRAHEDRON] = 'stone';
		texturePack.textureMap[MESH_OCTAHEDRON] = 'stone';
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
							this.loadSkybox().then(function(){ console.log('loaded skybox..');
							}, function(err){ console.error('error loading skybox..'); });
							succeeded(renderer);
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

									var loading = texturePack.textureList[textureToLoad.id].loading;
									for (var i=0; i<loading.length; ++i) {
										var mesh = loading[i],
											material = new THREE.MeshBasicMaterial( { map: texture } );
										mesh.material = material;
									}
									delete texturePack.textureList[textureToLoad.id].loading;

									if (texturesToLoad.length === 0) {
										_me.loadSkybox().then(function(){ console.log('loaded skybox..');
										}, function(err){ console.error('error loading skybox..'); });
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
					succeeded(renderer);

					// this.loadSkybox().then(function(){
					// 	loadNext();
					// }, function(err){
					// 	failed(err);
					// });

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
		
		this.removeMesh = function(mesh){
			scene.remove(mesh.body);
		};

		this.applyTexture = function(mesh, texture){
			
			if (texturePack.textureList[texture].hasOwnProperty('texture')) {
				// Texture ready
				var material = new THREE.MeshBasicMaterial( { map: texturePack.textureList[texture].texture } );
				mesh.material = material;
			} else {
				texturePack.textureList[texture].loading.push(mesh);
			}
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

				var x=1, y=1, z=1;
				if (_meshProps.hasOwnProperty('dimensions')) {
					x = _meshProps.dimensions.x;
					y = _meshProps.dimensions.y;
					z = _meshProps.dimensions.z;
				}

				geometry = new THREE.BoxGeometry( x, y, z, 1, 1, 1 );
				material = new THREE.MeshBasicMaterial( { color: 0x00ff00, transparent: true, opacity: 0.5 } );
				mesh = new THREE.Mesh( geometry );
				this.applyTexture(mesh, texturePack.textureMap[MESH_BOX]);

				if (_meshProps.hasOwnProperty('position')) mesh.position.add(_meshProps.position);

			} else if (meshProps.type === MESH_SPHERE) {

				meshProps = _.defaults(_meshProps, {
					dimensions: { radius: 0.75 }
				});

				geometry = new THREE.SphereGeometry( meshProps.dimensions.radius, 12, 9 );
				material = new THREE.MeshBasicMaterial( { color: 0x000000 } );
				mesh = new THREE.Mesh( geometry, material );
				mesh.radius = meshProps.dimensions.radius;
				this.applyTexture(mesh, texturePack.textureMap[MESH_SPHERE]);

				if (_meshProps.hasOwnProperty('position')) mesh.position.add(_meshProps.position);

			} else if (meshProps.type === MESH_PLANE) {

				var x=400, y=5, z=400;
				if (_meshProps.hasOwnProperty('dimensions')) {
					x = _meshProps.dimensions.x;
					y = _meshProps.dimensions.y;
					z = _meshProps.dimensions.z;
				}


				geometry = new THREE.BoxGeometry( x, y, z, 1, 1, 1 );
				material = new THREE.MeshBasicMaterial( { color: 0x000000 } );
				mesh = new THREE.Mesh( geometry, material );
				this.applyTexture(mesh, texturePack.textureMap[MESH_PLANE]);

				if (_meshProps.hasOwnProperty('position')) mesh.position.add(_meshProps.position);
			} else if (meshProps.type === MESH_TETRAHEDRON) {

				meshProps = _.defaults(_meshProps, {
					dimensions: { radius: 0.75 }
				});


				geometry = new THREE.TetrahedronGeometry( meshProps.dimensions.radius );
				material = new THREE.MeshBasicMaterial( { color: 0x000000 } );
				mesh = new THREE.Mesh( geometry, material );
				mesh.radius = meshProps.dimensions.radius;
				this.applyTexture(mesh, texturePack.textureMap[MESH_TETRAHEDRON]);

				if (_meshProps.hasOwnProperty('position')) mesh.position.add(_meshProps.position);

			} else if (meshProps.type === MESH_OCTAHEDRON) {

				meshProps = _.defaults(_meshProps, {
					dimensions: { radius: 0.75 }
				});


				geometry = new THREE.OctahedronGeometry( meshProps.dimensions.radius );
				material = new THREE.MeshBasicMaterial( { color: 0x000000 } );
				mesh = new THREE.Mesh( geometry, material );
				mesh.radius = meshProps.dimensions.radius;
				this.applyTexture(mesh, texturePack.textureMap[MESH_OCTAHEDRON]);

				if (_meshProps.hasOwnProperty('position')) mesh.position.add(_meshProps.position);
			} else {
				throw new Error("Adding undefined mesh: "+ meshProps.type);
			}

			if (_meshProps.hasOwnProperty('rotation')) {
				mesh.rotation.x = _meshProps.rotation.x;
				mesh.rotation.y = _meshProps.rotation.y;
				mesh.rotation.z = _meshProps.rotation.z;

				// if (Math.random() > 0.3) mesh.rotation.x = Math.PI * 0.25;
				// if (Math.random() > 0.3) mesh.rotation.y = -Math.PI * 0.25;
				// if (Math.random() > 0.3) mesh.rotation.z = -Math.PI * 0.25;
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
