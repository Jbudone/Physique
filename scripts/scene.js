define(function(){

	var Scene = function(){

		this.onAddedMesh = new Function();

		this.meshes = {};
		this.meshCount = 0;

		this.addMesh = function(settings){

			var mesh = {
				settings: settings,
				uid: (++this.meshCount)
			};

			this.meshes[mesh.uid] = mesh;
			this.onAddedMesh(mesh);

			return mesh;
		};
	};

	return Scene;
});
