define(function(){

	var Scene = function(){

		this.onAddedMesh = new Function();
		this.onRemovedMesh = new Function();

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

		this.reset = function(){

			for (var meshID in this.meshes) {
				this.onRemovedMesh(this.meshes[meshID]);
				delete this.meshes[meshID];
			}

			this.meshCount = 0;
		};
	};

	return Scene;
});
