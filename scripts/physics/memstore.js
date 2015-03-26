define(function(){

	// NOTE: MemStore is a global Factory class, so even though its included in requirejs through multiple
	// places, they all use the same MemStore
	var MemStore = function(){

		this.Vector3 = (function(){

			var stored = [];

			THREE.Vector3.prototype.markedForMemStore = false;
			THREE.Vector3.prototype.delete = function(){
				if (this.markedForMemStore === true) {
					stored.push(this);
				}
			};
			THREE.Vector3.prototype.free = THREE.Vector3.prototype.delete;

			var c = 0,
				reportMalloc = function(){ console.log("Created new Vector3 ("+(++c)+")"); };

			return function(x, y, z){
				if (stored.length == 0) {
					// var vec = new (Function.prototype.bind.apply(THREE.Vector3, arguments))(); // FIXME:
					// for some reason the arguments is starting from index 1
					var vec;
					if (x instanceof THREE.Vector3) {
						vec = x.clone();
					} else {
						vec = new THREE.Vector3(x, y, z);
					}
					vec.markedForMemStore = true;
					// reportMalloc();
					return vec;
				}

				var vec = stored.shift();
				if (x instanceof THREE.Vector3) {
					vec.copy(x);
				} else {
					vec.set(x || 0, y || 0, z || 0);
				}
				return vec;
			};

		}());
	};

	return (new MemStore());
});
