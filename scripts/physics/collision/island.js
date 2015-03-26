define(function(){

	var Island = function(uid, manifoldLeader){

		this.uid = uid;
		this.manifoldLeader = null;
		this.inTheAir = true;
		this.manifolds = [];
		this.timeOfCreation = (new Date()).getTime();
		this.count = 0;
		this.staticContacts = [];

		this.addManifold = function(manifold){
			if (!this.manifoldLeader && manifold.uid == this.uid) {
				this.manifoldLeader = manifold;
			} else if (this.inTheAir) {
				// Perhaps our new manifold is on the ground?
				if (manifold.bodyA.static || manifold.bodyB.static) {
					this.uid = manifold.uid;
					this.manifoldLevel = manifold;
					this.inTheAir = false;
				}
			}

			if (manifold.bodyA.static || manifold.bodyB.static) {
				manifold.shockLevel = 0;
				this.staticContacts.push(manifold);
			} else {

				// TODO: find shock level, add manifold
				var nearestOnA = null,
					nearestOnB = null;
				for (var manifoldID in manifold.bodyA.manifolds) {
					var _manifold = manifold.bodyA.manifolds[manifoldID];
					if (_manifold == manifold) continue;
					if (_manifold.island != this) debugger; // this should never occur!
					if (_manifold.shockLevel !== null && (nearestOnA === null || _manifold.shockLevel < nearestOnA)) {
						nearestOnA = _manifold.shockLevel;
					}
				}
				for (var manifoldID in manifold.bodyB.manifolds) {
					var _manifold = manifold.bodyB.manifolds[manifoldID];
					if (_manifold == manifold) continue;
					if (_manifold.island != this) debugger; // this should never occur!
					if (_manifold.shockLevel !== null && (nearestOnA === null || _manifold.shockLevel < nearestOnA)) {
						nearestOnB = _manifold.shockLevel;
					}
				}

				if (nearestOnA !== null) manifold.shockLevel = nearestOnA + 1;
				if (nearestOnB !== null && (nearestOnA === null || nearestOnB < nearestOnA)) manifold.shockLevel = nearestOnB + 1;
				if (manifold.shockLevel === null) debugger; // This should never occur!
			}

			if (!this.manifolds[manifold.shockLevel]) this.manifolds[manifold.shockLevel] = [];
			this.manifolds[manifold.shockLevel].push(manifold);
			manifold.indexInIsland = this.manifolds[manifold.shockLevel].length - 1;
			manifold.island = this;
			++this.count;
		};

		this.updateManifoldShockLevel = function(manifold){
			// TODO: find all neighbour manifolds
		};

		this.removeManifold = function(manifold){

			// Update indexInIsland of other manifolds in the same shock level
			this.manifolds[manifold.shockLevel].splice( manifold.indexInIsland, 1 );
			--this.count;
			if (this.count === 0) return;

			for (var i=manifold.indexInIsland; i<this.manifolds[manifold.shockLevel].length; ++i) {
				--this.manifolds[manifold.shockLevel][i].indexInIsland;
			}

			
			if (manifold.uid === this.uid) {
				var newManifoldLeader = null;
				for (var j=0; j<this.manifolds.length; ++j) {
					var manifoldLevel = this.manifolds[j];
					if (!manifoldLevel) continue;
					for (var k=0; k<manifoldLevel.length; ++k) {
						var manifold = manifoldLevel[k];
						if (manifold.bodyA.static || manifold.bodyB.static) {
							newManifoldLeader = manifold;
							this.inTheAir = false;
							// FIXME: make sure its not static from the user holding it!! That its
							// permanently static
							break;
						}
					}
				}

				if (!newManifoldLeader) {
					for (var i=0; i<this.manifolds.length; ++i) {
						for (var j=0; j<this.manifolds[i].length; ++j) {
							newManifoldLeader = this.manifolds[i][j];
							this.inTheAir = true;
							break;
						}
					}
				}
				
				this.manifoldLeader = newManifoldLeader;
				this.uid = newManifoldLeader.uid;
				// FIXME: update shock levels for EVERYTHING
			}

		};

		this.mergeWith = function(island){
			// FIXME FIXME FIXME FIXME FIXME FIXME!
		};

	};

	return Island;
});
