define(function(){

	var totalIslandsEver = -1,
		airShockLevel = 10;
	var Island = function(){

		this.uid = (++totalIslandsEver);
		// this.manifoldLeader = null;
		this.inTheAir = true;
		this.manifolds = [];
		this.timeOfCreation = (new Date()).getTime();
		this.count = 0;
		// this.staticContacts = [];


		this.addManifold = function(manifold) {

			if (manifold.bodyA.static || manifold.bodyB.static) {
				manifold.shockLevel = 0;
				this.inTheAir = false;
				// this.staticContacts.push(manifold);
			} else {
				if (this.updateManifoldShockLevel(manifold)) {

				} else {
					debugger; // this should never occur
				}
			}

			var disconnected = this.updateManifolds();
			if (disconnected) debugger; // This should never occur!

			if (!this.manifolds[manifold.shockLevel]) this.manifolds[manifold.shockLevel] = [];
			this.manifolds[manifold.shockLevel].push(manifold);
			manifold.indexInIsland = this.manifolds[manifold.shockLevel].length - 1;
			manifold.island = this;
			manifold.bodyA.island = this;
			manifold.bodyB.island = this;
			++this.count;

			// if (manifold.shockLevel == 0) {
				// this.updateManifolds();
			// }
		};


		this.removeManifold = function(manifold){

			// Update indexInIsland of other manifolds in the same shock level
			// this.manifolds[manifold.shockLevel].splice( manifold.indexInIsland, 1 );
			--this.count;
			if (this.count < 0) debugger;
			if (this.count === 0) return;

			if (manifold.shockLevel === null) debugger; // this should never occur
			if (!this.manifolds[manifold.shockLevel]) debugger; // this should never occur
			for (var i=manifold.indexInIsland; i<this.manifolds[manifold.shockLevel].length; ++i) {
				--this.manifolds[manifold.shockLevel][i].indexInIsland;
			}

			this.inTheAir = true; // Assume the worst case: we've lost our ground manifold
			return this.updateManifolds();
		};

		this.mergeWith = function(island){

			for (var shockLevel=0; shockLevel<island.manifolds.length; ++shockLevel) {
				var shock = island.manifolds[shockLevel];
				if (!shock) continue;
				for (var i=0; i<shock.length; ++i) {
					this.addManifold[shock[i]]; // FIXME: this is beyond inefficient because of the repeated updateManifolds!!!
				}
			}
			island.count == 0;
		};

		this.updateManifolds = function(){
			var manifoldsToShock = [],
				staticManifolds = [];
			for (var shockLevel=0; shockLevel<this.manifolds.length; ++shockLevel) {
				if (this.manifolds[shockLevel]) {
					for (var i=0; i<this.manifolds[shockLevel].length; ++i) {
						var _manifold = this.manifolds[shockLevel][i];
						// if (_manifold == manifold) continue; // same manifold that's being removed
						if (_manifold.bodyA.static || _manifold.bodyB.static) { // FIXME: user moving object!
							this.inTheAir = false;
							staticManifolds.push(_manifold);
							_manifold.indexInIsland = staticManifolds.length - 1;
						} else {
							manifoldsToShock.push(_manifold);
							_manifold.shockLevel = null;
						}
					}
				}
			}
			this.manifolds = [];

			if (staticManifolds.length) {
				this.manifolds[0] = staticManifolds;
			}

			var iFroze = 100,
				hasChanged = false;
			while (manifoldsToShock.length) {

				hasChanged = false;
				for (var i=0; i<manifoldsToShock.length; ++i) {
					var manifold = manifoldsToShock[i];
					if (this.updateManifoldShockLevel(manifold)) {
						manifoldsToShock.splice(i, 1);
						--i;
						hasChanged = true;

						if (!this.manifolds[manifold.shockLevel]) this.manifolds[manifold.shockLevel] = [];
						this.manifolds[manifold.shockLevel].push(manifold);
						manifold.indexInIsland = this.manifolds[manifold.shockLevel].length - 1;
					}

					if (--iFroze <= 0) {
						console.error("Oops :(");
						debugger;
					}
				}

				if (!hasChanged) {
					// There has been no changes, which means we're done.. except any remaining manifolds
					// must therefore be disconnected
					for (var i=0; i<manifoldsToShock.length; ++i) {
						var manifold = manifoldsToShock[i];
						manifold.island = null;
						manifold.bodyA.island = null;
						manifold.bodyB.island = null;
					}

					return manifoldsToShock;
				}
			}

			return false;
		};

		this.updateManifoldShockLevel = function(manifold){
				// Find shock level, add manifold

			if (this.count === 0) {
				manifold.shockLevel = airShockLevel;
				return true;
			}

				var nearestOnA = null,
					nearestOnB = null;
				for (var manifoldID in manifold.bodyA.manifolds) {
					var _manifold = manifold.bodyA.manifolds[manifoldID];
					if (_manifold == manifold) continue;
					if (_manifold.island != this && _manifold.island !== null) {
						// debugger; // this should never occur!
						this.mergeWith( _manifold.island );
					}
					if (_manifold.shockLevel !== null && (nearestOnA === null || _manifold.shockLevel < nearestOnA)) {
						nearestOnA = _manifold.shockLevel;
					}
				}
				for (var manifoldID in manifold.bodyB.manifolds) {
					var _manifold = manifold.bodyB.manifolds[manifoldID];
					if (_manifold == manifold) continue;
					if (_manifold.island != this && _manifold.island !== null) {
						this.mergeWith( _manifold.island );
						// debugger; // this should never occur!
					}
					if (_manifold.shockLevel !== null && (nearestOnA === null || _manifold.shockLevel < nearestOnA)) {
						nearestOnB = _manifold.shockLevel;
					}
				}

				if (nearestOnA !== null) manifold.shockLevel = nearestOnA + 1;
				if (nearestOnB !== null && (nearestOnA === null || nearestOnB < nearestOnA)) manifold.shockLevel = nearestOnB + 1;
				if (manifold.shockLevel === null) {
					return false;
				}
				

			return true;
		};


		/*
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

		*/

	};

	return Island;
});
