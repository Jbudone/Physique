define(function(){

	// TODO: keep track of min/max outliers (n number of outliers) to remove from avg mean
	var profilesRunning = {},
		profiles = {};

	var Profile = function(label){

		this.label = label;

		this.runs = 0;
		this.mean = 0;
		this.min = 0;
		this.max = 0;

		this.addRun = function(time){

			if (this.runs == 0) {
				this.mean = time;
				this.runs = 1;
				this.min = time;
				this.max = time;
			} else {
				this.mean = (this.mean * this.runs + time) / (this.runs + 1);
				++this.runs;
				if (time < this.min) this.min = time;
				if (time > this.max) this.max = time;
			}
		};
	};


	var Profiler = {

		profile: function(label){
			if (!profiles.hasOwnProperty(label)) profiles[label] = new Profile(label);
			if (profilesRunning.hasOwnProperty(label)) return false;
			profilesRunning[label] = profiles[label];
			profiles[label].start = (new Date()).getTime();
		},

		profileEnd: function(label){
			if (!profilesRunning.hasOwnProperty(label)) return false;
			var profile = profiles[label],
				time = (new Date()).getTime() - profile.start;
			delete profile.start;
			profile.addRun(time);
			delete profilesRunning[label];
		},

		report: function(label){
			
			console.log("Profiler Report");
			console.log("==================");
			if (label == undefined) {

				for (var label in profiles) {
					this.report(label);
				}
				console.log("==================");
			} else {
				var profile = profiles[label];
				console.log("	"+label+" [ mean: ("+ profile.mean +"ms),  min: ("+ profile.min +"ms),  max: ("+ profile.max +"ms),  runs: "+ profile.runs +"]");
			}
		}
	};

	return Profiler;
});
