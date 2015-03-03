define(function(){

	var Keys = {

		_keys: [],
		add: function(name){
			if (window.hasOwnProperty(name)) {
				throw new Error("Key ("+name+") already defined!");
			}

			this._keys.push(name);
			window[name] = this._keys.length; // NOTE: key[0] will have value 1
		},
	};

	return Keys;
});
