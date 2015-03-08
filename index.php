<html>
<head>

	<title>Physics Engine</title>


	<!--

	TODO list

		* CD
			- Spatial data structure - BS
			- Architecture (separate physique components...compile into 1 file)
			- Broadphase
				- islands
				- data structure for world
				- improve SAP add/update body (linked list; cache pointers)
				- temporal coherence for SAP axis list
			- SAT
				- IMPLEMENT: for each object find all axis (normal of each face on polygon), merge axis between object pairs; for each axis, project each objects' vertices onto axis to find min/max line on axis; if ALL axis have collision then there is a collision, and the normal is the axis)
				- NOTE: circles need a separate case for testing
				- cache object axes (rotate only when necessary)
				- cache object world vertices (only rotate when necessary)
				- boxes should only have 3 distinct axes for faces (skip the scalar multiples on opposite side)
				- look into SAT colliding on edges issue
				- ERROR:
					* check face/normal (debug lines) - is the normal updated interally already? no need to automatically apply quaternion? are normals facing the right direction?
					* 
			- GJK
				- pick a more intuitive initial direction
				- improve support function
				- look into GJK numerical stability issues
					- Erin Catto GDC 2010: GJK lecture pg. ~113
				- look into GJK penetration & margins: http://hacktank.net/blog/?p=129
				- look into GJK early-out: why do people claim that GJK can finish without being a tetrahedron? (pg. 159 of Collision Detection / 3d Environments book)
				- continuous collision detection (extend support function from initial position of body to end position of body)
			- EPA
				- DEBUG: draw contact points (turn colliding meshes transparent); draw contact normals; draw penetration depth (line from start to end to show precise contact area)
				- Look into Goblin Physics silhouette algorithm (efficiency)
				- BUG: creating new face sometimes has reverse normal
				- BUG: sometimes hits infinite loop (redundant vertices added from support point)
			- report contact manifold
		* CR
			- Angular velocity
			- restitution?
			- baumgarte?
			- pre-sequential solvers?
			- friction
			- slop
			- FIX contact.tangent and contact.tangent2 (EVERYTHING that uses them needs to have the correct sign)

			- BUG: resting contact
			- BUG: box sometimes slips

			- check if point is colliding: check against normal of every face
		* UI: edit clock
			- edit clock
			- keep track of snapshots of world; then step backwards in time (debug)
			- improve raycasting / moving object


	-->




	<!-- disable evil cache -->
	<meta http-equiv="cache-control" content="max-age=0" />
	<meta http-equiv="cache-control" content="no-cache" />
	<meta http-equiv="expires" content="0" />
	<meta http-equiv="expires" content="Tue, 01 Jan 1980 1:00:00 GMT" />
	<meta http-equiv="pragma" content="no-cache" />

	<!-- styles -->
	<link href="styles/libs/bootstrap.min.css" rel="stylesheet">
	<link rel="stylesheet" href="styles/main.css">


	<!-- scripts -->
	<script src="scripts/libs/jquery-2.1.3.min.js"></script>
	<script src="scripts/libs/three.js"></script>
	<script src="scripts/libs/bootstrap.min.js"></script>
	<script src="scripts/libs/underscore-min.js"></script>
	<script src="scripts/libs/modernizr.min.js"></script>
	<script src="scripts/libs/require.js"></script>
	<script src="scripts/GoblinPhysics/build/goblin.js"></script>
	<script>
		requirejs.config({
			"baseUrl": "scripts",
		});

		require(['keys'], function(Keys){

			window['Keys'] = Keys;

			var readyToStartup = function(){

				$(document).ready(function(){

					// Startup
					require(['main']);
				});

			};

			// Make sure that we have ES6 Promises before starting webapp
			Modernizr.load([
				{
					test: Promise,
					nope: ['scripts/libs/es6-promise-2.0.1.min.js'],
					complete: readyToStartup
				}
			]);
		
		});

		var scene    = null,
			renderer = null,
			physique = null;
	</script>

</head>
<body>

	<div id="options">
		<div class="section">
			<span class="section-title">Physics Debugger</span>
			<div class="details">
				<input id="objectsInput" list="objects" name="object">
				<datalist id="objects">
				</datalist>
				<span class=option-text">Meshes</span>
				<br/>

				<input id="rotX" type="range" value="0" min="-314.1" max="314.1" />
				<span class="option-text">Rotate X</span>

				<br/>

				<input id="rotY" type="range" value="0" min="-314.1" max="314.1" />
				<span class="option-text">Rotate Y</span>

				<br/>

				<input id="rotZ" type="range" value="0" min="-314.1" max="314.1" />
				<span class="option-text">Rotate Z</span>

				<br/>
			</div>
		</div>
	</div>

</body>
</html>
