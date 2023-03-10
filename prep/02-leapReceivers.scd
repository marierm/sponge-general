(
// A clone of this Prototype (like a class) will be created for each
// OSC path we listen too.  The default behavior is to scale all
// messages between 0 and 1 and to set a bus to that value.  Some OSC
// path have more than one parameter (x,y,z or pitch,roll,yaw); this
// requires multichannel buses.

// When this file is loaded, ~leapReceivers is an IdentityDictionary
// that contains names -> ~leapReceiver Protos.  The ~leapConfig array
// contains all configuration: name, oscPaths and specArray
// (calibration data).

~leapReceiver = Proto({
	~port = 49999;	// Ephemeral port.
	~print = true;
	~initialize = { |name, oscPath, specArray|
		~name = name ? \defaultLeap;
		~oscPath = oscPath ? "/leapmotion";
		~specArray = specArray;	
		~specs = ~specArray.collect(_.asSpec);
		~numChan = ~specs.size;
		~bus = Bus.control(Server.default, ~numChan);
		~def = OSCdef( ~name, { |msg, timeStamp|
			~print.if({
				(~name ++ ": ").post;
				msg[1..].postln;
			});
			~bus.setn(
				msg[1..].collect({ |i, j|
					~specs[j].unmap(i);
				});
			);
		}.inEnvir, ~oscPath, recvPort: ~port);
		currentEnvironment;
	};
	~calibrate = {
		~specs.do({ |i|
			i.minval_(32000);
			i.maxval_(-32000);
		});
		~calibrationDef = OSCdef( (~name ++ "--Cal").asSymbol, { |msg|
			msg[1..].do({ |i, j|
				(i < ~specs[j].minval).if({
					~specs[j].minval_(i);
				});
				(i > ~specs[j].maxval).if({
					~specs[j].maxval_(i);
				})
			});
		}.inEnvir, ~oscPath, recvPort: ~port);
	};
	~stopCalibration = {
		~calibrationDef.free;
		~dumpCalibration.value;
	};
	~dumpCalibration = {
		"[ ".post;
		~specs.do({|i|
			[i.minval, i.maxval].post;
			", ".post;
		});
		("], " + "// " + ~name).postln;
	};
	~free = {
		~def.free;
		try { ~calibrationDef.free };
		~bus.free;
	}
});

// A compact way of declaring oscDef names with corresponding OSC
// paths.
// gives : [
// [\name, "path", [ [spec], [spec], ... ] ],
// [\name, "path", [ [spec], [spec], ... ] ], ... ]
~leapConfig = [
	[
		\numHands,
		\left, \lOrient, \lPos, \lSphere, 
		\right, \rOrient, \rPos, \rSphere
	],
	["/leapmotion/numhands"] ++
	[
		["/leapmotion"], ["/hand"], ["/left","/right"],
		["", "/orientation", "/position", "/sphere/radius"]
	].allTuples.collect(_.join),
	[ // This is the "default" calibration.
		// Copy the output of ~leapReceivers.do(_.dumpCalibration);
		// Arrays are [minval,maxval,warp, step]
		[ [ 0, 2 ], ],  //  leapNumHands
		[ [ 0, 1 ], ],  // left (assumes only one left hand)
		[ [ -179.79426574707, 175.31391906738 ], [ -179.96960449219, 179.98629760742 ], [ -179.63610839844, 178.93190002441 ], ],  //  leapLeftOrient
		[ [ -347.53311157227, 228.9963684082 ], [ 71.353790283203, 617.65570068359 ], [ -305.54360961914, 264.82501220703 ], ],  //  leapLeftPos
		[ [ 36.359676361084, 411.03643798828 ], ],  //  leapLeftSphere
		[ [ 0, 1 ], ],  // right (assumes only one right hand)
		[ [ -179.97616577148, 179.61235046387 ], [ -179.90298461914, 179.96981811523 ], [ -179.19645690918, 179.06399536133 ], ],  //  leapRightOrient
		[ [ -253.4771270752, 277.05780029297 ], [ 43.974826812744, 607.38232421875 ], [ -20.264762878418, 278.33212280273 ], ],  //  leapRightPos
		[ [ 32.889392852783, 484.68316650391 ], ],  //  leapRightSphere
	]
].flop;

~leapReceivers = IdentityDictionary.newFrom (
	~leapConfig.collect({ |item, index|
		[ item[0],
			~leapReceiver.new(
				item[0], // name
				item[1], // oscPath
				item[2] // specArray
			);
		]
	}).flatten;
);
)
