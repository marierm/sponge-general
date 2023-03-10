~cSponge = {
	arg ip, defName = \sponge01, server;
	var numBits = 5;
	// This table is used to invert the bit order of the buttons.  This is
	// useful when controling pitch with the buttons.
	var conversionTable = (0..(2.pow(numBits) - 1)).collect({|i|
		i.asInteger.asBinaryDigits(numBits).reverse.convertDigits(2)
	});
	var netAddress, sponge, butts;
	var pitchMode = 1;
	var harmoMode = 2;
	var pitchModeRzn = 3;
	var harmoModeRzn = 4;
	var pitchModePop = 5;  // Useful only for loop recording.
	var harmoModePop = 6;
	var fmVoice = 0;
	var meloVoice = 1;
	var popVoice = 2;
	var flmVoice = 3;
	var feuVoice = 4;

	var prep = {
		server = server ? Server.default;
		ip.isNil.if({
			netAddress = nil;
		},{
			netAddress = NetAddr(ip, 50502);
		});
		server.waitForBoot({
			sponge = SpongeFeather(
				"/sponge",
				netAddress,
				50501,
				currentEnvironment // To execute the OSCfunc inside this environment.
			);
			// activate the features needed
			[
				\acc1xHP, \acc1yHP, \acc1zHP,
				\acc2xHP, \acc2yHP, \acc2zHP, 
				\acc1xBP, \acc1yBP, \acc1zBP,
				\acc2xBP, \acc2yBP, \acc2zBP, 
				\bend
			].do{|i,j|
				sponge.activateFeature(i);
			};
			// create button bank
			butts = ButtonBank(ButtonInput.sponge(sponge));
			// Buttons 9 to 5 are used to make operations on corresponding voices.
			// Modifier keys will change their behavior.
			// Level 0 stops the voice.
			// Level 1 starts the voice.
			// Level 16 (bit 4) changes to "pitch control" mode.
			// ~butts.modes[0].buttonFunctions[0..4].do{|i,j|
			butts.modes[0].buttonFunctions[9..5].do{|i,j|
				i.functions_( // Right hand, finger 1.
					Dictionary.newFrom([
						1, Array.newClear(32).putEach([0,1],[
							{ // When no modifier key is pressed.
								\stop.post; j.postln;
								~voices.at(defName)[j].stopMe;
							},
							{ //  When left hand, key 1 is held.
								\play.post; j.postln;
								~voices.at(defName)[j].playMe;
							},
						]),
						0, Array.newClear(32).putEach([8, 16],[
							{ // When left hand, key 4 is held.
								("mode harmo " + (j + 1)).postln;
								butts.mode_((j*2) + 2); // modes 2, 4, 6, ...
							},
							{ // When left hand, key 5 is held.
								("mode pitch" + (j + 1)).postln;
								butts.mode_((j*2) + 1); // modes 1, 3, 5, ...
							},
						])
					]);
				)
			};
			// in mode 0, buttons 0 to 4 are modifier keys.
			butts.modes[0].buttonFunctions[4..0].do{|i,j| i.makeModifier(j)};


			//////////////////////////////////////
			///////////// PITCH MODE /////////////
			//////////////////////////////////////
			// In pitchMode (1), buttons 0 to 4 set the pitch.
			butts.newMode; // pitch mode is created
			// buttons 0 to 4 are not really modifiers.  They set the level
			// and then, the level is used to set the pitch.
			// butts.modes[pitchMode].buttonFunctions[0..4].do{|i,j| i.makeModifier(j)};
			butts.modes[pitchMode].buttonFunctions[0..4].do{|i,j| i.makeModifier(j)};

			// Enable double clicks for button 9.
			// butts.modes[pitchMode].buttonFunctions[8].enableNclick;
			butts.modes[pitchMode].buttonFunctions[9].enableNclick;

			// for each button(0 to 4), add function to level 0.
			(1..4).do{|i,j|
				butts.modes[pitchMode].buttonFunctions[i].functions[1][0] = (
					butts.modes[pitchMode].buttonFunctions[i].functions[1][0]
				).addFunc({
					// This func offsets the pitch of the voice.
					~voices.at(defName)[fmVoice].pitchOsBus.set(
						// & 30 gets rid of LSB (no button 5)
						conversionTable[ butts.modes[pitchMode].level & 30 ]
					);
				});
				butts.modes[pitchMode].buttonFunctions[i].functions[0][0] = (
					butts.modes[pitchMode].buttonFunctions[i].functions[0][0]
				).addFunc({
					// do it when button is released too
					~voices.at(defName)[fmVoice].pitchOsBus.set(
						conversionTable[ butts.modes[pitchMode].level & 30 ]
					);
				});
			};


			// button 5 switch back to mode 0 (any level works).
			butts.modes[pitchMode].addFunc(
				button: 5,
				function: {
					butts.mode_(0);
					("mode:" + 0).postln; 
					butts.mode.level_(0);
				}.inEnvir,
				levels: (0..15),
				buttState: 1
			);

			// button 9 starts the voice.
			// Double click stops it.
			butts.modes[pitchMode].addFunc(
				button: 9,
				function: {
					("Stop fm pitch" + defName).postln;
					~voices.at(defName)[fmVoice].stopMe;
					// \play.post; h.postln;
					// ~voices.at( (~defName ++ 0).asSymbol ).playMe(0,doReset: true);
				},
				levels: (0..15),
				buttState: 2
			);
			butts.modes[pitchMode].addFunc(
				button: 9,
				function: {
					("Play fm pitch" + defName).postln;
					~voices.at(defName)[fmVoice].playMe;
					// \unfreeze.post; h.postln;
					// ~voices.at( (~defName ++ 0).asSymbol ).unfreezeMe;
				},
				levels: (0..15),
				buttState: 1
			);

			// button 6 (right hand, key 4) = one octave lower
			butts.modes[pitchMode].addFunc(
				button: 6,
				function: {
					\octVbStuck.post; h.postln;
					~voices.at(defName)[fmVoice].octVb;
				},
				levels: (0..15),
				buttState: 1
			);

			// button 7 (right hand, key 3) one octave higher
			butts.modes[pitchMode].addFunc(
				button: 7,
				function: {
					\octVaStuck.post; h.postln;
					~voices.at(defName)[fmVoice].octVa;
				},
				levels: (0..15),
				buttState: 1
			);

			// button 8 (right hand, key 2) reset octave (to octave 1)
			// butts.modes[pitchMode].addFunc(
			// 	button: 8,
			// 	function: {
			// 		\octReset.post; h.postln;
			// 		~voices.at(defName)[fmVoice].octReset;
			// 	},
			// 	levels: (0..15),
			// 	buttState: 0
			// );

			// button 8 controls looper.
			butts.modes[pitchMode].addFunc(
				button: 8,
				function: {
					\toggleRecording.post; h.postln;
					~voices.at(defName)[fmVoice].looper.toggleRec;
				},
				levels: [0] ++ (2..31),
				buttState: 1
			);
			butts.modes[pitchMode].addFunc(
				button: 8,
				function: {
					"Stop loop playback ".post; h.postln;
					~voices.at(defName)[fmVoice].looper.stopPlayback;
				},
				levels: [1],
				buttState: 1
			);
			// Mode 2 is harmoMode 8, buttons 0 to 4 sets notes of chords.
			////////////////////////////
			// ////////////////////// //
			////////////////////////////

			
			
			/////////////////////////////////////////////
			///////////// HARMO MODE FOR FM /////////////
			/////////////////////////////////////////////
			butts.newMode;
			// buttons 0 to 4 are not really modifiers.  They set the level
			// and then, the level is used to set the chord.
			butts.modes[harmoMode].buttonFunctions[0..4].do{|i,j| i.makeModifier(j)};

			butts.modes[harmoMode].buttonFunctions[9].enableNclick;
			// for each button(0 to 4), add function to level 0.
			(1..4).do{ |i,j|
				butts.modes[harmoMode].buttonFunctions[i].functions[1][0] = (
					butts.modes[harmoMode].buttonFunctions[i].functions[1][0]
				).addFunc({
					// This func offsets the pitch of the voice.
					~voices.at(defName)[fmVoice].setChord(
						conversionTable[ butts.modes[harmoMode].level & 30]
					);
				});
				butts.modes[harmoMode].buttonFunctions[i].functions[0][0] = (
					butts.modes[harmoMode].buttonFunctions[i].functions[0][0]
				).addFunc({
					// do it when button is released too
					~voices.at(defName)[fmVoice].setChord(
						conversionTable[ butts.modes[harmoMode].level & 30]
					);
				});
			};


			// button 5 switches back to mode 0 (any level works).
			butts.modes[harmoMode].addFunc(
				button: 5,
				function: {
					butts.mode_(0);
					("mode:" + 0).postln; 
					butts.mode.level_(0);
				}.inEnvir,
				levels: (0..15),
				buttState: 1
			);

			// button 9 starts the voice (this is for harmo mode).
			// Double click stops it (this is for harmo mode).
			butts.modes[harmoMode].addFunc(
				button: 9,
				function: {
					\stop.post; h.postln;
					~voices.at(defName)[fmVoice].stopMe;
				},
				levels: (0..15),
				buttState: 2
			);
			butts.modes[harmoMode].addFunc(
				button: 9,
				function: {
					\play.post; h.postln;
					~voices.at(defName)[fmVoice].playMe;
				},
				levels: (0..15),
				buttState: 1
			);

			// Octave controls for harmo mode.
			butts.modes[harmoMode].addFunc(
				button: 6,
				function: {
					\octVbStuck.post; h.postln;
					~voices.at(defName)[fmVoice].octVb;
				},
				levels: (0..15),
				buttState: 1
			);
			// butts.modes[harmoMode].addFunc(
			// 	button: 8,
			// 	function: {
			// 		\octReset.post; h.postln;
			// 		~voices.at(defName)[fmVoice].octReset;
			// 	},
			// 	levels: (0..15),
			// 	buttState: 0
			// );
			butts.modes[harmoMode].addFunc(
				button: 8,
				function: {
					\toggleRecording.post; h.postln;
					~voices.at(defName)[fmVoice].looper.toggleRec;
				},
				levels: (0..15),
				buttState: 1
			);
			butts.modes[harmoMode].addFunc(
				button: 8,
				function: {
					\stopPlayback.post; h.postln;
					~voices.at(defName)[fmVoice].looper.stopPlayback;
				},
				levels: [1],
				buttState: 1
			);

			butts.modes[harmoMode].addFunc(
				button: 7,
				function: {
					\octVaStuck.post; h.postln;
					~voices.at(defName)[fmVoice].octVa;
				},
				levels: (0..15),
				buttState: 1
			);

			////////////////////////////////////////////////////
			///////////// PITCH MODE FOR RZN ///////////////////
			////////////////////////////////////////////////////
			// In pitchMode for Rzn (3), buttons 0 to 3 set the pitch.
			butts.newMode; // pitch mode melo is created
			// buttons 0 to 3 are not really modifiers.  They set the level
			// and then, the level is used to set the pitch.
			// butts.modes[pitchMode].buttonFunctions[0..4].do{|i,j| i.makeModifier(j)};
			butts.modes[pitchModeRzn].buttonFunctions[0..4].do{|i,j| i.makeModifier(j)};

			// Enable double clicks for button 9.
			// butts.modes[pitchMode].buttonFunctions[8].enableNclick;
			butts.modes[pitchModeRzn].buttonFunctions[9].enableNclick;

			// for each button(0 to 4), add function to level 0.
			(1..4).do{|i,j|
				butts.modes[pitchModeRzn].buttonFunctions[i].functions[1][0] = (
					butts.modes[pitchModeRzn].buttonFunctions[i].functions[1][0]
				).addFunc({
					// This func offsets the pitch of the voice.
					~voices.at(defName)[meloVoice].pitchOsBus.set(
						// & 30 gets rid of LSB (no button 5)
						conversionTable[ butts.modes[pitchModeRzn].level & 30 ]
					);
				});
				butts.modes[pitchModeRzn].buttonFunctions[i].functions[0][0] = (
					butts.modes[pitchModeRzn].buttonFunctions[i].functions[0][0]
				).addFunc({
					// do it when button is released too
					~voices.at(defName)[meloVoice].pitchOsBus.set(
						conversionTable[ butts.modes[pitchModeRzn].level & 30 ]
					);
				});
			};


			// button 5 switch back to mode 0 (any level works).
			butts.modes[pitchModeRzn].addFunc(
				button: 5,
				function: {
					butts.mode_(0);
					("mode:" + 0).postln; 
					butts.mode.level_(0);
				}.inEnvir,
				levels: (0..15),
				buttState: 1
			);

			// button 9 starts the voice.
			// Double click stops it.
			butts.modes[pitchModeRzn].addFunc(
				button: 9,
				function: {
					"Stop pop melo.".postln;
					~voices.at(defName)[meloVoice].stopMe;
					// \play.post; h.postln;
					// ~voices.at( (~defName ++ 0).asSymbol ).playMe(0,doReset: true);
				},
				levels: (0..15),
				buttState: 2
			);
			butts.modes[pitchModeRzn].addFunc(
				button: 9,
				function: {
					"Play pop melo.".postln;
					~voices.at(defName)[meloVoice].playMe;
					// \unfreeze.post; h.postln;
					// ~voices.at( (~defName ++ 0).asSymbol ).unfreezeMe;
				},
				levels: (0..15),
				buttState: 1
			);

			// button 6 (right hand, key 4) = one octave lower
			butts.modes[pitchModeRzn].addFunc(
				button: 6,
				function: {
					\octVbStuck.post; h.postln;
					~voices.at(defName)[meloVoice].octVb;
				},
				levels: (0..15),
				buttState: 1
			);

			// button 7 (right hand, key 3) one octave higher
			butts.modes[pitchModeRzn].addFunc(
				button: 7,
				function: {
					\octVaStuck.post; h.postln;
					~voices.at(defName)[meloVoice].octVa;
				},
				levels: (0..15),
				buttState: 1
			);

			// button 8 controls looper.
			butts.modes[pitchModeRzn].addFunc(
				button: 8,
				function: {
					\toggleRecording.post; h.postln;
					~voices.at(defName)[meloVoice].looper.toggleRec;
				},
				levels: [0] ++ (2..31),
				buttState: 1
			);
			butts.modes[pitchModeRzn].addFunc(
				button: 8,
				function: {
					"Stop loop playback ".post; h.postln;
					~voices.at(defName)[meloVoice].looper.stopPlayback;
				},
				levels: [1],
				buttState: 1
			);
			// Mode 2 is harmoMode 8, buttons 0 to 4 sets notes of chords.

			/////////////////////////////////////////////
			///////////// HARMO MODE RZN ////////////////
			/////////////////////////////////////////////
			butts.newMode;
			// buttons 0 to 4 are not really modifiers.  They set the level
			// and then, the level is used to set the chord.
			butts.modes[harmoModeRzn].buttonFunctions[0..4].do{|i,j| i.makeModifier(j)};

			butts.modes[harmoModeRzn].buttonFunctions[9].enableNclick;
			// for each button(0 to 4), add function to level 0.
			(1..4).do{ |i,j|
				butts.modes[harmoModeRzn].buttonFunctions[i].functions[1][0] = (
					butts.modes[harmoModeRzn].buttonFunctions[i].functions[1][0]
				).addFunc({
					// This func offsets the pitch of the voice.
					~voices.at(defName)[meloVoice].setChord(
						conversionTable[ butts.modes[harmoModeRzn].level & 30]
					);
				});
				butts.modes[harmoModeRzn].buttonFunctions[i].functions[0][0] = (
					butts.modes[harmoModeRzn].buttonFunctions[i].functions[0][0]
				).addFunc({
					// do it when button is released too
					~voices.at(defName)[meloVoice].setChord(
						conversionTable[ butts.modes[harmoModeRzn].level & 30]
					);
				});
			};


			// button 5 switches back to mode 0 (any level works).
			butts.modes[harmoModeRzn].addFunc(
				button: 5,
				function: {
					butts.mode_(0);
					("mode:" + 0).postln; 
					butts.mode.level_(0);
				}.inEnvir,
				levels: (0..15),
				buttState: 1
			);

			// button 9 starts the voice (this is for harmo mode).
			// Double click stops it (this is for harmo mode).
			butts.modes[harmoModeRzn].addFunc(
				button: 9,
				function: {
					\stop.post; h.postln;
					~voices.at(defName)[meloVoice].stopMe;
				},
				levels: (0..15),
				buttState: 2
			);
			butts.modes[harmoModeRzn].addFunc(
				button: 9,
				function: {
					\play.post; h.postln;
					~voices.at(defName)[meloVoice].playMe;
				},
				levels: (0..15),
				buttState: 1
			);

			// Octave controls for harmo mode.
			butts.modes[harmoModeRzn].addFunc(
				button: 6,
				function: {
					\octVbStuck.post; h.postln;
					~voices.at(defName)[meloVoice].octVb;
				},
				levels: (0..15),
				buttState: 1
			);
			// butts.modes[harmoModeRzn].addFunc(
			// 	button: 8,
			// 	function: {
			// 		\octReset.post; h.postln;
			// 		~voices.at(defName)[meloVoice].octReset;
			// 	},
			// 	levels: (0..15),
			// 	buttState: 0
			// );
			butts.modes[harmoModeRzn].addFunc(
				button: 8,
				function: {
					\toggleRecording.post; h.postln;
					~voices.at(defName)[meloVoice].looper.toggleRec;
				},
				levels: (0..15),
				buttState: 1
			);
			butts.modes[harmoModeRzn].addFunc(
				button: 8,
				function: {
					\stopPlayback.post; h.postln;
					~voices.at(defName)[meloVoice].looper.stopPlayback;
				},
				levels: [1],
				buttState: 1
			);

			butts.modes[harmoModeRzn].addFunc(
				button: 7,
				function: {
					\octVaStuck.post; h.postln;
					~voices.at(defName)[meloVoice].octVa;
				},
				levels: (0..15),
				buttState: 1
			);

			////////////////////////////////////////////////////
			///////////// PITCH MODE FOR POP ///////////////////
			////////////////////////////////////////////////////
			butts.newMode; // pitch mode pops is created
			// buttons 0 to 3 are not really modifiers.  They set the level
			// and then, the level is used to set the pitch.
			// butts.modes[pitchMode].buttonFunctions[0..4].do{|i,j| i.makeModifier(j)};
			butts.modes[pitchModePop].buttonFunctions[0].do{|i,j| i.makeModifier(j)};

			// Enable double clicks for button 9.
			// butts.modes[pitchMode].buttonFunctions[8].enableNclick;
			butts.modes[pitchModePop].buttonFunctions[9].enableNclick;


			// button 5 switch back to mode 0 (any level works).
			butts.modes[pitchModePop].addFunc(
				button: 5,
				function: {
					butts.mode_(0);
					("mode:" + 0).postln; 
					butts.mode.level_(0);
				}.inEnvir,
				levels: (0..15),
				buttState: 1
			);

			// button 9 starts the voice.
			// Double click stops it.
			butts.modes[pitchModePop].addFunc(
				button: 9,
				function: {
					"Stop pops.".postln;
					~voices.at(defName)[popVoice].stopMe;
					// \play.post; h.postln;
					// ~voices.at( (~defName ++ 0).asSymbol ).playMe(0,doReset: true);
				},
				levels: (0..15),
				buttState: 2
			);
			butts.modes[pitchModePop].addFunc(
				button: 9,
				function: {
					"Play pops.".postln;
					~voices.at(defName)[popVoice].playMe;
					// \unfreeze.post; h.postln;
					// ~voices.at( (~defName ++ 0).asSymbol ).unfreezeMe;
				},
				levels: (0..15),
				buttState: 1
			);

			// button 8 controls looper.
			butts.modes[pitchModePop].addFunc(
				button: 8,
				function: {
					\toggleRecording.post; h.postln;
					~voices.at(defName)[popVoice].looper.toggleRec;
				},
				levels: [0] ++ (2..31),
				buttState: 1
			);
			butts.modes[pitchModePop].addFunc(
				button: 8,
				function: {
					"Stop loop playback of pops.".post;
					~voices.at(defName)[popVoice].looper.stopPlayback;
				},
				levels: [1],
				buttState: 1
			);
			// Mode 2 is harmoMode 8, buttons 0 to 4 sets notes of chords.

			/////////////////////////////////////////////
			///////////// HARMO MODE POP ////////////////
			/////////////////////////////////////////////
			butts.newMode;
			// buttons 0 to 4 are not really modifiers.  They set the level
			// and then, the level is used to set the chord.
			butts.modes[harmoModePop].buttonFunctions[0].do{|i,j| i.makeModifier(j)};

			butts.modes[harmoModePop].buttonFunctions[9].enableNclick;

			// button 5 switches back to mode 0 (any level works).
			butts.modes[harmoModePop].addFunc(
				button: 5,
				function: {
					butts.mode_(0);
					("mode:" + 0).postln; 
					butts.mode.level_(0);
				}.inEnvir,
				levels: (0..15),
				buttState: 1
			);

			// button 9 starts the voice (this is for harmo mode).
			// Double click stops it (this is for harmo mode).
			butts.modes[harmoModePop].addFunc(
				button: 9,
				function: {
					\stop.post; h.postln;
					~voices.at(defName)[popVoice].stopMe;
				},
				levels: (0..15),
				buttState: 2
			);
			butts.modes[harmoModePop].addFunc(
				button: 9,
				function: {
					\play.post; h.postln;
					~voices.at(defName)[popVoice].playMe;
				},
				levels: (0..15),
				buttState: 1
			);

			butts.modes[harmoModePop].addFunc(
				button: 8,
				function: {
					\toggleRecording.post; h.postln;
					~voices.at(defName)[popVoice].looper.toggleRec;
				},
				levels: (0..15),
				buttState: 1
			);
			butts.modes[harmoModePop].addFunc(
				button: 8,
				function: {
					\stopPlayback.post; h.postln;
					~voices.at(defName)[popVoice].looper.stopPlayback;
				},
				levels: [1],
				buttState: 1
			);
		});
	};

	var close = {
		{ sponge.close; }.try;
	};

	(
		netAddress: { netAddress },
		sponge: { sponge },
		prep: { prep.value },
		butts: { butts },
		close: { close.value },
		defName: { defName }
	);
};

// ~s1 = ~cSponge.value("192.168.109.97", Server.default);
// ~s1.prep;
// ~s1.sponge.gui


