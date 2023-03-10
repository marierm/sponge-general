//  This is the version for children/installations.
~cSponge = {
	arg ip, defName = \sponge01, server;
	var numBits = 5;
	// This table is used to invert the bit order of the buttons.  This is
	// useful when controling pitch with the buttons.
	var conversionTable = (0..(2.pow(numBits) - 1)).collect({|i|
		i.asInteger.asBinaryDigits(numBits).reverse.convertDigits(2)
	});
	var netAddress, sponge, butts;
	var pitchMode = 0;
	var fmVoice = 0;

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
				// \acc1xBP, \acc1yBP, \acc1zBP,
				// \acc2xBP, \acc2yBP, \acc2zBP, 
				// \bend
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
			// butts.modes[0].buttonFunctions[9..5].do{|i,j|
			// 	i.functions_( // Right hand, finger 1.
			// 		Dictionary.newFrom([
			// 			1, Array.newClear(32).putEach([0,1],[
			// 				{ // When no modifier key is pressed.
			// 					\stop.post; j.postln;
			// 					~voices.at(defName)[j].stopMe;
			// 				},
			// 				{ //  When left hand, key 1 is held.
			// 					\play.post; j.postln;
			// 					~voices.at(defName)[j].playMe;
			// 				},
			// 			]),
			// 			0, Array.newClear(32).putEach([8, 16],[
			// 				{ // When left hand, key 4 is held.
			// 					("mode harmo " + (j + 1)).postln;
			// 					butts.mode_((j*2) + 2); // modes 2, 4, 6, ...
			// 				},
			// 				{ // When left hand, key 5 is held.
			// 					("mode pitch" + (j + 1)).postln;
			// 					butts.mode_((j*2) + 1); // modes 1, 3, 5, ...
			// 				},
			// 			])
			// 		]);
			// 	)
			// };
			// // in mode 0, buttons 0 to 4 are modifier keys.
			// butts.modes[0].buttonFunctions[4..0].do{|i,j| i.makeModifier(j)};


			//////////////////////////////////////
			///////////// PITCH MODE /////////////
			//////////////////////////////////////
			// In pitchMode (1), buttons 0 to 4 set the pitch.
			// butts.newMode; // pitch mode is created
			// buttons 0 to 4 are not really modifiers.  They set the level
			// and then, the level is used to set the pitch.
			// butts.modes[pitchMode].buttonFunctions[0..4].do{|i,j| i.makeModifier(j)};
			butts.modes[pitchMode].buttonFunctions[0..4].do{|i,j| i.makeModifier(j)};

			// Enable double clicks for button 9.
			// butts.modes[pitchMode].buttonFunctions[8].enableNclick;
			butts.modes[pitchMode].buttonFunctions[9].enableNclick;

			// for each button(0 to 4), add function to level 0.
			(0..4).do{|i,j|
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


			// button 5 stops fm.
			butts.modes[pitchMode].addFunc(
				button: 5,
				function: {
					("Stop fm pitch" + defName).postln;
					~voices.at(defName)[fmVoice].stopMe;
					// \play.post; h.postln;
					// ~voices.at( (~defName ++ 0).asSymbol ).playMe(0,doReset: true);
				},
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
			////////////////////////////
			// ////////////////////// //
			////////////////////////////
		
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


