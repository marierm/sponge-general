PR(\withChords).clone({ //popper
	~out = 0;
	~tetra = false;
	~prepList = ~prepList.deepCopy.addFunc({
		~tetra.if({ // Quad
			SynthDef(\manualPops, {
				|out=0, outBus=30, dust=0, amp=1.0, rq=0.35, delRange=0.0017, fltMin=100, fltMax=8000|
				var posFront, posRear, busOffset;
				posFront = WhiteNoise.kr(1.0);
				posRear = WhiteNoise.kr(1.0);

				dust = DelayN.ar(
					dust,
					0.2, // Random position in space with delays.
					[
						posFront.clip(0,1.0) * delRange,
						posFront.clip(-1.0,0).abs * delRange,
						posRear.clip(-1.0,0).abs * delRange,
						posRear.clip(0,1.0) * delRange
					],
					amp
				);
				// dust = BPF.ar(dust, WhiteNoise.kr().range(fltMin,fltMax), rq );
				dust = BPF.ar(PinkNoise.ar(Decay.ar(dust,0.001)), WhiteNoise.kr().range(80,4000));
				dust = GVerb.ar(
					in:dust,
					roomsize:50,
					revtime:0.5,
					damping: 0.5,
					inputbw: 0.5,
					spread: 100,
					drylevel: 1,
					earlyreflevel: 0.005,
					taillevel:0.1
				);
				dust = LeakDC.ar(dust);
				Out.ar( outBus, dust);
				Out.ar( out, dust );
			}).add;
			~popBuses = ~numNotes.collect({
				Bus.audio(Server.default, 4);
			});
		}, { // Stereo
			SynthDef(\manualPops, {
				|out=0, outBus=30, dust=0, amp=1.0, rq=0.35, delRange=0.0017, fltMin=100, fltMax=8000|
				var pos, busOffset;
				pos = WhiteNoise.kr(1.0);
				
				dust = DelayN.ar(
					dust,
					0.2, // Random position in space with delays.
					[pos.clip(0,1.0) * delRange, pos.clip(-1.0,0).abs * delRange],
					amp
				);
				// dust = BPF.ar(dust, WhiteNoise.kr().range(fltMin,fltMax), rq );
				
				dust = BPF.ar(PinkNoise.ar(Decay.ar(dust,0.001)), WhiteNoise.kr().range(80,4000));
				dust = GVerb.ar(
					in:dust,
					roomsize:50,
					revtime:0.5,
					damping: 0.5,
					inputbw: 0.5,
					spread: 100,
					drylevel: 1,
					earlyreflevel: 0.005,
					taillevel:0.1
				);
				dust = LeakDC.ar(dust);
				
				Out.ar( outBus, dust);
				Out.ar( out, dust );
			}).add;
			~popBuses = ~numNotes.collect({
				Bus.audio(Server.default, 2);
			});
		});

		~trigFeatures = [
			\acc1xTrig, \acc1yTrig, \acc1zTrig, \acc2xTrig, \acc2yTrig, \acc2zTrig
		];

		~hpFeatures = [
			\acc1xHP, \acc1yHP, \acc1zHP, \acc2xHP, \acc2yHP, \acc2zHP
		];
		
		{
			(~hpFeatures ++ ~trigFeatures).do({|i|
				~sponge.activateFeature(i);
				Server.default.sync;
			});
		}.fork;
	});

	~asPatternList = ~asPatternList.deepCopy.addFunc({
		// make 6 manual pops: one for each note.
		// maybe send only the first one to the actual output.
		~manualPops = ~numNotes.collect({ |i|
			Synth.after(~pitcher, \manualPops, [
				\out, ~out,
				\outBus, ~popBuses[i]
			]).map(
				\dust, ~sponge[ ~trigFeatures[i] ].bus
			);
		});

		1.asPattern;
	});

	~stopCleanupList = ~stopCleanupList.deepCopy.addFunc({
		{
			~manualPops.do({|i|
				i.free;
			});
		}.try;
	});

	~freeCleanupList = ~freeCleanupList.deepCopy.addFunc({
		~popBuses.do({|i|
			i.free;
		});
	});

}).chuck(PR(\popper));

