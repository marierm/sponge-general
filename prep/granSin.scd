PR(\withPitch).clone({
	~out = 0;
	~prepList = ~prepList.deepCopy.addFunc({

		~envBuffers = [
			Env.sine(),
			Env.triangle(),
			Env.perc(0.01, 1, 1, -4),
			Env.perc(0.001, 1, 1, -4),
			Env.perc(0.01, 1, 1, -8),
			Env.perc(0.001, 1, 1, -8),
			Env.perc(1, 0.01, 1, 4)		// reverse
		].collect({|i|
			Buffer.sendCollection(Server.default, i.asSignal(4096))
		});

		SynthDef(\granSin, {|out=0|
			var trig, envsBuf, envChoice1, envChoice2, env1, env2;
			trig = \trigger.ar(0.0);
			// Choose the envelope buffer
			envsBuf = LocalBuf.newFrom(~envBuffers.collect(_.bufnum));
			envChoice1 = \envelope1.kr(0);
			envChoice2 = \envelope2.kr(2);
			env1 = IndexL.ar(envsBuf, envChoice1);
			env2 = IndexL.ar(envsBuf, envChoice2);
			Out.ar(
				out,
				GrainSin.ar(
					numChannels: 2, 
					trigger: trig,
					dur: \dur.kr(0.01),
					freq: TExpRand.ar(
						lo: \freqLo.kr(220),
						hi: \freqHi.kr(440),
						trig: trig
					),
					pan: WhiteNoise.ar(\pan.kr(0.0)),
					envbufnum: TWChoose.ar(
						trig,
						[env1, env2],
						[envChoice1.frac, 1 - envChoice1.frac]
					),
					maxGrains: 4096,
					mul: \amp.kr(0.1) * \ampComp.kr(1.0)
				)
			);
		}).add;

		// Generate triggers at freq.  Range is the maximum amplitude of the
		// modulation of freq.  The modulation is random within that range.
		// The range is expressed in semitones.

		// The amp input has an effect on the density of grains, but only when
		// the freq is not precise (when range is high).  The higher range is,
		// the more freq is affected by amp.
		SynthDef(\trigGenerator, {|out=100|
			var fr, ratio, amp, range;
			fr = \freq.kr(220);
			range = \range.kr(0);
			ratio = range.midiratio;
			amp = \amp.kr(0.1);
			fr = amp.lincurve(
				0.0, 1.0,				// in min & max
				4.0, fr,				// out min & max
				range.lincurve(
					0,96,
					\curveMin.kr(-1000),
					\curveMax.kr(0),
					\curveCurve.kr(-10)
				)							// curve
			);
			Out.ar(
				out,
				TDuty.ar(
					Dwhite().exprange(
						ratio / fr,
						1 / (fr * ratio)
					),
					0,
					amp
				)
			)
		}).add;


		// (
		// 	(0,6..96).do({|i|
		// 		(1.0,0.99 .. 0.01).collect(_.lincurve(
		// 			0.0,1.0,4.0,100,i.lincurve(0,96,-1000,0,-10)
		// 		)).plot
		// 	});
		// )

		~trigBus = Bus.audio(Server.default, 1);
		~bus = Bus.audio(Server.default, 2);

	});

	~asPatternList = ~asPatternList.deepCopy.addFunc({
		~server.makeBundle(nil,{

			~trigGenerator = Synth.after(~pitcher, \trigGenerator, [
				\out, ~trigBus
			]).map(
				\range, ~interpolator.cursor.parameters[0].bus
			);

			// Map output of pitcher to trigger freq.
			~trigGenerator.map(	\freq, ~freqBuses );

			~trigGenerator.map( \amp, ~globalAmpBus );

			~granSin = Synth.after(~trigGenerator, \granSin, [
				\out, ~out
			]);

			~granSin.map(
				\trigger, ~trigBus,
				\dur, ~interpolator.cursor.parameters[1].bus,
				\freqLo, ~interpolator.cursor.parameters[2].bus,
				\freqHi, ~interpolator.cursor.parameters[3].bus,
				\pan, ~interpolator.cursor.parameters[4].bus,
				\ampComp, ~interpolator.cursor.parameters[5].bus,
				\envelope1, ~interpolator.cursor.parameters[7].bus,
				\envelope2, ~interpolator.cursor.parameters[8].bus
			);

			~granSin.map( \amp, ~globalAmpBus );

			// Map pitchbend from interpolator to pitcher
			~pitcher.map(
				~interpolId, ~interpolator.cursor.parameters[6].bus
			);

			NodeWatcher.register(~granSin);
			NodeWatcher.register(~trigGenerator);
		});
		1.asPattern;
	});

	~stopCleanupList = ~stopCleanupList.deepCopy.addFunc({
		~granSin.isPlaying.if({
			~granSin.free;
			~trigGenerator.free;
		});
	});

	~freeCleanupList = ~freeCleanupList.deepCopy.addFunc({
		~bus.free;
		~envBuffers.do(_.free);
	});

}).chuck(PR(\granSin));

