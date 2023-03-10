~cPopper = {
	arg interpolatorPath, cSponge, notes, outBus=0, tetra=false, amp=1.0, fsrMuteThreshold=0.85, popId=210, relTime=0.2, server;
	var interpolator, frozen = false, numInterpolatorDim = 6, numAccelAxes = 6;
	var numFSR = 2, numNotes;
	var ampBus, globalAmpBus, group, inputs, damper, ampExtractor, globalAmp;
	var damperCurve = 4, damperMin = 100, damperMax = 900;
	var damperBus, damperInA, damperInB;
	var sponge, defName;
	var looperBus, looperXfade, myLooper;
	var popBuses, trigFeatures, hpFeatures, popsOnly;


	var changed = { |what ... args|
		what.post;
		args.postln;
	};

	var prep = {
		server = server ? Server.default;
		interpolator = PresetInterpolatorServer.newLoad( interpolatorPath );
		sponge = cSponge.sponge;
		defName = cSponge.defName;
		numNotes = 1;
		SynthDef(\ampExtractor, {
			arg out=0, in = #[0,0,0,0,0,0];
			Out.kr(
				out,
				EnvFollow.kr(in * 0.005, \decayCoef.kr(0.999))
			);
		}).add;
		SynthDef(\globalAmp, {|out=0, inBus=0|
			var amp;
			amp = In.kr(inBus, 6).sum * 0.166667;
			Out.kr(out, amp);
		}).add;
		SynthDef(\damper, {
			arg out=0, min=0, max=900, curve=4;
			var sig;
			sig = [\inA.kr(0), \inB.kr(0)].max;
			Out.kr(
				out,
				[sig, sig].linlin(
					min,max,0.01,1.0
				);
			)
		}).add;
		// Four controls for the pitch are combined:
		//  1. Base pitch: ~notes (midi keynum)
		//  2. Offset from sponge buttons: ~pitchOsBus (in semitones)
		//  3. Pitch bend: ~pbBus (in semitones)
		//  4. Fine pitch variations from interpolator: ~interpolFreqs


		ampBus = Bus.control(s, 6);
		globalAmpBus = Bus.control(s, 1);
		group = Group.tail();
		damperBus = Bus.control(server, numFSR);
		damperInA = sponge[\fsr1].bus;
		damperInB = sponge[\fsr2].bus;

		looperBus = Bus.audio(server, 2);
		looperXfade = 1.0;
		myLooper = ~cLooper.value(
			inBus: looperBus,
			id: popId,
			outBus: outBus,
			numChan: 2,
			lag: looperXfade,
			bufDur: 60,
			server: server
		);
		myLooper.prep;

		tetra.if({ // Quad
			SynthDef(\popsOnly, {
				|out=0, outBus=30, dust=0, amp=1.0, rq=0.35, delRange=0.0017, fltMin=100, fltMax=8000, outToLooper|
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
				Out.ar( outToLooper, dust );
			}).add;
			popBuses = numNotes.collect({
				Bus.audio(server, 4);
			});
		}, { // Stereo
			SynthDef(\popsOnly, {
				|out=0, outBus=30, dust=0, amp=1.0, rq=0.35, delRange=0.0017, fltMin=100, fltMax=8000, outToLooper|
				var pos, busOffset;
				pos = WhiteNoise.kr(1.0);
				dust = DelayN.ar(
					dust,
					0.2, // Random position in space with delays.
					[pos.clip(0,1.0) * delRange, pos.clip(-1.0,0).abs * delRange],
					amp
				);
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
				Out.ar( outToLooper, dust );
			}).add;
			popBuses = numNotes.collect({
				Bus.audio(server, 2);
			});
		});

		trigFeatures = [
			\acc1xTrig, \acc1yTrig, \acc1zTrig, \acc2xTrig, \acc2yTrig, \acc2zTrig
		];

		hpFeatures = [
			\acc1xHP, \acc1yHP, \acc1zHP, \acc2xHP, \acc2yHP, \acc2zHP
		];
		
		{
			(hpFeatures ++ trigFeatures).do({|i|
				sponge.activateFeature(i);
				server.sync;
			});
		}.fork;
	};

	var playMe = {
		// interpolator stuff
		frozen = false;
		numInterpolatorDim.do({ |i|
			interpolator.connect( i, sponge.features[i] );
		});
		// Amplitude control stuff
		inputs = [
			\acc1xBP, \acc1yBP, \acc1zBP,
			\acc2xBP, \acc2yBP, \acc2zBP
		].collect({|i| sponge[i].bus});

		damper.isPlaying.if({
			ampExtractor.free;
			damper.free;
		});
		
		server.makeBundle(nil, {
			ampExtractor = Synth.tail(Server.default, \ampExtractor, [
				\out, ampBus,
				\decayCoef, 0.9985
			]).map( *
				6.collect({|i| [i+1, inputs[i]]}).flat;
			);
			globalAmp = Synth.after(ampExtractor, \globalAmp, [
				\out, globalAmpBus,
				\inBus, ampBus
			]);
			damper = Synth.after(ampExtractor, \damper, [
				\out, damperBus.index,
				\min, damperMin,
				\max, damperMax,
				\curve, damperCurve
			]).map(
				\inA, damperInA,
				\inB, damperInB
			);
			// make 6 manual pops: one for each note.
			// maybe send only the first one to the actual output.
			popsOnly = numNotes.collect({ |i|
				Synth.after(damper, \popsOnly, [
					\out, outBus,
					\outBus, popBuses[i],
					\outToLooper, looperBus,
					\amp, amp
				]).map(
					\dust, sponge[ trigFeatures[i] ].bus
				);
			});

			NodeWatcher.register(ampExtractor);
			NodeWatcher.register(globalAmp);
			NodeWatcher.register(damper);
			
		});
	};

	var freezeMe = {
		numInterpolatorDim.do({ |i| interpolator.disconnect(i);	});
		frozen = true;
	};

	var unfreezeMe = {
		frozen.if({
			numInterpolatorDim.do({ |i|
				interpolator.connect(i, sponge.features[i]);
			});
			frozen = false;
		});
	};

	var stopMe = {
		numInterpolatorDim.do({ |i|
			try { interpolator.disconnect(i) };
		});
		ampExtractor.isPlaying.if({
			// wait for ~relTime seconds before freeing ~ampExtractor.
			server.sendBundle(
				relTime,
				ampExtractor.freeMsg,
				globalAmp.freeMsg
			);
		});
		damper.isPlaying.if({
			// wait for ~relTime seconds before freeing
			server.sendBundle(relTime, damper.freeMsg );
		});

		{
			popsOnly.do({|i|
				i.free;
			});
		}.try;
	};

	var close = {
		interpolator.free;
		ampExtractor.free;
		globalAmp.free;
		ampBus.free;
		globalAmpBus.free;
		group.free;
		damperBus.free;
		popBuses.do({|i|
			i.free;
		});
	};

	(
		interpolatorPath: { interpolatorPath },
		cSponge: { cSponge },
		sponge: { sponge },
		popBuses: { popBuses },
		popsOnly: { popsOnly },
		looper: { myLooper },
		prep: { prep.value },
		playMe: { playMe.value },
		freezeMe: { freezeMe.value },
		unfreezeMe: { unfreezeMe.value },
		stopMe: { stopMe.value },
		close: { close.value }
	);
}
