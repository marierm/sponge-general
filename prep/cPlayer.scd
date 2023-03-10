~cPlayer = {
	arg audioFilePath, cSponge, outBus=0, tetra=false, amp=1.0, attackTime = 10.0, relTime=1.0, ampCtl=false, playerId=880, server;
	var startPosB;
	var numFSR = 2;
	var bus, buf, feu, feu2;
	var ampBus, globalAmpBus, group, inputs, damper, ampExtractor, globalAmp;
	var damperCurve = 4, damperMin = 100, damperMax = 900;
	var damperBus, damperInA, damperInB;
	var sponge, defName;
	var looperBus, looperXfade, myLooper;

	var prep = {
		audioFilePath = audioFilePath.resolveRelative;
		startPosB = SoundFile.openRead(audioFilePath).numFrames / 2;
		server = server ? Server.default;
		sponge = cSponge.sponge;
		defName = cSponge.defName;
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
			id: playerId,
			outBus: outBus,
			numChan: 2,
			lag: looperXfade,
			bufDur: 60,
			server: server
		);
		myLooper.prep;

		SynthDef(\player, {|out=0, bufnum=0, rate=1, trig=1, sPos=0, lp=1, amp=1.0, gate=1.0, atk=10.0|
			Out.ar(
				out,
				PlayBuf.ar(
					numChannels: 2,
					bufnum: bufnum,
					rate: rate,
					trigger: trig,
					startPos: sPos,
					loop: lp
				) * EnvGen.kr(
					envelope: Env.asr(atk,1.0, 20.0),
					gate: gate,
					levelScale: 1.0,
					doneAction: 2
				) * amp;
			);
		}).add;

		buf = Buffer.read(Server.default, audioFilePath);

		// ~envBuf = Buffer.sendCollection(Server.default, Env.perc().asSignal(4096));

		bus = Bus.audio(Server.default, 2);
		group = ParGroup();
	};

	var playMe = {
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

			feu = Synth.tail(group, \player, [
				\out, outBus,
				\bufnum, buf.bufnum,
				\amp, amp,
				\atk, attackTime,
				\sPos, 0
			]);

			ampCtl.if({
				feu.map(\amp, globalAmpBus);
			});

			tetra.if({
				feu2 = Synth.tail(group, \player, [
					\out, outBus + 2,
					\bufnum, buf.bufnum,
					\amp, amp,
					\atk, attackTime,
					\sPos, startPosB
				]);

				ampCtl.if({
					feu2.map(\amp, globalAmpBus);
				});
				NodeWatcher.register(feu2);
			});
			
			NodeWatcher.register(ampExtractor);
			NodeWatcher.register(globalAmp);
			NodeWatcher.register(damper);
			NodeWatcher.register(feu);
			
		});
	};

	var stopMe = {
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

		{ feu.release(relTime); }.try;
		{ feu2.release(relTime); }.try;
	};

	var close = {
		ampExtractor.free;
		globalAmp.free;
		ampBus.free;
		globalAmpBus.free;
		group.free;
		damperBus.free;
		bus.free;
		buf.free
	};

	(
		audioFilePath: {audioFilePath },
        outBus: { outBus },
        tetra: { tetra },
        amp: { amp },
        attackTime: { attackTime },
        relTime: { relTime },
        ampCtl: { ampCtl },
        playerId: { playerId },
        server: { server },
		cSponge: { cSponge },
		sponge: { sponge },
		looper: { myLooper },
		prep: { prep.value },
		playMe: { playMe.value },
		stopMe: { stopMe.value },
		close: { close.value }
	)
};
