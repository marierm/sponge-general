PR(\withDamp).clone({
	~audioFilePath = "../samples/05-feu.wav".resolveRelative;
	~startPosB = SoundFile.openRead(~audioFilePath).numFrames / 2;
	~tetra = false;
	~amp = 1.0;
	~out = 0;
	~attackTime = 10.0;
	~ampCtl = false;
	~prepList = ~prepList.deepCopy.addFunc({
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

		~buf = Buffer.read(Server.default, ~audioFilePath);

		// ~envBuf = Buffer.sendCollection(Server.default, Env.perc().asSignal(4096));

		~bus = Bus.audio(Server.default, 2);
		~group = ParGroup();
	});

	~asPatternList = ~asPatternList.deepCopy.addFunc({
		~feu = Synth.tail(~group, \player, [
			\out, ~out,
			\bufnum, ~buf.bufnum,
			\amp, ~amp,
			\atk, ~attackTime,
			\sPos, 0
		]);

		~ampCtl.if({
			~feu.map(\amp, ~globalAmpBus);
		});

		~tetra.if({
			~feu2 = Synth.tail(~group, \player, [
				\out, ~out + 2,
				\bufnum, ~buf.bufnum,
				\amp, ~amp,
				\atk, ~attackTime,
				\sPos, ~startPosB
			]);

			~ampCtl.if({
				~feu2.map(\amp, ~globalAmpBus);
			});

		});

		1.asPattern;
	});

	~stopCleanupList = ~stopCleanupList.deepCopy.addFunc({
		{ ~feu.release(~relTime); }.try;
		{ ~feu2.release(~relTime); }.try;
	});

	~freeCleanupList = ~freeCleanupList.deepCopy.addFunc({
		~bus.free;
		~group.free;
		~buf.free;
	});

}).chuck(PR(\player));

