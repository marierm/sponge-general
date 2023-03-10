PR(\abstractProcess).v.clone({
	~sponge = BP(\sponge).sponge;
	~out = 8;
	~nc=2; //number of audio channels
	~bufnc=2; // number of channels in audio files
	~files = (//just bassClar.
		Document.current.dir ++ "/samplesClari/loudestNormStereo/*Bass*"
	).pathMatch;
	~bufs = List[];
	~sineFreqs = [\dn1, \dn2].keynum.hertz;
	~prep = {
		SynthDef(\filter, {
			arg out=0, in=0, freq=1, mul=1.0;
			Out.ar(
				out,
				LPF.ar(
					In.ar(in, 2),
					Lag.kr(freq.linexp(0, 850, 22000, 80),0.05)
				),
				mul
			);
		}).add;

		//Granulator SynthDef;
		SynthDef(\granClarinet, {
			arg out=0, bufnum=0, pan=0, rate=1, amp=1, centerPos=0,
			granFreq=15, granDur=0.2, rndAmp=0.1, rndFreq=20, rndDC=0, mulFin = 0,
			gate=1, att=0.2;
			
			var trig;
			trig = Impulse.ar(granFreq);
			Out.ar(out,
				TGrains.ar(
					numChannels: ~nc,
					trigger: trig,
					bufnum: bufnum,
					rate: rate,
					centerPos: LFDNoise3.ar(rndFreq,rndAmp,centerPos + rndDC),
					dur: granDur,
					pan: pan,
					amp: amp,
					interp: 4
				) * (0.8 - mulFin.squared).clip *
				EnvGen.kr(Env.asr(att), gate, doneAction:2);
			)
		}).add;
		SynthDef(\sinus, {
			arg out=0, pan=0, amp=1, freq=73, mulFin = 0, gate=1, att=0.2;
			Out.ar(out,Pan2.ar(
				SinOsc.ar(freq, 0, amp * (0.8-mulFin.squared).clip),
				pan
			) * EnvGen.kr(Env.asr(att), gate, doneAction:2));
		},[0,0.1,0.1]).add;

		~files.do({|i|
			~bufnc.do({ |j|
				~bufs.add(Buffer.readChannel(Server.default, i, channels:[j]));
			});
		});
		~group = ParGroup(Server.default);
		~bus = Bus.audio(Server.default,2);
		~interpolator = PresetInterpolatorServer.load(
			Document.current.dir ++ "/clari_1.07.pri"
		);
		

	};

	~asPattern = {
		~claris.do(_.release(3));
		~sines.do(_.release(3));
		~filter = Synth.after(~group, \filter, [
			\out, ~out,
			\in, ~bus
		]).map(
			\freq, ~sponge[\fsr1].bus
		);
		
		~claris = Array.fill( ~bufs.size, {|i|
			Synth.head(~group, \granClarinet, [
				\out, ~bus,
				\pan, [-0.5,0.5,-0.5,0.5,-0.5,0.5,-0.5,0.5][i],
				\bufnum, ~bufs.collect(_.bufnum)[i],
				\att, 4
			]).map(
				// change bit shift on next line if ther are more
				// channels in buffers;
				// i>>1 if 2 chans, i>>2 if 4 chans.
				\centerPos, Array.fill(
					~bufs.size, {|i|
						~interpolator.cursor.parameters[i>>1].bus
					}
				)[i],
				\granFreq, ~interpolator.cursor.parameters[4].bus,
				\granDur, ~interpolator.cursor.parameters[5].bus,
				\rndAmp, ~interpolator.cursor.parameters[6].bus,
				\rndFreq, ~interpolator.cursor.parameters[7].bus,
				\rndDC, ~interpolator.cursor.parameters[8].bus,
				\amp, ~interpolator.cursor.parameters[10].bus
			)
		});
		~sines = Array.fill(2, {|i|
			Synth.head(~group, \sinus, [
				\out, ~bus,
				\pan, 0,
				\freq, ~sineFreqs[i],
				\att, 4
			]).map(
				\amp, ~interpolator.cursor.parameters[9].bus
			)
		});
		6.do({ |i|
			~interpolator.connect(i, ~sponge.features[i]);
		});
	};
	
	~freezeMe = {
		6.do({ |i|
			~interpolator.disconnect(i, ~sponge.features[i]);
		});
		\frozenClari.postln;
	};

	~stopCleanup = {
		~claris.do(_.release(3));
		~sines.do(_.release(3));
		Server.default.sendBundle(3.0, ~filter.freeMsg);
		~freezeMe.();
	};

	~freeCleanup = {
		~bus.free;
		~group.free;
		~bufs.do(_.free);
		~interpolator.free;
	};

}) => PR(\clarinet);

PR(\clarinet) => BP(\clari);