PR(\abstractProcess).v.clone({
	~prep = {
		~sponge = BP(\sponge).sponge;

		SynthDef(\rod, {
			arg out=0, in=0, trig = 0, freq=440, freqMul=1.0,
			att=0.01, rel=1, decay=8.0, damp=1.0, amp=1.0, odd = 1.0, gate=1;
			var z, env;
			env = EnvGen.kr(Env.adsr(att,0,1,rel), gate);
			z = CombC.ar(
				in * env * 0.005,
				0.2,
				[ freq.reciprocal, freq.reciprocal * 1.01] * freqMul,
				decay * damp * odd,
				amp
			);
			FreeSelf.kr(TDelay.kr(Done.kr(env),6));
			Out.ar(out, z );
		}).add;

		SynthDef(\pitchBender, {
			arg out = 0, in = 0, bend = 0.0, range=2;
			var freqs, mul;
			freqs = In.kr(in, 6);
			range = range.midiratio;
			mul = bend.linlin(-pi, pi, 1/range, range);
			ReplaceOut.kr(out, freqs*mul);
		}).add;

		~bus = Bus.audio(s, 2);
		~freqsBus = Bus.control(s,6);
		~pbBus = Bus.control(s,6);

		~freqs = Pn(
			~freqsBus.numChannels.collect({|i|
				\c ++ (i + ~freqsBus.index).asSymbol
			});
		);

		~freqsBus.set(*[ \dn2, \an2, \dn3, \fs3, \an3, \dn4 ].hertz);
		~pargroup = ParGroup.tail();
		~dur = 0.01;
		~legato = 1.0;
		~decay = 8.0;
		~odd = 1.0;
		~damp = BP(\sponge).damperBus.asMap;
	};

	~asPattern = {
		~inputs = Pn([
			\acc1xHP, \acc1yHP, \acc1zHP,
			\acc2xHP, \acc2yHP, \acc2zHP
		].collect({|i| ~sponge[i].bus.asMap}));

		~pitchBend = Synth.head(~pargroup,\pitchBender, [
			\out, ~pbBus,
			\in, ~freqsBus
		]).map(
			\bend, ~sponge[\bend].bus;
		);
		
		~rod = PmonoArtic(
			// type: \set, 
			// \instrument, 
			\rod,
			\dur, BPStream(\dur),
			\legato, BPStream(\legato),
			\freq, BPStream(\freqs),
			\in, BPStream(\inputs),
			\group, BPStream(\pargroup),
			\damp, BPStream(\damp),
			\decay, BPStream(\decay),
			\odd, BPStream(\odd),
			\out, ~bus.index
		);
	};

	~stopCleanup = {
		~pitchBend.free;
	};

	~freeCleanup = {
		~bus.free;
		~pargroup.free;
	};

}) => PR(\rod);