// formK
PR(\abstractProcess).v.clone({
	~prep = {
		~sponge = BP(\sponge).sponge;
		~interpolator = PresetInterpolatorServer.load(
			// Document.current.dir ++ "/preset6D.pri"
			Document.current.dir ++ "/formants_v3.pri"
		);
	};

	~asPattern = {
		// ~sponge.connect(~interpolator);
		6.do({ |i|
			~interpolator.connect(i, ~sponge.features[i]);
		});
	};

	~stopCleanup = {
		// ~sponge.disconnect(~interpolator);
		6.do({ |i|
			~interpolator.disconnect(i);
		});
	};

	~freeCleanup = {
		~bus.free;
		~interpolator.free;
	};
}) => BP(\formK);

// formants
PR(\abstractProcess).v.clone({
	~out = 0;
	~voiceNum = 0;

	~prep = {
		~sponge = BP(\sponge).sponge;
		~kBuses = BP(\formK).interpolator.cursor.parameters.collect({|i| i.bus});
		~rod = BP(~voiceNum);

		SynthDef(\formants, {
			arg out = 0, in0 = 0, gate=1, att=0.3, amp=0.1,
			formFreqs = #[80, 400, 750, 1100, 1400],
			formAttacks = #[0.001, 0.001, 0.001, 0.001, 0.001],
			formReleases = #[0.02, 0.02, 0.02, 0.02, 0.02];
			var sig;
			sig = Formlet.ar(
				In.ar(in0, 2),
				[ formFreqs, formFreqs ],
				[formAttacks],
				[formReleases]
			);
			Out.ar(out, sig * EnvGen.kr(Env.asr(att,amp), gate, doneAction:2));
		}).add;
	};
	
	~asPattern = {
		~forms.release(3);
		~forms = Synth.after(~rod.pargroup, \formants, [
			\out, ~out,
			\in0, ~rod.bus,
			\att, 2
		]).map(*15.collect({|i| [i+5,~kBuses[i]]}).flat;);
	};

	~freezeMe = {
		~kBuses.do({|bus,j|
			bus.get({ |val| ~forms.set(j+5, val); }.inEnvir);
		});
	};

	~stopCleanup = {
		~forms.release(3);
	};

	~freeCleanup = {
		~forms.free;
	};

}) => PR(\formants);

// BP(\formK).free;
// BP(\formants).free;

PR(\abstractProcess).v.clone({
	~voiceNum = 0;

	~prep = {
		~rod = BP(~voiceNum);
		SynthDef(\disto, {
			arg out = 0, in = 0, gain=2.0, att=0.01, amp=1.0, gate=1;
			var sig;
			sig = In.ar(in, 2) * gain;
			sig = sig.distort;
			ReplaceOut.ar(out, sig * EnvGen.kr(Env.asr(att,amp), gate, doneAction:2));
		}).add;
	};
	
	~asPattern = {
		~disto.release(3);
		~disto = Synth.after(~rod.pargroup, \disto, [
			\out, ~rod.bus,
			\in, ~rod.bus,
			\gain, [1.0,1.0,1.0,30.0][~voiceNum]
		]);
	};

	~freezeMe = {
	};

	~stopCleanup = {
		~disto.release(3);
	};

	~freeCleanup = {
		~disto.free;
	};

}) => PR(\disto);