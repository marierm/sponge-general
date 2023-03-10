// withLists
// -> withSponge
//    -> withInterpolator
//       -> withAmp
//          -> withDamp
//             -> withChords
//                -> fm
//          -> granule

Proto({   // withLists
	// Do this when cloning:
	// ~prepList = ~prepList.deepCopy.addFunc({ });
	~server = Server.default;
	~prep = { ~prepList.value; };
	~asPattern = { ~asPatternList.value.last; };
	~stopCleanup = { ~stopCleanupList.value; };
	~freeCleanup = { ~freeCleanupList.value; };
	~changed = { |what ... args|
		what.post;
		args.postln;
	};
}).chuck(PR(\withLists));

PR(\withLists).clone({					// withSponge
	// ~spongeBP = \sponge_a;
	// ~sponge = BP(~spongeBP).sponge;
}).chuck(PR(\withSponge));

PR(\withSponge).clone({					// withInterpolator
	~interpolatorPath = nil;

	~prepList = ~prepList.deepCopy.addFunc({
		~interpolator = PresetInterpolatorServer.newLoad(
			~interpolatorPath
		);
	});
	~asPatternList = ~asPatternList.deepCopy.addFunc({
		~frozen = false;
		6.do({ |i|
			~interpolator.connect(i, ~sponge.features[i]);
		});
		1.asPattern;
	});
	~freezeMe = {
		6.do({ |i| ~interpolator.disconnect(i);	});
		~frozen = true;
	};
	~unfreezeMe = {
		~frozen.if({
			6.do({ |i|
				~interpolator.connect(i, ~sponge.features[i]);
			});
			~frozen = false;
		});
	};
	~stopCleanupList = ~stopCleanupList.deepCopy.addFunc({
		6.do({ |i|
			try { ~interpolator.disconnect(i) };
		});
	});
	~freeCleanupList = ~freeCleanupList.deepCopy.addFunc({
		~interpolator.free;
	});
}).chuck(PR(\withInterpolator));

PR(\withInterpolator).clone({ 			// withAmp
	// Provide ~ampBus to control amp
	// 
	~relTime = 0.2;
	~prepList = ~prepList.deepCopy.addFunc({
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

		~ampBus = Bus.control(s, 6);
		~globalAmpBus = Bus.control(s, 1);
		~group = Group.tail();
		// ~damp = ~sponge.damperBus.asMap;
	});
	~asPatternList = ~asPatternList.deepCopy.addFunc({
		~inputs = [
			\acc1xBP, \acc1yBP, \acc1zBP,
			\acc2xBP, \acc2yBP, \acc2zBP
		].collect({|i| ~sponge[i].bus});

		~damper.isPlaying.if({
			~ampExtractor.free;
		});

		~server.makeBundle(nil, {
			~ampExtractor = Synth.tail(Server.default, \ampExtractor, [
				\out, ~ampBus,
				\decayCoef, 0.9985
			]).map( *
				6.collect({|i| [i+1, ~inputs[i]]}).flat;
			);
			~globalAmp = Synth.after(~ampExtractor, \globalAmp, [
				\out, ~globalAmpBus,
				\inBus, ~ampBus
			]);
			
			NodeWatcher.register(~ampExtractor);
			NodeWatcher.register(~globalAmp);
		});
		1.asPattern;
	});
	~stopCleanupList = ~stopCleanupList.deepCopy.addFunc({
		// wait for ~relTime seconds before freeing ~ampExtractor.
		~ampExtractor.isPlaying.if({
			~server.sendBundle(
				~relTime,
				~ampExtractor.freeMsg,
				~globalAmp.freeMsg
			);
		});
	});
	~freeCleanupList = ~freeCleanupList.deepCopy.addFunc({
		~ampExtractor.free;
		~globalAmp.free;
		~ampBus.free;
		~globalAmpBus.free;
		~group.free;
	});
}).chuck(PR(\withAmp));

PR(\withAmp).clone({ 			// withDamp
	~damperCurve = 4;
	~damperMin = 0;
	~damperMax = 900;
	~prepList = ~prepList.deepCopy.addFunc({
		~damperBus = Bus.control(Server.default, 2);
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
		~damperInA = ~sponge[\fsr1].bus;
		~damperInB = ~sponge[\fsr2].bus;
	});

	~asPatternList = ~asPatternList.deepCopy.addFunc({
		~damper.isPlaying.if({
			~damper.free;
		});
		~server.makeBundle(nil,{
			~damper = Synth.after(~ampExtractor, \damper, [
				\out, ~damperBus.index,
				\min, ~damperMin,
				\max, ~damperMax,
				\curve, ~damperCurve
			]).map(
				\inA, ~damperInA,
				\inB, ~damperInB
			);
			// do not use damper.
			// ~ampExtractor.map(\decayCoef, ~damperBus);
			NodeWatcher.register(~damper);
		});
		1.asPattern;
	});

	~stopCleanupList = ~stopCleanupList.deepCopy.addFunc({
		// wait for ~relTime seconds before freeing
		~damper.isPlaying.if({
			~server.sendBundle(~relTime, ~damper.freeMsg );
		});
	});
	~freeCleanupList = ~freeCleanupList.deepCopy.addFunc({
		~damperBus.free;
	});
}).chuck(PR(\withDamp));

PR(\withDamp).clone({ 			// withPitch
	~notes = ([ \en1, \en2, \bn2].keynum.mirror2.clump(3)).flat;
	~numNotes = 6;
	// ~notes = [ \en6 ].keynum;
	// ~numNotes = 1;

	~octVa = { ~octVaList.value; };
	~octVb = { ~octVbList.value; };
	~octVaList = ~octVaList.deepCopy.addFunc({
		~notes = ~notes + 12;
		~pitcher.set(\base, ~notes);
		~changed.value(\octave, ~notes[0].noteOctave);
	});
	~octVbList = ~octVbList.deepCopy.addFunc({
		~notes = ~notes - 12;
		~pitcher.set(\base, ~notes);
		~changed.value(\octave, ~notes[0].noteOctave);
	});
	~octTmp = { |oct|
		~pitcher.set(\base, ~notes + (12 * oct));
	};
	~octReset = {
		~pitcher.set(\base, ~notes);
	};
	~prepList = ~prepList.deepCopy.addFunc({
		// Four controls for the pitch are combined:
		//  1. Base pitch: ~notes (midi keynum)
		//  2. Offset from sponge buttons: ~pitchOsBus (in semitones)
		//  3. Pitch bend: ~pbBus (in semitones)
		//  4. Fine pitch variations from interpolator: ~interpolFreqs

		// ~freqBuses hold the resultant.  
		SynthDef(\pitcher, {
			arg out = 0, bendRange=2;
			var base, offset, bend, interpol, freqs;
			base = NamedControl.kr(\base, ~notes);
			offset = NamedControl.kr(\offset, 0);
			bend = NamedControl.kr(\bend, 0);
			bend = bend.linlin(-pi, pi, bendRange * -1, bendRange); 
			interpol = NamedControl.kr(
				\interpol, [0,0,0,0,0,0]
			) * NamedControl.kr(
				\coefImpactHauteur, 1.0 // adjust the impact of interpolator (for experiment);
			);

			freqs = (base + offset + bend + interpol).midicps;
			ReplaceOut.kr(out, freqs);
		}).add;

		~pitchOsBus = Bus.control(s, 1);
		~freqBuses = Bus.control(s, ~notes.size);

		// This stores the index of the \interpol control input.
		// Is there a better way to do this?
		~interpolId = SynthDescLib.global[
			\pitcher
		].controls.collect(_.name).indexOf(\interpol);

		// ~pargroup = ParGroup.tail();
	});

	~asPatternList = ~asPatternList.deepCopy.addFunc({
		~pitcher.isPlaying.if({
			~pitcher.free;
		});

		~server.makeBundle(nil,{
			~pitcher = Synth.head(~group, \pitcher, [
				\out, ~freqBuses.index,
				\bendRange, 2,
				\base, ~notes,
				\bend, 0
			]).map(
				// \bend, ~sponge[\bend].bus,
				\offset, ~pitchOsBus
			);

			// map freq interpolator to pitcher synth
			// first 6 parameters in interpolator are pitch.
			// We use only the first one.
			~pitcher.map( *
				6.collect({|i|
					[~interpolId + i , ~interpolator.cursor.parameters[i].bus]
				}).flat;			
			);
			NodeWatcher.register(~pitcher);
		});
		1.asPattern;
	});

	~stopCleanupList = ~stopCleanupList.deepCopy.addFunc({
		// wait for ~relTime seconds before freeing
		~pitcher.isPlaying.if({
			~server.sendBundle(~relTime, ~pitcher.freeMsg );
		});
	});
	~freeCleanupList = ~freeCleanupList.deepCopy.addFunc({
		~pitcher.free;
		// ~pargroup.free;
	});
}).chuck(PR(\withPitch));

PR(\withPitch).clone({ 			// withChords
	// ~notes = ([ \en1, \en2, \bn2].keynum.mirror2.clump(3)).flat;
	~chords = ([
		~notes.deepCopy,
		[ \en2, \en3, \gn3, \dn4, \fs4, \bn4].keynum,
		[ \en2, \dn3, \fs3, \dn4, \fs4, \bn4].keynum,
		[ \en2, \cn3, \gn3, \dn4, \fs4, \bn4].keynum,
		[ \en2, \bn2, \an3, \dn4, \fs4, \bn4].keynum,

		[ \en2, \en3, \cn4, \dn4, \fs4, \bn4].keynum,
		[ \en2, \dn3, \gn3, \cn4, \fs4, \bn4].keynum,
		[ \en2, \cn3, \gn3, \bn3, \fs4, \bn4].keynum,
		[ \en2, \dn3, \fs3, \bn3, \en4, \an4].keynum
		// [ \en1, \bn1, \cn3, \dn4, \fs4, \bn4].keynum,
	]);

	~octVaList = ~octVaList.deepCopy.addFunc({
		// ~notes = ~notes + 12;
		~chords = ~chords + 12;
		// ~pitcher.set(\base, ~notes);
	});
	~octVbList = ~octVbList.deepCopy.addFunc({
		// ~notes = ~notes - 12;
		~chords = ~chords - 12;
		// ~pitcher.set(\base, ~notes);
	});

	~setChord = { |id|
		~notes = ~chords.clipAt(id);
		~pitcher.set(\base, ~notes);
		~changed.value(\chord, ~notes.note);
	};
}).chuck(PR(\withChords));
