// withLists
// -> withSponge
//    -> withInterpolator
//       -> withAmp
//          -> withDamp
//             -> fm
//          -> granule


PR(\withInterpolator).clone({ 			// withAmp
	~relTime = 0.2;
	~fsrMuteThreshold = 0.95;
	~prepList = ~prepList.deepCopy.addFunc({
		SynthDef(\ampExtractorFm, {
			arg out=0, in = #[0,0,0,0,0,0], threshold=0.95;
			var decay = \decayCoef.kr(0.999);
			Out.kr(
				out,
				Select.kr(
					decay > threshold, [
						0,
						EnvFollow.kr(in * 0.005, decay)
					]
				)
			);
		}).add;

		SynthDef(\globalAmpFm, {|out=0, inBus=0|
			var amp;
			amp = In.kr(inBus, 6).sum * 0.166667;
			Out.kr(out, amp);
		}).add;

		~ampBus = Bus.control(s, 6);
		~globalAmpBus = Bus.control(s, 1);
		~group = ParGroup.tail();
		// ~damp = ~sponge.damperBus.asMap;
	});
	~asPatternList = ~asPatternList.deepCopy.addFunc({
		~inputs = [
			\acc1xHP, \acc1yHP, \acc1zHP,
			\acc2xHP, \acc2yHP, \acc2zHP
		].collect({|i| ~sponge[i].bus});

		~damper.isPlaying.if({
			~ampExtractor.free;
		});


		~server.makeBundle(nil, {
			~ampExtractor = Synth.tail(Server.default, \ampExtractorFm, [
				\out, ~ampBus,
				\threshold, ~fsrMuteThreshold
			]).map( *
				6.collect({|i| [i+1, ~inputs[i]]}).flat;
			);
			~globalAmp = Synth.after(~ampExtractor, \globalAmpFm, [
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
}).chuck(PR(\withAmpFm));

PR(\withAmpFm).clone({ 			// withDamp
	~damperCurve = 4;
	~damperMin = 100;
	~damperMax = 900;
	~prepList = ~prepList.deepCopy.addFunc({
		~damperBus = Bus.control();
		SynthDef(\damperFm, {
			arg out=0, min=100, max=900, curve=4;
			Out.kr(
				out,
				\inA.kr(0).max(\inB.kr(0)).lincurve(
					min,max,0.99999,0.7, curve
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
			~damper = Synth.after(~ampExtractor, \damperFm, [
				\out, ~damperBus.index,
				\min, ~damperMin,
				\max, ~damperMax,
				\curve, ~damperCurve
			]).map(
				\inA, ~damperInA,
				\inB, ~damperInB
			);
			~ampExtractor.map(\decayCoef, ~damperBus);
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
}).chuck(PR(\withDampFm));

PR(\withDampFm).clone({ 			// withPitch
	~notes = ([ \en1, \en2, \bn2].keynum.mirror2.clump(3)).flat;
	~numNotes = 6;
	~octVa = { ~octVaList.value; };
	~octVb = { ~octVbList.value; };
	~octVaList = ~octVaList.deepCopy.addFunc({
		~notes = ~notes + 12;
		~pitcher.set(\base, ~notes);
		~changed.value(\octave, ~notes[0].noteOctave);
		NetAddr.localAddr.sendMsg("/" ++ ~defName, ~notes[0].noteOctave );
	});
	~octVbList = ~octVbList.deepCopy.addFunc({
		~notes = ~notes - 12;
		~pitcher.set(\base, ~notes);
		~changed.value(\octave, ~notes[0].noteOctave);
		NetAddr.localAddr.sendMsg("/" ++ ~defName, ~notes[0].noteOctave );
	});
	~octTmp = { |oct|
		~pitcher.set(\base, ~notes + (12 * oct));
	};
	~octReset = {
		~notes = ([ \en1, \en2, \en3].keynum.mirror2.clump(3)).flat + (0 * 12);
		~pitcher.set(\base, ~notes);
		NetAddr.localAddr.sendMsg("/" ++ ~defName, ~notes[0].noteOctave );
	};
	~prepList = ~prepList.deepCopy.addFunc({
		// Four controls for the pitch are combined:
		//  1. Base pitch: ~notes (midi keynum)
		//  2. Offset from sponge buttons: ~pitchOsBus (in semitones)
		//  3. Pitch bend: ~pbBus (in semitones)
		//  4. Fine pitch variations from interpolator: ~interpolFreqs

		// ~freqBuses hold the resultant.  
		SynthDef(\pitcherFm, {
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
			\pitcherFm
		].controls.collect(_.name).indexOf(\interpol);

		~pargroup = ParGroup.tail();
	});

	~asPatternList = ~asPatternList.deepCopy.addFunc({
		~pitcher.isPlaying.if({
			~pitcher.free;
		});

		~server.makeBundle(nil,{
			~pitcher = Synth.head(~pargroup,\pitcherFm, [
				\out, ~freqBuses.index,
				\bendRange, 2,
				\base, ~notes,
				\bend, 0
			]).map(
				// \bend, ~sponge[\bend].bus,
				\offset, ~pitchOsBus
			);

			// map freq interpolator to pitcher synth
			// ~pitcher.map( *
			// 	6.collect({|i|
			// 		[~interpolId + i , ~interpolator.cursor.parameters[i*2].bus]
			// 	}).flat;			
			// );
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
		~pargroup.free;
	});
}).chuck(PR(\withPitchFm));

PR(\withPitchFm).clone({ 			// withChords
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
}).chuck(PR(\withChordsFm));
