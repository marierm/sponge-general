// provides interpolator, amplitude, damping, pitch control, chords.
// Needs a path to an interpolator, a sponge, list of notes and list of chords.

~cFmManager = {
	arg interpolatorPath, cSponge, notes, chords, outBus=0, fsrMuteThreshold=0.85, fmId=20, relTime=0.2, server, withLoop=true;
	var interpolator, frozen = false, numInterpolatorDim = 6, numAccelAxes = 6;
	var numFSR = 2, numNotes;
	var ampBus, globalAmpBus, group, inputs, damper, ampExtractor, globalAmp;
	var damperCurve = 4, damperMin = 100, damperMax = 900;
	var damperBus, damperInA, damperInB;
	var pitcher, pitchOsBus, freqBuses, interpolId;
	var sponge, defName;
	var looperBus, looperXfade, myLooper;
	var fm;

	var changed = { |what ... args|
		what.post;
		args.postln;
	};

	var prep = {
		server = server ? Server.default;
		interpolator = PresetInterpolatorServer.newLoad( interpolatorPath );
		notes = notes ? ([ \en1, \en2, \bn2].keynum.mirror2.clump(3)).flat;
		sponge = cSponge.sponge;
		defName = cSponge.defName;
		numNotes = notes.size;
		chords = chords ? ([
			notes.deepCopy,
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
			amp = In.kr(inBus, numAccelAxes).sum * 0.166667;
			Out.kr(out, amp);
		}).add;
		SynthDef(\damperFm, {
			arg out=0, min=100, max=900, curve=4;
			var sig;
			sig = [\inA.kr(0), \inB.kr(0)].max;
			Out.kr(
				out,
				\inA.kr(0).max(\inB.kr(0)).lincurve(
					min,max,0.99999,0.7, curve
				);
			)
		}).add;
		// Four controls for the pitch are combined:
		//  1. Base pitch: ~notes (midi keynum)
		//  2. Offset from sponge buttons: ~pitchOsBus (in semitones)
		//  3. Pitch bend: ~pbBus (in semitones)
		//  4. Fine pitch variations from interpolator: ~interpolFreqs

		// ~freqBuses hold the resultant.  
		SynthDef(\pitcherFm, {
			arg out = 0, bendRange=2;
			var base, offset, bend, interpol, freqs;
			base = NamedControl.kr(\base, notes);
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
		// fm synthesizer
		SynthDef(\fm, {
			arg out = 0, gate=1, att=0.3, outToLooper=100;
			var ctls, mods, sig, in;
			ctls = Array.fill(6, {|i|
				[
					(\freq++i).asSymbol.kr(220),
					0,
					(\amp++i).asSymbol.kr(1)
				]
			});
			mods = Array.fill2D(6, 6, {|x,y|
				var name;
				name = ("mod" ++ x ++ "_" ++ y).asSymbol;
				NamedControl.kr(name, 0);
			});
			sig = FM7.ar(ctls, mods);
			sig = sig * \amps.kr( 0.1!6 );
			sig = Splay.arFill( numNotes, { |i|
				sig[i] * EnvGen.kr(
					Env.asr(att,0.1),gate, doneAction:2
				)
			});
			Out.ar(	out, sig );
			Out.ar( outToLooper, sig);
		}).add;


		ampBus = Bus.control(s, numAccelAxes);
		globalAmpBus = Bus.control(s, 1);
		group = Group.tail();
		damperBus = Bus.control(server, numFSR);
		damperInA = sponge[\fsr1].bus;
		damperInB = sponge[\fsr2].bus;
		pitchOsBus = Bus.control(server, 1);
		freqBuses = Bus.control(server, notes.size);
		
		looperBus = Bus.audio(server, 2);
		looperXfade = 1.0;
		withLoop.if{ // If this is the master, allow loops.
			myLooper = ~cLooper.value(
				inBus: looperBus,
				id: fmId,
				outBus: outBus,
				numChan: 2,
				lag: looperXfade,
				server: server
			);
			myLooper.prep;
		};
		// This stores the index of the \interpol control input.
		// Is there a better way to do this?
		interpolId = SynthDescLib.global[
			\pitcherFm
		].controls.collect(_.name).indexOf(\interpol);
	};
	var playMe = {
		// interpolator stuff
		frozen = false;
		numInterpolatorDim.do({ |i|
			interpolator.connect( i, sponge.features[i] );
		});
		
		// Amplitude control stuff
		inputs = [
			\acc1xHP, \acc1yHP, \acc1zHP,
			\acc2xHP, \acc2yHP, \acc2zHP
		].collect({|i| sponge[i].bus});
		damper.isPlaying.if({
			ampExtractor.free;
			damper.free;
		});
		pitcher.isPlaying.if({ pitcher.free; });
		fm.isPlaying.if({ fm.free; });
		server.makeBundle(nil, {
			ampExtractor = Synth.tail(Server.default, \ampExtractorFm, [
				\out, ampBus,
				\threshold, fsrMuteThreshold
			]).map( *
				numAccelAxes.collect({|i| [i+1, inputs[i]]}).flat;
			);
			globalAmp = Synth.after( ampExtractor, \globalAmpFm, [
				\out, globalAmpBus,
				\inBus, ampBus
			]);
			damper = Synth.after(ampExtractor, \damperFm, [
				\out, damperBus.index,
				\min, damperMin,
				\max, damperMax,
				\curve, damperCurve
			]).map(
				\inA, damperInA,
				\inB, damperInB
			);
			ampExtractor.map(\decayCoef, damperBus);
			pitcher = Synth.head(group, \pitcherFm, [
				\out, freqBuses.index,
				\bendRange, 2,
				\base, notes,
				\bend, 0
			]).map(
				// \bend, ~sponge[\bend].bus,
				\offset, pitchOsBus
			);
			// map freq interpolator to pitcher synth
			// first 6 parameters in interpolator are pitch.
			// We use only the first one.
			pitcher.map( *
				numNotes.collect({|i|
					[interpolId + i , interpolator.cursor.parameters[i*2].bus]
				}).flat;			
			);
			fm = Synth.after(pitcher, \fm, [
				// \out, ~bus.index,
				\out, outBus,
				\outToLooper, looperBus
				// \amp, 0.1
			]);

			fm.map( \amps, ampBus);
			
			// Map output of pitcher to fm freqs.
			fm.map( *
				notes.size.collect({|i|
					[ (\freq ++ i).asSymbol, freqBuses.subBus(i) ]
				}).flat;
			);

			// Map interpolator amps to fm amps
			fm.map( *
				interpolator.cursor.parameters.select({|i|
					i.name.beginsWith("amp");
				}).collect({|param|
					[param.name.asSymbol, param.bus]
				}).flat;
			);

			// Map interpolator mods to fm mods
			fm.map( *
				interpolator.cursor.parameters.select({|i|
					i.name.beginsWith("mod");
				}).collect({ |param|
					[param.name.asSymbol, param.bus]
				}).flat;
			);

			pitcher.map( *
				notes.size.collect({|i|
					[interpolId + i , interpolator.cursor.parameters[i*2].bus]
				}).flat;			
			);


			NodeWatcher.register(ampExtractor);
			NodeWatcher.register(globalAmp);
			NodeWatcher.register(damper);
			NodeWatcher.register(pitcher);
			NodeWatcher.register(fm);

			NetAddr.localAddr.sendMsg("/" ++ defName, notes[0].noteOctave );
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
	var octVa = {
		notes = notes + 12;
		chords = chords + 12;
		pitcher.set(\base, notes);
		changed.value(\octave, notes[0].noteOctave);
		NetAddr.localAddr.sendMsg("/" ++ defName, notes[0].noteOctave );
	};
	var octVb = {
		notes = notes - 12;
		chords = chords - 12;
		pitcher.set(\base, notes);
		changed.value(\octave, notes[0].noteOctave);
		NetAddr.localAddr.sendMsg("/" ++ defName, notes[0].noteOctave );
	};
	var octTmp = { |oct|
		pitcher.set(\base, notes + (12 * oct));
	};
	var octReset = {
		notes = ([ \en1, \en2, \en3].keynum.mirror2.clump(3)).flat + (0 * 12);
		pitcher.set(\base, notes);
		NetAddr.localAddr.sendMsg("/" ++ defName, notes[0].noteOctave );
	};
	var setChord = { |id|
		notes = chords.clipAt(id);
		pitcher.set(\base, notes);
		changed.value(\chord, notes.note);
		NetAddr.localAddr.sendMsg("/" ++ defName, notes[0].noteOctave );
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
		pitcher.isPlaying.if({
			// wait for ~relTime seconds before freeing
			server.sendBundle(relTime, pitcher.freeMsg );
		});
		fm.isPlaying.if({
			fm.release(relTime);
		});
	};
	var close = {
		interpolator.free;
		ampExtractor.free;
		globalAmp.free;
		ampBus.free;
		globalAmpBus.free;
		group.free;
		damperBus.free;
		pitcher.free;
	};

	(
		interpolatorPath: { interpolatorPath },
		interpolator: { interpolator },
		cSponge: { cSponge },
		sponge: { sponge },
		fm: { fm },
		pitchOsBus: { pitchOsBus },
		looper: { myLooper },
		freqBuses: { freqBuses },
		prep: { prep.value },
		playMe: { playMe.value },
		freezeMe: { freezeMe.value },
		unfreezeMe: { unfreezeMe.value },
		octVa: { octVa.value },
		octVb: { octVb.value },
		octReset: { octReset.value },
		setChord: { |self, id| setChord.value(id); },
		stopMe: { stopMe.value },
		close: { close.value }
	);
}
