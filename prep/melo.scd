// withLists
// -> withSponge
//  -> withInterpolator
//   -> withAmp
//    -> withDamp
//     -> withPitch
//      -> withChords
//       -> popper
//        -> melo

PR(\popper).clone({
	~prepList = ~prepList.deepCopy.addFunc({
		~tetra.if({
			SynthDef(\rzn, {
				arg out=0, in=0, coef=0.4, decay=5, freqRec=440, amp=0.2;
				var sig, freq;
				freq = freqRec.reciprocal;
				freq = [freq, freq * 1.001];
				coef = [coef, coef * 0.99];
				sig = Pluck.ar(
					in:In.ar(in, 4),
					trig:In.ar(in, 4),
					maxdelaytime:0.5,
					delaytime:freq,
					decaytime:decay,
					coef: coef.clip(-0.999, 0.999),
					mul: amp
				);
				// sig = sig * \amps.kr( 0.1!6 );
				Out.ar(	out, LeakDC.ar(sig) );
			}).add;
		},{
			SynthDef(\rzn, {
				arg out=0, in=0, coef=0.4, decay=5, freqRec=440, amp=0.2;
				var sig, freq;
				freq = freqRec.reciprocal;
				freq = [freq, freq * 1.001];
				coef = [coef, coef * 0.99];
				sig = Pluck.ar(
					in:In.ar(in, 2),
					trig:In.ar(in, 2),
					maxdelaytime:0.5,
					delaytime:freq,
					decaytime:decay,
					coef: coef.clip(-0.999, 0.999),
					mul: amp
				);
				// sig = sig * \amps.kr( 0.1!6 );
				Out.ar(	out, LeakDC.ar(sig) );
			}).add;
		});

		~coefBus = Bus.control(Server.default, 2);

		SynthDef(\coefBlend, {
			arg out=0;
			Out.kr(
				out,
				\inA.kr(0) * \inB.kr([0,0])
			)
		}).add;
	});

	~asPatternList = ~asPatternList.deepCopy.addFunc({
		//Make sure everything is clean.
		~rzn.do({ |rzn|
			rzn.isPlaying.if({
				rzn.free;
				// ~pitcher.free;
			});
		});
		~coefBlend.isPlaying.if({
			~coefBlend.free;
		});

		~server.makeBundle(nil,{

			
			// One rzn for each note
			// ~manualPops Synths excite the rzns.
			~rzn = ~numNotes.collect({|i|
				Synth.after(~manualPops[i], \rzn, [
					// \out, ~bus.index,
					\out, 0,
					\in, ~popBuses[i]
					// \amp, 0.1
				]);

			});

			// Calculate the coef from fsrs and interpoltor.
			~coefBlend = Synth.before( ~rzn[0], \coefBlend, [
				\out, ~coefBus
			]);
			~coefBlend.map(
				\inA, ~interpolator.cursor.parameters.select({|i|
					i.name.beginsWith("coef")
				})[0].bus,
				\inB, ~damperBus
			);

			NodeWatcher.register(~coefBlend);
					
			

			~rzn.do({ |rzn, id|


				// Map output of pitcher to rzn freq.
				rzn.map( \freqRec, ~freqBuses.subBus(id) );
				// Map dampers (fsr) coef.
				(id < 3).if({
					rzn.map(\coef, ~coefBus.subBus(0));
				},{
					rzn.map(\coef, ~coefBus.subBus(1));
				});
					
				// // Map decay from interpolator to rzn
				// rzn.map( *
				// 	~interpolator.cursor.parameters.select({|i|
				// 		i.name.beginsWith("decay");
				// 	}).collect({|param|
				// 		[param.name.asSymbol, param.bus]
				// 	}).flat;
				// );
				// Map coef from interpolator to rzn
				// rzn.map( *
				// 	~interpolator.cursor.parameters.select({|i|
				// 		i.name.beginsWith("coef");
				// 	}).collect({ |param|
				// 		[param.name.asSymbol, param.bus]
				// 	}).flat;
				// );
				NodeWatcher.register(rzn);
			});
			

			// ~pitcher.map( *
			// 	~notes.size.collect({|i|
			// 		[~interpolId + i , ~interpolator.cursor.parameters[i*2].bus]
			// 	}).flat;			
			// );
			
		});
		1.asPattern;
	});

	~stopCleanupList = ~stopCleanupList.deepCopy.addFunc({
		fork {
			// 5.wait;
			~rzn.do({ |rzn|
				rzn.isPlaying.if({
					~rzn.free;
				});
			});
			~coefBlend.isPlaying.if({
				~coefBlend.free;
			});
		}
	});

	~freeCleanupList = ~freeCleanupList.deepCopy.addFunc({
		~coefBus.free;
		// ~bus.free;
	});
}).chuck(PR(\melo));