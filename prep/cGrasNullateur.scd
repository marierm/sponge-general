(
~cGrasNullateur = { |name='gras'|
	var numChan = 2; // Set to number of speakers.
	var envelopeSize = 2048, envs, envBufs;
	var fileNames, buffers, buf;
	var granSynth;
	var group = ParGroup.tail(Server.default);
	var liveRecMaxDur = 60; // 60 seconds max for live recording.
	var liveRecBufTmp;
	var inputNum = 0;
	var liveRecBuffers;
	var oscFunc;
	var prep =  {
		// A bunch of different envelopes to choose from.
		envs = [
			// 20 perc envelopes.
			Array.geom(20, 0.001, 0.001.reciprocal.pow(1/20)).collect({ |i|
				Env.perc(i, 1-i).asSignal(envelopeSize);
			}),
			// 20 reverse perc envelopes.
			Array.geom(20, 0.001, 0.001.reciprocal.pow(1/20)).collect({ |i|
				Env.perc(1-i, i, 1, 4).asSignal(envelopeSize);
			}),
			// 20 sine envelopes (with flat section of varying length in middle).
			Array.series(40, 0.04, 0.02).collect({ |i|
				Env([0,1,0], [i, 1-i],'sine').asSignal(envelopeSize);
			}),
			// 25 sine envelopes (with flat section of varying length in middle).
			// Last one is a hanning enveloppe.
			Array.series(25, 0.01, 0.02).collect({ |i|
				Env([0,1,1,0], [i, 1 - (i * 2), i],'sine').asSignal(envelopeSize);
			})
		].flatten;
		
		envBufs = envs.collect({ |env|
				Buffer.alloc(Server.default, envelopeSize);
		});

		envs.do({|env, i|
			envBufs[i].setn(0, env)
		});

		// Load test samples in buffers.
		fileNames = List[];
		buffers = "../media/*.wav".resolveRelative.pathMatch.collect({ |path|
			fileNames.add(path.basename); 
			Buffer.read(Server.default, path);
		});

		// mono buffer for live recording.
		liveRecBufTmp = Buffer.alloc(Server.default, Server.default.sampleRate * liveRecMaxDur, 1);
		liveRecBuffers = List[];

		buf = buffers[0];

		SynthDef(\grasNullateur, {
			arg granFreq=8, out=0, granDur=0.2, bufnum=0, rate=1,
			pos=0.5, panW=0.0, panC=0.0, amp=0.1, env= -1, gate=1,
			t_trig=1, trigSel=0;
			
			var autoTrig, chosenTrig, sig, pan, envelope;
			envelope = EnvGen.kr(Env.asr(0.01,1,0.1), gate, doneAction:2);
			autoTrig = Impulse.ar(granFreq);
			chosenTrig = Select.ar( trigSel, [K2A.ar(t_trig), autoTrig]);
			pan = TRand.ar( // Center and Width of pan
				lo: (panC - panW).clip(-1.0, 1.0),
				hi: (panC + panW).clip(-1.0, 1.0),
				trig: chosenTrig
			);

			
			sig = GrainBuf.ar(
				numChannels: 2,
				trigger: chosenTrig,
				dur: granDur,
				sndbuf: bufnum,
				rate: rate,
				pos: pos,
				interp: 2,
				pan: pan,
				envbufnum: env,
				rate: rate,
				maxGrains: 512,
				mul: amp,
				add: 0
			);
			Out.ar(out,	sig * envelope)
		}).add;

		SynthDef(\record, { 
			arg in=0, bufnum=0, gate=1, id=0;
			var phasor, onsets, chain, input, env, fadeTime=2.0;
			env = EnvGen.kr(Env.asr(fadeTime, 1.0, fadeTime), gate, doneAction:2);
			input = SoundIn.ar(in) * env;
			phasor = Phasor.ar(0, 1, 0, BufFrames.kr(bufnum));
			BufWr.ar(
				input,
				bufnum: bufnum,
				phase: phasor
			);
			SendReply.kr(1 - gate, "/buffer_pos", phasor, id);
		}).add;

		oscFunc = OSCFunc({ |msg|
			var nodeID, bufnum, bufPos, startPos, first;
			#nodeID, bufnum, bufPos = msg[1..3];
			startPos = (bufPos - ~frames) % ~liveRecBuf.numFrames;
			(bufnum == ~buf.bufnum and:	//make sure the message is for us.
				~liveRecSynth.nodeID == nodeID ).if ({ 
					~copyFunc.value(bufPos); // and copy.
					// Reposition onsets according to this ~buf
					~onsets = ((~tmpOnsets - startPos) % ~frames).sort;
					// "Onsets are at: ".post;
					// ~onsets.postln;
				});
		}.inEnvir, "/buffer_pos", Server.default.addr);

		currentEnvironment;
	};


	
	var playMe = {
		granSynth = Synth.tail(group, \grasNullateur, [
			\bufnum, buf
		]);
	};

	var startLiveRec = {
		recSynth = Synth.tail(group, \record, [
			\bufnum, liveRecBuf,
			\in, inputNum
		]);
	};

	var stopLiveRec = {
		recSynth.set(\gate, 0);
	};

	var stopMe = {
		granSynth.release;
	};

	var update = { arg envir, what ... args;	// respond to a change in a model (MVC)
		forkIfNeeded {
			Server.default.sync;
			args.postln;
			what.switch(
				\setBuffer, {
					var buf, na;
					#buf, name  = args;
					(na == name).if({
						buf = buf;
						granSynth.set(\bufnum, buf );
					});
				}
			)
		}
	};

	var close = {
		stopMe.value;
		envBufs.do(_.free);
	};

	(
		granSynth: { granSynth },
		prep: { prep.value },
		playMe: { playMe.value },
		stopMe: { stopMe.value },
		close: { close.value },
		update: { update.value },
		buffers: { buffers },
		buf: { buf }
	);
};
~gras = ~cGrasNullateur.value('gras');
~gras.prep;
)

