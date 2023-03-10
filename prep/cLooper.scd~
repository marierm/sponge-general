~cLooper = {
	arg inBus, id=0, outBus=0, numChan=2, lag=1.0, bufDur=45, numLoops=30, server, defName;
	var isRec, isPlaying, recBuffers, bus;
	var recLev = 0.9, preLev = 0.9;
	var oscFunc, recorder, loopPlayers;
	var loopDur, nextLoopId=0, task, nextLoopToStop=0;
	var prep = {
		forkIfNeeded {
			isRec = false;
			isPlaying = Array.fill(numLoops, { false });
			loopPlayers = Array.fill(numLoops, nil);
			server = server ? Server.default;
			recBuffers = numLoops.collect({
				Buffer.alloc(
					server, // add 1 second to allow some latency. 
					server.sampleRate * (bufDur + lag + 1.0),
					numChan
				);
			});
			SynthDef(\loopPlayer, {
				arg out=0, bufnum=0, atk=5, rel=6, amp=1, gate=0, rate=1.0, lag=1.0, loopDur=10.0;
				var sig, env;
				env = EnvGen.kr(Env.asr(atk,1,rel, curve:\sqr), gate, amp, doneAction:2);
				sig = PlayBufCF.ar(
					numChan,
					bufnum,
					BufRateScale.kr(bufnum) * rate,
					trigger: LFPulse.ar(((loopDur/rate)).reciprocal),
					lag: lag);
				Out.ar(out, sig * env);
			}).add;
			bus = Bus.audio(s, 2);
			SynthDef(\loopRecorder, {
				arg in=0, buf=0, id=0, recLev=0.9, preLev=0.9, t_trig=0;
				var phasor;
				in = In.ar(in, numChan);
				phasor = Phasor.ar(0, 1, 0, BufFrames.kr(buf));
				RecordBuf.ar(
					in,
					bufnum: buf,
					offset: 0,
					recLevel: recLev,
					preLevel: preLev,
					run: 1,
					loop: 1,
					trigger: 1,
					doneAction: 0
				);
				SendTrig.kr(t_trig, id, phasor); // send duration of loop to sclang.
			}).add;
			server.sync;
			oscFunc = OSCFunc({ |msg| // Triggered when recording is stopped.
				var msgId;
				msgId = msg[2];
				(msgId == id).if({
					loopDur = msg[3] / server.sampleRate;
					loopPlayers[nextLoopId].release(0.2);
					loopPlayers[nextLoopId] = Synth.after(recorder, \loopPlayer, [
						\out, outBus,
						\bufnum, recBuffers[nextLoopId],
						\atk, 1,
						\amp, 1,
						\gate, 1,
						\rate, 1.0,
						\lag, lag
					]);
					loopPlayers[nextLoopId].set(\loopDur, loopDur);
					"Set loop dur ".post; nextLoopId.post; " to ".post; loopDur.postln;
					server.sendBundle(lag, recorder.freeMsg );
					isPlaying[nextLoopId] = true;
					nextLoopId = (nextLoopId + 1) % numLoops;
				});
			}.inEnvir, "/tr", server.addr);
			server.sync;
		};
		currentEnvironment;
	};

	var stopPlayback = { |dur=17|
		loopPlayers.do({ |i|
			i.release(dur);
		});
		isPlaying = Array.fill(numLoops, { false });
		server.makeBundle(dur, {  
			recBuffers.do({ |buffer| // zero the buffers;
				buffer.zero
			});
		});
		nextLoopId = 0;
	};

	var stopPlaybackSingleLoop = { |dur=1|
		isPlaying[nextLoopToStop].if({
			loopPlayers[nextLoopToStop].release(dur);
			isPlaying[nextLoopToStop] = false;
			server.makeBundle(dur, {  
				recBuffers[nextLoopToStop].zero;
			});
			nextLoopToStop = (nextLoopToStop + 1) % numLoops;
			(nextLoopToStop == nextLoopId).if({
				nextLoopToStop = 0;
				nextLoopId = 0;
			});
		},{
			"Trying to stop non-playing loop.".postln;
			"Reseting nextLoopToStop to zero.".postln;
			nextLoopToStop = 0;
			nextLoopId = 0;
		});
		// nextLoopId = 0;
	};

	var killAllLoops = {
		stopPlayback.value(0.5);
		"Stopping all loops.".postln;
	};

	var toggleRec = {
		// isPlaying.not.if({
		isRec.if({
			recorder.set(\t_trig, 1.0);
			"Stop rec loop ".post; nextLoopId.postln;
			isRec = false;
			task.stop;
			task.reset;

			NetAddr.localAddr.sendMsg("" +/+ defName +/+ "recording", 0);
			// task.postln;
		},{
			isPlaying[nextLoopId].if({
				"Trying to record loop num ".post; nextLoopId.post;
				", but it is already playing.".postln;
			}, {
				recorder = Synth.tail(server, \loopRecorder, [
					\in, inBus,
					\buf, recBuffers[nextLoopId],
					\id, id,
					\recLev, recLev,
					\preLev, preLev
				]);
				isRec = true;
				"Starting rec on buf ".post; nextLoopId.postln;
				NetAddr.localAddr.sendMsg("" +/+ defName +/+ "recording", 1);
				// Do not record for more than bufDur seconds.
				task.stop; task.reset; // this may not be necessary. // Double check.
				task = Task({
					bufDur.wait;
					recorder.set(\t_trig, 1.0);
					"Auto stopping loop rec.".postln;
					NetAddr.localAddr.sendMsg("" +/+ defName +/+ "recording", 0);
					isRec = false;
				}).start;
			});
		});
	};

	var freeMe = {
		recBuffers.do(_.free);
		recorder.free;
		loopPlayers.do(_.free);
		oscFunc.free;
	};

	(
		inBus: { inBus },
		id: { id },
		outBus: { outBus },
		numChan: { numChan },
		lag: { lag },
		bufDur: { bufDur },
		server: { server },
		isRec: { isRec },
		isPlaying: { isPlaying },
		recBuffers: { recBuffers },
		loopPlayers: { loopPlayers },
		bus: { bus },
		loopDur: { loopDur },
		prep: { prep.value },
		toggleRec: { toggleRec.value },
		stopPlayback: { |event, dur| stopPlayback.value(dur) },
		stopPlaybackSingleLoop: { stopPlaybackSingleLoop.value },
		killAllLoops: { killAllLoops.value },
		freeMe: { freeMe.value }
	);
}
