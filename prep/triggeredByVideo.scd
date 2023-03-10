(
// Keep track of number of noiseAccu synths to avoid glitches.
~noiseAccuGroup = Group.tail(Server.default); // all noiseAccus go in a group.
~numNoiseAccu = 0;
// Receives messages from the server that reply the number of synths in noiseAccuGroup.
OSCdef(\numNoiseAccu, {|...msg| ~numNoiseAccu = msg[0][3];}, "/g_queryTree.reply");

OSCdef(\sinusDidactique, { |msg, time, src|
	var x;
	// First argument is an index (of the projected sine wave).
	msg.postln;
	msg[1].switch(
		0, {
			//
			s.sendMsg("/g_queryTree", ~noiseAccuGroup.nodeID, 0);
			// (s.numSynths < 300).if({
				Synth.tail(~noiseAccuGroup, \noiseAccu, [
					\ffreq, msg[2].linexp(0,1920, 300,8000),
					\amp, msg[2].linexp(0,1920, 0.5,0.05),
					\rq, msg[2].linexp(0,1920, 10, 0.4),
					\startDens, 10000,
					\endDens, 0.001,
					\durDens, ~numNoiseAccu.linexp(0,500,6.0,0.1),
					\out, 8
				]);
			// });
		},
		1, {
			//
			s.sendMsg("/g_queryTree", ~noiseAccuGroup.nodeID, 0);
			// (s.numSynths < 300).if({
			Synth.tail(~noiseAccuGroup, \noiseAccu, [
				\ffreq, msg[2].linexp(0,1920, 300,8000),
				\amp, msg[2].linexp(0,1920, 0.5,0.05),
				\rq, msg[2].linexp(0,1920, 10, 0.4),
				\startDens, 10000,
				\endDens, 0.001,
				\durDens, ~numNoiseAccu.linexp(0,500,6.0,0.1),
				\out, 8
			]);
			// });
		},
		2, {
			//
			s.sendMsg("/g_queryTree", ~noiseAccuGroup.nodeID, 0);
			// (s.numSynths < 300).if({
			Synth.tail(~noiseAccuGroup, \noiseAccu, [
				\ffreq, msg[2].linexp(0,1920, 300,8000),
				\amp, msg[2].linexp(0,1920, 0.5,0.05),
				\rq, msg[2].linexp(0,1920, 10, 0.4),
				\startDens, 10000,
				\endDens, 0.001,
				\durDens, ~numNoiseAccu.linexp(0,500,6.0,0.1),
				\out, 8
			]);
			// });
		},
		3, {
			//
			s.sendMsg("/g_queryTree", ~noiseAccuGroup.nodeID, 0);
			// (s.numSynths < 300).if({
			Synth.tail(~noiseAccuGroup, \noiseAccu, [
				\ffreq, msg[2].linexp(0,1920, 300,8000),
				\amp, msg[2].linexp(0,1920, 0.5,0.05),
				\rq, msg[2].linexp(0,1920, 10, 0.4),
				\startDens, 10000,
				\endDens, 0.001,
				\durDens, ~numNoiseAccu.linexp(0,500,6.0,0.1),
				\out, 8
			]);
			// });
		}		
	);
}, "/video_trig_s", nil, 51127);

OSCdef(\cercleCycliste, { |msg, time, src|
	var x;
	// First argument is an index (of the projected sine wave).
	msg[1].switch(
		0, {
			Synth.tail(s, \fmForCercleCycliste, [
				\freq, msg[2].linexp(0,1920, 30, 90).midicps,
				\amp, msg[2].linexp(0,1920, 0.08,0.05),
				\release, 1.0,
				\out, 8
			]);
		}
	);
}, "/video_trig_c", nil, 51127);

SynthDef(\noiseAccu, {
	arg startDens=1000, endDens=1, durDens=0.3, ffreq=440, rq=1, amp=0.1, out=0;
	var sig;
	sig = Dust2.ar(
		XLine.kr(startDens, endDens, durDens, doneAction:2).dup(4)
	);
	sig = RLPF.ar(sig, ffreq, rq);
	Out.ar(out, sig*amp);
}).add;

SynthDef(\toneFeuille, {
	arg freq=440, amp=0.1, dur=2.0;
	var sig, env, freqs, freqMuls, ampMuls,noise, noise2, numHarmo=20, numChan=4;
	var maxDelay = 0.1, delays;
	noise =  {LFNoise1.kr(40).range(0.98,1.02)}.dup(numHarmo*numChan).clump(numHarmo);
	noise2 =  {LFNoise1.kr(40).range(0.5,1.5)}.dup(numHarmo*numChan).clump(numHarmo);
	freqMuls = {Array.series(numHarmo, 1, 1)}.dup(numChan);
	ampMuls = {Array.geom(numHarmo, 1.0, 0.8)}.dup(numChan);
	delays = {Array.fill(numHarmo, {maxDelay.rand})}.dup(numChan);
	XLine.kr(1.0,0.1, dur + maxDelay, doneAction:2);  // Just for removing the synth.
	env = XLine.kr(1.0, 0.000001, dur, amp, 0);
	env = DelayN.kr(env, maxDelay, delays);
	sig = SinOsc.ar(freq*freqMuls*noise, 0, env*noise2*ampMuls);
	Out.ar(0, sig);
}).add;

SynthDef(\fmForCercleCycliste, {
	arg freq=2000, mod, amp=0.1, gate=1, release=4.0, out=0;
	var sig, ctls, mods, chans, env1, env2;
	env1 = EnvGen.kr(Env.perc(0.001, curve: -8), gate, amp, 0, release, doneAction:2);
	env2 = EnvGen.kr(Env.perc(0.1, curve: -4), gate, amp, 0, release);
	ctls = [
		// freq, phase, amp
		[freq, 0, 1],
		[freq, 0, 1],
		[freq*1.1, 0, 1],
		[freq*0.9, 0, 1],
		[freq*Line.kr(4.05,4.0, release*0.6), 0, 1],
		[freq*4.0, 0, 1]
	];
	mods = [
		[LFNoise1.ar(40).range(0.9,1.1), env1, env2, 0, 1, 1],
		[env1, LFNoise1.ar(40).range(0.9,1.1), 0, env2, 1, 1],
		[env1, env1, 0, 0, 0, 0],
		[env1, env1, 0, 0, 0, 0],
		[0, 0, 0, 0, 0, 0],
		[0, 0, 0, 0, 0, 0]
	];
	chans = [0, 1, 2, 3];
	FM7.ar(ctls, mods).slice(chans) * -12.dbamp; // using "slice" to select specific channels
	sig = FM7.ar(ctls, mods).slice(chans) * env1;
	Out.ar(out, sig);
}).add;
)
(
OSCdef(\sinusDidactique, {});
OSCdef(\cercleCycliste, {});
)


OSCdef(\numSynths, {|...msg| msg[0][3].postln}, "/g_queryTree.reply");

	
// Ports pour touchOSC:
// Pat: 23001
// Martin: 23002

x = fork {
loop{
	NetAddr("127.0.0.1", 51127).sendMsg("/video_trig_s", 0, 600);
	(1/300).wait;
}
}
x.stop
s.meter
1/30