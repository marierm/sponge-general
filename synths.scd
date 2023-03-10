~num = 0;
(
NetAddr("192.168.0.150", 8895).sendMsg("/eraser_on", 1); 
NetAddr.localAddr.sendMsg("/eraser/start", 1);
~num = ~num + 1 % 9;
)

(
~lasers = Array.fill(9, {nil});
~isPlaying = Array.fill(9, {false});



OSCdef(\eraserStart, {|msg|
	msg.postln;
	~isPlaying[msg[1]].if({
		~isPlaying[msg[1]] = false;
		~lasers[msg[1]].do(_.release(0.2));
	});
	~lasers[msg[1]] = [
		Synth.tail(s, \lampPostEraser, [\freq, 40 + (msg[1]), \amp, 0.2]),
		Synth.tail(s, \lampPostEraser, [\freq, 40 + (msg[1] * 1), \amp, 0.2])
	];
	~isPlaying[msg[1]] = true;
},"/eraser/start");

OSCdef(\eraserGrouille, {|msg|
	msg.postln;
	// index, length (0.0-600.0), x (0.0-1280.0), y (0.0-720.0)
	// On va avoir x2 et y2 pour la suite.
	~lasers[msg[1]][0].set(\grit, msg[3]);
},"/eraser/grouille");

OSCdef(\eraserStop, {|msg|
	msg.postln;
	~lasers[msg[1]].do(_.release(0.1));
	~isPlaying[msg[1]] = false;
},"/eraser/stop");
)


(
SynthDef(\lampPostEraser, {
	|out=0, freq=50, amp=0.1, gate=1, scale=1, grit=1.0|
	Out.ar(out,
		LeakDC.ar(
			VarSaw.ar(
				LFDNoise3.kr(2,0.03, 1.0) * [freq,freq*1.02],
				0,
				LFDNoise3.kr(1, LFDNoise3.kr(1).range(0,0.3) ), //width
				grit * EnvGen.kr(Env.adsr(0.02, 0.15, 0.02, 5, 10, -2), gate, scale, doneAction:2),
				1).wrap(-0.65,0.2);
		) * amp;
	);
}).add();
)

(
var num = 4;
var baseFreq = 40;
x = Array.fill(num , { |i| Synth.tail(s, \lampPostEraser, [\freq, baseFreq + (baseFreq*i), \amp, (i+1)/(num+i)]); });
)

x.do(_.release(15.5));

s.scope(2);

(instrument: \lampPostEraser, freq: 40, scale: 0.8, dur: 3, amp:1,).play;


