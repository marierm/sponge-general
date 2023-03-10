// A link is a connection between a ~leapReceiver and a sound
// parameter.  It takes these parameters:
// - source : a Symbol that is a key in the ~leapReceivers dictionary.
// - dests : an Array of Symbols that are parameters of the grasNullateur Synth.
// - specs : an Array of  ControlSpecs for mapping the data.
// - granu : The granulator synth.
// - offset : A channel offset to acces leap data that has multiple parameters.
//            For example, lOrient has pitch, roll and yaw.  To access only yaw,
//            use offset of 2. dests and specs should be arrays with only one
//            element.
// 
// This will only do one-to-one mapping.

// GrasNullateur takes these arguments:

// arg granFreq=8, out=0, granDur=0.2, bufnum=0, rate=1,
// pos=0.5, panW=0.0, panC=0.0, amp=0.1, env= -1;

(
forkIfNeeded {
	~link = Proto({
		~initialize = { |source, dests, specs, granu, offset=0|
			forkIfNeeded {
				~leapReceivers = topEnvironment.at(\leapReceivers);
				~source = ~leapReceivers.at(source);
				~specs = specs ? Array.fill(~source.numChan, {});
				~dests = dests ? Array.fill(~source.numChan, {});
				~granu = granu;
				~bus = Bus.control(Server.default, ~dests.size);
				~defs = ~specs.collect({|spec, i|
					SynthDef(
						(source++"-to-"++dests[i]), {
							|in=0, out=0|
							var input;
							input = In.kr(in, 1);
							input = spec.map(input);
							Out.kr(out, input);
						});
				});
				~defs.do(_.add);
				Server.default.sync;
				~synths = ~defs.collect({ |def, i|
					Synth.head(Server.default, def.name, [
						\in, ~source.bus.subBus(i + offset),
						\out, ~bus.subBus(i)
					])
				});
				
				~dests.do({|dest, i|
					~granu.granSynth.map(dest, ~bus.subBus(i));
				});
			};
			currentEnvironment;
		};
		~free = {
			~synths.do(_.free);
			~bus.free;
		};

	});
});



// ~link1.free;
// ~link2.free;

// ~link1.bus.get;

// ~link2.bus.get;

// ~gr.granSynth.set(\granFreq, 1, \pos, 0.1, \env, -1, \bufnum, 4, \granDur, 3, \rate, 1);

// ~gr.help