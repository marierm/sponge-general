( forkIfNeeded {
// The ~preRecorder is a prerec mechanism.  It continuously record in
// a 30 second buffer (by default; duration can be changed).  When the
// ~recorder synth receives a trigger message, it replies its current
// recording position.  A preRecBuffer uses that position to copy a
// section of the ~preRecorder buffer into its own buffer.

//  By default, I create 4 ~preRecBuffers that hold 5 and 7 
//  seconds of sound.  The same ~preRecorder is used for all
//  ~preRecBuffers.  Only one channel is assumed at this point.

// There is also an Onset detector.  Every time an onset is detected,
// a reply (an OSC message /onset) is sent back to the language with
// argument a position in samples in the ~preRecorder buffer.  The
// ~preRecBuffers store these onsets in their ~onsets array so that
// other processes can use them.  (Granulation, for example.)

~preRecorder = Proto({
	~sr = Server.default.sampleRate;
	~initialize = { |dur=30|
		forkIfNeeded {
			~dur = ~sr * dur; // Defaults to 2 minutes seconds at 48000.
			// Buffer to continuously record input.
			~buf = Buffer.alloc( Server.default, ~dur, 1);
			// FFT Buffer for onset detection.
			~fftBuf = Buffer.alloc(Server.default, 1024);
			// Set default values here.
			~onsetThresh = 0.7;
			~gateThresh = 0.03;
			~latencyComp = 480;
			SynthDef(\record, { 
				arg in=0, buf=0, t_trig, id=0, fftBuf=500,
				gateThresh=0.01, onsetThresh=0.7, latencyComp=480;
				var phasor, onsets, chain, input;
				input = SoundIn.ar(in);
				phasor = Phasor.ar(0, 1, 0, BufFrames.kr(buf));
				BufWr.ar(
					input,
					bufnum: buf,
					phase: phasor
				);
				// Onset detection: first, intense noise gating to
				// avoid false onsets from environmental noise.
				input = Compander.ar(input, input, gateThresh, 100, 1, 0.002);
				// Then actual onset detection.
				chain = FFT(fftBuf, input);
				onsets = Onsets.kr(chain, onsetThresh, \complex);
				SendReply.kr(t_trig, "/buffer_pos", phasor, id);
				SendReply.kr(onsets, "/onset", phasor - latencyComp);
			}).add;
			
			Server.default.sync; //
			// Always recording, looping.
			~recorder = Synth.head(Server.default, \record, [
				\buf, ~buf,
				\fftBuf, ~fftBuf
			]);
			Server.default.sync; //
			~setOnsetThresh.value(~onsetThresh);
			~setGateThresh.value(~gateThresh);
			~setLatencyComp.value(~latencyComp);
		};
		currentEnvironment;
	};
	// Change the defaults here.
	~setOnsetThresh = { |val=0.7|
		~onsetThresh = val;
		~recorder.set(\onsetThresh, ~onsetThresh);
		currentEnvironment.changed(\onsetThresh, val);
	};
	~setGateThresh = { |val=0.01|
		~gateThresh = val;
		~recorder.set(\gateThresh, ~gateThresh);
		currentEnvironment.changed(\gateThresh, val);
	};
	~setLatencyComp = { |val=480|
		~latencyComp = val;
		~recorder.set(\latencyComp, ~latencyComp);
		currentEnvironment.changed(\latencyComp, val);
	};

	~free = {
		~recorder.free;
		~buf.free;
		~fftBuf.free;
	};
	~makeGui = { | winPos |
		topEnvironment[\preRecorderGui].new(currentEnvironment, winPos);
	};
});

~preRecBuffer = Proto({
	~initialize = { |liveRecorder, dur=4, numChan=1, grasNullateur|
		~sr = Server.default.sampleRate;
		~dur = dur; // default is 4 seconds.  This is decided at prep time.
		~frames = ~sr * ~dur;
		~numChan = numChan;
		~buf = Buffer.alloc(Server.default, ~frames, ~numChan);
		~liveRecBuf = liveRecorder.buf;
		~liveRecSynth = liveRecorder.recorder;
		~tmpOnsets = List[];
		~onsets = List[];
		~copyFunc = { |bufPos|
			~liveRecBuf.copyLoopTo(
				buf: ~buf,
				dstStartAt: 0,
				srcStartAt: (bufPos - ~frames),
				numSamples: ~frames
			);
			"Copying buffer ".post;
			~liveRecBuf.bufnum.post;
			" to buffer ".post;
			~buf.bufnum.postln;
			"Length: ".post;
			~dur.post;
			" seconds.".postln;
			~getBufferData.value();
			/// MVC: update the GUI if present. 
			currentEnvironment.changed(\bufferUpdated);
		};
		~oscFunc = OSCFunc({ |msg|
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
		~onsetOscFunc = OSCFunc({ |msg|
			var bufPos, startPos, first;
			bufPos = msg[3];
			~tmpOnsets.add(bufPos);
			// Remove temporary onsets that are older than ~dur.
			startPos = (bufPos - ~frames) % ~liveRecBuf.numFrames;
			first = ~tmpOnsets.first;
			(startPos < bufPos).if ({ // normal situation;
				while {
					first.notNil.and({
						(first <= startPos) or: (first > bufPos);
					});
				} {
					~tmpOnsets.removeAt(0);
					first = ~tmpOnsets.first;
				};
			},{ // recording looped and start point is ahead of end.
				while {
					first.notNil.and({
						(first <= startPos) and: (first > bufPos);
					});
				} {
					~tmpOnsets.removeAt(0);
					first = ~tmpOnsets.first;
				};
			});

			// "Temporary onsets: ".post;
			// ~tmpOnsets.postln;
			currentEnvironment.changed(\onsetDetected);
		}.inEnvir, "/onset", Server.default.addr);
		currentEnvironment.addDependant(grasNullateur);
		currentEnvironment;
	};
	~preRec = { // Prerecord the last ~dur seconds.
		// Tell ~liveRecSynth that we want to copy the
		// last ~dur recorded to our ~buf.
		// ID is the bufnum of ~buf so we know where to copy.
		~liveRecSynth.set(\id, ~buf.bufnum, \t_trig, 1.0);
	};
	// Tell a subscriber to use this buffer.
	// ~grL and ~grR subscribe to these messages.
	~setBuffer = { |name|
		currentEnvironment.changed(\setBuffer, ~buf, name);
	};

	~free = {
		~buf.free;
		~oscFunc.disable; // not "free"
		~onsetOscFunc.disable; // not "free"
	};
	~getBufferData = {
		forkIfNeeded {
			~buf.loadToFloatArray(0,-1, { |data|
				~bufferData = data;
			}.inEnvir);
			Server.default.sync;
			~bufferData;
		}
	};
	~makeGui = { | winPos |
		//
		topEnvironment[\preRecBufferGui].new(currentEnvironment, winPos);
	};
});

                  /////////////////////
                  // Fantastic Gui ! //
                  /////////////////////

~preRecBufferGui = Proto({
	~initialize = { |preRecBuffer, winPos|
		~selectionSize = 480*3;  // 30 ms, big enough to see the
								 // onsets in sf views of longest
								 // bufers.
		~prb = preRecBuffer;
		~prb.addDependant(currentEnvironment); // MVC
		winPos = winPos ? 700@700;
		{
			~window = Window(
				~prb.dur.asString ++ " second Buffer",
				Rect(winPos.x, winPos.y, 440, 100);
			);
			~window.layout = HLayout(
				~recButton = Button(~window),
				~sf = SoundFileView(~window),
				VLayout(
					~assignButton = Button(~window)
				)
			);
			~sf.gridOn = false;
			~recButton.states_(
				[["PreRecord\n" ++ ~prb.dur.asString ++ " seconds" ]]
			);
			~assignButton.states_(
				[["Assign to gras"]]
			);
			// ~rightHandButton.states_(
			// 	[["Right Hand"]]
			// );
			~recButton.action_({ ~prb.preRec }.inEnvir);
			~assignButton.action_({~prb.setBuffer('gras')}.inEnvir);
			// ~rightHandButton.action_({~prb.setBuffer('right')}.inEnvir);
			~window.front;
			~updateBufferData.value();
			~window.onClose_({ // MVC
				~prb.removeDependant(currentEnvironment);
			}.inEnvir);
		}.inEnvir.defer;
		currentEnvironment;
	};

	~updateBufferData = {
		forkIfNeeded {
			~prb.getBufferData;
			Server.default.sync;
			{
				~sf.setData(~prb.bufferData, ~prb.sr);
				// Make a short selection at each onset.
				64.do({|id| // The max number of selections.
					~prb.onsets[id].notNil.if({
						\frame.post; ~prb.onsets[id].postln;
						~sf.setSelectionStart(id, ~prb.onsets[id]);
						~sf.setSelectionSize(id, ~selectionSize);
						~sf.setEditableSelectionStart(id, false);
						~sf.setEditableSelectionSize(id, false);
						~sf.setSelectionColor(id, Color.grey(0.6));
					},{
						~sf.setSelectionSize(id, 0);
					});
				});
			}.inEnvir.defer;
		};
	};

	~update = { arg envir, what ... args;	// respond to a change in a model (MVC)
		forkIfNeeded {
			Server.default.sync;
			what.switch(
				\bufferUpdated,{ ~updateBufferData.value(); }
			)
		}
	};
});

~preRecorderGui = Proto({
	~initialize = { |preRecorder, winPos|
		~pr = preRecorder;
		~pr.addDependant(currentEnvironment); // MVC
		winPos = winPos ? 700@700;
		{
			~window = Window(
				"Pre-Recorder Settings",
				Rect(winPos.x, winPos.y, 230, 500);
			);
			~window.layout = VLayout(
				~onsetButton = Button(~window),
				HLayout(
					VLayout(
						~gateThresh = Slider(~window),
						~gateText = StaticText(~window)
					),
					VLayout(
						~onsetThresh = Slider(~window),
						~onsetText = StaticText(~window)
					),
					VLayout(
						~latencyComp = Slider(~window),
						~latencyText = StaticText(~window)
					)
				);
			);
			~onsetButton.states_(
				[["Onset\nIndicator"],["BOOM\n!!!!",nil,Color.red]]
			);
			~onsetThresh.action_({ |sl|
				// make the fader follow an amp warp.
				~pr.setOnsetThresh(sl.value.squared);
			}.inEnvir);
			~gateThresh.action_({ |sl|
				// make the fader follow an amp warp.
				~pr.setGateThresh(sl.value.squared);
			}.inEnvir);
			~latencyComp.action_({ |sl|
				// make the fader follow an amp warp.
				~pr.setLatencyComp(
					(sl.value * 2047).round(1)
				);
			}.inEnvir);
			~window.front;
			~window.onClose_({ // MVC
				~pr.removeDependant(currentEnvironment);
				~onsetOscFunc.disable;
			}.inEnvir);
			~gateThresh.value_(~pr.gateThresh.sqrt);
			~onsetThresh.value_(~pr.onsetThresh.sqrt);
			~latencyComp.value_(~pr.latencyComp / 2047);

			~gateText.string_(
				~pr.gateThresh.asString ++ "\nGate\nThreshold"
			).align_(\center);
			~onsetText.string_(
				~pr.onsetThresh.asString ++ "\nOnset\nThreshold"
			).align_(\center);
			~latencyText.string_(
				~pr.latencyComp.asString ++ "\nLatency\nComp."
			).align_(\center);


			// ~updateGateThresh.value(~pr.gateThresh);
			// ~updateOnsetThresh.value(~pr.onsetThresh);
			// ~updateLatencyComp.value(~pr.latencyComp);
		}.inEnvir.defer;

		// Show onsets using OSC, not MVC
		~onsetOscFunc = OSCFunc({ |msg|
			~updateOnsetButton.value();
		}.inEnvir, "/onset", Server.default.addr);
		currentEnvironment;
	};

	~updateOnsetButton = {
		{
			{
				~onsetButton.value_(1);
				0.1.wait;
				~onsetButton.value_(0);
			}.forkIfNeeded(AppClock);
		}.inEnvir.defer;
	};

	~updateGateThresh = { |val|
		{
			~gateThresh.value_( val.sqrt );
			~gateText.string_(
				val.round(0.001).asString ++ "\nGate\nThreshold"
			).align_(\center);
		}.inEnvir;
	};

	
	~updateOnsetThresh = { |val|
		{
			~onsetThresh.value_( val.sqrt );
			~onsetText.string_(
				val.round(0.001).asString ++ "\nOnset\nThreshold"
			).align_(\center);
		}.inEnvir;
	};

	~updateLatencyComp = { |val|
		{
			~latencyComp.value_( val / 2047 );
			~latencyText.string_(
				val.round(1).asString ++ "\nLatency\nComp."
			).align_(\center);
		}.inEnvir;
	};

	~update = { arg envir, what ... args;	// respond to a change in a model (MVC)
		{
			what.switch(
				\onsetThresh, ~updateOnsetThresh.value(args[0]),
				\gateThresh, ~updateGateThresh.value(args[0]),
				\latencyComp, ~updateLatencyComp.value(args[0])
			)
		}.forkIfNeeded(AppClock);
	};
});


~lc = ~preRecorder.new(30);
Server.default.sync;
// make 5 preRecBuffers
// init them with 5, 5, 7, 7 second buffers
~recBufs = [5,5,7,7].collect({|i|
	~preRecBuffer.new(~lc, i, 1, ~gras);
});
// And make a gui for this stuff.
// ~bufferData = Array.fill(~recBufs.size, { [] });

Server.default.sync;
// make guis;
~guis = ~recBufs.collect({|i,j|
	i.makeGui(Point(800, 600 - (j * 120) ));
});
~guig = ~lc.makeGui(470@300);
}
);

