// Make sure Leap motion daemon is started.
// Required quarks:
// dewdrop_lib, wslib
// 2. Execute this section.
(
forkIfNeeded {
	s.sync;
	"./prep/cGrasNullateur.scd".loadRelative; // creates ~gras
	s.sync;
	"./prep/04-liveRecording.scd".loadRelative;
	"./prep/cGranGui.scd".loadRelative;
	~gras.playMe;
	defer {
		~gui = ~cGranGui.value(~gras);
		~gui.front;
	};
}
)


// Cleanup.
(
~gras.close;
~lc.free;
~guig.window.close;
~recBufs.do(_.free);
~guis.do({|i| i.window.close});
)