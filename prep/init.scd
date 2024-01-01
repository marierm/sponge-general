

(
Platform.case(
	\osx, {
		ServerOptions.devices.includesEqual("BlackHole 64ch").if({
			~audioDevice = "BlackHole 64ch";
		});
		~spongeGeneralRoot = "/Users/martinm/docs/sponge-general";
		~reaperCommand = "open -a reaper --args -template" + "~/Library/Application\\ Support/REAPER/ProjectTemplates/scBasicRouting.RPP";
	},
	\linux, {

	},
	\windows, {

	}
);
~reaperCommand.unixCmd;
Server.default.options.device_(~audioDevice);
// Server.supernova;
// Server.default.options.outDevice_("Haut-parleurs MacBook Pro").inDevice_("Micro MacBook Pro");
// Server.default.options.device_("Dante Virtual Soundcard");
Server.default.options.hardwareBufferSize_(64);
Server.default.options.sampleRate_(48000);
Server.default.options.maxNodes_(16384);
Server.default.options.numAudioBusChannels_((2.pow(11)).asInteger);
Server.default.options.numOutputBusChannels_(48);
Server.default.options.numInputBusChannels_(2);
Server.default.options.numBuffers_(2048);
Server.default.options.memSize_(2.pow(20));
Server.default.options.numWireBufs_(2.pow(16));
Server.default.options.loadDefs_(false);
MIDIClient.init;
MIDIIn.connectAll;
Server.default.boot;
)