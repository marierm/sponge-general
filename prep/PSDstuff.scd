PSD_generic_grabber {
	*init {
		arg nChannels, chan_offset, s;
		var time, level_meter, input_synth, analyse_synth;
		level_meter = List.new(); input_synth = List.new(); analyse_synth =
		List.new();
		~slider_ch = List.new();
		s = s ? Server.default;

		~generic_grabber_window = Window("Generic grabber", Rect(20, 920, 310, (33
			* nChannels) + 10));
		~generic_grabber_window.front;
		~generic_grabber_window.view.decorator =
		FlowLayout(~generic_grabber_window.view.bounds);

		nChannels.do{ arg i;
			analyse_synth.add(Synth.new(\analyse_send_passe_lag, [inbus: 100+i, level:
				1.0, median: 7, pre_lpf: 5000,
				amp_thresh: 0.0, max_freq: 2000, exec_freq: 20, lag_time: 0.3, index:
				100+i, outbus: 20+i], ~analyse));
			input_synth.add(Synth.new(\sound_in, [inbus: i + chan_offset, level: 1.0,
				outbus: 100+i], ~sources));


			~slider_ch.add( EZSlider(~generic_grabber_window, 300@15, "ch " ++
				i.asString, \db,
				{|ez| input_synth[i].set(\level, ez.value.dbamp )}, unitWidth:30, initVal:
				0, initAction: false,
				layout: 'horz').setColors(Color.grey,Color.white,
					Color.grey(0.7),Color.grey, Color.white, Color.yellow);
				level_meter.add(LevelIndicator(~generic_grabber_window, 300@10));
				OSCFunc({arg msg; if((msg[3]) == (100+i), {{level_meter[i].value =
					(msg[6]).ampdb.linlin(-40, 0, 0, 1);}.defer; });
				}, '/analysis', s.addr);
			)};
		~generic_grabber_window.onClose_({
			analyse_synth.collect{ arg i; i.free; };
			input_synth.collect{ arg i; i.free; };
		});
	}
}


s.boot;

SynthDef(\analyse_send_passe_lag, { arg inbus, level, median, pre_lpf,
	amp_thresh, max_freq, exec_freq, lag_time, index, outbus;
	var sound, freq, has_freq, fft, entropy, amp_follow, trig;
	sound = In.ar(inbus, 1);
	trig = Impulse.kr(exec_freq);
	//analyse
	# freq, has_freq = Pitch.kr(LPF.ar(sound, pre_lpf),
		initFreq: 440.0,
		minFreq: 50.0,
		maxFreq: max_freq,
		execFreq: exec_freq,
		maxBinsPerOctave: 16,
		median: median,
		ampThreshold: amp_thresh,
		peakThreshold: 0.8,
		downSample: 1,
		clar: 0);
	fft = FFT(LocalBuf(2048), sound);
	entropy = SpectralEntropy.kr(fft,2048,1);
	amp_follow = Amplitude.kr(sound, 0.01, 0.3);
	SendReply.kr(trig, '/analysis', [
		index,
		Lag.kr(freq, lag_time),
		Lag.kr(entropy, lag_time),
		Lag.kr(amp_follow, lag_time)
	]);
	Out.ar(outbus, sound * level);
}).send(s);

PSD_generic_grabber.init(8, 0, s);

(
~h_start = 25;
~h_end = 35;
~s_start = 255;
~s_end = 0;
~b_start = 105;
~b_end = 255;

OSCdef.new(\suiveur_to_color, { arg msg;
	var freq, entropy, amp, id;
	//msg.postln;
	freq = msg[4];
	entropy =  msg[5];
	amp = msg[6];
	id = msg[3].switch(
		100, { 0 },
		101, { 1 },
		102, { 2 },
		103, { 3 }
	);
	
	~osc_to_background.sendMsg(
		"/bckg_hsb",
		id,
		freq.linlin(
			0, 2000, ~h_start, ~h_end
		).asInt,
		entropy.linlin(
			0, 35, ~s_start, ~s_end
		).asInt,
		amp.linlin(
			0, 0.1, ~b_start, ~b_end
		).asInt
	);
}, '/analysis',  recvPort: 57120);
)
