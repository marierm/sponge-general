(
{
	// "nmcli con up sponger".unixCmd; // Make this computer a hotspot.
	// "nmcli con up sudafed".unixCmd; // Connect to sudafed
	s.bootSync;
	"prep/cSponge.scd".loadRelative;
	"prep/cSimpleSponge.scd".loadRelative;
	"prep/cLooper.scd".loadRelative;
	"prep/cFm.scd".loadRelative;
	"prep/gui.scd".loadRelative;
	s.sync;
	~ipsPrefix = "192.168.0."; // for sudafed
	~ips = [174];
	~spongeNames = ["zebra"];
	~classes = [~cSimpleSponge];
	~transpose = 0;
	~sponges = IdentityDictionary.new();
	~voices = IdentityDictionary.new();
	~ips.do({ |i, j|
		// var sym = ("sponge_" ++ i.asAscii).asSymbol;
		var sym = (~spongeNames[j] ++ "_" ++ i).asSymbol;
		sym.postln;
		~sponges.put( sym, ~classes[j].value( ~ipsPrefix ++ i, sym, Server.default ) );
		~sponges[sym].postln;
		~sponges[sym].prep;
		s.sync;
		1.wait;
		~sponges[sym].sponge.hold_(true);
		~voices.put( sym, List[] );
		~voices[sym].add( // add fm synth to voice 0 of each sponge.
			~cFmManager.value(
				interpolatorPath: ("fm0*").resolveRelative.pathMatch.sort.last.resolveRelative,
				cSponge: ~sponges[sym],
				notes: ([ \en1, \en2, \en3].keynum.mirror2.clump(3)).flat + (0 * 12) + ~transpose,
				chords: [
					[ \en1, \en2, \en3, \en3, \en2, \en1].keynum + ~transpose,
					// [ \fs7, \bn7, \en8, \en7, \bf7, \cn8].keynum - 12,
					[ \en1, \en2, \gn2, \dn3, \fs3, \bn3].keynum + ~transpose,
					[ \en1, \dn2, \fs2, \dn3, \fs3, \bn3].keynum + ~transpose,
					[ \en1, \cn2, \gn2, \dn3, \fs3, \bn3].keynum + ~transpose,
					[ \en1, \bn1, \an2, \dn3, \fs3, \bn3].keynum + ~transpose,
					[ \en1, \en2, \cn3, \dn3, \fs3, \bn3].keynum + ~transpose,
					[ \en1, \dn2, \gn2, \cn3, \fs3, \bn3].keynum + ~transpose,
					[ \en1, \cn2, \gn2, \bn2, \fs3, \bn3].keynum + ~transpose,
					[ \en1, \dn2, \fs2, \bn2, \en3, \an3].keynum + ~transpose,
					[ \fs1, \bn1, \en2, \en1, \bf1, \cn2].keynum + ~transpose,
					[ \fn1, \bf1, \ef2, \en1, \bf1, \cn2].keynum + ~transpose,
					[ \en1, \an1, \cs2, \ef1, \af1, \dn2].keynum + ~transpose,
					[ \gf1, \an1, \cn2, \fn1, \af1, \df2].keynum + ~transpose,
					[ \gf1, \an1, \cn2, \fn1, \bf1, \dn2].keynum + ~transpose,
					[ \gf1, \an1, \cn2, \fn1, \bn1, \dn2].keynum + ~transpose,
					[ \gn1, \an1, \cn2, \fn1, \bn1, \dn2].keynum + ~transpose,
					[ \gn1, \an1, \cn2, \fn1, \fn2, \dn2].keynum + ~transpose
				],
				outBus: j * 2,
				fsrMuteThreshold: 0.85, // larger number = muting is easier
				fmId: j + 20,
				relTime: 0.2,
				server: s,
				withLoop: (~classes[0] == ~cSponge )
			)
		);
		///////////////
		s.sync;
		1.wait;
		~voices.at(sym).do({|i| i.prep});
		s.sync;
		// 1.wait;
		sym.post; " ready.".postln;
	});
	s.sync;

	~ips.do({ |i,j|
		var sym = (~spongeNames[j] ++ "_" ++ i).asSymbol;
		~sponges[sym].sponge.hold_(false);
	});
	{
		~gui = ~quintetGui.value(~ips, ~spongeNames);
		~gui.window.front;
	}.defer;
}.fork;
)
~sponges[\zebra_174].sponge.hold_(false)