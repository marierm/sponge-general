(
{
	var sym = "sponge_solo".asSymbol;
	s.bootSync;
	"prep/sponge.scd".loadRelative;
	"prep/parents.scd".loadRelative;
	"prep/parentsFm.scd".loadRelative;
	"prep/fm.scd".loadRelative;
	s.sync;
	PR(\sponge).chuck(BP(sym), parms:(
		defName: sym
	));
	BP(sym).play;
	s.sync;
	BP(sym).sponge.hold_(true);
	PR(\fm).chuck(BP( (sym ++ 0).asSymbol ), parms:(
		interpolatorPath: ("fm0*").resolveRelative.pathMatch.sort.last.resolveRelative,
		sponge: BP(sym).sponge,
		outBus: 0,
		notes: ([ \en1, \en2, \en3].keynum.mirror2.clump(3)).flat + (0 * 12),
		// notes: ([ \bb0, \bb1, \bb2].keynum.mirror2.clump(3)).flat + (i * 12),
		numNotes: 6,
		fmId: 20,
		fsrMuteThreshold: 0.85, // larger number = muting is easier
		chords: [
			[ \en1, \en2, \en3, \en3, \en2, \en1].keynum,
			// [ \fs7, \bn7, \en8, \en7, \bf7, \cn8].keynum - 12,
			[ \en2, \en3, \gn3, \dn4, \fs4, \bn4].keynum,
			[ \en2, \dn3, \fs3, \dn4, \fs4, \bn4].keynum,
			[ \en2, \cn3, \gn3, \dn4, \fs4, \bn4].keynum,
			[ \en2, \bn2, \an3, \dn4, \fs4, \bn4].keynum,
			[ \en2, \en3, \cn4, \dn4, \fs4, \bn4].keynum,
			[ \en2, \dn3, \gn3, \cn4, \fs4, \bn4].keynum,
			[ \en2, \cn3, \gn3, \bn3, \fs4, \bn4].keynum,
			[ \en2, \dn3, \fs3, \bn3, \en4, \an4].keynum,
			[ \fs7, \bn7, \en8, \en7, \bf7, \cn8].keynum - 48,
			[ \fn7, \bf7, \ef8, \en7, \bf7, \cn8].keynum - 48,
			[ \en7, \an7, \cs8, \ef7, \af7, \dn8].keynum - 48,
			[ \gf7, \an7, \cn8, \fn7, \af7, \df8].keynum - 48,
			[ \gf7, \an7, \cn8, \fn7, \bf7, \dn8].keynum - 48,
			[ \gf7, \an7, \cn8, \fn7, \bn7, \dn8].keynum - 48,
			[ \gn7, \an7, \cn8, \fn7, \bn7, \dn8].keynum - 48,
			[ \gn7, \an7, \cn8, \fn7, \fn8, \dn8].keynum - 48
		],
		defName: sym
	));
	s.sync;
	BP(sym).sponge.hold_(false);
	s.sync;
	sym.post; " ready.".postln;
	"prep/gui.scd".loadRelative;
	{
		a = ~quintetGui.value(["solo".ascii]);
		a.window.front;
	}.defer;
}.fork
)


