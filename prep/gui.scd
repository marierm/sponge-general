~quintetGui = { |ips=#[100,98,97,101,99], names=#["","","","",""]|
	var window, spongeViews, octaveViews;
	// var names = ips.collect(_.asAscii);
	var numSponges = ips.size;
	var oscFuncs;
	var spongeNames;
	spongeNames = names.collect({ |i,j|
		// (i ++ "_" ++ ips[j]);
		i;
	});

	octaveViews = ips.collect({|i,j|
		StaticText().string_("1").align_(\center).font_(Font(nil, 250));
	});
	window = Window("Sponges", (80*numSponges)@170);
	spongeViews = names.collect({ |i,j|
		View(window, 70@120).layout_(
			VLayout(
				StaticText().string_(ips[j].asString + names[j].asString).align_(\center),
				octaveViews[j]
			)
		)
	});

	window.layout_(
		HLayout(*spongeViews);
	);

	oscFuncs = spongeNames.collect({ |spongeName,i|
		[
			OSCFunc({ |msg|
				{
					octaveViews[i].string_(msg[1].asInteger.asString);
				}.defer;
				msg.postln;
            }, "/" ++ spongeName),
			OSCFunc({ |msg|
				(msg[1] == 1).if({
					{
						octaveViews[i].stringColor_(Color.red());
					}.defer;
				},{
					{
						octaveViews[i].stringColor_(Color.black());
					}.defer;
				});
				msg.postln;
			}, "/" ++ spongeName ++ "/recording");
		]
	});

	(
		window: { window },
		oscFuncs: { oscFuncs }
	)

};
