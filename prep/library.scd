// Not sure how to avoid copying code in multiple places.


(
freezeMe: { |interpolator|
	numInterpolatorDim.do({ |i| interpolator.disconnect(i);	});
	frozen = true;
}
)

