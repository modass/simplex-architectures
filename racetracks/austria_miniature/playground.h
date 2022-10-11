#include <hypro/representations/GeometricObjectBase.h>
#include <vector>

#pragma once

namespace simplexArchitectures {
template<typename Number>
typename hypro::Box<Number> createPlayground() {
	using I = carl::Interval<Number>;
	using IV = std::vector<I>;
	return hypro::Box<Number>{IV{I{52.2737,339.147},I{63.610600000000005,250.102}}};
}
} // namespace
