#include <hypro/representations/GeometricObjectBase.h>
#include <vector>

#pragma once

namespace simplexArchitectures {
template<typename Number>
typename hypro::Box<Number> createPlayground() {
	using I = carl::Interval<Number>;
	using IV = std::vector<I>;
	return hypro::Box<Number>{IV{I{12.8462,88.7194},I{11.8814,83.7075}}};
}
} // namespace
