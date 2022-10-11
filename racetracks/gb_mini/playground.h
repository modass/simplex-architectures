#include <hypro/representations/GeometricObjectBase.h>
#include <vector>

#pragma once

namespace simplexArchitectures {
template<typename Number>
typename hypro::Box<Number> createPlayground() {
	using I = carl::Interval<Number>;
	using IV = std::vector<I>;
	return hypro::Box<Number>{IV{I{47.7307,391.642},I{46.8808,298.397}}};
}
} // namespace
