#include <hypro/representations/GeometricObjectBase.h>
#include <vector>

#pragma once

namespace simplexArchitectures {
template<typename Number>
typename hypro::Box<Number> createPlayground() {
	using I = carl::Interval<Number>;
	using IV = std::vector<I>;
	return hypro::Box<Number>{IV{I{61.6172,67.5706},I{620.959,431.414}}};
}
} // namespace
