#include <hypro/datastructures/Point.h>
#include <vector>

#pragma once

namespace simplexArchitectures {
template<typename Number>
std::vector<hypro::Point<Number>> createOptimizedWaypoints() {
	auto res = std::vector<hypro::Point<Number>>{
	hypro::Point<Number>{  60.1706532 ,  20.2246792 },
	hypro::Point<Number>{  47.8326707 ,  14.6890303 },
	hypro::Point<Number>{  33.8479571 ,  14.4502300 },
	hypro::Point<Number>{  22.1981268 ,  21.4437596 },
	hypro::Point<Number>{  20.1255124 ,  34.8059257 },
	hypro::Point<Number>{  31.2229288 ,  41.7105125 },
	hypro::Point<Number>{  44.7586555 ,  38.1323712 },
	hypro::Point<Number>{  56.9267607 ,  42.0947877 },
	hypro::Point<Number>{  58.1742617 ,  55.3299855 },
	hypro::Point<Number>{  61.4369607 ,  68.2271891 },
	hypro::Point<Number>{  74.5880578 ,  68.9473583 },
	hypro::Point<Number>{  81.0805809 ,  56.5889388 },
	hypro::Point<Number>{  80.5476146 ,  42.8339595 },
	hypro::Point<Number>{  74.1796605 ,  30.6235952 },
	hypro::Point<Number>{  63.6391506 ,  21.5911153 }};
	return res;
}
} // namespace
