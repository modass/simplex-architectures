#include <hypro/datastructures/Point.h>
#include <vector>

#pragma once

namespace simplexArchitectures {
template<typename Segment>
std::vector<Segment> createSegments() {
	auto res = std::vector<Segment>{
	{ hypro::Point<Number>{ 63.51 , 12.8814 } , hypro::Point<Number>{ 59.2047, 21.655 }, hypro::Point<Number>{ 13.8462, 22.0116 }, hypro::Point<Number>{ 21.9423, 28.5727 } },
	{ hypro::Point<Number>{ 13.8462 , 22.0116 } , hypro::Point<Number>{ 21.9423, 28.5727 }, hypro::Point<Number>{ 18.5212, 46.6404 }, hypro::Point<Number>{ 23.8719, 37.5105 } },
	{ hypro::Point<Number>{ 18.5212 , 46.6404 } , hypro::Point<Number>{ 23.8719, 37.5105 }, hypro::Point<Number>{ 50.5554, 40.2056 }, hypro::Point<Number>{ 56.7873, 31.6536 } },
	{ hypro::Point<Number>{ 50.5554 , 40.2056 } , hypro::Point<Number>{ 56.7873, 31.6536 }, hypro::Point<Number>{ 61.0571, 82.7075 }, hypro::Point<Number>{ 65.6292, 67.4722 } },
	{ hypro::Point<Number>{ 61.0571 , 82.7075 } , hypro::Point<Number>{ 65.6292, 67.4722 }, hypro::Point<Number>{ 87.7194, 63.0645 }, hypro::Point<Number>{ 77.9377, 60.5205 } },
	{ hypro::Point<Number>{ 87.7194 , 63.0645 } , hypro::Point<Number>{ 77.9377, 60.5205 }, hypro::Point<Number>{ 63.51, 12.8814 }, hypro::Point<Number>{ 59.2047, 21.655 } }};
	return res;
}
} // namespace
