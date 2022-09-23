#include <hypro/datastructures/Point.h>
#include <vector>

#pragma once

namespace simplexArchitectures {
template<typename Number>
std::vector<hypro::Point<Number>> createWaypoints() {
	auto res = std::vector<hypro::Point<Number>>{
	hypro::Point<Number>{ 61.35735 , 17.2682 },
	hypro::Point<Number>{ 17.89425 , 25.29215 },
//      hypro::Point<Number>{ 40 , 25.29215 },
	hypro::Point<Number>{ 21.196550000000002 , 42.075450000000004 },
	hypro::Point<Number>{ 53.671350000000004 , 35.9296 },
	hypro::Point<Number>{ 63.343149999999994 , 75.08985 },
	hypro::Point<Number>{ 82.82855 , 61.792500000000004 }};
	return res;
}
} // namespace
