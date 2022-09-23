#include <hypro/datastructures/Point.h>
#include <vector>

#pragma once

namespace simplexArchitectures {
template<typename Number>
std::vector<hypro::Point<Number>> createWaypoints() {
	auto res = std::vector<hypro::Point<Number>>{
    hypro::Point<Number>{ 229.95999999999998 , 82.0086 },
    hypro::Point<Number>{ 176.553 , 68.7601 },
    hypro::Point<Number>{ 170.144 , 68.4011 },
    hypro::Point<Number>{ 165.853 , 76.2541 },
    hypro::Point<Number>{ 151.52550000000002 , 95.9411 },
    hypro::Point<Number>{ 140.1055 , 111.992 },
    hypro::Point<Number>{ 101.4503 , 189.053 },
    hypro::Point<Number>{ 59.85275 , 236.81900000000002 },
    hypro::Point<Number>{ 57.90625 , 239.98 },
    hypro::Point<Number>{ 62.4105 , 244.538 },
	hypro::Point<Number>{ 98.32845 , 245.42700000000002 },
	hypro::Point<Number>{ 123.411 , 240.661 },
	hypro::Point<Number>{ 160.687 , 233.85750000000002 },
	hypro::Point<Number>{ 194.8315 , 230.4105 },
	hypro::Point<Number>{ 230.3 , 227.6775 },
	hypro::Point<Number>{ 234.288 , 225.629 },
	hypro::Point<Number>{ 235.3605 , 216.9375 },
	hypro::Point<Number>{ 227.195 , 206.1575 },
	hypro::Point<Number>{ 201.46550000000002 , 193.29399999999998 },
	hypro::Point<Number>{ 170.4085 , 196.1815 },
	hypro::Point<Number>{ 144.2015 , 198.055 },
	hypro::Point<Number>{ 138.387 , 194.064 },
	hypro::Point<Number>{ 133.09199999999998 , 180.336 },
	hypro::Point<Number>{ 135.2105 , 173.477 },
	hypro::Point<Number>{ 152.24 , 141.8655 },
	hypro::Point<Number>{ 158.9915 , 137.18349999999998 },
	hypro::Point<Number>{ 175.95350000000002 , 139.118 },
	hypro::Point<Number>{ 192.58350000000002 , 156.49200000000002 },
	hypro::Point<Number>{ 211.37650000000002 , 163.086 },
	hypro::Point<Number>{ 294.327 , 162.9015 },
	hypro::Point<Number>{ 314.2645 , 162.891 },
	hypro::Point<Number>{ 325.1095 , 155.538 },
	hypro::Point<Number>{ 334.26149999999996 , 120.98949999999999 },
	hypro::Point<Number>{ 329.94 , 111.75200000000001 },
	hypro::Point<Number>{ 310.312 , 102.37880000000001 }};
	return res;
}
} // namespace
