#include <hypro/datastructures/Point.h>
#include <vector>

#pragma once

namespace simplexArchitectures {
template<typename Number>
std::vector<hypro::Point<Number>> createWaypoints() {
	auto res = std::vector<hypro::Point<Number>>{
	hypro::Point<Number>{ 147.2885 , 288.3135 },
	hypro::Point<Number>{ 173.267 , 282.69100000000003 },
	hypro::Point<Number>{ 191.53199999999998 , 286.101 },
	hypro::Point<Number>{ 201.6395 , 284.539 },
	hypro::Point<Number>{ 213.1485 , 276.528 },
	hypro::Point<Number>{ 224.596 , 271.62350000000004 },
	hypro::Point<Number>{ 236.4235 , 273.582 },
	hypro::Point<Number>{ 245.8725 , 276.773 },
	hypro::Point<Number>{ 255.0515 , 274.0705 },
	hypro::Point<Number>{ 260.6 , 267.688 },
	hypro::Point<Number>{ 265.96299999999997 , 250.773 },
	hypro::Point<Number>{ 276.21799999999996 , 237.9105 },
	hypro::Point<Number>{ 346.5085 , 174.6945 },
	hypro::Point<Number>{ 380.7985 , 139.6395 },
	hypro::Point<Number>{ 385.599 , 128.1895 },
	hypro::Point<Number>{ 381.779 , 116.83500000000001 },
	hypro::Point<Number>{ 372.2415 , 110.134 },
	hypro::Point<Number>{ 349.863 , 105.116 },
	hypro::Point<Number>{ 307.2655 , 83.48975 },
	hypro::Point<Number>{ 304.399 , 77.11455000000001 },
	hypro::Point<Number>{ 308.214 , 69.42555 },
	hypro::Point<Number>{ 303.89200000000005 , 58.560249999999996 },
	hypro::Point<Number>{ 292.21500000000003 , 52.8249 },
	hypro::Point<Number>{ 278.57849999999996 , 55.55285 },
	hypro::Point<Number>{ 265.91700000000003 , 68.32625 },
	hypro::Point<Number>{ 209.024 , 132.249 },
	hypro::Point<Number>{ 207.406 , 143.2425 },
	hypro::Point<Number>{ 211.9365 , 151.6985 },
	hypro::Point<Number>{ 223.91 , 178.47 },
	hypro::Point<Number>{ 211.778 , 209.07600000000002 },
	hypro::Point<Number>{ 200.3005 , 225.688 },
	hypro::Point<Number>{ 214.4665 , 230.4105 },
	hypro::Point<Number>{ 228.4735 , 234.15800000000002 },
	hypro::Point<Number>{ 223.91 , 244.576 },
	hypro::Point<Number>{ 202.1705 , 256.6095 },
	hypro::Point<Number>{ 185.4435 , 251.52800000000002 },
	hypro::Point<Number>{ 87.38535 , 177.0885 },
	hypro::Point<Number>{ 83.67914999999999 , 169.912 },
	hypro::Point<Number>{ 89.69575 , 163.6685 },
	hypro::Point<Number>{ 102.5706 , 158.9685 },
	hypro::Point<Number>{ 107.421 , 152.7175 },
	hypro::Point<Number>{ 106.608 , 143.83550000000002 },
	hypro::Point<Number>{ 99.25489999999999 , 138.8275 },
	hypro::Point<Number>{ 88.2653 , 143.2645 },
	hypro::Point<Number>{ 64.03565 , 165.426 },
	hypro::Point<Number>{ 53.5844 , 186.7975 },
	hypro::Point<Number>{ 60.04600000000001 , 240.94 },
	hypro::Point<Number>{ 66.41640000000001 , 276.5925 },
	hypro::Point<Number>{ 72.59765 , 286.10450000000003 },
	hypro::Point<Number>{ 85.8784 , 291.63 },
	hypro::Point<Number>{ 116.9245 , 292.664 }};
	return res;
}
} // namespace
