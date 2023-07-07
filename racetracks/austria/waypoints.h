#include <hypro/datastructures/Point.h>
#include <vector>

#pragma once

namespace simplexArchitectures {
template<typename Number>
std::vector<hypro::Point<Number>> createWaypoints() {
	auto res = std::vector<hypro::Point<Number>>{
	hypro::Point<Number>{ 193.99 , 417.19550000000004 },
	hypro::Point<Number>{ 268.6105 , 403.5765 },
	hypro::Point<Number>{ 336.96000000000004 , 396.676 },
	hypro::Point<Number>{ 407.961 , 391.207 },
	hypro::Point<Number>{ 418.08950000000004 , 369.70550000000003 },
	hypro::Point<Number>{ 350.24 , 322.37649999999996 },
	hypro::Point<Number>{ 288.0705 , 328.156 },
	hypro::Point<Number>{ 235.6105 , 331.906 },
	hypro::Point<Number>{ 213.37 , 296.4355 },
	hypro::Point<Number>{ 217.6105 , 282.706 },
	hypro::Point<Number>{ 251.7 , 219.4265 },
	hypro::Point<Number>{ 299.16999999999996 , 213.9265 },
	hypro::Point<Number>{ 332.46000000000004 , 248.706 },
	hypro::Point<Number>{ 370.08000000000004 , 261.906 },
	hypro::Point<Number>{ 536.13 , 261.536 },
	hypro::Point<Number>{ 576.04 , 261.51599999999996 },
	hypro::Point<Number>{ 597.75 , 246.7955 },
	hypro::Point<Number>{ 616.0705 , 177.6365 },
	hypro::Point<Number>{ 607.4200000000001 , 159.14600000000002 },
	hypro::Point<Number>{ 567.624 , 140.1815 },
	hypro::Point<Number>{ 407.2805 , 99.6063 },
	hypro::Point<Number>{ 300.37 , 73.0851 },
	hypro::Point<Number>{ 287.54 , 72.3656 },
	hypro::Point<Number>{ 278.95050000000003 , 88.0851 },
	hypro::Point<Number>{ 250.2695 , 127.4955 },
	hypro::Point<Number>{ 227.41 , 159.62650000000002 },
	hypro::Point<Number>{ 150.02949999999998 , 313.8855 },
	hypro::Point<Number>{ 66.75975 , 409.506 },
	hypro::Point<Number>{ 71.87989999999999 , 424.95550000000003 },
	hypro::Point<Number>{ 143.78 , 426.7355 }};
	return res;
}
} // namespace
