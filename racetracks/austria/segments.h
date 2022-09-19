#include <hypro/datastructures/Point.h>
#include <vector>

#pragma once

namespace simplexArchitectures {
template<typename Segment>
std::vector<Segment> createSegments() {
	auto res = std::vector<Segment>{
	{ hypro::Point<Number>{ 194.662 , 420.803 } , hypro::Point<Number>{ 193.318, 413.588 }, hypro::Point<Number>{ 269.125, 407.213 }, hypro::Point<Number>{ 268.096, 399.94 } },
	{ hypro::Point<Number>{ 269.125 , 407.213 } , hypro::Point<Number>{ 268.096, 399.94 }, hypro::Point<Number>{ 337.285, 400.332 }, hypro::Point<Number>{ 336.635, 393.02 } },
	{ hypro::Point<Number>{ 337.285 , 400.332 } , hypro::Point<Number>{ 336.635, 393.02 }, hypro::Point<Number>{ 410.371, 394.702 }, hypro::Point<Number>{ 405.551, 387.712 } },
	{ hypro::Point<Number>{ 410.371 , 394.702 } , hypro::Point<Number>{ 405.551, 387.712 }, hypro::Point<Number>{ 422.73, 368.467 }, hypro::Point<Number>{ 413.449, 370.944 } },
	{ hypro::Point<Number>{ 422.73 , 368.467 } , hypro::Point<Number>{ 413.449, 370.944 }, hypro::Point<Number>{ 351.238, 318.598 }, hypro::Point<Number>{ 349.242, 326.155 } },
	{ hypro::Point<Number>{ 351.238 , 318.598 } , hypro::Point<Number>{ 349.242, 326.155 }, hypro::Point<Number>{ 287.77, 324.499 }, hypro::Point<Number>{ 288.371, 331.813 } },
	{ hypro::Point<Number>{ 287.77 , 324.499 } , hypro::Point<Number>{ 288.371, 331.813 }, hypro::Point<Number>{ 237.549, 328.089 }, hypro::Point<Number>{ 233.672, 335.723 } },
	{ hypro::Point<Number>{ 237.549 , 328.089 } , hypro::Point<Number>{ 233.672, 335.723 }, hypro::Point<Number>{ 217.373, 295.911 }, hypro::Point<Number>{ 209.367, 296.96 } },
	{ hypro::Point<Number>{ 217.373 , 295.911 } , hypro::Point<Number>{ 209.367, 296.96 }, hypro::Point<Number>{ 221.012, 284.132 }, hypro::Point<Number>{ 214.209, 281.28 } },
	{ hypro::Point<Number>{ 221.012 , 284.132 } , hypro::Point<Number>{ 214.209, 281.28 }, hypro::Point<Number>{ 254.023, 222.852 }, hypro::Point<Number>{ 249.377, 216.001 } },
	{ hypro::Point<Number>{ 254.023 , 222.852 } , hypro::Point<Number>{ 249.377, 216.001 }, hypro::Point<Number>{ 297.779, 217.782 }, hypro::Point<Number>{ 300.561, 210.071 } },
	{ hypro::Point<Number>{ 297.779 , 217.782 } , hypro::Point<Number>{ 300.561, 210.071 }, hypro::Point<Number>{ 330.416, 251.878 }, hypro::Point<Number>{ 334.504, 245.534 } },
	{ hypro::Point<Number>{ 330.416 , 251.878 } , hypro::Point<Number>{ 334.504, 245.534 }, hypro::Point<Number>{ 369.459, 265.577 }, hypro::Point<Number>{ 370.701, 258.235 } },
	{ hypro::Point<Number>{ 369.459 , 265.577 } , hypro::Point<Number>{ 370.701, 258.235 }, hypro::Point<Number>{ 536.135, 265.206 }, hypro::Point<Number>{ 536.125, 257.866 } },
	{ hypro::Point<Number>{ 536.135 , 265.206 } , hypro::Point<Number>{ 536.125, 257.866 }, hypro::Point<Number>{ 577.168, 265.186 }, hypro::Point<Number>{ 574.912, 257.846 } },
	{ hypro::Point<Number>{ 577.168 , 265.186 } , hypro::Point<Number>{ 574.912, 257.846 }, hypro::Point<Number>{ 600.945, 249.063 }, hypro::Point<Number>{ 594.555, 244.528 } },
	{ hypro::Point<Number>{ 600.945 , 249.063 } , hypro::Point<Number>{ 594.555, 244.528 }, hypro::Point<Number>{ 619.959, 177.288 }, hypro::Point<Number>{ 612.182, 177.985 } },
	{ hypro::Point<Number>{ 619.959 , 177.288 } , hypro::Point<Number>{ 612.182, 177.985 }, hypro::Point<Number>{ 610.188, 156.401 }, hypro::Point<Number>{ 604.652, 161.891 } },
	{ hypro::Point<Number>{ 610.188 , 156.401 } , hypro::Point<Number>{ 604.652, 161.891 }, hypro::Point<Number>{ 568.701, 136.669 }, hypro::Point<Number>{ 566.547, 143.694 } },
	{ hypro::Point<Number>{ 568.701 , 136.669 } , hypro::Point<Number>{ 566.547, 143.694 }, hypro::Point<Number>{ 408.172, 96.0456 }, hypro::Point<Number>{ 406.389, 103.167 } },
	{ hypro::Point<Number>{ 408.172 , 96.0456 } , hypro::Point<Number>{ 406.389, 103.167 }, hypro::Point<Number>{ 300.92, 69.4396 }, hypro::Point<Number>{ 299.82, 76.7306 } },
	{ hypro::Point<Number>{ 300.92 , 69.4396 } , hypro::Point<Number>{ 299.82, 76.7306 }, hypro::Point<Number>{ 285.432, 68.5706 }, hypro::Point<Number>{ 289.648, 76.1606 } },
	{ hypro::Point<Number>{ 285.432 , 68.5706 } , hypro::Point<Number>{ 289.648, 76.1606 }, hypro::Point<Number>{ 275.844, 86.1176 }, hypro::Point<Number>{ 282.057, 90.0526 } },
	{ hypro::Point<Number>{ 275.844 , 86.1176 } , hypro::Point<Number>{ 282.057, 90.0526 }, hypro::Point<Number>{ 247.291, 125.352 }, hypro::Point<Number>{ 253.248, 129.639 } },
	{ hypro::Point<Number>{ 247.291 , 125.352 } , hypro::Point<Number>{ 253.248, 129.639 }, hypro::Point<Number>{ 224.256, 157.729 }, hypro::Point<Number>{ 230.564, 161.524 } },
	{ hypro::Point<Number>{ 224.256 , 157.729 } , hypro::Point<Number>{ 230.564, 161.524 }, hypro::Point<Number>{ 146.957, 311.825 }, hypro::Point<Number>{ 153.102, 315.946 } },
	{ hypro::Point<Number>{ 146.957 , 311.825 } , hypro::Point<Number>{ 153.102, 315.946 }, hypro::Point<Number>{ 62.6172, 408.674 }, hypro::Point<Number>{ 70.9023, 410.338 } },
	{ hypro::Point<Number>{ 62.6172 , 408.674 } , hypro::Point<Number>{ 70.9023, 410.338 }, hypro::Point<Number>{ 69.209, 428.561 }, hypro::Point<Number>{ 74.5508, 421.35 } },
	{ hypro::Point<Number>{ 69.209 , 428.561 } , hypro::Point<Number>{ 74.5508, 421.35 }, hypro::Point<Number>{ 144.08, 430.414 }, hypro::Point<Number>{ 143.48, 423.057 } },
	{ hypro::Point<Number>{ 144.08 , 430.414 } , hypro::Point<Number>{ 143.48, 423.057 }, hypro::Point<Number>{ 194.662, 420.803 }, hypro::Point<Number>{ 193.318, 413.588 } }};
	return res;
}
} // namespace
