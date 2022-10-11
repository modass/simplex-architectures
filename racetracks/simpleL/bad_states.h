#include <hypro/representations/GeometricObjectBase.h>
#include <vector>

#pragma once

namespace simplexArchitectures {
template<typename Automaton>
typename Automaton::conditionVector createBadStates() {
	using Number = typename Automaton::NumberType;
	auto res = typename Automaton::conditionVector();

{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({21.9423,28.5727}),
		hypro::Point<Number>({56.7873,31.6536}),
		hypro::Point<Number>({23.8719,37.5105}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({56.7873,31.6536}),
		hypro::Point<Number>({21.9423,28.5727}),
		hypro::Point<Number>({59.2047,21.655}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({77.9377,60.5205}),
		hypro::Point<Number>({65.6292,67.4722}),
		hypro::Point<Number>({56.7873,31.6536}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({77.9377,60.5205}),
		hypro::Point<Number>({56.7873,31.6536}),
		hypro::Point<Number>({59.2047,21.655}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({12.8462,11.8814}),
		hypro::Point<Number>({13.8462,22.0116}),
		hypro::Point<Number>({12.8462,83.7075}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({61.0571,82.7075}),
		hypro::Point<Number>({12.8462,83.7075}),
		hypro::Point<Number>({18.5212,46.6404}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({18.5212,46.6404}),
		hypro::Point<Number>({12.8462,83.7075}),
		hypro::Point<Number>({13.8462,22.0116}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({61.0571,82.7075}),
		hypro::Point<Number>({18.5212,46.6404}),
		hypro::Point<Number>({50.5554,40.2056}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({13.8462,22.0116}),
		hypro::Point<Number>({12.8462,11.8814}),
		hypro::Point<Number>({63.51,12.8814}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({88.7194,11.8814}),
		hypro::Point<Number>({63.51,12.8814}),
		hypro::Point<Number>({12.8462,11.8814}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({61.0571,82.7075}),
		hypro::Point<Number>({87.7194,63.0645}),
		hypro::Point<Number>({88.7194,83.7075}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({88.7194,83.7075}),
		hypro::Point<Number>({87.7194,63.0645}),
		hypro::Point<Number>({88.7194,11.8814}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({61.0571,82.7075}),
		hypro::Point<Number>({88.7194,83.7075}),
		hypro::Point<Number>({12.8462,83.7075}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({63.51,12.8814}),
		hypro::Point<Number>({88.7194,11.8814}),
		hypro::Point<Number>({87.7194,63.0645}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}
	return res;
}
} // namespace
