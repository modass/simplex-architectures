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
		hypro::Point<Number>({67.8837,168.305}),
		hypro::Point<Number>({90.8231,147.324}),
		hypro::Point<Number>({87.0522,159.607}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({67.8837,168.305}),
		hypro::Point<Number>({87.0522,159.607}),
		hypro::Point<Number>({77.9315,169.072}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({67.8837,168.305}),
		hypro::Point<Number>({83.6799,180.204}),
		hypro::Point<Number>({58.4381,187.621}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({83.6799,180.204}),
		hypro::Point<Number>({67.8837,168.305}),
		hypro::Point<Number>({77.9315,169.072}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({58.4381,187.621}),
		hypro::Point<Number>({83.6799,180.204}),
		hypro::Point<Number>({64.7065,240.179}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({87.0522,159.607}),
		hypro::Point<Number>({90.8231,147.324}),
		hypro::Point<Number>({99.6742,154.999}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({98.6826,144.151}),
		hypro::Point<Number>({102.549,151.294}),
		hypro::Point<Number>({99.6742,154.999}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({102.549,151.294}),
		hypro::Point<Number>({98.6826,144.151}),
		hypro::Point<Number>({102.109,146.484}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({98.6826,144.151}),
		hypro::Point<Number>({99.6742,154.999}),
		hypro::Point<Number>({90.8231,147.324}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({70.8966,274.821}),
		hypro::Point<Number>({86.8964,286.939}),
		hypro::Point<Number>({75.7604,282.306}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({86.8964,286.939}),
		hypro::Point<Number>({70.8966,274.821}),
		hypro::Point<Number>({64.7065,240.179}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({64.7065,240.179}),
		hypro::Point<Number>({116.666,287.931}),
		hypro::Point<Number>({86.8964,286.939}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({116.666,287.931}),
		hypro::Point<Number>({64.7065,240.179}),
		hypro::Point<Number>({146.453,283.663}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({83.6799,180.204}),
		hypro::Point<Number>({146.453,283.663}),
		hypro::Point<Number>({64.7065,240.179}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({146.453,283.663}),
		hypro::Point<Number>({83.6799,180.204}),
		hypro::Point<Number>({183.262,255.8}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({210.847,272.377}),
		hypro::Point<Number>({202.71,261.708}),
		hypro::Point<Number>({224.006,266.74}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({183.262,255.8}),
		hypro::Point<Number>({202.71,261.708}),
		hypro::Point<Number>({191.607,281.311}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({173.198,277.875}),
		hypro::Point<Number>({183.262,255.8}),
		hypro::Point<Number>({191.607,281.311}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({199.839,280.039}),
		hypro::Point<Number>({191.607,281.311}),
		hypro::Point<Number>({202.71,261.708}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({210.847,272.377}),
		hypro::Point<Number>({199.839,280.039}),
		hypro::Point<Number>({202.71,261.708}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({183.262,255.8}),
		hypro::Point<Number>({173.198,277.875}),
		hypro::Point<Number>({146.453,283.663}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({216.178,149.617}),
		hypro::Point<Number>({212.305,142.387}),
		hypro::Point<Number>({213.49,134.334}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({299.177,77.0149}),
		hypro::Point<Number>({280.918,59.9003}),
		hypro::Point<Number>({291.571,57.769}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({299.177,77.0149}),
		hypro::Point<Number>({291.571,57.769}),
		hypro::Point<Number>({300.177,61.9965}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({216.178,149.617}),
		hypro::Point<Number>({213.49,134.334}),
		hypro::Point<Number>({303.642,86.9459}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({303.642,86.9459}),
		hypro::Point<Number>({213.49,134.334}),
		hypro::Point<Number>({269.36,71.5601}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({299.177,77.0149}),
		hypro::Point<Number>({269.36,71.5601}),
		hypro::Point<Number>({280.918,59.9003}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({303.642,86.9459}),
		hypro::Point<Number>({269.36,71.5601}),
		hypro::Point<Number>({299.177,77.0149}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({300.177,61.9965}),
		hypro::Point<Number>({303.048,69.2139}),
		hypro::Point<Number>({299.177,77.0149}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({343.239,171.285}),
		hypro::Point<Number>({303.642,86.9459}),
		hypro::Point<Number>({348.253,109.594}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({370.294,114.536}),
		hypro::Point<Number>({376.792,136.982}),
		hypro::Point<Number>({348.253,109.594}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({376.792,136.982}),
		hypro::Point<Number>({370.294,114.536}),
		hypro::Point<Number>({377.797,119.808}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({377.797,119.808}),
		hypro::Point<Number>({380.556,128.007}),
		hypro::Point<Number>({376.792,136.982}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({303.642,86.9459}),
		hypro::Point<Number>({229.033,178.359}),
		hypro::Point<Number>({216.178,149.617}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({343.239,171.285}),
		hypro::Point<Number>({348.253,109.594}),
		hypro::Point<Number>({376.792,136.982}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({235.004,231.018}),
		hypro::Point<Number>({215.825,225.886}),
		hypro::Point<Number>({215.969,211.317}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({215.825,225.886}),
		hypro::Point<Number>({207.76,223.197}),
		hypro::Point<Number>({215.969,211.317}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({235.004,231.018}),
		hypro::Point<Number>({215.969,211.317}),
		hypro::Point<Number>({229.033,178.359}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({237.572,268.986}),
		hypro::Point<Number>({224.006,266.74}),
		hypro::Point<Number>({227.594,247.934}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({224.006,266.74}),
		hypro::Point<Number>({202.71,261.708}),
		hypro::Point<Number>({227.594,247.934}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({261.726,248.513}),
		hypro::Point<Number>({227.594,247.934}),
		hypro::Point<Number>({235.004,231.018}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({229.033,178.359}),
		hypro::Point<Number>({272.77,234.661}),
		hypro::Point<Number>({235.004,231.018}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({256.395,265.328}),
		hypro::Point<Number>({227.594,247.934}),
		hypro::Point<Number>({261.726,248.513}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({343.239,171.285}),
		hypro::Point<Number>({272.77,234.661}),
		hypro::Point<Number>({229.033,178.359}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({245.97,271.822}),
		hypro::Point<Number>({237.572,268.986}),
		hypro::Point<Number>({256.395,265.328}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({256.395,265.328}),
		hypro::Point<Number>({252.394,269.931}),
		hypro::Point<Number>({245.97,271.822}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({227.594,247.934}),
		hypro::Point<Number>({256.395,265.328}),
		hypro::Point<Number>({237.572,268.986}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({261.726,248.513}),
		hypro::Point<Number>({235.004,231.018}),
		hypro::Point<Number>({272.77,234.661}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({229.033,178.359}),
		hypro::Point<Number>({303.642,86.9459}),
		hypro::Point<Number>({343.239,171.285}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({47.7307,46.8808}),
		hypro::Point<Number>({85.7075,139.205}),
		hypro::Point<Number>({60.1876,162.547}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({85.7075,139.205}),
		hypro::Point<Number>({47.7307,46.8808}),
		hypro::Point<Number>({99.8272,133.504}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({47.7307,46.8808}),
		hypro::Point<Number>({60.1876,162.547}),
		hypro::Point<Number>({48.7307,185.974}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({92.3393,167.73}),
		hypro::Point<Number>({91.0908,173.973}),
		hypro::Point<Number>({89.4268,170.752}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({47.7307,46.8808}),
		hypro::Point<Number>({48.7307,185.974}),
		hypro::Point<Number>({47.7307,298.397}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({99.8272,133.504}),
		hypro::Point<Number>({204.558,130.164}),
		hypro::Point<Number>({111.107,141.187}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({204.558,130.164}),
		hypro::Point<Number>({99.8272,133.504}),
		hypro::Point<Number>({47.7307,46.8808}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({111.107,141.187}),
		hypro::Point<Number>({204.558,130.164}),
		hypro::Point<Number>({202.507,144.098}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({91.0908,173.973}),
		hypro::Point<Number>({105.467,162.938}),
		hypro::Point<Number>({192.841,228.179}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({105.467,162.938}),
		hypro::Point<Number>({112.293,154.141}),
		hypro::Point<Number>({192.841,228.179}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({202.507,144.098}),
		hypro::Point<Number>({112.293,154.141}),
		hypro::Point<Number>({111.107,141.187}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({105.467,162.938}),
		hypro::Point<Number>({91.0908,173.973}),
		hypro::Point<Number>({92.3393,167.73}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({112.293,154.141}),
		hypro::Point<Number>({202.507,144.098}),
		hypro::Point<Number>({207.695,153.78}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({47.7307,46.8808}),
		hypro::Point<Number>({262.474,65.0924}),
		hypro::Point<Number>({204.558,130.164}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({207.587,206.835}),
		hypro::Point<Number>({112.293,154.141}),
		hypro::Point<Number>({207.695,153.78}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({61.9362,278.364}),
		hypro::Point<Number>({69.4349,289.903}),
		hypro::Point<Number>({47.7307,298.397}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({55.3855,241.701}),
		hypro::Point<Number>({61.9362,278.364}),
		hypro::Point<Number>({47.7307,298.397}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({47.7307,298.397}),
		hypro::Point<Number>({84.8604,296.321}),
		hypro::Point<Number>({117.183,297.397}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({84.8604,296.321}),
		hypro::Point<Number>({47.7307,298.397}),
		hypro::Point<Number>({69.4349,289.903}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({117.183,297.397}),
		hypro::Point<Number>({148.124,292.964}),
		hypro::Point<Number>({191.457,290.891}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({47.7307,298.397}),
		hypro::Point<Number>({117.183,297.397}),
		hypro::Point<Number>({391.642,298.397}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({55.3855,241.701}),
		hypro::Point<Number>({47.7307,298.397}),
		hypro::Point<Number>({48.7307,185.974}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({201.631,251.511}),
		hypro::Point<Number>({187.625,247.256}),
		hypro::Point<Number>({192.841,228.179}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({187.625,247.256}),
		hypro::Point<Number>({91.0908,173.973}),
		hypro::Point<Number>({192.841,228.179}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({213.108,234.935}),
		hypro::Point<Number>({201.631,251.511}),
		hypro::Point<Number>({192.841,228.179}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({173.336,287.507}),
		hypro::Point<Number>({191.457,290.891}),
		hypro::Point<Number>({148.124,292.964}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({191.457,290.891}),
		hypro::Point<Number>({203.44,289.039}),
		hypro::Point<Number>({391.642,298.397}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({112.293,154.141}),
		hypro::Point<Number>({207.587,206.835}),
		hypro::Point<Number>({192.841,228.179}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({117.183,297.397}),
		hypro::Point<Number>({191.457,290.891}),
		hypro::Point<Number>({391.642,298.397}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({292.859,47.8808}),
		hypro::Point<Number>({276.239,51.2054}),
		hypro::Point<Number>({47.7307,46.8808}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({307.607,55.124}),
		hypro::Point<Number>({292.859,47.8808}),
		hypro::Point<Number>({391.642,46.8808}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({313.38,69.6372}),
		hypro::Point<Number>({310.889,80.0336}),
		hypro::Point<Number>({309.621,77.2142}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({276.239,51.2054}),
		hypro::Point<Number>({262.474,65.0924}),
		hypro::Point<Number>({47.7307,46.8808}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({313.38,69.6372}),
		hypro::Point<Number>({391.642,46.8808}),
		hypro::Point<Number>({351.473,100.638}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({391.642,46.8808}),
		hypro::Point<Number>({313.38,69.6372}),
		hypro::Point<Number>({307.607,55.124}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({351.473,100.638}),
		hypro::Point<Number>({391.642,46.8808}),
		hypro::Point<Number>({374.189,105.732}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({390.642,128.372}),
		hypro::Point<Number>({385.761,113.862}),
		hypro::Point<Number>({391.642,46.8808}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({374.189,105.732}),
		hypro::Point<Number>({391.642,46.8808}),
		hypro::Point<Number>({385.761,113.862}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({384.805,142.297}),
		hypro::Point<Number>({390.642,128.372}),
		hypro::Point<Number>({391.642,298.397}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({391.642,46.8808}),
		hypro::Point<Number>({391.642,298.397}),
		hypro::Point<Number>({390.642,128.372}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({351.473,100.638}),
		hypro::Point<Number>({310.889,80.0336}),
		hypro::Point<Number>({313.38,69.6372}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({292.859,47.8808}),
		hypro::Point<Number>({47.7307,46.8808}),
		hypro::Point<Number>({391.642,46.8808}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({213.108,234.935}),
		hypro::Point<Number>({221.943,237.298}),
		hypro::Point<Number>({220.226,241.218}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({225.186,276.507}),
		hypro::Point<Number>({235.275,278.178}),
		hypro::Point<Number>({215.45,280.679}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({215.45,280.679}),
		hypro::Point<Number>({235.275,278.178}),
		hypro::Point<Number>({245.775,281.724}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({201.631,251.511}),
		hypro::Point<Number>({213.108,234.935}),
		hypro::Point<Number>({220.226,241.218}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({264.805,270.048}),
		hypro::Point<Number>({270.2,253.033}),
		hypro::Point<Number>({279.666,241.16}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({391.642,298.397}),
		hypro::Point<Number>({279.666,241.16}),
		hypro::Point<Number>({349.778,178.104}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({245.775,281.724}),
		hypro::Point<Number>({257.709,278.21}),
		hypro::Point<Number>({391.642,298.397}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({264.805,270.048}),
		hypro::Point<Number>({391.642,298.397}),
		hypro::Point<Number>({257.709,278.21}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({391.642,298.397}),
		hypro::Point<Number>({264.805,270.048}),
		hypro::Point<Number>({279.666,241.16}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({391.642,298.397}),
		hypro::Point<Number>({203.44,289.039}),
		hypro::Point<Number>({245.775,281.724}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({349.778,178.104}),
		hypro::Point<Number>({384.805,142.297}),
		hypro::Point<Number>({391.642,298.397}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({245.775,281.724}),
		hypro::Point<Number>({203.44,289.039}),
		hypro::Point<Number>({215.45,280.679}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({207.587,206.835}),
		hypro::Point<Number>({207.695,153.78}),
		hypro::Point<Number>({218.787,178.581}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}
	return res;
}
} // namespace
