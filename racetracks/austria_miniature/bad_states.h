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
		hypro::Point<Number>({131.813,172.053}),
		hypro::Point<Number>({143.257,113.888}),
		hypro::Point<Number>({149.429,139.353}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({143.257,113.888}),
		hypro::Point<Number>({131.813,172.053}),
		hypro::Point<Number>({104.52,191.111}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({104.52,191.111}),
		hypro::Point<Number>({129.213,180.469}),
		hypro::Point<Number>({135.375,196.444}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({129.213,180.469}),
		hypro::Point<Number>({104.52,191.111}),
		hypro::Point<Number>({131.813,172.053}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({158.036,133.384}),
		hypro::Point<Number>({154.502,98.0826}),
		hypro::Point<Number>({177.686,135.625}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({149.429,139.353}),
		hypro::Point<Number>({143.257,113.888}),
		hypro::Point<Number>({158.036,133.384}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({158.036,133.384}),
		hypro::Point<Number>({143.257,113.888}),
		hypro::Point<Number>({154.502,98.0826}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({62.5388,239.451}),
		hypro::Point<Number>({62.8159,238.999}),
		hypro::Point<Number>({63.9789,240.909}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({98.0284,241.752}),
		hypro::Point<Number>({63.9789,240.909}),
		hypro::Point<Number>({62.8159,238.999}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({104.52,191.111}),
		hypro::Point<Number>({98.0284,241.752}),
		hypro::Point<Number>({62.8159,238.999}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({122.74,237.056}),
		hypro::Point<Number>({143.184,201.804}),
		hypro::Point<Number>({160.173,230.224}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({143.184,201.804}),
		hypro::Point<Number>({122.74,237.056}),
		hypro::Point<Number>({135.375,196.444}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({160.173,230.224}),
		hypro::Point<Number>({143.184,201.804}),
		hypro::Point<Number>({170.709,199.836}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({170.709,199.836}),
		hypro::Point<Number>({194.506,226.758}),
		hypro::Point<Number>({160.173,230.224}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({122.74,237.056}),
		hypro::Point<Number>({98.0284,241.752}),
		hypro::Point<Number>({104.52,191.111}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({135.375,196.444}),
		hypro::Point<Number>({122.74,237.056}),
		hypro::Point<Number>({104.52,191.111}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({177.686,135.625}),
		hypro::Point<Number>({154.502,98.0826}),
		hypro::Point<Number>({168.957,78.2196}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({176.004,72.4016}),
		hypro::Point<Number>({177.686,135.625}),
		hypro::Point<Number>({168.957,78.2196}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({229.069,85.5646}),
		hypro::Point<Number>({194.625,153.323}),
		hypro::Point<Number>({177.686,135.625}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({177.686,135.625}),
		hypro::Point<Number>({176.004,72.4016}),
		hypro::Point<Number>({229.069,85.5646}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({176.004,72.4016}),
		hypro::Point<Number>({168.957,78.2196}),
		hypro::Point<Number>({172.251,72.1916}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({294.322,159.235}),
		hypro::Point<Number>({229.069,85.5646}),
		hypro::Point<Number>({308.896,105.766}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({308.896,105.766}),
		hypro::Point<Number>({330.376,121.337}),
		hypro::Point<Number>({321.916,153.273}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({330.376,121.337}),
		hypro::Point<Number>({308.896,105.766}),
		hypro::Point<Number>({327.175,114.495}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({229.069,85.5646}),
		hypro::Point<Number>({211.997,159.418}),
		hypro::Point<Number>({194.625,153.323}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({294.322,159.235}),
		hypro::Point<Number>({308.896,105.766}),
		hypro::Point<Number>({321.916,153.273}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({194.506,226.758}),
		hypro::Point<Number>({224.789,209.054}),
		hypro::Point<Number>({229.282,224.078}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({224.789,209.054}),
		hypro::Point<Number>({194.506,226.758}),
		hypro::Point<Number>({200.762,197.042}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({194.506,226.758}),
		hypro::Point<Number>({170.709,199.836}),
		hypro::Point<Number>({200.762,197.042}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({313.138,159.225}),
		hypro::Point<Number>({294.322,159.235}),
		hypro::Point<Number>({321.916,153.273}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({229.282,224.078}),
		hypro::Point<Number>({231.539,217.965}),
		hypro::Point<Number>({230.887,223.254}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({231.539,217.965}),
		hypro::Point<Number>({229.282,224.078}),
		hypro::Point<Number>({224.789,209.054}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({211.997,159.418}),
		hypro::Point<Number>({229.069,85.5646}),
		hypro::Point<Number>({294.322,159.235}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({98.3806,186.995}),
		hypro::Point<Number>({52.2737,63.610600000000005}),
		hypro::Point<Number>({136.954,110.096}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({136.954,110.096}),
		hypro::Point<Number>({52.2737,63.610600000000005}),
		hypro::Point<Number>({148.549,93.7996}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({141.399,191.684}),
		hypro::Point<Number>({136.971,180.203}),
		hypro::Point<Number>({138.608,174.901}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({162.749,74.2886}),
		hypro::Point<Number>({148.549,93.7996}),
		hypro::Point<Number>({52.2737,63.610600000000005}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({145.219,194.306}),
		hypro::Point<Number>({138.608,174.901}),
		hypro::Point<Number>({170.108,192.527}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({174.221,142.611}),
		hypro::Point<Number>({155.051,144.378}),
		hypro::Point<Number>({159.947,140.983}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({162.749,74.2886}),
		hypro::Point<Number>({52.2737,63.610600000000005}),
		hypro::Point<Number>({168.037,64.6106}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({52.2737,63.610600000000005}),
		hypro::Point<Number>({339.147,63.610600000000005}),
		hypro::Point<Number>({168.037,64.6106}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({138.608,174.901}),
		hypro::Point<Number>({155.051,144.378}),
		hypro::Point<Number>({170.108,192.527}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({98.3806,186.995}),
		hypro::Point<Number>({56.8896,234.639}),
		hypro::Point<Number>({52.2737,63.610600000000005}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({52.2737,250.102}),
		hypro::Point<Number>({53.2737,240.509}),
		hypro::Point<Number>({60.8421,248.167}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({53.2737,240.509}),
		hypro::Point<Number>({52.2737,250.102}),
		hypro::Point<Number>({52.2737,63.610600000000005}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({52.2737,250.102}),
		hypro::Point<Number>({60.8421,248.167}),
		hypro::Point<Number>({98.6285,249.102}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({56.8896,234.639}),
		hypro::Point<Number>({53.2737,240.509}),
		hypro::Point<Number>({52.2737,63.610600000000005}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({145.219,194.306}),
		hypro::Point<Number>({141.399,191.684}),
		hypro::Point<Number>({138.608,174.901}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({339.147,250.102}),
		hypro::Point<Number>({98.6285,249.102}),
		hypro::Point<Number>({124.082,244.266}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({339.147,250.102}),
		hypro::Point<Number>({124.082,244.266}),
		hypro::Point<Number>({161.201,237.491}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({52.2737,250.102}),
		hypro::Point<Number>({98.6285,249.102}),
		hypro::Point<Number>({339.147,250.102}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({230.851,78.4526}),
		hypro::Point<Number>({177.102,65.1186}),
		hypro::Point<Number>({339.147,63.610600000000005}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({190.542,159.661}),
		hypro::Point<Number>({170.108,192.527}),
		hypro::Point<Number>({155.051,144.378}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({230.851,78.4526}),
		hypro::Point<Number>({339.147,63.610600000000005}),
		hypro::Point<Number>({311.728,98.9916}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({190.542,159.661}),
		hypro::Point<Number>({155.051,144.378}),
		hypro::Point<Number>({174.221,142.611}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({332.705,109.009}),
		hypro::Point<Number>({339.147,63.610600000000005}),
		hypro::Point<Number>({338.147,120.642}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({328.303,157.803}),
		hypro::Point<Number>({338.147,120.642}),
		hypro::Point<Number>({339.147,250.102}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({315.391,166.557}),
		hypro::Point<Number>({328.303,157.803}),
		hypro::Point<Number>({339.147,250.102}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({339.147,63.610600000000005}),
		hypro::Point<Number>({339.147,250.102}),
		hypro::Point<Number>({338.147,120.642}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({332.705,109.009}),
		hypro::Point<Number>({311.728,98.9916}),
		hypro::Point<Number>({339.147,63.610600000000005}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({177.102,65.1186}),
		hypro::Point<Number>({168.037,64.6106}),
		hypro::Point<Number>({339.147,63.610600000000005}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({202.169,189.546}),
		hypro::Point<Number>({210.756,166.754}),
		hypro::Point<Number>({229.601,203.261}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({202.169,189.546}),
		hypro::Point<Number>({170.108,192.527}),
		hypro::Point<Number>({190.542,159.661}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({210.756,166.754}),
		hypro::Point<Number>({294.332,166.568}),
		hypro::Point<Number>({229.601,203.261}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({202.169,189.546}),
		hypro::Point<Number>({190.542,159.661}),
		hypro::Point<Number>({210.756,166.754}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({239.182,215.91}),
		hypro::Point<Number>({294.332,166.568}),
		hypro::Point<Number>({339.147,250.102}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({294.332,166.568}),
		hypro::Point<Number>({239.182,215.91}),
		hypro::Point<Number>({229.601,203.261}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({231.318,231.277}),
		hypro::Point<Number>({237.689,228.004}),
		hypro::Point<Number>({339.147,250.102}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({339.147,250.102}),
		hypro::Point<Number>({237.689,228.004}),
		hypro::Point<Number>({239.182,215.91}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({231.318,231.277}),
		hypro::Point<Number>({339.147,250.102}),
		hypro::Point<Number>({195.157,234.063}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({294.332,166.568}),
		hypro::Point<Number>({315.391,166.557}),
		hypro::Point<Number>({339.147,250.102}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}{
		std::vector<hypro::Point<Number>> points{
		hypro::Point<Number>({195.157,234.063}),
		hypro::Point<Number>({339.147,250.102}),
		hypro::Point<Number>({161.201,237.491}),
	};
	hypro::HPolytope<Number> poly{points};
	res.push_back(hypro::Condition<Number>(poly.matrix(), poly.vector()));
	}
	return res;
}
} // namespace
