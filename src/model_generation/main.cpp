/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 23.03.22.
 */
#include <string>
#include <CLI/CLI.hpp>
#include <CLI/App.hpp>
#include <spdlog/spdlog.h>
#include "generateBicycle.h"

enum MODEL { PURE_PURSUIT };

int main( int argc, char** argv ) {
  spdlog::set_level( spdlog::level::debug );

  std::map<std::string, MODEL> map{ { "pursuit", MODEL::PURE_PURSUIT } };
  std::string keys{"pursuit"};

  CLI::App app{ "Model generation tool" };

  std::string                                     storagefilename = "out";
  std::size_t delta_discretization = 7;
  std::size_t theta_discretization = 12;
  std::pair<double,double> delta_range{-60.0, 60.0};
  MODEL model = MODEL::PURE_PURSUIT;
  app.add_option( "-d,--delta_discretization", delta_discretization, "Delta discretization for hybridization" )->check( CLI::PositiveNumber );
  app.add_option( "-t,--theta_discretization", theta_discretization, "Theta discretization for hybridization" )->check( CLI::PositiveNumber );
  app.add_option( "-m,--model", model,
                  "What model should be generated. Possible values: " + keys )
      ->transform( CLI::CheckedTransformer( map, CLI::ignore_case ) );
  app.add_option( "-o,--outfile", storagefilename, "Path to output file" );

  CLI11_PARSE( app, argc, argv );

  switch ( model ) {
    case MODEL::PURE_PURSUIT:{
      modelGenerator::generateBicycleModelFile(storagefilename,delta_range,delta_discretization, theta_discretization);
      break;
    }
  }
}