#include <string>
#include <iostream>
#include <thread>
#include <chrono> 

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/meshcat/joint_sliders.h"
#include "drake/geometry/meshcat_visualizer.h"

using namespace drake;

int main(int argc, char* argv[]){
  systems::DiagramBuilder<double> builder;
  auto meshcat = std::make_shared<geometry::Meshcat>();

  auto [plant, scene_graph] = multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);
  multibody::Parser parser(&plant);
  multibody::parsing::ModelDirectives directives =
    multibody::parsing::LoadModelDirectives(
        FindResourceOrThrow(
          "drake/manipulation/models/tri_homecart/homecart.dmd.yaml"
        )
    );
  multibody::parsing::ProcessModelDirectives(directives, &plant, nullptr, &parser);
  plant.Finalize();
  geometry::MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph.get_query_output_port(), meshcat);

  auto* slider = builder.AddSystem<multibody::meshcat::JointSliders<double>>(meshcat, &plant);

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();

  // Run Visualization:
  slider->Run(*diagram);

  return 0;
}