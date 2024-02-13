#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

#define EXPECT_TRUE(pred)                                                      \
  if (!(pred)) {                                                               \
    throw std::runtime_error(#pred);                                           \
  }

static auto ParseArguments(int argc, const char *argv[]) {
  EXPECT_TRUE((argc == 1u) || (argc == 3u));
  using ResultType = std::tuple<std::string, uint16_t>;
  return argc == 3u ? ResultType{argv[1u], std::stoi(argv[2u])}
                    : ResultType{"localhost", 2000u};
}

int main(int argc, const char *argv[]) {
  try {
    std::string host;
    uint16_t port;
    std::tie(host, port) = ParseArguments(argc, argv);
    auto client = cc::Client(host, port);
    client.SetTimeout(40s);

    std::cout << "Client API version : " << client.GetClientVersion() << '\n';
    std::cout << "Server API version : " << client.GetServerVersion() << '\n';

    // Load a random town.
    auto world = client.LoadWorld("Town05");

    // Get a random vehicle blueprint.
    auto blueprint_library = world.GetBlueprintLibrary();
    carla::client::ActorBlueprint blueprint =
        (*blueprint_library->Filter("vehicle.ford.mustang"))[0];

    // Spawn the vehicle.
    carla::geom::Transform spawn_transform(
        carla::geom::Location(-1.69883, -13.9062, 0.5),
        carla::geom::Rotation(0.0, 270.0, 0.0));

    auto actor = world.SpawnActor(blueprint, spawn_transform);
    std::cout << "Spawned " << actor->GetDisplayId() << '\n';
    auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);

    // Apply control to vehicle.
    cc::Vehicle::Control control;
    control.throttle = 0.0f;
    vehicle->ApplyControl(control);

    // // Move spectator so we can see the vehicle from the simulator window.
    auto spectator = world.GetSpectator();

    // Spawn a camera attached to the vehicle.
    auto camera_bp =
        blueprint_library->Find("sensor.camera.rgb");
    auto camera_transform =
        cg::Transform{cg::Location{-5.5f, 0.0f, 2.8f},   // x, y, z.
                      cg::Rotation{-15.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.

    auto cam_actor =
        world.SpawnActor(*camera_bp, camera_transform, actor.get());
    auto camera = boost::static_pointer_cast<cc::Sensor>(cam_actor);


    // Create segmentation cam
    // SegCam segCam(world);
    float time = 0;
    float setTime = 1;
    while (true) {
      // Update the vehicle's control or perform other simulation steps

      // Get the latest vehicle transformcd
      auto camera_transform = camera->GetTransform();

      // Calculate the offset position for the spectator
      cg::Location offset_location = camera_transform.location;

      // Create the spectator transform with the offset and rotation
      cg::Transform spectator_transform{
          offset_location, camera_transform.rotation}; // pitch, yaw, roll

      // Set the spectator's transform
      spectator->SetTransform(spectator_transform);

      if (time > setTime){
        // segCam.takePicture();
        setTime = time + 5;
      }

      // print out where spectator is
      // auto loc = spectator->GetTransform();
      // std::cout << "location: " << loc.location.x << "," << loc.location.y <<
      // "," << loc.location.z;

      // Other simulation/rendering logic here
      std::this_thread::sleep_for(
           std::chrono::milliseconds(16)); // Adjust the sleep duration as needed
           time += 0.016;
    }

  } catch (const cc::TimeoutException &e) {
    std::cout << '\n' << e.what() << std::endl;
    return 1;
  } catch (const std::exception &e) {
    std::cout << "\nException: " << e.what() << std::endl;
    return 2;
  }
}
