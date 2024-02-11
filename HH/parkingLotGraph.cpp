#include "parkingLotGraph.h"
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

SegCam::SegCam(cc::World &world) : world_(world) { 
    SegCam::addCamera(); };

void SegCam::addCamera() {
  // Spawn a camera attached to the vehicle.
  auto blueprint_library = world_.GetBlueprintLibrary();
  auto camera_bp =
      blueprint_library->Find("sensor.camera.instance_segmentation");
  auto camera_transform =
      cg::Transform{cg::Location{-5.5f, 0.0f, 2.8f},   // x, y, z.
                    cg::Rotation{-15.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.

  // Spawn the camera actor
  this->camera_actor_ =
      world_.SpawnActor(*camera_bp, camera_transform);
}

void SegCam::takePicture() {
  // Assuming camera_actor_ is a member variable of SegCam
  auto sensor = boost::static_pointer_cast<cc::Sensor>(this->camera_actor_);
  if (sensor != nullptr) {
    sensor->Listen([&](auto data) {
        auto image_data = boost::static_pointer_cast<csd::Image>(data);
        if (image_data != nullptr) {
                // Save the image data to disk
                saveImageToFile(*image_data);
            }

    });
  }
}

void SegCam::saveImageToFile(const csd::Image &image) {
  using namespace carla::image;

  char buffer[9u];
  std::snprintf(buffer, sizeof(buffer), "%08zu", image.GetFrame());
  auto filename = "_images/"s + buffer + ".png";

  auto view = ImageView::MakeColorConvertedView(
      ImageView::MakeView(image),
      ColorConverter::CityScapesPalette());
  ImageIO::WriteView(filename, view);
}