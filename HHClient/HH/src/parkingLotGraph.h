#pragma once
#include "carla/Memory.h"
#include "carla/client/Actor.h"
#include "carla/client/World.h"
#include "carla/sensor/data/Image.h"
#include "carla/sensor/data/SemanticLidarMeasurement.h"
#include <carla/client/Sensor.h>
#include <memory>
#include <queue>

namespace cc = carla::client;
namespace csd = carla::sensor::data;

class SegCam {
public:
  SegCam(cc::World& world);
  void addCamera();
  void takePicture();
  void saveImageToFile(const csd::Image &image);
  void imageParse();

  private:
    cc::World world_;
    std::queue<std::shared_ptr<csd::Image>> instance_image_queue_;
    carla::SharedPtr<cc::Actor> camera_actor_; // Added member variable for camera sensor


};

struct ColorRGB{
  ColorRGB() : r(0), g(0), b(0) {}
  ColorRGB(const int r, const int g, const int b);
  int r;
  int g;
  int b;
};

struct Test{
  void imageParse();
};

class LidarCam {
  public:
  LidarCam(cc::World& world, float x, float y, float z );
  void addCamera(float x, float y, float z);
  void takePicture();
  void saveImageToFile(const carla::sensor::data::SemanticLidarMeasurement& lidar_data);
  std::string getCurrentDateTimeAsString();
  void disconnectSensor();
  private:
    carla::SharedPtr<cc::Sensor> sensor_ref;
    cc::World world_;
    std::queue<std::shared_ptr<csd::Image>> instance_image_queue_;
    carla::SharedPtr<cc::Actor> camera_actor_; // Added member variable for camera sensor
};
