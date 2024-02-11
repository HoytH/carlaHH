#include "carla/client/Actor.h"
#include "carla/client/World.h"
#include "carla/sensor/data/Image.h"
#include <carla/client/Sensor.h>
#include <queue>

namespace cc = carla::client;
namespace csd = carla::sensor::data;

class SegCam {
public:
  SegCam(cc::World& world);
  void addCamera();
  void takePicture();
  void saveImageToFile(const csd::Image &image);

  private:
    cc::World world_;
    std::queue<std::shared_ptr<csd::Image>> instance_image_queue_;
    carla::SharedPtr<cc::Actor> camera_actor_; // Added member variable for camera sensor


};

