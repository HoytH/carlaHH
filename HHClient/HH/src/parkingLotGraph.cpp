#include "parkingLotGraph.h"
#include "carla/Buffer.h"
#include "carla/ros2/ROS2.h"
#include "carla/sensor/data/SemanticLidarData.h"
#include "carla/sensor/data/SemanticLidarMeasurement.h"
#include <fstream>
#include <iostream>
#include <ostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <opencv2/opencv.hpp>

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


ColorRGB::ColorRGB(const int r, const int g, const int b): r(r), g(g), b(b){};

void Test::imageParse(){
  std::map<std::string, ColorRGB> colorKey;
  colorKey["None"] = ColorRGB(0,0,0);
  colorKey["Building"] = ColorRGB(70,70,70);
  colorKey["Fence"] = ColorRGB(190,153,153);
  colorKey["Other"] = ColorRGB(72,0,90);
  colorKey["Pedestrian"] = ColorRGB(220,20,60);
  colorKey["Pole"] = ColorRGB(153,153,153);
  colorKey["Roadline"] = ColorRGB(157,234,50);
  colorKey["Road"] = ColorRGB(128,64,128);
  colorKey["Sidewalk"] = ColorRGB(244,35,232);
  colorKey["Vegetation"] = ColorRGB(107,142,35);
  colorKey["Vehicle"] = ColorRGB(0,0,255);
    // Read the image
  cv::Mat image = cv::imread("/home/hoyt/Desktop/00001396.png");

  if (image.empty()) {
      std::cout << "Could not open or find the image" << std::endl;
      std::exit;
  }

  // Define the RGB value you want to find
  cv::Vec3b target_color(50, 234, 157); // Example: Blue color

  // Create a mask with pixels equal to the target color
  cv::Mat mask;
  cv::inRange(image, target_color, target_color, mask);

  // Create a black overlay image
  cv::Mat overlay(image.rows, image.cols, CV_8UC3, cv::Scalar(0, 0, 0));

  // Overlay the pixels found with the target color on the black image
  for (int y = 0; y < image.rows; ++y) {
      for (int x = 0; x < image.cols; ++x) {
          if (mask.at<uchar>(y, x) != 0) {
              overlay.at<cv::Vec3b>(y, x) = target_color;
          }
      }
  }

  // Display the overlay image
  cv::imshow("Overlay Image", overlay);
  cv::waitKey(0);

}

SegCam::SegCam(cc::World &world) : world_(world) { 
    SegCam::addCamera(); };

void SegCam::addCamera() {
  // Spawn a camera attached to the vehicle.
  auto blueprint_library = world_.GetBlueprintLibrary();
  auto camera_bp =
      blueprint_library->Find("sensor.camera.instance_segmentation");
  auto camera_transform =
      cg::Transform{cg::Location{-1.69883, -33.9062, 30},   // x, y, z.
                    cg::Rotation{-90.0f, 270.0f, 0.0f}}; // pitch, yaw, roll.

  // Spawn the camera actor
  this->camera_actor_ =
      world_.SpawnActor(*camera_bp, camera_transform);
}

void SegCam::imageParse(){
  std::map<std::string, ColorRGB> colorKey;
  colorKey["None"] = ColorRGB(0,0,0);
  colorKey["Building"] = ColorRGB(70,70,70);
  colorKey["Fence"] = ColorRGB(190,153,153);
  colorKey["Other"] = ColorRGB(72,0,90);
  colorKey["Pedestrian"] = ColorRGB(220,20,60);
  colorKey["Pole"] = ColorRGB(153,153,153);
  colorKey["Roadline"] = ColorRGB(157,234,50);
  colorKey["Road"] = ColorRGB(128,64,128);
  colorKey["Sidewalk"] = ColorRGB(244,35,232);
  colorKey["Vegetation"] = ColorRGB(107,142,35);
  colorKey["Vehicle"] = ColorRGB(0,0,255);
    // Read the image
  cv::Mat image = cv::imread("/home/hoyt/Desktop/00001396.png");

  if (image.empty()) {
      std::cout << "Could not open or find the image" << std::endl;
      std::exit;
  }

  // Define the RGB value you want to find
  cv::Vec3b target_color(157, 234, 50); // Example: Blue color

  // Create a mask with pixels equal to the target color
  cv::Mat mask;
  cv::inRange(image, target_color, target_color, mask);

  // Create a black overlay image
  cv::Mat overlay(image.rows, image.cols, CV_8UC3, cv::Scalar(0, 0, 0));

  // Overlay the pixels found with the target color on the black image
  for (int y = 0; y < image.rows; ++y) {
      for (int x = 0; x < image.cols; ++x) {
          if (mask.at<uchar>(y, x) != 0) {
              overlay.at<cv::Vec3b>(y, x) = target_color;
          }
      }
  }

  // Display the overlay image
  cv::imshow("Overlay Image", overlay);
  cv::waitKey(0);

}

void SegCam::takePicture() {
  // Assuming camera_actor_ is a member variable of SegCam
  auto sensor = boost::static_pointer_cast<cc::Sensor>(this->camera_actor_);
  if (sensor != nullptr) {
    sensor->Listen([&](auto data) {
        if (data != nullptr) { // Check if data is not null
            auto image_data = boost::static_pointer_cast<csd::Image>(data);
            if (image_data != nullptr) {
                // Save the image data to disk
                saveImageToFile(*image_data);
            } else {
                std::cerr << "Received null image data pointer!" << std::endl;
            }
        } else {
            std::cerr << "Received null data pointer!" << std::endl;
        }
    });
  } else {
    std::cerr << "Sensor is null!" << std::endl;
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

LidarCam::LidarCam(cc::World &world, float x, float y, float z ) : world_(world) { 
    LidarCam::addCamera(x,y,z); };

void LidarCam::addCamera(float x, float y, float z ) {
  // Spawn a camera attached to the vehicle.
  auto blueprint_library = world_.GetBlueprintLibrary();
  cc::BlueprintLibrary::const_pointer camera_bp =
      blueprint_library->Find("sensor.lidar.ray_cast_semantic");
  auto camera_transform =
      cg::Transform{cg::Location{x,y,z},   // x, y, z.
                    cg::Rotation{-90.0f, 270.0f, 0.0f}}; // pitch, yaw, roll.
  // Make a copy of the blueprint
  cc::BlueprintLibrary::value_type modified_camera_bp = *camera_bp;

  // Set attributes on the copied blueprint
  modified_camera_bp.SetAttribute("range", "50");
  modified_camera_bp.SetAttribute("channels", "100");

  // Spawn the camera actor
  this->camera_actor_ =
      world_.SpawnActor(modified_camera_bp, camera_transform);
}

void LidarCam::takePicture() {
  // Assuming camera_actor_ is a member variable of LidarCam
  this->sensor_ref = boost::static_pointer_cast<cc::Sensor>(this->camera_actor_);
  if (this->sensor_ref != nullptr) {
    this->sensor_ref->Listen([&](auto data) {
      // auto pcd = boost::static_pointer_cast<csd::SemanticLidarData>(data);
      auto pcd = boost::static_pointer_cast<csd::SemanticLidarMeasurement>(data);
      saveImageToFile(*pcd);
    });
  } else {
    std::cerr << "sensor_ref is null!" << std::endl;
  }
}

void LidarCam::saveImageToFile(const carla::sensor::data::SemanticLidarMeasurement& lidar_data) {
    // Assuming you have a way to obtain the Lidar data, let's say it's stored in a variable called lidar_measurement

    // Assuming filename to write to
    std::string filename = "lidar_data.xyz";

    // Open the file for writing
    std::ofstream outfile(filename, std::ios::app);
    if (!outfile.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
    }
    std::cout << "PRINT POINTS: " << "\n";
    for (const csd::SemanticLidarDetection a: lidar_data){
        outfile << a.point.x << " " << a.point.y << " " << a.point.z << " " << a.object_tag << std::endl;
    };

    // Close the file
    outfile.close();
    std::cout << "Successfully wrote Lidar data to file: " << filename << std::endl;
}

std::string LidarCam::getCurrentDateTimeAsString() {
    // Implementation to get current date and time as a string
    // You can implement this function using your preferred method
    // Here's an example using std::chrono and std::put_time:

    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

void LidarCam::disconnectSensor() {
  if (sensor_ref != nullptr) {
    std::cout <<"DISCONNECTING"<< std::endl;
    sensor_ref->Stop();  // Disconnect externally
  }
}