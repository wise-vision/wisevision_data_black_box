/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef ZENOH_EXTRACTOR_HPP
#define ZENOH_EXTRACTOR_HPP

#include <curl/curl.h>
#include <fstream>
#include <json/json.h>
#include <lora_msgs/msg/full_date_time.hpp>
#include <lora_msgs/msg/gps_device_data.hpp>
#include <lora_msgs/msg/micro_publisher.hpp>
#include <lora_msgs/srv/get_messages.hpp>
#include <memory>
#include <openssl/bio.h>
#include <openssl/evp.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <regex>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sstream>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <vector>
#include <wisevision_msgs/msg/gps_devices_publisher.hpp>
#include <wisevision_msgs/srv/add_data_to_data_base.hpp>
#include <wisevision_msgs/srv/add_storage_to_data_base.hpp>
#include <wisevision_msgs/srv/create_data_base.hpp>
#include <wisevision_msgs/srv/delete_data_from_data_base.hpp>

using GetMessages = lora_msgs::srv::GetMessages;
using CreateDataBase = wisevision_msgs::srv::CreateDataBase;
using AddStorageToDataBase = wisevision_msgs::srv::AddStorageToDataBase;
using AddDataToDataBase = wisevision_msgs::srv::AddDataToDataBase;
using DeleteDataFromDataBase = wisevision_msgs::srv::DeleteDataFromDataBase;
using GpsDeviceData = lora_msgs::msg::GpsDeviceData;
using GpsDevicesPublisher = wisevision_msgs::msg::GpsDevicesPublisher;

using CurlPerformFunc = CURLcode (*)(CURL*);

class DataBaseHandler : public rclcpp::Node {
public:
  // Constructor
  DataBaseHandler(const std::string& storage_type, const std::string& url);

  // Destructor
  ~DataBaseHandler();

  CurlPerformFunc curlPerform;

private:
  // Function to handle the GetMessages service
  //
  // @request: Request message
  //
  // @response: Response message
  void handleGetMessages(const std::shared_ptr<GetMessages::Request> request,
                         std::shared_ptr<GetMessages::Response> response);

  // Function to handle Int32 messages
  //
  // @jsonData: JSON data from the Zenoh query
  //
  // @topic_name: Topic name
  //
  // @request: Request message
  //
  // @response: Response message
  void handleInt32Messages(const Json::Value& jsonData,
                           const std::string& topic_name,
                           const std::shared_ptr<GetMessages::Request> request,
                           std::shared_ptr<GetMessages::Response> response);

  // Function to handle SensorReadingsTempPressureBinary messages
  //
  // @jsonData: JSON data from the Zenoh query
  //
  // @topic_name: Topic name
  //
  // @request: Request message
  //
  // @response: Response message
  void handleMicroPublisherMessages(const Json::Value& jsonData,
                                    const std::string& topic_name,
                                    const std::shared_ptr<GetMessages::Request> request,
                                    std::shared_ptr<GetMessages::Response> response);

  void handleGpsDevicesPublisherMessages(const Json::Value& jsonData,
                                         const std::string& topic_name,
                                         const std::shared_ptr<GetMessages::Request> request,
                                         std::shared_ptr<GetMessages::Response> response);

  // Prepare response for Int32 messages
  //
  // @response: Response message
  //
  // @int32_msgs: Vector of Int32 messages
  //
  // @timestamps: Vector of timestamps
  //
  // @request: Request message
  void prepareResponseInt32(std::shared_ptr<GetMessages::Response> response,
                            const std::vector<int32_t>& int32_msgs,
                            const std::vector<lora_msgs::msg::FullDateTime>& timestamps,
                            const std::shared_ptr<GetMessages::Request> request);

  // Prepare response for MicroPublisher messages
  //
  // @response: Response message
  //
  // @micro_publisher_msgs: Vector of MicroPublisher messages
  //
  // @timestamps: Vector of timestamps
  //
  // @request: Request message
  void prepareResponseMicroPublisher(std::shared_ptr<GetMessages::Response> response,
                                     const std::vector<lora_msgs::msg::MicroPublisher>& micro_publisher_msgs,
                                     const std::vector<lora_msgs::msg::FullDateTime>& timestamps,
                                     const std::shared_ptr<GetMessages::Request> request);
  // Function to load configuration
  void loadConfiguration();

  // Parse ISO8601 string to FullDateTime
  //
  // @time_str: ISO8601 string
  static lora_msgs::msg::FullDateTime parseIso8601ToFullDatetime(const std::string& time_str);

  // Convert FullDateTime to string
  //
  // @datetime: FullDateTime message
  static std::string fullDatetimeToString(const lora_msgs::msg::FullDateTime& datetime);

  // Callback function for writing data
  //
  // @contents: Data
  //
  // @size: Size of data
  //
  // @nmemb: Number of members
  //
  // @userp: User pointer
  static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp);

  // new funcionality
  // Function to handle any type of messages
  //
  // @jsonData: JSON data from the Zenoh query
  //
  // @topic_name: Topic name
  //
  void handleMessages(const Json::Value& jsonData,
                      const std::string& topic_name,
                      const std::shared_ptr<GetMessages::Request> request,
                      std::shared_ptr<GetMessages::Response> response);
  // Function to prepare response for any type of messages
  //
  // @response: Response message
  //
  // @serialized_data: Vector of serialized data
  //
  // @timestamps: Vector of timestamps
  void prepareResponse(std::shared_ptr<GetMessages::Response> response,
                       const std::vector<uint8_t>& serialized_data,
                       const std::vector<lora_msgs::msg::FullDateTime>& timestamps);

  // Function to parse string to JSON
  //
  // @input: Input string
  std::optional<Json::Value> parseToJson(const std::string& input);

  // Function to CreateDataBase
  //
  // @request: Request message
  //
  // @response: Response message
  void handleCreateDataBase(const std::shared_ptr<CreateDataBase::Request> request,
                            std::shared_ptr<CreateDataBase::Response> response);

  // Function to AddStorageToDataBase
  //
  // @request: Request message
  //
  // @response: Response message
  void handleAddStorageToDataBase(const std::shared_ptr<AddStorageToDataBase::Request> request,
                                  std::shared_ptr<AddStorageToDataBase::Response> response);

  // Function to AddDataToDataBase
  //
  // @request: Request message
  //
  // @response: Response message
  void handleAddDataToDataBase(const std::shared_ptr<AddDataToDataBase::Request> request,
                               std::shared_ptr<AddDataToDataBase::Response> response);

  // Function to DeleteDataFromDataBase
  //
  // @request: Request message
  //
  // @response: Response message
  void handleDeleteDataFromDataBase(const std::shared_ptr<DeleteDataFromDataBase::Request> request,
                                    std::shared_ptr<DeleteDataFromDataBase::Response> response);

  // Decode base64 string
  //
  // @encoded_string: Encoded string
  std::string base64Decode(const std::string& encoded_string);

  rclcpp::Service<GetMessages>::SharedPtr m_service;
  rclcpp::Service<CreateDataBase>::SharedPtr m_service_create_database;
  rclcpp::Service<AddStorageToDataBase>::SharedPtr m_service_add_storage_to_database;
  rclcpp::Service<AddDataToDataBase>::SharedPtr m_service_add_data_to_database;
  rclcpp::Service<DeleteDataFromDataBase>::SharedPtr m_service_delete_data_from_database;
  std::string m_zenoh_url;
  std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> m_curl;
  CURLcode m_res;
  std::string m_read_buffer;
  std::map<std::string, std::string> m_storage_url_map;
};

#endif // ZENOH_EXTRACTOR_HPP
