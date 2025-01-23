/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "zenoh_extractor.hpp"

DataBaseHandler::DataBaseHandler(const std::string& storage_type, const std::string& url)
  : curlPerform(curl_easy_perform),
    Node("message_retriever_service"),
    m_curl(curl_easy_init(), &curl_easy_cleanup) {
  m_storage_url_map = {{storage_type, url}};
  loadConfiguration();
  m_service = this->create_service<GetMessages>(
      "get_messages",
      std::bind(&DataBaseHandler::handleGetMessages, this, std::placeholders::_1, std::placeholders::_2));
  m_service_create_database = this->create_service<CreateDataBase>(
      "create_database",
      std::bind(&DataBaseHandler::handleCreateDataBase, this, std::placeholders::_1, std::placeholders::_2));
  m_service_add_storage_to_database = this->create_service<AddStorageToDataBase>(
      "add_storage_to_database",
      std::bind(&DataBaseHandler::handleAddStorageToDataBase, this, std::placeholders::_1, std::placeholders::_2));
  m_service_add_data_to_database = this->create_service<AddDataToDataBase>(
      "add_data_to_database",
      std::bind(&DataBaseHandler::handleAddDataToDataBase, this, std::placeholders::_1, std::placeholders::_2));
  m_service_delete_data_from_database = this->create_service<DeleteDataFromDataBase>(
      "delete_data_from_database",
      std::bind(&DataBaseHandler::handleDeleteDataFromDataBase, this, std::placeholders::_1, std::placeholders::_2));
}

DataBaseHandler::~DataBaseHandler() {
  curl_global_cleanup();
}

void DataBaseHandler::handleGetMessages(const std::shared_ptr<GetMessages::Request> request,
                                        std::shared_ptr<GetMessages::Response> response) {
  if (!request) {
    RCLCPP_ERROR(this->get_logger(), "Received null request in handleGetMessages");
    response->success = false;
    response->error_message = "Received null request.";
    return;
  }

  std::string topic_name = request->topic_name;
  std::string message_type = request->message_type;

  if (topic_name.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Topic name is empty in request");
    response->success = false;
    response->error_message = "Topic name is empty.";
    return;
  }
  if (message_type.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Message type is empty in request");
    response->success = false;
    response->error_message = "Message type is empty.";
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Received request for topic: %s", topic_name.c_str());
  RCLCPP_DEBUG(this->get_logger(), "Received request for message type: %s", message_type.c_str());

  std::string url;
  if (message_type == "wisevision_msgs/GpsDevicesPublisher") {
    url = m_zenoh_url + topic_name + "/**";
  } else {
    url = m_zenoh_url + topic_name;
    bool is_time_range_provided =
        !(request->time_start.year == 0 && request->time_start.month == 0 && request->time_start.day == 0 &&
          request->time_end.year == 0 && request->time_end.month == 0 && request->time_end.day == 0);

    if (is_time_range_provided) {
      url +=
          "?_time=[" + fullDatetimeToString(request->time_start) + ".." + fullDatetimeToString(request->time_end) + "]";
    } else {
      url += "?_time=[..]";
    }
  }

  if (url.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Constructed URL is empty");
    response->success = false;
    response->error_message = "Constructed URL is empty.";
    return;
  }

  curl_global_init(CURL_GLOBAL_DEFAULT);
  if (!m_curl.get()) {
    response->success = false;
    response->error_message = "Failed to initialize CURL.";
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize CURL");
    return;
  }

  m_read_buffer.clear();

  curl_easy_setopt(m_curl.get(), CURLOPT_URL, url.c_str());
  curl_easy_setopt(m_curl.get(), CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(m_curl.get(), CURLOPT_WRITEDATA, &m_read_buffer);
  m_res = curlPerform(m_curl.get());

  if (m_res != CURLE_OK) {
    RCLCPP_ERROR(this->get_logger(), "HTTP request error: %s", curl_easy_strerror(m_res));
    response->success = false;
    response->error_message = std::string("HTTP request error: ") + curl_easy_strerror(m_res);
    return;
  }

  if (m_read_buffer.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Received empty response buffer from server");
    response->success = false;
    response->error_message = "Received empty response buffer from server.";
    return;
  }

  Json::Value jsonData;
  Json::Reader jsonReader;
  if (!jsonReader.parse(m_read_buffer, jsonData)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON response");
    response->success = false;
    response->error_message = "Failed to parse JSON response.";
    return;
  }

  if (message_type == "std_msgs/Int32") {
    handleInt32Messages(jsonData, topic_name, request, response);
  } else if (message_type == "lora_msgs/MicroPublisher") {
    handleMicroPublisherMessages(jsonData, topic_name, request, response);
  } else if (message_type == "any") {
    handleMessages(jsonData, topic_name, request, response);
  } else if (message_type == "wisevision_msgs/GpsDevicesPublisher") {
    handleGpsDevicesPublisherMessages(jsonData, topic_name, request, response);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown message type: %s", message_type.c_str());
    response->success = false;
    response->error_message = "Unknown message type: " + message_type;
  }
}

void DataBaseHandler::loadConfiguration() {
  std::ifstream config_file("config.json");
  if (!config_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Can't load configuration file.");
    return;
  }

  Json::Value config;
  config_file >> config;

  if (!config["zenoh_url"].isNull()) {
    m_zenoh_url = config["zenoh_url"].asString();

    std::regex url_pattern(R"(^(http|https)://([a-zA-Z0-9.-]+)(:[0-9]+)?(/.*)?$)");

    if (std::regex_match(m_zenoh_url, url_pattern)) {
      RCLCPP_INFO(this->get_logger(), "Loaded zenoh config URL : %s", m_zenoh_url.c_str());

      if (m_zenoh_url.back() != '/') {
        m_zenoh_url += '/';
      }

    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid zenoh_url format in config file: %s", m_zenoh_url.c_str());
      m_zenoh_url.clear();
      return;
    }

  } else {
    RCLCPP_ERROR(this->get_logger(), "Lack of 'zenoh_url' in config file.");
    return;
  }
}

void DataBaseHandler::handleInt32Messages(const Json::Value& jsonData,
                                          const std::string& topic_name,
                                          const std::shared_ptr<GetMessages::Request> request,
                                          std::shared_ptr<GetMessages::Response> response) {
  std::vector<int32_t> int32_msgs;
  std::vector<lora_msgs::msg::FullDateTime> timestamps;
  rclcpp::Serialization<std_msgs::msg::Int32> serializer;

  for (const auto& entry : jsonData) {
    if (entry["key"].asString() == topic_name) {
      std::string decoded_value = base64Decode(entry["value"].asString());

      if (decoded_value.size() >= sizeof(int32_t)) {
        std_msgs::msg::Int32 message;
        rclcpp::SerializedMessage serialized_msg;
        serialized_msg.reserve(decoded_value.size());
        std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, decoded_value.data(), decoded_value.size());
        serialized_msg.get_rcl_serialized_message().buffer_length = decoded_value.size();

        serializer.deserialize_message(&serialized_msg, &message);
        int32_msgs.push_back(message.data);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid decoded message size: %zu bytes.", decoded_value.size());
        continue;
      }

      lora_msgs::msg::FullDateTime full_datetime = parseIso8601ToFullDatetime(entry["time"].asString());
      timestamps.push_back(full_datetime);
    }
  }

  prepareResponseInt32(response, int32_msgs, timestamps, request);
}

void DataBaseHandler::prepareResponseInt32(std::shared_ptr<GetMessages::Response> response,
                                           const std::vector<int32_t>& int32_msgs,
                                           const std::vector<lora_msgs::msg::FullDateTime>& timestamps,
                                           const std::shared_ptr<GetMessages::Request> request) {
  if (request->number_of_msgs == 0) {
    response->int32_msgs = int32_msgs;
    response->timestamps = timestamps;
  } else {
    size_t total_msgs = int32_msgs.size();
    size_t num_msgs_to_return = std::min(static_cast<size_t>(request->number_of_msgs), total_msgs);

    std::vector<int32_t> last_msgs(int32_msgs.end() - num_msgs_to_return, int32_msgs.end());
    std::vector<lora_msgs::msg::FullDateTime> last_timestamps(timestamps.end() - num_msgs_to_return, timestamps.end());

    response->int32_msgs = last_msgs;
    response->timestamps = last_timestamps;
  }
}

void DataBaseHandler::handleMicroPublisherMessages(const Json::Value& jsonData,
                                                   const std::string& topic_name,
                                                   const std::shared_ptr<GetMessages::Request> request,
                                                   std::shared_ptr<GetMessages::Response> response) {
  std::vector<lora_msgs::msg::MicroPublisher> micro_publisher_msgs;
  std::vector<lora_msgs::msg::FullDateTime> timestamps;
  rclcpp::Serialization<lora_msgs::msg::MicroPublisher> serializer;

  for (const auto& entry : jsonData) {
    if (entry["key"].asString() == topic_name) {
      std::string decoded_value = base64Decode(entry["value"].asString());

      if (decoded_value.size() < sizeof(lora_msgs::msg::MicroPublisher)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid decoded message size: %zu bytes.", decoded_value.size());
        continue;
      }

      lora_msgs::msg::MicroPublisher message;
      rclcpp::SerializedMessage serialized_msg;
      serialized_msg.reserve(decoded_value.size());
      std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, decoded_value.data(), decoded_value.size());
      serialized_msg.get_rcl_serialized_message().buffer_length = decoded_value.size();

      try {
        serializer.deserialize_message(&serialized_msg, &message);
        micro_publisher_msgs.push_back(message);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Deserialization error: %s", e.what());
        continue;
      }

      lora_msgs::msg::FullDateTime full_datetime = parseIso8601ToFullDatetime(entry["time"].asString());
      timestamps.push_back(full_datetime);
    }
  }

  prepareResponseMicroPublisher(response, micro_publisher_msgs, timestamps, request);
}

void DataBaseHandler::prepareResponseMicroPublisher(
    std::shared_ptr<GetMessages::Response> response,
    const std::vector<lora_msgs::msg::MicroPublisher>& micro_publisher_msgs,
    const std::vector<lora_msgs::msg::FullDateTime>& timestamps,
    const std::shared_ptr<GetMessages::Request> request) {
  if (request->number_of_msgs == 0) {
    response->micro_publisher_data = micro_publisher_msgs;
    response->timestamps = timestamps;
  } else {
    size_t total_msgs = micro_publisher_msgs.size();
    size_t num_msgs_to_return = std::min(static_cast<size_t>(request->number_of_msgs), total_msgs);

    std::vector<lora_msgs::msg::MicroPublisher> last_msgs(micro_publisher_msgs.end() - num_msgs_to_return,
                                                          micro_publisher_msgs.end());
    std::vector<lora_msgs::msg::FullDateTime> last_timestamps(timestamps.end() - num_msgs_to_return, timestamps.end());

    response->micro_publisher_data = last_msgs;
    response->timestamps = last_timestamps;
  }
}

void DataBaseHandler::handleMessages(const Json::Value& jsonData,
                                     const std::string& topic_name,
                                     const std::shared_ptr<GetMessages::Request> request,
                                     std::shared_ptr<GetMessages::Response> response) {
  std::vector<lora_msgs::msg::FullDateTime> timestamps;
  std::vector<uint8_t> serialized_data; // Vector for serialized messages in binary form

  uint32_t messages_processed = 0;

  for (const auto& entry : jsonData) {
    if (entry["key"].asString() == topic_name) {
      if (request->number_of_msgs > 0 && messages_processed >= request->number_of_msgs) {
        break;
      }

      std::string decoded_value = base64Decode(entry["value"].asString());

      uint32_t message_length = static_cast<uint32_t>(decoded_value.size());
      uint8_t length_prefix[4];
      length_prefix[0] = (message_length >> 24) & 0xFF;
      length_prefix[1] = (message_length >> 16) & 0xFF;
      length_prefix[2] = (message_length >> 8) & 0xFF;
      length_prefix[3] = message_length & 0xFF;

      serialized_data.insert(serialized_data.end(), length_prefix, length_prefix + 4);

      serialized_data.insert(serialized_data.end(), decoded_value.begin(), decoded_value.end());

      try {
        lora_msgs::msg::FullDateTime full_datetime = parseIso8601ToFullDatetime(entry["time"].asString());
        timestamps.push_back(full_datetime);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error parsing timestamp: %s", e.what());
        continue;
      }

      messages_processed++;
    }
  }

  prepareResponse(response, serialized_data, timestamps);
}

void DataBaseHandler::prepareResponse(std::shared_ptr<GetMessages::Response> response,
                                      const std::vector<uint8_t>& serialized_data,
                                      const std::vector<lora_msgs::msg::FullDateTime>& timestamps) {
  response->data = serialized_data; // Assign serialized binary data
  response->timestamps = timestamps;
}

void DataBaseHandler::handleGpsDevicesPublisherMessages(const Json::Value& jsonData,
                                                        const std::string& topic_name,
                                                        const std::shared_ptr<GetMessages::Request> request,
                                                        std::shared_ptr<GetMessages::Response> response) {

  GpsDevicesPublisher gps_devices_msg;

  for (const auto& device : jsonData) {
    GpsDeviceData gps_device;

    std::string key = device["key"].asString();
    std::string eui_string = key.substr(key.find_last_of('/') + 1);

    lora_msgs::msg::EUI64 eui64;
    std::stringstream ss(eui_string);
    std::string byte;
    int i = 0;
    while (std::getline(ss, byte, ':') && i < 8) {
      eui64.data[i++] = static_cast<uint8_t>(std::stoi(byte, nullptr, 16));
    }
    gps_device.device_eui = eui64;

    gps_device.device_name = device["value"]["device_name"].asString();

    const Json::Value& location = device["value"]["location"];
    sensor_msgs::msg::NavSatFix nav_data;

    if (location.isMember("latitude") && location["latitude"].isNumeric()) {
      nav_data.latitude = location["latitude"].asDouble();
    } else {
      nav_data.latitude = 0.0;
    }

    if (location.isMember("longitude") && location["longitude"].isNumeric()) {
      nav_data.longitude = location["longitude"].asDouble();
    } else {
      nav_data.longitude = 0.0;
    }

    if (location.isMember("altitude") && location["altitude"].isNumeric()) {
      nav_data.altitude = location["altitude"].asDouble();
    } else {
      nav_data.altitude = 0.0;
    }

    gps_device.nav_value = nav_data;

    gps_devices_msg.devices_data.push_back(gps_device);

    lora_msgs::msg::FullDateTime full_datetime = parseIso8601ToFullDatetime(device["time"].asString());
    response->timestamps.push_back(full_datetime);
  }

  response->gps_devices_data = gps_devices_msg.devices_data;
}

lora_msgs::msg::FullDateTime DataBaseHandler::parseIso8601ToFullDatetime(const std::string& time_str) {
  lora_msgs::msg::FullDateTime full_datetime;
  std::sscanf(time_str.c_str(),
              "%d-%d-%dT%d:%d:%d.%uZ",
              &full_datetime.year,
              &full_datetime.month,
              &full_datetime.day,
              &full_datetime.hour,
              &full_datetime.minute,
              &full_datetime.second,
              &full_datetime.nanosecond);
  return full_datetime;
}

std::string DataBaseHandler::fullDatetimeToString(const lora_msgs::msg::FullDateTime& datetime) {
  std::ostringstream oss;
  oss << datetime.year << "-" << std::setw(2) << std::setfill('0') << datetime.month << "-" << std::setw(2)
      << std::setfill('0') << datetime.day << "T" << std::setw(2) << std::setfill('0') << datetime.hour << ":"
      << std::setw(2) << std::setfill('0') << datetime.minute << ":" << std::setw(2) << std::setfill('0')
      << datetime.second << "." << std::setw(9) << std::setfill('0') << datetime.nanosecond << "Z";
  return oss.str();
}

size_t DataBaseHandler::WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
  auto* buffer = static_cast<std::string*>(userp);
  buffer->append(static_cast<char*>(contents), size * nmemb);
  return size * nmemb;
}

std::string DataBaseHandler::base64Decode(const std::string& encoded_string) {
  std::unique_ptr<BIO, decltype(&BIO_free_all)> bio(BIO_new_mem_buf(encoded_string.c_str(), -1), &BIO_free_all);

  std::unique_ptr<BIO, decltype(&BIO_free_all)> b64(BIO_new(BIO_f_base64()), &BIO_free_all);

  bio.reset(BIO_push(b64.release(), bio.release()));
  BIO_set_flags(bio.get(), BIO_FLAGS_BASE64_NO_NL);
  std::vector<char> buffer(encoded_string.size());
  int decoded_size = BIO_read(bio.get(), buffer.data(), buffer.size());

  if (decoded_size < 0) {
    throw std::runtime_error("Error during base64 decoding");
  }

  return std::string(buffer.begin(), buffer.begin() + decoded_size);
}

std::optional<Json::Value> DataBaseHandler::parseToJson(const std::string& input) {
  Json::CharReaderBuilder builder;
  Json::Value result;
  std::string errs;

  std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  if (reader->parse(input.c_str(), input.c_str() + input.length(), &result, &errs)) {
    return result;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", errs.c_str());
    return std::nullopt;
  }
}

void DataBaseHandler::handleCreateDataBase(const std::shared_ptr<CreateDataBase::Request> request,
                                           std::shared_ptr<CreateDataBase::Response> response) {
  if (!request || !response) {
    RCLCPP_ERROR(this->get_logger(), "Received null request or response in handleCreateDataBase");
    if (response) {
      response->success = false;
      response->message = "Request or response is null.";
    }
    return;
  }

  if (request->key_expr.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Received empty key_expr in create database request.");
    response->success = false;
    response->message = "key_expr is empty.";
    return;
  }
  if (request->storage_name.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Received empty storage_name in create database request.");
    response->success = false;
    response->message = "storage_name is empty.";
    return;
  }
  if (request->db_name.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Received empty db_name in create database request.");
    response->success = false;
    response->message = "db_name is empty.";
    return;
  }

  RCLCPP_DEBUG(this->get_logger(),
              "Received create database request: key_expr=%s, storage_name=%s, db_name=%s, create_db=%d",
              request->key_expr.c_str(),
              request->storage_name.c_str(),
              request->db_name.c_str(),
              request->create_db);

  if (m_zenoh_url.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Zenoh URL is not initialized.");
    response->success = false;
    response->message = "Zenoh URL is not initialized.";
    return;
  }

  Json::Value root;
  root["key_expr"] = request->key_expr;
  Json::Value volume;
  volume["id"] = request->storage_name;
  volume["db"] = request->db_name;
  volume["create_db"] = request->create_db;
  root["volume"] = volume;

  Json::StreamWriterBuilder writer;
  std::string json_data = Json::writeString(writer, root);

  if (!m_curl) {
    m_curl.reset(curl_easy_init());
  }

  if (m_curl) {
    std::string url = m_zenoh_url + "@/router/local/config/plugins/storage_manager/storages/" + request->storage_name;

    curl_easy_setopt(m_curl.get(), CURLOPT_URL, url.c_str());
    curl_easy_setopt(m_curl.get(), CURLOPT_CUSTOMREQUEST, "PUT");
    curl_easy_setopt(m_curl.get(), CURLOPT_POSTFIELDS, json_data.c_str());

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(m_curl.get(), CURLOPT_HTTPHEADER, headers);

    CURLcode res = curlPerform(m_curl.get());
    if (res != CURLE_OK) {
      response->success = false;
      response->message = curl_easy_strerror(res);
      RCLCPP_ERROR(this->get_logger(), "Request failed: %s", response->message.c_str());
    } else {
      response->success = true;
      response->message = "Database created successfully!";
      RCLCPP_DEBUG(this->get_logger(), "Database creation succeeded.");
    }

    curl_slist_free_all(headers);
  } else {
    response->success = false;
    response->message = "Failed to initialize cURL.";
  }
}

void DataBaseHandler::handleAddStorageToDataBase(const std::shared_ptr<AddStorageToDataBase::Request> request,
                                                 std::shared_ptr<AddStorageToDataBase::Response> response) {
  if (!request) {
    response->success = false;
    response->message = "Request is null.";
    RCLCPP_ERROR(this->get_logger(), "Received null request in handleAddStorageToDataBase");
    return;
  }

  if (request->storage_name.empty()) {
    response->success = false;
    response->message = "storage_name is empty.";
    RCLCPP_ERROR(this->get_logger(), "Storage name is empty in request");
    return;
  }

  RCLCPP_DEBUG(this->get_logger(),
              "Received add storage to database request: storage_name=%s",
              request->storage_name.c_str());

  auto it = m_storage_url_map.find(request->storage_name);
  if (it == m_storage_url_map.end()) {
    response->success = false;
    response->message = "Unknown storage type: " + request->storage_name;
    RCLCPP_ERROR(this->get_logger(), "Unknown storage type: %s", request->storage_name.c_str());
    return;
  }

  std::string storage_url = it->second;
  if (storage_url.empty()) {
    response->success = false;
    response->message = "URL for storage_name is empty.";
    RCLCPP_ERROR(this->get_logger(), "URL for storage_name %s is empty.", request->storage_name.c_str());
    return;
  }

  Json::Value root;
  root["url"] = storage_url;

  Json::StreamWriterBuilder writer;
  std::string json_data = Json::writeString(writer, root);

  if (!m_curl) {
    m_curl.reset(curl_easy_init());
  }

  if (!m_curl) {
    response->success = false;
    response->message = "Failed to initialize cURL.";
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize CURL");
    return;
  }

  std::string url = m_zenoh_url + "@/router/local/config/plugins/storage_manager/volumes/" + request->storage_name;
  if (m_zenoh_url.empty()) {
    response->success = false;
    response->message = "Zenoh URL is not initialized.";
    RCLCPP_ERROR(this->get_logger(), "Zenoh URL is not initialized.");
    return;
  }

  curl_easy_setopt(m_curl.get(), CURLOPT_URL, url.c_str());
  curl_easy_setopt(m_curl.get(), CURLOPT_CUSTOMREQUEST, "PUT");
  curl_easy_setopt(m_curl.get(), CURLOPT_POSTFIELDS, json_data.c_str());

  struct curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  curl_easy_setopt(m_curl.get(), CURLOPT_HTTPHEADER, headers);

  CURLcode res = curlPerform(m_curl.get());
  if (res != CURLE_OK) {
    response->success = false;
    response->message = curl_easy_strerror(res);
    RCLCPP_ERROR(this->get_logger(), "Request failed: %s", response->message.c_str());
  } else {
    response->success = true;
    response->message = "Storage added successfully to database!";
    RCLCPP_DEBUG(this->get_logger(), "Storage addition succeeded.");
  }

  curl_slist_free_all(headers);
}

void DataBaseHandler::handleAddDataToDataBase(const std::shared_ptr<AddDataToDataBase::Request> request,
                                              std::shared_ptr<AddDataToDataBase::Response> response) {
  if (!request) {
    RCLCPP_ERROR(this->get_logger(), "Received null request in handleAddDataToDataBase");
    response->success = false;
    response->message = "Request is null";
    return;
  }

  if (request->db_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Database path (db_path) is empty in request");
    response->success = false;
    response->message = "Database path (db_path) is empty";
    return;
  }

  if (request->query.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Query string is empty in request");
    response->success = false;
    response->message = "Query string is empty";
    return;
  }

  RCLCPP_DEBUG(this->get_logger(),
              "Received add data to database request: db_path=%s, query=%s",
              request->db_path.c_str(),
              request->query.c_str());

  auto parsed_data = parseToJson(request->query);

  if (!parsed_data) {
    response->success = false;
    response->message = "Invalid JSON format in query";
    return;
  }

  Json::StreamWriterBuilder writer;
  std::string json_data = Json::writeString(writer, *parsed_data);

  if (!m_curl) {
    m_curl.reset(curl_easy_init());
  }

  if (m_curl) {
    std::string url = m_zenoh_url + request->db_path;

    curl_easy_setopt(m_curl.get(), CURLOPT_URL, url.c_str());
    curl_easy_setopt(m_curl.get(), CURLOPT_CUSTOMREQUEST, "PUT");
    curl_easy_setopt(m_curl.get(), CURLOPT_POSTFIELDS, json_data.c_str());

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(m_curl.get(), CURLOPT_HTTPHEADER, headers);

    CURLcode res = curlPerform(m_curl.get());

    if (res != CURLE_OK) {
      response->success = false;
      response->message = curl_easy_strerror(res);
      RCLCPP_ERROR(this->get_logger(), "Request failed: %s", response->message.c_str());
    } else {
      response->success = true;
      response->message = "Data added successfully to the database!";
      RCLCPP_DEBUG(this->get_logger(), "Data addition succeeded.");
    }

    curl_slist_free_all(headers);
    curl_easy_reset(m_curl.get());
  } else {
    response->success = false;
    response->message = "Failed to initialize cURL.";
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize cURL.");
  }
}

void DataBaseHandler::handleDeleteDataFromDataBase(const std::shared_ptr<DeleteDataFromDataBase::Request> request,
                                                   std::shared_ptr<DeleteDataFromDataBase::Response> response) {

  if (!request) {
    response->success = false;
    response->message = "Request is null";
    RCLCPP_ERROR(this->get_logger(), "Received null request for delete data operation.");
    return;
  }

  if (request->db_path.empty()) {
    response->success = false;
    response->message = "Database path (db_path) is empty";
    RCLCPP_ERROR(this->get_logger(), "Database path is empty in delete data request.");
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Received delete data from database request: db_path=%s", request->db_path.c_str());

  if (!m_curl) {
    m_curl.reset(curl_easy_init());
  }

  if (m_curl) {
    std::string url = m_zenoh_url + request->db_path;

    curl_easy_setopt(m_curl.get(), CURLOPT_URL, url.c_str());
    curl_easy_setopt(m_curl.get(), CURLOPT_CUSTOMREQUEST, "DELETE");

    CURLcode res = curlPerform(m_curl.get());
    if (res != CURLE_OK) {
      response->success = false;
      response->message = curl_easy_strerror(res);
      RCLCPP_ERROR(this->get_logger(), "Request failed: %s", response->message.c_str());
    } else {
      response->success = true;
      response->message = "Data deleted successfully from the database!";
      RCLCPP_DEBUG(this->get_logger(), "Data deletion succeeded.");
    }
  } else {
    response->success = false;
    response->message = "Failed to initialize cURL.";
    RCLCPP_ERROR(this->get_logger(), "cURL initialization failed.");
  }
}
