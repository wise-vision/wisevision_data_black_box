/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "zenoh_extractor.hpp"

class DataBaseHandlerTest : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<DataBaseHandler> data_service;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

  void SetUp() override {
    std::ofstream config_file("config.json");
    config_file << R"({
      "zenoh_url": "http://40.50.60.70:8000/"
    })";
    config_file.close();

    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("database_handler_test_node");

    data_service = std::make_shared<DataBaseHandler>("black_box", "http://example.com");
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(data_service);

    std::thread([this]() { executor->spin(); }).detach();
  }

  void TearDown() override {
    executor->cancel();
    rclcpp::shutdown();
    std::remove("config.json");
  }

  template <typename ServiceT, typename RequestT>
  typename ServiceT::Response::SharedPtr callService(const std::string& service_name,
                                                     typename RequestT::SharedPtr request) {
    auto client = node->create_client<ServiceT>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(node->get_logger(), "Service %s not available", service_name.c_str());
      return nullptr;
    }
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
      return future.get();
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to call service %s", service_name.c_str());
      return nullptr;
    }
  }
};

TEST_F(DataBaseHandlerTest, GetMessagesService_EmptyTopicName) {
  auto request = std::make_shared<lora_msgs::srv::GetMessages::Request>();
  request->topic_name = "";
  request->message_type = "any";

  auto response =
      callService<lora_msgs::srv::GetMessages, lora_msgs::srv::GetMessages::Request>("get_messages", request);

  ASSERT_NE(response, nullptr) << "Service response is null";
  EXPECT_FALSE(response->success) << "Expected success to be false due to empty topic name";
  EXPECT_EQ(response->error_message, "Topic name is empty.") << "Expected error message for empty topic name";
}

TEST_F(DataBaseHandlerTest, GetMessagesService_EmptyMessageType) {
  auto request = std::make_shared<lora_msgs::srv::GetMessages::Request>();
  request->topic_name = "test_topic";
  request->message_type = "";

  auto response =
      callService<lora_msgs::srv::GetMessages, lora_msgs::srv::GetMessages::Request>("get_messages", request);

  ASSERT_NE(response, nullptr) << "Service response is null";
  EXPECT_FALSE(response->success) << "Expected success to be false due to empty message type";
  EXPECT_EQ(response->error_message, "Message type is empty.") << "Expected error message for empty message type";
}

bool logContains(const std::string& output, const std::string& substring) {
  std::cout << output << std::endl;
  std::cout << substring << std::endl;
  return output.find(substring) != std::string::npos;
}

void createConfigFile(const std::string& url) {
  std::ofstream config_file("config.json");
  config_file << R"({
      "zenoh_url": ")"
              << url << R"("
    })";
  config_file.close();
}

TEST(DataBaseHandlerTestUrl, Constructor_ValidHttpUrl) {
  createConfigFile("http://123.123.123.123:8000/");

  rclcpp::init(0, nullptr);
  testing::internal::CaptureStderr();

  auto data_service = std::make_shared<DataBaseHandler>("black_box", "http://example.com");
  std::string output = testing::internal::GetCapturedStderr();

  EXPECT_TRUE(logContains(output, "Loaded zenoh config URL : http://123.123.123.123:8000/"));

  std::remove("config.json");
  rclcpp::shutdown();
}

TEST(DataBaseHandlerTestUrl, Constructor_MissingZenohUrl) {
  std::ofstream config_file("config.json");
  config_file << R"({})";
  config_file.close();

  rclcpp::init(0, nullptr);
  testing::internal::CaptureStderr();

  auto data_service = std::make_shared<DataBaseHandler>("black_box", "http://example.com");
  std::string output = testing::internal::GetCapturedStderr();

  EXPECT_TRUE(logContains(output, "Lack of 'zenoh_url' in config file."));

  std::remove("config.json");
  rclcpp::shutdown();
}

TEST(DataBaseHandlerTestUrl, Constructor_InvalidUrlPrefix) {
  createConfigFile("ftp://123.123.123.123:8000/");

  rclcpp::init(0, nullptr);
  testing::internal::CaptureStderr();

  auto data_service = std::make_shared<DataBaseHandler>("black_box", "http://example.com");
  std::string output = testing::internal::GetCapturedStderr();

  EXPECT_TRUE(logContains(output, "Invalid zenoh_url format in config file: ftp://123.123.123.123:8000/"));

  std::remove("config.json");
  rclcpp::shutdown();
}

TEST_F(DataBaseHandlerTest, CreateDatabase_EmptyKeyExpr) {
  auto request = std::make_shared<CreateDataBase::Request>();
  request->key_expr = "";
  request->storage_name = "test_storage";
  request->db_name = "test_db";
  request->create_db = true;

  auto response = callService<CreateDataBase, CreateDataBase::Request>("create_database", request);

  ASSERT_NE(response, nullptr) << "Response is null";
  EXPECT_FALSE(response->success) << "Expected failure response";
  EXPECT_EQ(response->message, "key_expr is empty.");
}

TEST_F(DataBaseHandlerTest, CreateDatabase_EmptyStorageName) {
  auto request = std::make_shared<CreateDataBase::Request>();
  request->key_expr = "some_key_expr";
  request->storage_name = "";
  request->db_name = "test_db";
  request->create_db = true;

  auto response = callService<CreateDataBase, CreateDataBase::Request>("create_database", request);

  ASSERT_NE(response, nullptr) << "Response is null";
  EXPECT_FALSE(response->success) << "Expected failure response";
  EXPECT_EQ(response->message, "storage_name is empty.");
}

TEST_F(DataBaseHandlerTest, CreateDatabase_EmptyDbName) {
  auto request = std::make_shared<CreateDataBase::Request>();
  request->key_expr = "some_key_expr";
  request->storage_name = "test_storage";
  request->db_name = "";
  request->create_db = true;

  auto response = callService<CreateDataBase, CreateDataBase::Request>("create_database", request);

  ASSERT_NE(response, nullptr) << "Response is null";
  EXPECT_FALSE(response->success) << "Expected failure response";
  EXPECT_EQ(response->message, "db_name is empty.");
}

TEST_F(DataBaseHandlerTest, AddStorageToDatabase_EmptyStorageName) {
  auto request = std::make_shared<AddStorageToDataBase::Request>();
  request->storage_name = "";

  auto response = callService<AddStorageToDataBase, AddStorageToDataBase::Request>("add_storage_to_database", request);

  ASSERT_NE(response, nullptr) << "Service response is null";
  EXPECT_FALSE(response->success) << "Expected failure, but received success";
  EXPECT_EQ(response->message, "storage_name is empty.") << "Unexpected error message";
}

TEST_F(DataBaseHandlerTest, AddStorageToDatabase_UnknownStorageName) {
  auto request = std::make_shared<AddStorageToDataBase::Request>();
  request->storage_name = "unknown_storage";

  auto response = callService<AddStorageToDataBase, AddStorageToDataBase::Request>("add_storage_to_database", request);

  ASSERT_NE(response, nullptr) << "Service response is null";
  EXPECT_FALSE(response->success) << "Expected failure, but received success";
  EXPECT_EQ(response->message, "Unknown storage type: unknown_storage") << "Unexpected error message";
}

TEST_F(DataBaseHandlerTest, AddDataToDataBase_EmptyDbPath) {
  auto request = std::make_shared<AddDataToDataBase::Request>();
  request->db_path = "";
  request->query = R"({"key": "value"})";

  auto response = callService<AddDataToDataBase, AddDataToDataBase::Request>("add_data_to_database", request);

  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "Database path (db_path) is empty");
}

TEST_F(DataBaseHandlerTest, EmptyQuery) {
  auto request = std::make_shared<AddDataToDataBase::Request>();
  request->db_path = "/valid_db_path";
  request->query = "";

  auto response = callService<AddDataToDataBase, AddDataToDataBase::Request>("add_data_to_database", request);

  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "Query string is empty");
}

TEST_F(DataBaseHandlerTest, AddDataToDataBase_InvalidJsonQuery) {
  auto request = std::make_shared<AddDataToDataBase::Request>();
  request->db_path = "/valid_db_path";
  request->query = "{invalid_json}";

  auto response = callService<AddDataToDataBase, AddDataToDataBase::Request>("add_data_to_database", request);

  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "Invalid JSON format in query");
}

TEST_F(DataBaseHandlerTest, DeleteDataFromDatabase_EmptyDbPath) {
  auto request = std::make_shared<DeleteDataFromDataBase::Request>();
  request->db_path = "";

  auto response =
      callService<DeleteDataFromDataBase, DeleteDataFromDataBase::Request>("delete_data_from_database", request);

  ASSERT_NE(response, nullptr) << "Response is null";
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "Database path (db_path) is empty");
}
