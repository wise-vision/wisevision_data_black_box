/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Copyright (c) 2024, WiseVision. All rights reserved.
#include <cstdlib>

#include "zenoh_extractor.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::string db_type = "influxdb";
  std::string db_address = std::getenv("DB_ADDRESS");
  std::string db_port = std::getenv("DB_PORT");
  std::string db_url = "http://" + db_address + ":" + db_port;
  auto node = std::make_shared<DataBaseHandler>(db_type, db_url);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
