/**
 * Copyright (c) 2026, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <unistd.h>

#include <functional>
#include <rclcpp/logger.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <variant>
/**
 * Generic reader for queue which returns a filename and variant value.
 *
 * The caller is expected to know the datatype add add time but will
 * be verified with ?datatype=<type> qpp option
 */
class QPPReader {
 public:
  enum class QPPType {
    BOOL,    // Stored in bool
    BINARY,  // Stored in std::string
    STRING,  // NOTE string arrays are not supported.
    UINT8,   // all stored in uint64_t
    UINT16,
    UINT32,
    UINT64,
    INT8,  // all stored in int64_t
    INT16,
    INT32,
    INT64,
    FLOAT32,  // stored in double
    FLOAT64
  };

  using QPPIntArray = std::vector<int64_t>;
  using QPPUIntArray = std::vector<uint64_t>;
  using QPPBoolArray = std::vector<bool>;
  using QPPFloatArray = std::vector<double>;
  using QPPValue =
      std::variant<std::string, uint64_t, int64_t, bool, double, QPPIntArray,
                   QPPUIntArray, QPPBoolArray, QPPFloatArray>;

  struct QPPTypeInfo {
    QPPType type;
    bool is_array = false;
    bool is_ctrl = false;
  };

  using ValueCallback = std::function<void(
      const std::string &name, const QPPValue &value, const QPPTypeInfo &info)>;

 private:
  struct ListenMapData {
    QPPTypeInfo info;
    ValueCallback cb;
  };

 public:
  QPPReader(const std::string &qpp_path = "/dev/qpp");

  int go();
  int stop();

  // only can be done before starting
  int add_value_listener(const std::string &value_name, QPPTypeInfo qpp_type, ValueCallback cb);
  int remove_value_listener(const std::string &value_name);
  int clear_value_listener();

  /**
   * Writes to a value or control file
   */
  int write_value(const std::string &value_name, QPPTypeInfo qpp_type,
                  const QPPValue &value);

 private:
  void qpp_reader_loop();

  std::string qpp_type_str(QPPTypeInfo type_info) const;
  size_t qpp_type_size(QPPType qpp_type) const;
  int qpp_path(const std::string &value_name, QPPTypeInfo type_info, int access_type) const;

 private:
  std::string qpp_path_ = "/dev/qpp";

  std::thread qpp_reader_thread_;
  int ctrl_fd_wr_ = -1;
  int ctrl_fd_rd_ = -1;

  bool running_ = false;

  // how large of reads should be when getting qpp data.
  static constexpr int QPP_READ_SIZE = 4096;

  rclcpp::Logger logger_ = rclcpp::get_logger("QPPReader");

  std::unordered_map<std::string, ListenMapData> listener_map_{};
};