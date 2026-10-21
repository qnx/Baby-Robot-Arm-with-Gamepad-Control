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

#include "qpp_reader.hpp"

#include <fcntl.h>
#include <sys/poll.h>

#include <filesystem>
#include <rclcpp/logging.hpp>

QPPReader::QPPReader(const std::string &qpp_path) : qpp_path_(qpp_path) {
  if (!std::filesystem::exists(qpp_path_) ||
      !std::filesystem::is_directory(qpp_path_)) {
    RCLCPP_ERROR_STREAM(logger_, "Couldn't open QPP path " << qpp_path_);
    exit(1);
  }

  int fd[2];
  if (pipe(fd) < 0) {
    RCLCPP_ERROR_STREAM(logger_,
                        "Failed to create control pipe" << strerror(errno));
    exit(1);
  }
  ctrl_fd_rd_ = fd[0];
  ctrl_fd_wr_ = fd[1];
}

int QPPReader::go() {
  RCLCPP_INFO(logger_, "Reader Starting");
  if (running_ || qpp_reader_thread_.joinable()) {
    RCLCPP_WARN(logger_, "Event thread already started");
    return EAGAIN;
  }

  running_ = true;
  qpp_reader_thread_ = std::thread(&QPPReader::qpp_reader_loop, this);
  return EOK;
}

int QPPReader::stop() {
  if (!running_) {
    return EAGAIN;
  }

  running_ = false;
  int exit_val = 0;
  write(ctrl_fd_wr_, &exit_val, sizeof(exit_val));
  if (qpp_reader_thread_.joinable()) {
    qpp_reader_thread_.join();
  }
  return EOK;
}

size_t QPPReader::qpp_type_size(QPPType qpp_type) const {
  switch (qpp_type) {
    case QPPType::BOOL:
      return sizeof(bool);
    case QPPType::UINT8:
    case QPPType::INT8:
      return sizeof(uint8_t);
    case QPPType::UINT16:
    case QPPType::INT16:
      return sizeof(uint16_t);
    case QPPType::UINT32:
    case QPPType::INT32:
      return sizeof(uint32_t);
    case QPPType::UINT64:
    case QPPType::INT64:
      return sizeof(uint64_t);
    case QPPType::FLOAT32:
      return sizeof(float);
    case QPPType::FLOAT64:
      return sizeof(double);
    case QPPType::BINARY:  // dynamic and unknown
    case QPPType::STRING:
    default:
      return 0;
  }
}

std::string QPPReader::qpp_type_str(QPPTypeInfo type_info) const {
  std::string type_str = "";
  switch (type_info.type) {
    case QPPType::BOOL:
      type_str = "boolean";
      break;
    case QPPType::BINARY:
      type_str = "binary";
      break;
    case QPPType::STRING:
      type_str = "string";
      break;
    case QPPType::UINT8:
      type_str = "uint8";
      break;
    case QPPType::UINT16:
      type_str = "uint16";
      break;
    case QPPType::UINT32:
      type_str = "uint32";
      break;
    case QPPType::UINT64:
      type_str = "uint64";
      break;
    case QPPType::INT8:
      type_str = "int8";
      break;
    case QPPType::INT16:
      type_str = "int16";
      break;
    case QPPType::INT32:
      type_str = "int32";
      break;
    case QPPType::INT64:
      type_str = "int64";
      break;
    case QPPType::FLOAT32:
      type_str = "float";
      break;
    case QPPType::FLOAT64:
      type_str = "double";
      break;
  }

  if (type_info.is_array) {
    type_str += "[]";
  }
  return type_str;
}

int QPPReader::qpp_path(const std::string &value_name, QPPTypeInfo type_info,
                        int access_type) const {
  std::string value_file = std::filesystem::path(qpp_path_) / value_name;

  // Uses ?datatype= to ensure the right type is used during open
  value_file += "?datatype=" + qpp_type_str(type_info);
  if (type_info.is_ctrl) {
    value_file += ",control";
  }

  int fd = open(value_file.c_str(), access_type);
  if (fd < 0) {
    int ret = errno;
    RCLCPP_WARN_STREAM(logger_, "Failed to open value "
                                    << value_name
                                    << "with open(): " << strerror(ret));
    errno = ret;
    return -1;
  }
  return fd;
}

/**
 * @brief Writes a value to QPP using templated types
 *
 * @note This isn't the most efficient solution as it requires an every copy
 *       on the stack but to simply the solution it was done.
 *
 * @tparam I input type of the vector
 * @tparam O output type of the vector
 *
 * @param fd An open FD to the signal (note type is not checked at this point
 * and should be prechecked on open)
 * @param data vector containing the data.
 *
 * @return -1 on failure with errno set, or positive number of bytes.
 */
template <typename I, typename O = I>
static int qpp_write_value_vector(int fd, const std::vector<I> &data) {
  // Copy to a new vector with our expected data type
  std::vector<O> out_data(data.size());
  std::transform(data.begin(), data.end(), out_data.begin(),
                 [](auto v) { return static_cast<O>(v); });

  return write(fd, &out_data[0], data.size() * sizeof(O));
}

template <typename I, typename O = I>
static int qpp_write_value(int fd, I data) {
  O val = static_cast<O>(data);
  return write(fd, &val, sizeof(O));
}

int QPPReader::write_value(const std::string &value_name, QPPTypeInfo type_info,
                           const QPPValue &value) {
  int fd = qpp_path(value_name, type_info, O_WRONLY);
  if (fd < 0) {
    return fd;
  }

  // TODO(lmilne) log the value that is being written as well
  RCLCPP_INFO_STREAM(logger_, "Writing value to "
                                  << value_name
                                  << ((type_info.is_ctrl) ? "?control" : ""));

  int rc = 0;
  switch (type_info.type) {
    case QPPType::BOOL:
      if (type_info.is_array) {
        // std::vector<bool> are ... weird in c++ ... you would expect it to
        // have the same size as a bool per entry but really they are bit
        // packed. this means our helper can't do a copy from the first entry so
        // instead cast it to a uint8_t which has the same size as a bool
        rc = qpp_write_value_vector<bool, uint8_t>(
            fd, std::get<QPPBoolArray>(value));
      } else {
        rc = qpp_write_value<bool>(fd, std::get<bool>(value));
      }
      break;
    case QPPType::BINARY: {  // don't include null terminator
      const auto &val = std::get<std::string>(value);
      rc = write(fd, val.c_str(), val.length());
      break;
    }
    case QPPType::STRING: {  // include null terminator
      const auto &val = std::get<std::string>(value);
      rc = write(fd, val.c_str(), val.length() + 1);
      break;
    }
    case QPPType::UINT8:
      if (type_info.is_array) {
        rc = qpp_write_value_vector<uint64_t, uint8_t>(
            fd, std::get<QPPUIntArray>(value));
      } else {
        rc = qpp_write_value<uint64_t, uint8_t>(fd, std::get<uint64_t>(value));
      }
      break;
    case QPPType::UINT16:
      if (type_info.is_array) {
        rc = qpp_write_value_vector<uint64_t, uint16_t>(
            fd, std::get<QPPUIntArray>(value));
      } else {
        rc = qpp_write_value<uint64_t, uint16_t>(fd, std::get<uint64_t>(value));
      }
      break;
    case QPPType::UINT32:
      if (type_info.is_array) {
        rc = qpp_write_value_vector<uint64_t, uint32_t>(
            fd, std::get<QPPUIntArray>(value));
      } else {
        rc = qpp_write_value<uint64_t, uint32_t>(fd, std::get<uint64_t>(value));
      }
      break;
    case QPPType::UINT64:
      if (type_info.is_array) {
        rc =
            qpp_write_value_vector<uint64_t>(fd, std::get<QPPUIntArray>(value));
      } else {
        rc = qpp_write_value<uint64_t>(fd, std::get<uint64_t>(value));
      }
      break;
    case QPPType::INT8:
      if (type_info.is_array) {
        rc = qpp_write_value_vector<int64_t, int8_t>(
            fd, std::get<QPPIntArray>(value));
      } else {
        rc = qpp_write_value<int64_t, int8_t>(fd, std::get<int64_t>(value));
      }
      break;
    case QPPType::INT16:
      if (type_info.is_array) {
        rc = qpp_write_value_vector<int64_t, int16_t>(
            fd, std::get<QPPIntArray>(value));
      } else {
        rc = qpp_write_value<int64_t, int16_t>(fd, std::get<int64_t>(value));
      }
      break;
    case QPPType::INT32:
      if (type_info.is_array) {
        rc = qpp_write_value_vector<int64_t, int32_t>(
            fd, std::get<QPPIntArray>(value));
      } else {
        rc = qpp_write_value<int64_t, int32_t>(fd, std::get<int64_t>(value));
      }
      break;
    case QPPType::INT64:
      if (type_info.is_array) {
        rc = qpp_write_value_vector<int64_t>(fd, std::get<QPPIntArray>(value));
      } else {
        rc = qpp_write_value<int64_t>(fd, std::get<int64_t>(value));
      }
      break;
    case QPPType::FLOAT32: {
      if (type_info.is_array) {
        rc = qpp_write_value_vector<double, float>(
            fd, std::get<QPPFloatArray>(value));
      } else {
        rc = qpp_write_value<double, float>(fd, std::get<double>(value));
      }
      break;
    }
    case QPPType::FLOAT64:
      if (type_info.is_array) {
        rc = qpp_write_value_vector<double>(fd, std::get<QPPFloatArray>(value));
      } else {
        rc = qpp_write_value<double>(fd, std::get<double>(value));
      }
      break;
  }
  close(fd);

  if (rc == -1) {
    RCLCPP_WARN_STREAM(logger_, "Writing value to "
                                    << value_name
                                    << ((type_info.is_ctrl) ? "?control" : "")
                                    << " Failed with: " << strerror(errno));
  }

  return rc;
}

template <typename I, typename O = I>
static int qpp_read_value(void *buf, size_t buf_size, bool is_array,
                          QPPReader::QPPValue &out_value) {
  if (is_array) {
    if ((buf_size % sizeof(I)) != 0) {
      return EMSGSIZE;
    }

    std::vector<O> out_vec(buf_size / sizeof(I));
    I *buf_vec = (I *)buf;
    for (size_t i = 0; i < out_vec.size(); i++) {
      out_vec[i] = (O)buf_vec[i];
    }

    out_value = std::move(out_vec);
  } else {
    if (buf_size != sizeof(I)) {
      return EMSGSIZE;
    }

    out_value = (O)(*(I *)buf);
  }
  return EOK;
}

void QPPReader::qpp_reader_loop() {
  RCLCPP_INFO(logger_, "Reader loop started");
  // Nothing to listen to...
  if (listener_map_.empty()) {
    RCLCPP_WARN(logger_, "Empty listener map no values to poll.");
    running_ = false;
    return;
  }

  struct QPPPollData {
    std::string name;
    ListenMapData info;
  };

  /**
   * 2 vectors for our poll loop.
   *
   * - val_pollfd is used to detect which signal has a value and holds
   *   its file descriptors where val_pollfd.back() is our control pipe
   * - val_poll_data contains info the signal
   *
   * @note The size of val_poll_data is 1 less than val_pollfd
   */
  std::vector<struct pollfd> val_pollfd;
  std::vector<QPPPollData> val_poll_data;

  uint8_t qpp_read_buffer[QPP_READ_SIZE];

  for (const auto &val : listener_map_) {
    // First try to open the signal creating the name base on the info
    struct pollfd pfd = {
        .fd = qpp_path(val.first, val.second.info, O_RDONLY),
        .events = POLLIN,
        .revents = 0,
    };

    if (pfd.fd < 0) {
      exit(1);
    }
    RCLCPP_INFO_STREAM(logger_,
                       "Adding listener for " << val.first << " fd=" << pfd.fd);
    val_pollfd.push_back(pfd);
    val_poll_data.push_back({val.first, val.second});
  }

  {
    struct pollfd pfd = {
        .fd = ctrl_fd_rd_,
        .events = POLLIN,
        .revents = 0,
    };
    val_pollfd.push_back(pfd);
  }

  while (running_) {
    if (poll(&val_pollfd[0], val_pollfd.size(), -1) < 0) {
      RCLCPP_WARN(logger_, "Poll interupted");
      continue;
    }

    // Check for shutdown
    if (val_pollfd.back().revents & POLLIN) {
      int exit_val;
      read(ctrl_fd_rd_, &exit_val, sizeof(int));
      RCLCPP_INFO(logger_, "qpp_reader_loop: Exit requested");
      return;
    }

    for (size_t i = 0; i < val_poll_data.size(); i++) {
      if ((val_pollfd[i].revents & POLLIN) == 0) {
        continue;
      }

      int len = read(val_pollfd[i].fd, qpp_read_buffer, QPP_READ_SIZE);
      if (len < 0) {
        RCLCPP_WARN_STREAM(logger_, "Read failed for" << val_poll_data[i].name
                                                      << " with "
                                                      << strerror(errno));
        // Skip past the value (ignoring the return value)
        lseek(val_pollfd[i].fd, 1, SEEK_CUR);
        continue;
      }

      QPPValue val;
      int rc = EOK;
      const auto &type_info = val_poll_data[i].info.info;
      bool is_array = type_info.is_array;
      switch (type_info.type) {
        case QPPType::BOOL:
          rc = qpp_read_value<bool>(qpp_read_buffer, len, is_array, val);
          break;
        case QPPType::BINARY:
          val = std::string((char *)qpp_read_buffer, len);
          break;
        case QPPType::STRING:
          // QPP forces the null terminator so read it normally stripping it.
          val = std::string((char *)qpp_read_buffer);
          break;
        case QPPType::UINT8:
          rc = qpp_read_value<uint8_t, uint64_t>(qpp_read_buffer, len, is_array,
                                                 val);

          break;
        case QPPType::UINT16:
          rc = qpp_read_value<uint16_t, uint64_t>(qpp_read_buffer, len,
                                                  is_array, val);
          break;
        case QPPType::UINT32:
          rc = qpp_read_value<uint32_t, uint64_t>(qpp_read_buffer, len,
                                                  is_array, val);
          break;
        case QPPType::UINT64:
          rc = qpp_read_value<uint64_t>(qpp_read_buffer, len, is_array, val);
          break;
        case QPPType::INT8:
          rc = qpp_read_value<int8_t, int64_t>(qpp_read_buffer, len, is_array,
                                               val);
          break;
        case QPPType::INT16:
          rc = qpp_read_value<int16_t, int64_t>(qpp_read_buffer, len, is_array,
                                                val);
          break;
        case QPPType::INT32:
          rc = qpp_read_value<int32_t, int64_t>(qpp_read_buffer, len, is_array,
                                                val);
          break;
        case QPPType::INT64:
          rc = qpp_read_value<int64_t>(qpp_read_buffer, len, is_array, val);
          break;
        case QPPType::FLOAT32: {
          rc = qpp_read_value<float, double>(qpp_read_buffer, len, is_array,
                                             val);
          break;
        }
        case QPPType::FLOAT64:
          rc = qpp_read_value<double>(qpp_read_buffer, len, is_array, val);
          break;
      }

      if (rc == EOK) {
        val_poll_data[i].info.cb(val_poll_data[i].name, val, type_info);
      } else {
        RCLCPP_WARN_STREAM(
            logger_, "read " << val_poll_data[i].name << " with res=" << rc);
      }
    }
  }

  // Open up all the fd's in the listener map
}

int QPPReader::add_value_listener(const std::string &value_name,
                                  QPPTypeInfo type_info, ValueCallback cb) {
  if (running_) {
    RCLCPP_WARN_STREAM(logger_, "Tried to add a value when already started");
    return EBUSY;
  }
  if (!cb) {
    RCLCPP_WARN(logger_, "Invalid callback for QPPReader");
    return EINVAL;
  }

  ListenMapData data{.info = type_info, .cb = cb};
  listener_map_.insert_or_assign(value_name, data);
  return EOK;
}

int QPPReader::remove_value_listener(const std::string &value_name) {
  if (running_) {
    RCLCPP_WARN_STREAM(logger_, "Tried to remove a value when already started");
    return EBUSY;
  }
  auto it = listener_map_.find(value_name);
  if (it == listener_map_.end()) {
    return ENOENT;
  }
  listener_map_.erase(it);
  return EOK;
}

int QPPReader::clear_value_listener() {
  if (running_) {
    RCLCPP_WARN_STREAM(logger_, "Tried to remove a value when already started");
    return EBUSY;
  }
  listener_map_.clear();
  return EOK;
}
