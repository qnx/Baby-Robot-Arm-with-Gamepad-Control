/**
 * Copyright (c) 2025, BlackBerry Limited. All rights reserved.
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
 *
 * @file joy_teleop_node.cpp
 * @brief Safely translates supported QNX HIDDI gamepad reports to ROS Joy.
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <sys/hiddi.h>
#include <sys/hidut.h>

#include "joy_teleop_contract.h"
#include "parser.h"
#include "publication_gate.hpp"

using namespace std::chrono_literals;

// Retained for parser ABI compatibility and optional diagnostic use.
int verbose = 0;

class JoyTeleopNode : public rclcpp::Node {
public:
    JoyTeleopNode() : Node("joy_teleop_node") {
        // Depth one bounds command backlog while retaining reliable QoS
        // compatibility with existing default ROS Joy subscribers.
        auto joy_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        joy_qos.reliable().durability_volatile();
        publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", joy_qos);
        timer_ = this->create_wall_timer(20ms, std::bind(&JoyTeleopNode::publish_joy_message, this));

        std::lock_guard<std::mutex> callback_guard(instance_lifetime_lock_);
        if (instance_ != nullptr) {
            throw std::runtime_error("Only one JoyTeleopNode instance is supported");
        }
        instance_ = this;
    }

    ~JoyTeleopNode() override {
        stopping_.store(true, std::memory_order_release);
        if (timer_) {
            timer_->cancel();
        }

        // All HID callbacks hold this lock for their full lifetime. Clearing
        // the singleton under it prevents callbacks from racing destruction.
        {
            std::lock_guard<std::mutex> callback_guard(instance_lifetime_lock_);
            if (instance_ == this) {
                instance_ = nullptr;
            }
        }

        // A graceful stop should not leave the last nonzero sample behind.
        // Crashes still require the downstream command-age watchdog.
        publish_neutral_best_effort();

        if (hid_conn_ != nullptr) {
            const int status = hidd_disconnect(hid_conn_);
            if (status != EOK) {
                std::fprintf(stderr, "hidd_disconnect failed: %s (%d)\n", std::strerror(status), status);
            }
            hid_conn_ = nullptr;
        }

        std::lock_guard<std::mutex> state_guard(joy_state_lock_);
        clear_active_controller_locked(false);
    }

    /** Connect to HIDDI. A failed connection leaves the command source inert. */
    bool initialize_hid_connection() {
        if (!init_hidd()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize HIDDI connection; joystick input is disabled.");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "HIDDI Joy Teleop node started successfully.");
        return true;
    }

    bool terminal_input_failure() const noexcept {
        return terminal_input_failure_.load(std::memory_order_acquire);
    }

private:
    static constexpr std::size_t kAxisCount = JOY_TELEOP_AXIS_COUNT;
    static constexpr std::size_t kButtonCount = JOY_TELEOP_BUTTON_COUNT;
    static constexpr std::size_t kMappedButtonCount = 12;
    static constexpr std::array<int, kMappedButtonCount> kButtonMasks = {
        SCREEN_A_GAME_BUTTON,
        SCREEN_B_GAME_BUTTON,
        SCREEN_X_GAME_BUTTON,
        SCREEN_Y_GAME_BUTTON,
        SCREEN_L1_GAME_BUTTON,
        SCREEN_R1_GAME_BUTTON,
        SCREEN_L2_GAME_BUTTON,
        SCREEN_R2_GAME_BUTTON,
        SCREEN_MENU1_GAME_BUTTON,
        SCREEN_MENU2_GAME_BUTTON,
        SCREEN_L3_GAME_BUTTON,
        SCREEN_R3_GAME_BUTTON
    };
    static_assert(kAxisCount == 6 && kButtonCount == kMappedButtonCount,
        "Joy shape must match the actuator-side safety contract");
    static_assert(JOY_TELEOP_DEADMAN_BUTTON_INDEX < kButtonMasks.size() &&
        kButtonMasks[JOY_TELEOP_DEADMAN_BUTTON_INDEX] == SCREEN_L3_GAME_BUTTON,
        "Joy button 10 must remain the L3 operator dead-man");
    static constexpr std::uint32_t kReportContextMagic = 0x4a4f5931u; // "JOY1"
    static constexpr std::chrono::milliseconds kReportTimeout{250};
    static constexpr float kNeutralAxisThreshold = 0.10f;

    struct ReportContext {
        std::uint32_t magic;
        JoyTeleopNode *owner;
        hidd_device_instance_t *device;
        struct hidd_report *report_handle;
        parser_func_t parser;
        std::uint32_t devno;
        std::uint32_t vendor_id;
        std::uint32_t product_id;
        std::uint64_t generation;
        _Uint16t expected_report_length;
    };

    enum class AttachResult {
        ATTACHED,
        NOT_SUITABLE,
        BUSY
    };

    class ReportAttachmentGuard {
    public:
        explicit ReportAttachmentGuard(struct hidd_report *report) : report_(report) {}
        ~ReportAttachmentGuard() {
            if (report_ != nullptr) {
                hidd_report_detach(report_);
            }
        }
        void release() noexcept { report_ = nullptr; }

        ReportAttachmentGuard(const ReportAttachmentGuard &) = delete;
        ReportAttachmentGuard &operator=(const ReportAttachmentGuard &) = delete;

    private:
        struct hidd_report *report_;
    };

    bool init_hidd() {
        static hidd_funcs_t hid_funcs = {
            _HIDDI_NFUNCS,
            &JoyTeleopNode::on_hidd_insert,
            &JoyTeleopNode::on_hidd_remove,
            &JoyTeleopNode::on_hidd_report,
            nullptr
        };

        // This reduces unrelated callbacks; VID/PID remains parser selection,
        // not an authentication boundary.
        device_filter_ = {};
        device_filter_.vendor_id = 0x046d;
        device_filter_.product_id = HIDD_CONNECT_WILDCARD;
        device_filter_.version = HIDD_CONNECT_WILDCARD;

        hid_parms_ = {};
        hid_parms_.path = nullptr;
        hid_parms_.vhid = HID_VERSION;
        hid_parms_.vhidd = HIDD_VERSION;
        hid_parms_.device_ident = &device_filter_;
        hid_parms_.funcs = &hid_funcs;
        hid_parms_.connect_wait = HIDD_CONNECT_WAIT;

        const int status = hidd_connect(&hid_parms_, &hid_conn_);
        if (status != EOK) {
            RCLCPP_ERROR(this->get_logger(), "hidd_connect failed: %s (%d)", std::strerror(status), status);
            hid_conn_ = nullptr;
            return false;
        }
        return true;
    }

    static float normalize_axis(int value, float divisor) {
        return std::clamp(static_cast<float>(value) / divisor, -1.0f, 1.0f);
    }

    static bool is_neutral(
        const std::array<float, kAxisCount> &axes,
        const std::array<std::int32_t, kButtonCount> &buttons) {
        const bool axes_neutral = std::all_of(
            axes.begin(), axes.end(),
            [](float value) { return std::fabs(value) <= kNeutralAxisThreshold; });
        const bool buttons_neutral = std::all_of(
            buttons.begin(), buttons.end(),
            [](std::int32_t value) { return value == 0; });
        return axes_neutral && buttons_neutral;
    }

    static void parse_report(
        const ReportContext &context,
        const std::uint8_t *data,
        int data_len,
        std::array<float, kAxisCount> &axes,
        std::array<std::int32_t, kButtonCount> &buttons) {
        axes.fill(0.0f);
        buttons.fill(0);

        const int raw_buttons = context.parser(PARSER_MODE_BUTTON, data_len, data);
        const bool is_xinput = context.product_id == 0xc21d || context.product_id == 0xc21f;
        const float divisor = is_xinput ? 32768.0f : 128.0f;

        // Parsers first normalize physical direction across profiles; this
        // layer maps those normalized values to the ROS actuator convention.
        axes[0] = -normalize_axis(context.parser(PARSER_MODE_ANALOG1x, data_len, data), divisor);
        axes[1] = -normalize_axis(context.parser(PARSER_MODE_ANALOG1y, data_len, data), divisor);
        axes[2] = normalize_axis(context.parser(PARSER_MODE_ANALOG2x, data_len, data), divisor);
        axes[3] = normalize_axis(context.parser(PARSER_MODE_ANALOG2y, data_len, data), divisor);
        axes[4] = raw_buttons & SCREEN_DPAD_RIGHT_GAME_BUTTON ? 1.0f
            : (raw_buttons & SCREEN_DPAD_LEFT_GAME_BUTTON ? -1.0f : 0.0f);
        axes[5] = raw_buttons & SCREEN_DPAD_UP_GAME_BUTTON ? 1.0f
            : (raw_buttons & SCREEN_DPAD_DOWN_GAME_BUTTON ? -1.0f : 0.0f);

        for (std::size_t i = 0; i < kMappedButtonCount; ++i) {
            buttons[i] = raw_buttons & kButtonMasks[i] ? 1 : 0;
        }
    }

    void set_neutral_locked(bool require_neutral_before_resume) {
        latest_axes_.fill(0.0f);
        latest_buttons_.fill(0);
        have_fresh_report_ = false;
        publication_gate_.require_neutral();
        awaiting_neutral_ = require_neutral_before_resume;
    }

    void clear_active_controller_locked(bool publish_neutral) {
        latest_axes_.fill(0.0f);
        latest_buttons_.fill(0);
        have_fresh_report_ = false;
        publication_gate_.reset(publish_neutral);
        awaiting_neutral_ = false;
        neutral_wait_warning_emitted_ = false;
        active_device_ = nullptr;
        active_report_handle_ = nullptr;
        active_devno_ = 0;
        ++controller_generation_;
    }

    bool has_active_controller() const {
        std::lock_guard<std::mutex> state_guard(joy_state_lock_);
        return active_device_ != nullptr;
    }

    bool is_active_report(struct hidd_report *report_handle) const {
        std::lock_guard<std::mutex> state_guard(joy_state_lock_);
        return active_device_ != nullptr && active_report_handle_ == report_handle;
    }

    bool activate_controller(
        hidd_device_instance_t *device,
        struct hidd_report *report_handle,
        std::uint64_t generation) {
        std::lock_guard<std::mutex> publish_guard(publish_order_lock_);
        std::lock_guard<std::mutex> state_guard(joy_state_lock_);
        if (stopping_.load(std::memory_order_acquire) || active_device_ != nullptr) {
            return false;
        }
        active_device_ = device;
        active_report_handle_ = report_handle;
        active_devno_ = device->devno;
        controller_generation_ = generation;
        set_neutral_locked(true);
        neutral_wait_warning_emitted_ = false;
        return true;
    }

    std::uint64_t next_controller_generation() {
        std::lock_guard<std::mutex> state_guard(joy_state_lock_);
        return ++controller_generation_;
    }

    void fault_active_input(const char *reason) {
        {
            // The fault transition is the publication linearization point. Once
            // stopping_ is visible, the timer cannot send another fresh sample;
            // the terminal gate also rejects a report callback already in flight.
            std::lock_guard<std::mutex> publish_guard(publish_order_lock_);
            if (terminal_input_failure_.exchange(true, std::memory_order_acq_rel)) {
                return;
            }
            stopping_.store(true, std::memory_order_release);
            std::lock_guard<std::mutex> state_guard(joy_state_lock_);
            latest_axes_.fill(0.0f);
            latest_buttons_.fill(0);
            have_fresh_report_ = false;
            awaiting_neutral_ = false;
            publication_gate_.latch_terminal_neutral();
        }

        RCLCPP_ERROR(this->get_logger(),
            "Terminal HID input failure; making neutral the final command state and stopping: %s", reason);
        publish_neutral_best_effort();

        // Leave the timer alive to shut the ROS context down from an executor
        // callback after it has acquired the HID callback-lifetime lock.
    }

    void accept_report(
        const ReportContext &context,
        const std::uint8_t *data,
        std::uint32_t len) {
        {
            std::lock_guard<std::mutex> state_guard(joy_state_lock_);
            if (stopping_.load(std::memory_order_acquire) ||
                active_device_ != context.device || active_devno_ != context.devno ||
                controller_generation_ != context.generation || active_report_handle_ != context.report_handle) {
                return;
            }
        }

        std::array<float, kAxisCount> parsed_axes{};
        std::array<std::int32_t, kButtonCount> parsed_buttons{};
        parse_report(context, data, static_cast<int>(len), parsed_axes, parsed_buttons);
        const bool report_is_neutral = is_neutral(parsed_axes, parsed_buttons);
        const auto steady_stamp = std::chrono::steady_clock::now();
        const auto ros_stamp = this->get_clock()->now().to_msg();

        bool warn_waiting_for_neutral = false;
        bool report_gap_timed_out = false;
        {
            std::lock_guard<std::mutex> publish_guard(publish_order_lock_);
            std::lock_guard<std::mutex> state_guard(joy_state_lock_);
            if (stopping_.load(std::memory_order_acquire) ||
                active_device_ != context.device || active_devno_ != context.devno ||
                controller_generation_ != context.generation || active_report_handle_ != context.report_handle) {
                return;
            }

            // Enforce freshness at report receipt too, so a late nonneutral
            // sample cannot arrive between watchdog timer callbacks.
            if (have_fresh_report_ && steady_stamp - latest_report_steady_time_ > kReportTimeout) {
                report_gap_timed_out = true;
            } else if (awaiting_neutral_ && !report_is_neutral) {
                if (!neutral_wait_warning_emitted_) {
                    neutral_wait_warning_emitted_ = true;
                    warn_waiting_for_neutral = true;
                }
            } else {
                awaiting_neutral_ = false;
                neutral_wait_warning_emitted_ = false;
                latest_axes_ = parsed_axes;
                latest_buttons_ = parsed_buttons;
                latest_report_steady_time_ = steady_stamp;
                latest_report_ros_stamp_ = ros_stamp;
                have_fresh_report_ = true;
                // Coalesce ordinary reports, but retain the activation neutral
                // as an ordered publication barrier. Otherwise that neutral and
                // a later held-dead-man frame within one timer period could erase
                // actuator-side authority revocation.
                publication_gate_.queue_fresh();
            }
        }

        if (report_gap_timed_out) {
            fault_active_input("the HID report gap exceeded the allowed timeout");
            return;
        }
        if (warn_waiting_for_neutral) {
            RCLCPP_WARN(this->get_logger(),
                "Controller input is not neutral; commands remain inhibited until sticks and buttons are released.");
        }
    }

    void publish_joy_message() {
        std::array<float, kAxisCount> axes{};
        std::array<std::int32_t, kButtonCount> buttons{};
        builtin_interfaces::msg::Time report_stamp{};
        bool should_publish = false;
        bool synthetic_neutral = false;
        bool timed_out = false;
        bool request_terminal_shutdown = false;

        {
            // This lock is the publication linearization point. Fault/removal
            // either precedes sample selection or follows the completed publish.
            std::lock_guard<std::mutex> publish_guard(publish_order_lock_);
            if (stopping_.load(std::memory_order_acquire)) {
                request_terminal_shutdown =
                    terminal_input_failure_.load(std::memory_order_acquire);
            } else {
                std::lock_guard<std::mutex> state_guard(joy_state_lock_);
                if (active_device_ != nullptr && have_fresh_report_ &&
                    std::chrono::steady_clock::now() - latest_report_steady_time_ > kReportTimeout) {
                    timed_out = true;
                }

                if (!timed_out) {
                    const auto pending_publication = publication_gate_.take_next();
                    if (pending_publication == joy_teleop::PendingPublication::neutral) {
                        should_publish = true;
                        synthetic_neutral = true;
                    } else if (pending_publication == joy_teleop::PendingPublication::fresh &&
                        have_fresh_report_) {
                        axes = latest_axes_;
                        buttons = latest_buttons_;
                        report_stamp = latest_report_ros_stamp_;
                        should_publish = true;
                    }
                }

                if (should_publish) {
                    sensor_msgs::msg::Joy joy_msg;
                    joy_msg.header.stamp = synthetic_neutral ? this->get_clock()->now().to_msg() : report_stamp;
                    joy_msg.axes.assign(axes.begin(), axes.end());
                    joy_msg.buttons.assign(buttons.begin(), buttons.end());
                    publisher_->publish(joy_msg);
                }
            }
        }

        if (request_terminal_shutdown) {
            // A terminal source fault must fail the supervised unit, not
            // recover in-process and reacquire actuator authority. Waiting for
            // this lock also guarantees no raw-pointer HID callback is still
            // using the node when shutdown lets spin release shared ownership.
            std::lock_guard<std::mutex> callback_guard(instance_lifetime_lock_);
            if (rclcpp::ok()) {
                rclcpp::shutdown();
            }
            return;
        }
        if (timed_out) {
            fault_active_input("no valid HID report arrived before the watchdog timeout");
        }
    }

    void publish_neutral_best_effort() noexcept {
        try {
            // A timer that passed its first stop check must not publish after
            // this shutdown neutral; it rechecks stopping_ under this lock.
            std::lock_guard<std::mutex> publish_guard(publish_order_lock_);
            sensor_msgs::msg::Joy joy_msg;
            joy_msg.header.stamp = this->get_clock()->now().to_msg();
            joy_msg.axes.assign(kAxisCount, 0.0f);
            joy_msg.buttons.assign(kButtonCount, 0);
            publisher_->publish(joy_msg);
        } catch (const std::exception &error) {
            std::fprintf(stderr, "Could not publish shutdown neutral state: %s\n", error.what());
        } catch (...) {
            std::fprintf(stderr, "Could not publish shutdown neutral state: unknown error\n");
        }
    }

    static bool collection_is_game_controller(struct hidd_collection *collection) {
        if (collection == nullptr) {
            return false;
        }
        _Uint16t usage_page = 0;
        _Uint16t usage = 0;
        if (hidd_collection_usage(collection, &usage_page, &usage) != EOK) {
            return false;
        }
        return usage_page == HIDD_PAGE_DESKTOP &&
            (usage == HIDD_USAGE_GAMEPAD || usage == HIDD_USAGE_JOYSTICK);
    }

    static AttachResult try_attach_to_report(
        JoyTeleopNode *node,
        struct hidd_connection *conn,
        hidd_device_instance_t *inst,
        struct hidd_collection *collection) {
        if (node == nullptr || conn == nullptr || inst == nullptr || collection == nullptr) {
            return AttachResult::NOT_SUITABLE;
        }

        parser_func_t parser = get_parser(inst->device_ident.vendor_id, inst->device_ident.product_id);
        const int expected_length = get_parser_report_length(
            inst->device_ident.vendor_id, inst->device_ident.product_id);
        if (parser == nullptr || parser == prs_generic || expected_length < 0) {
            return AttachResult::NOT_SUITABLE;
        }

        struct hidd_report_instance *report_inst = nullptr;
        if (hidd_get_report_instance(collection, 0, HID_INPUT_REPORT, &report_inst) != EOK ||
            report_inst == nullptr) {
            return AttachResult::NOT_SUITABLE;
        }

        _Uint16t report_length = 0;
        if (hidd_report_len(report_inst, &report_length) != EOK ||
            report_length != static_cast<_Uint16t>(expected_length)) {
            return AttachResult::NOT_SUITABLE;
        }

        struct hidd_report *report_handle = nullptr;
        const int attach_status = hidd_report_attach(
            conn,
            inst,
            report_inst,
            HIDD_REPORT_EXCLUSIVE,
            sizeof(ReportContext),
            &report_handle);
        if (attach_status == EBUSY) {
            return AttachResult::BUSY;
        }
        if (attach_status != EOK || report_handle == nullptr) {
            return AttachResult::NOT_SUITABLE;
        }
        ReportAttachmentGuard attachment_guard(report_handle);

        void *extra_storage = hidd_report_extra(report_handle);
        if (extra_storage == nullptr) {
            return AttachResult::NOT_SUITABLE;
        }

        const std::uint64_t generation = node->next_controller_generation();
        auto *context = new (extra_storage) ReportContext{
            kReportContextMagic,
            node,
            inst,
            report_handle,
            parser,
            inst->devno,
            inst->device_ident.vendor_id,
            inst->device_ident.product_id,
            generation,
            report_length
        };

        if (!node->activate_controller(inst, report_handle, generation)) {
            context->~ReportContext();
            return AttachResult::BUSY;
        }
        attachment_guard.release();

        RCLCPP_INFO(node->get_logger(),
            "Attached exclusive HID report for devno=%u (VID=0x%04x, PID=0x%04x, len=%u).",
            static_cast<unsigned>(inst->devno),
            static_cast<unsigned>(inst->device_ident.vendor_id),
            static_cast<unsigned>(inst->device_ident.product_id),
            static_cast<unsigned>(report_length));
        return AttachResult::ATTACHED;
    }

    static void handle_callback_exception(const char *callback_name, const char *detail) noexcept {
        std::fprintf(stderr, "HIDDI %s callback failed: %s\n", callback_name, detail);
        try {
            std::lock_guard<std::mutex> callback_guard(instance_lifetime_lock_);
            JoyTeleopNode *node = instance_;
            if (node != nullptr && !node->stopping_.load(std::memory_order_acquire)) {
                node->fault_active_input("an exception interrupted HIDDI callback processing");
            }
        } catch (...) {
            // Never let recovery failure cross the C callback boundary.
        }
    }

    static void on_hidd_report(
        struct hidd_connection *conn,
        struct hidd_report *report_handle,
        void *data,
        std::uint32_t len,
        std::uint32_t flags,
        void *user_data) noexcept {
        try {
            on_hidd_report_impl(conn, report_handle, data, len, flags, user_data);
        } catch (const std::exception &error) {
            handle_callback_exception("report", error.what());
        } catch (...) {
            handle_callback_exception("report", "unknown exception");
        }
    }

    static void on_hidd_report_impl(
        struct hidd_connection *,
        struct hidd_report *report_handle,
        void *data,
        std::uint32_t len,
        std::uint32_t flags,
        void *user_data) {
        std::lock_guard<std::mutex> callback_guard(instance_lifetime_lock_);
        JoyTeleopNode *node = instance_;
        if (node == nullptr || node->stopping_.load(std::memory_order_acquire)) {
            return;
        }

        // Check live state before touching report-specific storage. A delayed
        // callback after detach must not dereference freed extra data.
        if (report_handle == nullptr || !node->is_active_report(report_handle)) {
            return;
        }
        if ((flags & (HIDD_REPORT_BUFFER_OVERFLOW | HIDD_REPORTS_RESUMED)) != 0u) {
            node->fault_active_input("HIDDI reported loss or resumption of the report stream");
            return;
        }
        if (user_data == nullptr || data == nullptr) {
            node->fault_active_input("HIDDI supplied a null report buffer or context");
            return;
        }
        void *expected_user_data = hidd_report_extra(report_handle);
        if (expected_user_data == nullptr || user_data != expected_user_data) {
            node->fault_active_input("HIDDI supplied context outside the active report storage");
            return;
        }

        auto *context = static_cast<ReportContext *>(user_data);
        if (context->magic != kReportContextMagic || context->owner != node ||
            context->device == nullptr || context->report_handle != report_handle ||
            context->parser == nullptr) {
            node->fault_active_input("HIDDI report context did not match the active controller");
            return;
        }
        if (len != context->expected_report_length) {
            node->fault_active_input("HID report length differed from its attached descriptor");
            return;
        }
        if (validate_parser_report(
                static_cast<int>(context->vendor_id),
                static_cast<int>(context->product_id),
                static_cast<int>(len),
                static_cast<const std::uint8_t *>(data)) != 1) {
            node->fault_active_input("HID report failed its VID/PID wire-profile validation");
            return;
        }

        node->accept_report(*context, static_cast<const std::uint8_t *>(data), len);
    }

    static void on_hidd_insert(
        struct hidd_connection *conn,
        hidd_device_instance_t *inst) noexcept {
        try {
            on_hidd_insert_impl(conn, inst);
        } catch (const std::exception &error) {
            handle_callback_exception("insertion", error.what());
        } catch (...) {
            handle_callback_exception("insertion", "unknown exception");
        }
    }

    static void on_hidd_insert_impl(struct hidd_connection *conn, hidd_device_instance_t *inst) {
        std::lock_guard<std::mutex> callback_guard(instance_lifetime_lock_);
        JoyTeleopNode *node = instance_;
        if (node == nullptr || node->stopping_.load(std::memory_order_acquire) || inst == nullptr) {
            return;
        }
        if (check_allowed(inst->device_ident.vendor_id, inst->device_ident.product_id) != 1) {
            return;
        }
        if (node->has_active_controller()) {
            RCLCPP_WARN(node->get_logger(),
                "Ignoring additional supported controller devno=%u; remove the active controller before handoff.",
                static_cast<unsigned>(inst->devno));
            return;
        }

        struct hidd_collection **collections = nullptr;
        _Uint16t num_collections = 0;
        if (hidd_get_collections(inst, nullptr, &collections, &num_collections) != EOK ||
            (num_collections > 0 && collections == nullptr)) {
            RCLCPP_WARN(node->get_logger(), "Could not enumerate HID collections for supported controller.");
            return;
        }

        for (_Uint16t i = 0; i < num_collections; ++i) {
            if (!collection_is_game_controller(collections[i])) {
                continue;
            }

            AttachResult result = try_attach_to_report(node, conn, inst, collections[i]);
            if (result == AttachResult::ATTACHED) {
                return;
            }
            if (result == AttachResult::BUSY) {
                node->fault_active_input("exclusive HID report is already in use");
                RCLCPP_ERROR(node->get_logger(), "Controller report is busy; refusing a non-exclusive fallback.");
                return;
            }

            // Nested report collections inherit the validated gamepad
            // application collection but are still length/profile checked.
            struct hidd_collection **nested_collections = nullptr;
            _Uint16t num_nested_collections = 0;
            if (hidd_get_collections(nullptr, collections[i], &nested_collections, &num_nested_collections) != EOK ||
                (num_nested_collections > 0 && nested_collections == nullptr)) {
                continue;
            }
            for (_Uint16t j = 0; j < num_nested_collections; ++j) {
                result = try_attach_to_report(node, conn, inst, nested_collections[j]);
                if (result == AttachResult::ATTACHED) {
                    return;
                }
                if (result == AttachResult::BUSY) {
                    node->fault_active_input("exclusive HID report is already in use");
                    RCLCPP_ERROR(node->get_logger(), "Controller report is busy; refusing a non-exclusive fallback.");
                    return;
                }
            }
        }

        RCLCPP_WARN(node->get_logger(),
            "Supported VID/PID did not expose a validated gamepad report; controller was rejected.");
    }

    static void on_hidd_remove(
        struct hidd_connection *conn,
        hidd_device_instance_t *inst) noexcept {
        try {
            on_hidd_remove_impl(conn, inst);
        } catch (const std::exception &error) {
            handle_callback_exception("removal", error.what());
        } catch (...) {
            handle_callback_exception("removal", "unknown exception");
        }
    }

    static void on_hidd_remove_impl(struct hidd_connection *conn, hidd_device_instance_t *inst) {
        std::lock_guard<std::mutex> callback_guard(instance_lifetime_lock_);
        JoyTeleopNode *node = instance_;
        if (node == nullptr || node->stopping_.load(std::memory_order_acquire) || inst == nullptr) {
            return;
        }
        if (check_allowed(inst->device_ident.vendor_id, inst->device_ident.product_id) != 1) {
            return;
        }

        bool was_active = false;
        {
            std::lock_guard<std::mutex> publish_guard(node->publish_order_lock_);
            std::lock_guard<std::mutex> state_guard(node->joy_state_lock_);
            if (node->active_device_ == inst && node->active_devno_ == inst->devno) {
                // Clear report ownership before detach; the terminal fault below
                // makes neutral final and forbids controller handoff.
                node->clear_active_controller_locked(false);
                was_active = true;
            }
        }

        const int detach_status = hidd_reports_detach(conn, inst);
        if (detach_status != EOK) {
            RCLCPP_WARN(node->get_logger(), "hidd_reports_detach failed: %s (%d)",
                std::strerror(detach_status), detach_status);
        }
        if (was_active) {
            node->fault_active_input("the active controller was removed");
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<bool> stopping_{false};
    std::atomic<bool> terminal_input_failure_{false};

    // Lock order is publish_order_lock_ then joy_state_lock_. It covers the
    // complete sample-selection-to-publish interval and safety revocations.
    mutable std::mutex publish_order_lock_;
    mutable std::mutex joy_state_lock_;
    std::array<float, kAxisCount> latest_axes_{};
    std::array<std::int32_t, kButtonCount> latest_buttons_{};
    builtin_interfaces::msg::Time latest_report_ros_stamp_{};
    std::chrono::steady_clock::time_point latest_report_steady_time_{};
    bool have_fresh_report_{false};
    joy_teleop::PublicationGate publication_gate_;
    bool awaiting_neutral_{false};
    bool neutral_wait_warning_emitted_{false};
    hidd_device_instance_t *active_device_{nullptr};
    struct hidd_report *active_report_handle_{nullptr};
    std::uint32_t active_devno_{0};
    std::uint64_t controller_generation_{0};

    hidd_device_ident_t device_filter_{};
    hidd_connect_parm_t hid_parms_{};
    struct hidd_connection *hid_conn_{nullptr};

    static std::mutex instance_lifetime_lock_;
    static JoyTeleopNode *instance_;
};

std::mutex JoyTeleopNode::instance_lifetime_lock_;
JoyTeleopNode *JoyTeleopNode::instance_ = nullptr;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    int exit_code = EXIT_SUCCESS;

    try {
        auto node = std::make_shared<JoyTeleopNode>();
        if (!node->initialize_hid_connection()) {
            exit_code = EXIT_FAILURE;
        } else {
            rclcpp::spin(node);
            if (node->terminal_input_failure()) {
                exit_code = EXIT_FAILURE;
            }
        }
    } catch (const std::exception &error) {
        std::fprintf(stderr, "joy_teleop_node failed: %s\n", error.what());
        exit_code = EXIT_FAILURE;
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return exit_code;
}
