#include "autonomous_vacuum_nodes/docking.hpp"
#include "autonomous_vacuum_nodes/energy_model.hpp"

#include "autonomous_vacuum_interfaces/srv/save_map.hpp"
#include "autonomous_vacuum_interfaces/msg/map_snapshot.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "autonomous_vacuum_interfaces/action/vacuum_task.hpp"
#include "autonomous_vacuum_interfaces/msg/coverage.hpp"
#include "autonomous_vacuum_interfaces/msg/floor_config.hpp"
#include "autonomous_vacuum_interfaces/msg/session_command.hpp"
#include "autonomous_vacuum_interfaces/msg/vacuum_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using SaveMap = autonomous_vacuum_interfaces::srv::SaveMap;
using MapSnapshot = autonomous_vacuum_interfaces::msg::MapSnapshot;
using VacuumTask = autonomous_vacuum_interfaces::action::VacuumTask;
using GoalHandle = rclcpp_action::ServerGoalHandle<VacuumTask>;
using FloorConfig = autonomous_vacuum_interfaces::msg::FloorConfig;
using SessionCommand = autonomous_vacuum_interfaces::msg::SessionCommand;
using Coverage = autonomous_vacuum_interfaces::msg::Coverage;
using VacuumStatus = autonomous_vacuum_interfaces::msg::VacuumStatus;
using Odometry = nav_msgs::msg::Odometry;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

enum class Phase { working, returning, charging };

struct CleaningSession
{
  std::shared_ptr<GoalHandle> goal;
  SessionCommand command;
  Coverage progress;
  Clock::time_point started = Clock::now();
  Clock::time_point next_feedback = started;
  Clock::time_point phase_started = started;
  unsigned int departures = 0;
  Phase phase = Phase::working;
  std::optional<Clock::time_point> stop_requested;
  rclcpp::Time stop_stamp{0, 0, RCL_ROS_TIME};
  bool stationary = false;
  MapSnapshot::ConstSharedPtr snapshot;
  bool saving_map = false;
  bool map_saved = false;
  uint8_t resource_stop = VacuumTask::Result::SUCCESS;
  std::string failure;
};

class TaskManager : public rclcpp::Node
{
public:
  TaskManager() : Node("task_manager")
  {
    command_pub_ = create_publisher<SessionCommand>("/vacuum/session_state", 10);
    docking_ = std::make_unique<Docking>(*this, command_pub_, [this]() { return session_.has_value(); });
    floor_sub_ = create_subscription<FloorConfig>(
      "/vacuum/floor/config", rclcpp::QoS(1).transient_local(), [this](const FloorConfig & floor) {
        if (session_) {
          session_->failure = "Floor configuration changed during the session";
        }
        docking_->set_floor(floor);
        floor_ = floor;

        odom_received_.reset();
      });
    save_client_ = create_client<SaveMap>("/vacuum/floor/save_map");
    snapshot_sub_ = create_subscription<MapSnapshot>("/vacuum/mapping/snapshot", 5,
      [this](MapSnapshot::ConstSharedPtr snapshot) {
        if (session_ && snapshot->session_id == session_->command.session_id && floor_ &&
          snapshot->grid.info.map_load_time == floor_->grid.info.map_load_time)
        {
          session_->snapshot = snapshot;
        }
      });
    status_sub_ = create_subscription<VacuumStatus>(
      "/vacuum/status", 10, [this](const VacuumStatus & status) { status_ = status; status_received_ = Clock::now(); });
    odom_sub_ = create_subscription<Odometry>(
      "/vacuum/sensors/odom", 1, [this](const Odometry & odom) { on_odometry(odom); });
    coverage_sub_ = create_subscription<Coverage>(
      "/vacuum/cleaning/coverage", 10, [this](const Coverage & progress) {
        if (session_ && progress.session_id == session_->command.session_id) {
          session_->progress = progress;
        }
      });

    server_ = rclcpp_action::create_server<VacuumTask>(this, "/vacuum/run_task",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const VacuumTask::Goal> goal) {
        return evaluate_goal(*goal);
      },
      [this](const std::shared_ptr<GoalHandle> handle) {
        if (session_ && session_->goal == handle &&
          (!session_->stop_requested || session_->resource_stop != VacuumTask::Result::SUCCESS)) {
          return rclcpp_action::CancelResponse::ACCEPT;
        }
        return rclcpp_action::CancelResponse::REJECT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) { start_session(handle); });

    timer_ = create_wall_timer(50ms, [this]() { tick(); });
  }

private:
  void on_odometry(const Odometry & odom)
  {
    docking_->observe(odom);
    const rclcpp::Time stamp(odom.header.stamp);
    if (!floor_ || stamp < rclcpp::Time(floor_->grid.header.stamp)) {
      return;
    }
    odom_received_ = Clock::now();
    if (session_ && session_->stop_requested && stamp > session_->stop_stamp) {
      const auto & velocity = odom.twist.twist;
      session_->stationary = std::abs(velocity.linear.x) < 0.001 &&
        std::abs(velocity.angular.z) < 0.001;
    }
  }

  rclcpp_action::GoalResponse evaluate_goal(const VacuumTask::Goal & goal)
  {
    std::string reason;
    if (session_ || docking_->busy()) {
      reason = "Another session is active";
    } else if (!floor_ || (goal.mode == "CLEAN_EXISTING_MAP" && floor_->map_state == "unexplored")) {
      reason = "Select a floor with a known map before cleaning";
    } else if (goal.mode != "CLEAN_EXISTING_MAP" && goal.mode != "MAP_ONLY" && goal.mode != "SLAM_AND_CLEAN") {
      reason = "Unknown task mode";
    } else if (goal.power_mode != "ECO" && goal.power_mode != "STANDARD" && goal.power_mode != "TURBO") {
      reason = "Unknown power mode";
    } else if (!status_ || Clock::now() - status_received_ > 2s || !odom_received_ || Clock::now() - *odom_received_ > 1s) {
      reason = "Waiting for live telemetry and floor odometry";
    } else if (goal.mode != "MAP_ONLY" && status_->dustbin_cells >= status_->dustbin_capacity_cells) {
      reason = "Dustbin is full";
    } else if (status_->battery_percent <= energy::return_reserve(0, floor_->grid.info.resolution) +
      energy::next_step_budget(floor_->grid.info.resolution, energy::work_rate(goal.mode, goal.power_mode)))
    {
      reason = "Insufficient battery to start a task";
    }
    if (!reason.empty()) {
      RCLCPP_WARN(get_logger(), "Rejected goal: %s", reason.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void start_session(const std::shared_ptr<GoalHandle> & handle)
  {
    auto & session = session_.emplace();
    session.goal = handle;
    session.command.task_id = rclcpp_action::to_string(handle->get_goal_id());
    session.command.floor_id = floor_->floor_id;
    session.command.mode = handle->get_goal()->mode;
    session.command.power_mode = handle->get_goal()->power_mode;
    depart_for_work();
  }

  void depart_for_work()
  {
    auto & session = *session_;
    session.phase = Phase::working;
    session.phase_started = Clock::now();
    session.next_feedback = session.phase_started;
    session.stop_requested.reset();
    session.stationary = false;
    session.snapshot.reset();
    session.saving_map = session.map_saved = false;
    session.resource_stop = VacuumTask::Result::SUCCESS;
    // A fresh segment ID excludes late coverage and map snapshots from the previous departure.
    session.command.session_id = session.command.task_id;
    if (session.departures > 0) {
      session.command.session_id += "/" + std::to_string(session.departures);
    }
    ++session.departures;
    session.command.running = true;
    session.command.cleaned_cells_at_start = session.progress.cleaned_cells;
    session.command.available_bin_cells = status_->dustbin_capacity_cells - status_->dustbin_cells;
    command_pub_->publish(session.command);
    RCLCPP_INFO(get_logger(), "Started %s (%s) on %s", session.command.mode.c_str(), session.command.power_mode.c_str(),
      session.command.floor_id.c_str());
  }

  void request_stop()
  {
    session_->stop_requested = Clock::now();
    session_->stop_stamp = now();
    session_->command.running = false;
    command_pub_->publish(session_->command);
  }

  void tick()
  {
    if (!session_) {
      return;
    }
    auto & session = *session_;
    const auto now_steady = Clock::now();
    publish_feedback(now_steady);
    if (session.phase != Phase::working) {
      handle_dock_visit(now_steady);
      return;
    }
    if (!session.stop_requested) {
      const bool planner_ready = session.progress.session_id == session.command.session_id;
      if (!odom_received_ || now_steady - *odom_received_ > 2s) {
        session.failure = "Lost floor odometry";
      } else if (!planner_ready && now_steady - session.phase_started > 3s) {
        session.failure = "Planner did not start cleaning";
      }
      if (planner_ready && !session.progress.complete) {
        const auto & command = session.command;
        const double headroom = energy::next_step_budget(floor_->grid.info.resolution,
          energy::work_rate(command.mode, command.power_mode));
        const auto cleaned_since_departure = session.progress.cleaned_cells - command.cleaned_cells_at_start;
        if (session.progress.cleaned_cells < command.cleaned_cells_at_start) {
          session.failure = "Planner lost saved coverage";
        } else if (session.progress.return_reserve_percent < 0) {
          session.failure = "No known free route to dock";
        } else if (command.mode != "MAP_ONLY" && cleaned_since_departure >= command.available_bin_cells) {
          session.resource_stop = VacuumTask::Result::DUSTBIN_FULL;
        } else if (status_->battery_percent <= session.progress.return_reserve_percent + headroom ||
          now_steady - status_received_ > 2s)
        {
          session.resource_stop = VacuumTask::Result::LOW_BATTERY;
        }
      }
      if (session.goal->is_canceling() || session.progress.complete || !session.failure.empty() ||
        session.resource_stop != VacuumTask::Result::SUCCESS)
      {
        request_stop();
      }
    }
    if (session.stop_requested) { finish_work_segment(now_steady); }
  }

  void finish_work_segment(Clock::time_point now_steady)
  {
    auto & session = *session_;
    const auto elapsed = now_steady - *session.stop_requested;
    const bool stopped = session.stationary && session.progress.stopped &&
      session.progress.session_id == session.command.session_id;
    if (!stopped) {
      if (elapsed > 3s) {
        session.failure = "Could not confirm planner and motion stop";
        finish_session();
      }
      return;
    }
    const bool needs_save = session.command.mode != "CLEAN_EXISTING_MAP" &&
      session.failure.empty() && !session.map_saved;
    if (needs_save) {
      if (elapsed > 5s) {
        session.failure = "Timed out saving the map";
        finish_session();
      } else if (session.snapshot && !session.saving_map) {
        save_session_map();
      }
      return;
    }
    if (session.resource_stop != VacuumTask::Result::SUCCESS && session.failure.empty() &&
      !session.goal->is_canceling() && !session.progress.complete)
    {
      return_for_service();
    } else {
      finish_session();
    }
  }

  void publish_feedback(Clock::time_point now_steady)
  {
    auto & session = *session_;
    if (now_steady < session.next_feedback) { return; }
    auto feedback = std::make_shared<VacuumTask::Feedback>();
    feedback->current_room = session.command.floor_id + "/main";
    feedback->completion_percent = session.progress.completion_percent;
    feedback->battery_percent = status_->battery_percent;
    if (session.phase == Phase::charging) {
      feedback->phase = "CHARGING";
    } else if (session.phase == Phase::returning) {
      feedback->phase = session.resource_stop == VacuumTask::Result::DUSTBIN_FULL ?
        "RETURNING_DUSTBIN_FULL" : "RETURNING_LOW_BATTERY";
    } else {
      feedback->phase = "WORKING";
    }
    session.goal->publish_feedback(feedback);
    session.next_feedback = now_steady + 1s;
  }

  void return_for_service()
  {
    auto & session = *session_;
    session.phase = Phase::returning;
    session.next_feedback = Clock::now();
    const auto segment = session.command.session_id;
    docking_->request_return(session.command.floor_id, segment, [this, segment](bool docked) {
      if (!session_ || session_->command.session_id != segment) { return; }
      if (!docked) {
        session_->failure = "Service return did not reach the dock";
        return;
      }
      session_->phase = Phase::charging;
      session_->phase_started = Clock::now();
      session_->next_feedback = session_->phase_started;
    });
  }

  void handle_dock_visit(Clock::time_point now_steady)
  {
    auto & session = *session_;
    if (session.goal->is_canceling() || !session.failure.empty()) {
      finish_session();
      return;
    }
    if (session.phase == Phase::returning) { return; }
    // Require a hardware reading after arrival, including the bin-empty acknowledgement.
    if (status_received_ > session.phase_started && status_->battery_percent == 100.0F &&
      status_->dustbin_cells == 0)
    {
      depart_for_work();
    } else if (now_steady - session.phase_started > 10s) {
      session.failure = "Dock did not report a full battery and empty bin";
      finish_session();
    }
  }

  void save_session_map()
  {
    if (!save_client_->service_is_ready()) {
      session_->failure = "Map storage unavailable";
      return;
    }
    auto request = std::make_shared<SaveMap::Request>();
    request->floor_id = session_->command.floor_id;
    request->grid = session_->snapshot->grid;
    request->map_state = session_->progress.complete || floor_->map_state == "complete" ? "complete" : "partial";
    session_->saving_map = true;
    const auto id = session_->command.session_id;
    save_client_->async_send_request(request,
      [this, id, state = request->map_state](rclcpp::Client<SaveMap>::SharedFuture future) {
        if (!session_ || session_->command.session_id != id) { return; }
        const auto response = future.get();
        if (response->success) {
          floor_->map_state = state;
          session_->map_saved = true;
        } else {
          session_->failure = response->message;
        }
      });
  }

  void finish_session()
  {
    auto & session = *session_;
    auto result = std::make_shared<VacuumTask::Result>();
    result->cleaned_area_m2 = session.progress.cleaned_area_m2;
    result->elapsed_seconds = std::chrono::duration<double>(Clock::now() - session.started).count();
    if (session.goal->is_canceling()) {
      result->completion_code = session.failure.empty() ? result->CANCELED : result->FAILED;
      result->message = session.failure.empty() ? "Session canceled; robot stopped" : session.failure;
      if (session.failure.empty() && session.phase == Phase::returning) {
        result->message = "Cleaning canceled; return to dock continues";
      }
      if (session.map_saved) { result->message += "; map saved"; }
      session.goal->canceled(result);
    } else if (!session.failure.empty()) {
      result->completion_code = result->FAILED;
      result->message = session.failure;
      session.goal->abort(result);
    } else {
      result->completion_code = result->SUCCESS;
      result->message = session.command.mode == "MAP_ONLY" ? "Reachable area mapped; robot stopped" :
        "All reachable known cells cleaned; robot stopped";
      if (session.map_saved) { result->message += "; map saved"; }
      session.goal->succeed(result);
    }
    RCLCPP_INFO(get_logger(), "%s (%.2f m2)", result->message.c_str(), result->cleaned_area_m2);
    const bool return_to_dock = session.failure.empty() && session.phase == Phase::working;
    const auto floor_id = session.command.floor_id;
    session_.reset();
    if (return_to_dock) {
      docking_->request_return(floor_id);
    }
  }

  rclcpp::Subscription<MapSnapshot>::SharedPtr snapshot_sub_;
  rclcpp::Client<SaveMap>::SharedPtr save_client_;
  std::unique_ptr<Docking> docking_;
  std::optional<CleaningSession> session_;
  std::optional<FloorConfig> floor_;
  std::optional<VacuumStatus> status_;
  Clock::time_point status_received_{};
  std::optional<Clock::time_point> odom_received_;
  rclcpp_action::Server<VacuumTask>::SharedPtr server_;
  rclcpp::Publisher<SessionCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<FloorConfig>::SharedPtr floor_sub_;
  rclcpp::Subscription<VacuumStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<Coverage>::SharedPtr coverage_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskManager>());
  rclcpp::shutdown();
  return 0;
}
