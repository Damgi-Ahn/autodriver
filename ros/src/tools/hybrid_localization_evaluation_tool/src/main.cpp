#include "hybrid_localization_evaluation_tool/evaluation_node.hpp"
#include "hybrid_localization_evaluation_tool/main_window.hpp"
#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"

#include <QApplication>

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <thread>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto bridge = std::make_shared<autodriver::tools::RosQtBridge>();
  auto node = std::make_shared<autodriver::tools::EvaluationNode>();
  node->SetBridge(bridge);

  autodriver::tools::EvaluationMainWindow window(bridge, node->GetExporter());
  window.SetNisGates(node->nis_gate_pos(), node->nis_gate_vel(), node->nis_gate_heading());
  window.show();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread ros_thread([&executor]() { executor.spin(); });

  const int result = app.exec();
  rclcpp::shutdown();
  ros_thread.join();
  return result;
}
