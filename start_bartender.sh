#!/bin/bash
# Start the Bartender Behavior Tree web server in Docker

cd "$(dirname "$0")" || exit 1

echo "🤖 Starting Bartender Robot Web Dashboard..."
echo ""

docker compose exec -d ros2_arm bash -c "cd /workspace && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run arm_controller bartender_behavior_tree"

sleep 2

echo "✅ Bartender started!"
echo ""
echo "📱 Web Dashboard: http://localhost:5000"
echo ""
echo "To trigger a drink order, run:"
echo "  ros2 topic pub /make_drink std_msgs/String \"data: 'make_drink'\""
echo ""
echo "To stop bartender, run:"
echo "  pkill -f bartender_behavior_tree"
