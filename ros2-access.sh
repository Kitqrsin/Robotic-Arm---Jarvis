#!/bin/bash
# ROS2 Docker Access Script for Jarvis Robotic Arm
# Author: Auto-generated
# Usage: ./ros2-access.sh [command]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
CONTAINER_NAME="ros2_robot_arm"
IMAGE_NAME="ros2-arm:latest"
WORKSPACE_PATH="/home/tnt/arm_project/Robotic-Arm---Jarvis"

# Function to print colored messages
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if container is running
check_container() {
    local running=$(sudo docker ps -q -f name="^${CONTAINER_NAME}$" 2>/dev/null)
    if [ -n "$running" ]; then
        return 0
    else
        return 1
    fi
}

# Function to check if container exists (running or stopped)
check_container_exists() {
    local exists=$(sudo docker ps -aq -f name="^${CONTAINER_NAME}$" 2>/dev/null)
    if [ -n "$exists" ]; then
        return 0
    else
        return 1
    fi
}

# Function to remove stopped container
remove_stopped_container() {
    if check_container_exists && ! check_container; then
        print_info "Removing stopped container: ${CONTAINER_NAME}..."
        sudo docker rm ${CONTAINER_NAME} >/dev/null 2>&1
        print_success "Stopped container removed."
    fi
}

# Function to check if image exists
check_image() {
    if sudo docker images -q ${IMAGE_NAME} 2>/dev/null; then
        return 0
    else
        return 1
    fi
}

# Display usage information
show_usage() {
    cat << EOF
${GREEN}ROS2 Docker Access Script for Jarvis Robotic Arm${NC}

Usage: ./ros2-access.sh [COMMAND]

${YELLOW}Commands:${NC}
  start       - Start a new ROS2 Docker container (default)
  exec        - Execute bash in running container
  stop        - Stop the running container
  restart     - Restart the container
  build       - Build the Docker image
  rebuild     - Rebuild the Docker image from scratch
  logs        - View container logs
  status      - Check container status
  clean       - Remove stopped containers
  help        - Show this help message

${YELLOW}Examples:${NC}
  ./ros2-access.sh                    # Start new container with bash
  ./ros2-access.sh exec               # Access running container
  ./ros2-access.sh start "colcon build" # Start with custom command
  ./ros2-access.sh build              # Build Docker image

${YELLOW}Quick ROS2 Commands (once inside):${NC}
  colcon build                        # Build workspace
  source install/setup.bash           # Source workspace
  ros2 launch arm_controller arm_control.launch.py  # Launch arm control
  ros2 topic list                     # List ROS2 topics
  ros2 node list                      # List ROS2 nodes

EOF
}

# Build Docker image
build_image() {
    print_info "Building Docker image: ${IMAGE_NAME}..."
    cd ${WORKSPACE_PATH}
    
    if [ "$1" == "--no-cache" ]; then
        print_info "Building without cache..."
        sudo docker build --no-cache -t ${IMAGE_NAME} .
    else
        sudo docker build -t ${IMAGE_NAME} .
    fi
    
    if [ $? -eq 0 ]; then
        print_success "Docker image built successfully!"
    else
        print_error "Failed to build Docker image"
        exit 1
    fi
}

# Start new container
start_container() {
    if check_container; then
        print_warning "Container ${CONTAINER_NAME} is already running!"
        print_info "Use './ros2-access.sh exec' to access it or './ros2-access.sh restart' to restart."
        exit 1
    fi
    
    # Remove stopped container if it exists
    remove_stopped_container
    
    if ! check_image; then
        print_warning "Docker image ${IMAGE_NAME} not found!"
        print_info "Building image first..."
        build_image
    fi
    
    print_info "Starting ROS2 Docker container: ${CONTAINER_NAME}..."
    
    # Allow X11 connections
    xhost +local:docker 2>/dev/null || true
    
    COMMAND="${1:-bash}"
    
    sudo docker run -it --rm \
        --privileged \
        --network host \
        --name ${CONTAINER_NAME} \
        -v ${WORKSPACE_PATH}:/workspace \
        -v /dev:/dev \
        -e DISPLAY=${DISPLAY} \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -w /workspace \
        ${IMAGE_NAME} \
        bash -c "source /opt/ros/humble/setup.bash && \
                 if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && \
                 ${COMMAND}"
    
    print_info "Container stopped."
}

# Execute command in running container
exec_container() {
    if ! check_container; then
        print_error "Container ${CONTAINER_NAME} is not running!"
        print_info "Use './ros2-access.sh start' to start it."
        exit 1
    fi
    
    print_info "Accessing running container: ${CONTAINER_NAME}..."
    
    sudo docker exec -it ${CONTAINER_NAME} \
        bash -c "source /opt/ros/humble/setup.bash && \
                 if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && \
                 bash"
}

# Stop container
stop_container() {
    if ! check_container; then
        print_warning "Container ${CONTAINER_NAME} is not running."
        exit 0
    fi
    
    print_info "Stopping container: ${CONTAINER_NAME}..."
    sudo docker stop ${CONTAINER_NAME}
    print_success "Container stopped."
}

# Restart container
restart_container() {
    print_info "Restarting container..."
    stop_container
    sleep 2
    start_container
}

# Show container logs
show_logs() {
    if ! check_container; then
        print_error "Container ${CONTAINER_NAME} is not running!"
        exit 1
    fi
    
    print_info "Showing logs for container: ${CONTAINER_NAME}..."
    sudo docker logs -f ${CONTAINER_NAME}
}

# Show container status
show_status() {
    print_info "Checking Docker status..."
    
    if check_image; then
        print_success "Docker image ${IMAGE_NAME} exists"
    else
        print_warning "Docker image ${IMAGE_NAME} not found"
    fi
    
    if check_container; then
        print_success "Container ${CONTAINER_NAME} is running"
        echo ""
        sudo docker ps -f name="^${CONTAINER_NAME}$" --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"
    elif check_container_exists; then
        print_warning "Container ${CONTAINER_NAME} exists but is stopped"
        echo ""
        sudo docker ps -a -f name="^${CONTAINER_NAME}$" --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"
        print_info "Run './ros2-access.sh start' to start a new container"
    else
        print_warning "Container ${CONTAINER_NAME} does not exist"
        print_info "Run './ros2-access.sh start' to create and start a new container"
    fi
}

# Clean up stopped containers
clean_containers() {
    print_info "Cleaning up stopped containers..."
    sudo docker container prune -f
    print_success "Cleanup complete."
}

# Main script logic
case "${1}" in
    start)
        start_container "${2}"
        ;;
    exec)
        exec_container
        ;;
    stop)
        stop_container
        ;;
    restart)
        restart_container
        ;;
    build)
        build_image
        ;;
    rebuild)
        build_image "--no-cache"
        ;;
    logs)
        show_logs
        ;;
    status)
        show_status
        ;;
    clean)
        clean_containers
        ;;
    help|--help|-h)
        show_usage
        ;;
    "")
        start_container
        ;;
    *)
        print_error "Unknown command: ${1}"
        echo ""
        show_usage
        exit 1
        ;;
esac
