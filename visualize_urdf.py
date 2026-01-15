#!/usr/bin/env python3
"""
Simple URDF structure visualizer - shows the kinematic chain
"""
import xml.etree.ElementTree as ET


def visualize_urdf(urdf_file):
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    robot_name = root.get("name")
    print(f"\n{'='*60}")
    print(f"  ROBOT: {robot_name}")
    print(f"{'='*60}\n")

    # Extract links
    links = {}
    for link in root.findall("link"):
        name = link.get("name")
        visual = link.find("visual")
        if visual is not None:
            geom = visual.find("geometry")
            if geom is not None:
                box = geom.find("box")
                if box is not None:
                    size = box.get("size")
                    links[name] = {"type": "box", "size": size}
        if name not in links:
            links[name] = {"type": "abstract"}

    # Extract joints
    joints = []
    for joint in root.findall("joint"):
        name = joint.get("name")
        jtype = joint.get("type")
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")

        origin = joint.find("origin")
        xyz = origin.get("xyz") if origin is not None else "0 0 0"

        axis_elem = joint.find("axis")
        axis = axis_elem.get("xyz") if axis_elem is not None else "N/A"

        joints.append(
            {
                "name": name,
                "type": jtype,
                "parent": parent,
                "child": child,
                "xyz": xyz,
                "axis": axis,
            }
        )

    # Build kinematic tree
    print("KINEMATIC CHAIN:")
    print("-" * 60)

    def print_chain(link_name, indent=0):
        prefix = "  " * indent
        link_info = links.get(link_name, {})
        if link_info.get("type") == "box":
            print(f"{prefix}📦 {link_name} (box: {link_info['size']})")
        else:
            print(f"{prefix}🔗 {link_name}")

        # Find children
        for joint in joints:
            if joint["parent"] == link_name:
                joint_symbol = "🔄" if joint["type"] == "revolute" else "🔒"
                print(f"{prefix}  {joint_symbol} {joint['name']} ({joint['type']})")
                print(f"{prefix}     offset: {joint['xyz']}")
                if joint["axis"] != "N/A":
                    print(f"{prefix}     axis: {joint['axis']}")
                print(f"{prefix}     ↓")
                print_chain(joint["child"], indent + 1)

    # Start from base_link
    print_chain("base_link")

    print("\n" + "=" * 60)
    print("JOINT SUMMARY:")
    print("-" * 60)
    for joint in joints:
        if joint["type"] == "revolute":
            print(f"  🔄 {joint['name']:15} - Rotates around axis {joint['axis']}")
        elif joint["type"] == "fixed":
            print(f"  🔒 {joint['name']:15} - Fixed connection")

    # Show ros2_control mapping
    ros2_control = root.find("ros2_control")
    if ros2_control is not None:
        print("\n" + "=" * 60)
        print("SERVO CHANNEL MAPPING:")
        print("-" * 60)
        for joint_elem in ros2_control.findall("joint"):
            joint_name = joint_elem.get("name")
            channel = joint_elem.find('param[@name="servo_channel"]')
            if channel is not None:
                print(f"  {joint_name:15} → Servo Channel {channel.text}")

    print("\n" + "=" * 60 + "\n")


if __name__ == "__main__":
    visualize_urdf("robot_arm.urdf")
