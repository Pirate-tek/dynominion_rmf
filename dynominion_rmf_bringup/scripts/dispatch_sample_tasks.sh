#!/bin/bash

# Helper script to dispatch sample RMF tasks for the 3 dynominion fleets
# Valid waypoints in nav_graphs/0.yaml:
#   A1, A2, B1, B2, C1, C2, D1, D2, I1, I2, I3, I4, I5, I6, S1, S2, X

FLEET_NAME="${2:-dynominion_fleet1}"

print_help() {
    echo "Usage: ./dispatch_sample_tasks.sh [task_type] [fleet_name]"
    echo "Available fleets: dynominion_fleet1 (default), dynominion_fleet2, dynominion_fleet3"
    echo "Available task types:"
    echo "  goto_a1       - Send a robot to A1"
    echo "  goto_a2       - Send a robot to A2"
    echo "  goto_b1       - Send a robot to B1"
    echo "  goto_b2       - Send a robot to B2"
    echo "  goto_d1       - Send a robot to D1"
    echo "  goto_d2       - Send a robot to D2"
    echo "  goto_x        - Send a robot to the central intersection X"
    echo "  patrol_a      - Patrol between A1 and I2"
    echo "  loop_ab       - Loop between A1 and B2"
}

if [ -z "$1" ]; then
    print_help
    exit 1
fi

case "$1" in
    goto_a1)
        echo "Dispatching GoTo A1 for fleet $FLEET_NAME..."
        ros2 run rmf_demos_tasks dispatch_go_to_place -p A1 -F $FLEET_NAME
        ;;
    goto_a2)
        echo "Dispatching GoTo A2 for fleet $FLEET_NAME..."
        ros2 run rmf_demos_tasks dispatch_go_to_place -p A2 -F $FLEET_NAME
        ;;
    goto_b1)
        echo "Dispatching GoTo B1 for fleet $FLEET_NAME..."
        ros2 run rmf_demos_tasks dispatch_go_to_place -p B1 -F $FLEET_NAME
        ;;
    goto_b2)
        echo "Dispatching GoTo B2 for fleet $FLEET_NAME..."
        ros2 run rmf_demos_tasks dispatch_go_to_place -p B2 -F $FLEET_NAME
        ;;
    goto_d1)
        echo "Dispatching GoTo D1 for fleet $FLEET_NAME..."
        ros2 run rmf_demos_tasks dispatch_go_to_place -p D1 -F $FLEET_NAME
        ;;
    goto_d2)
        echo "Dispatching GoTo D2 for fleet $FLEET_NAME..."
        ros2 run rmf_demos_tasks dispatch_go_to_place -p D2 -F $FLEET_NAME
        ;;
    goto_x)
        echo "Dispatching GoTo X for fleet $FLEET_NAME..."
        ros2 run rmf_demos_tasks dispatch_go_to_place -p X -F $FLEET_NAME
        ;;
    patrol_a)
        echo "Dispatching Patrol (A1 <-> I2 x2) for fleet $FLEET_NAME..."
        ros2 run rmf_demos_tasks dispatch_patrol -p A1 I2 -n 2 -F $FLEET_NAME
        ;;
    loop_ab)
        echo "Dispatching Loop (A1 <-> B2 x2) for fleet $FLEET_NAME..."
        ros2 run rmf_demos_tasks dispatch_loop -s A1 -f B2 -n 2 -F $FLEET_NAME
        ;;
    *)
        echo "Unknown task type: $1"
        print_help
        exit 1
        ;;
esac
