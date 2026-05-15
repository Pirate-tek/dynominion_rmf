#!/bin/bash

# Helper script to dispatch sample RMF tasks for the dynominion fleet
# Valid waypoints in nav_graphs/0.yaml:
#   charger_main, lobby_waypoint, hub_center, room_entry,
#   room_inside, corridor_bottom, bottom_zone, corridor_north

FLEET_NAME="dynominion_fleet"

print_help() {
    echo "Usage: ./dispatch_sample_tasks.sh [task_type]"
    echo "Available task types:"
    echo "  goto_lobby    - Send a robot to lobby_waypoint"
    echo "  goto_hub      - Send a robot to hub_center"
    echo "  goto_room     - Send a robot to room_entry"
    echo "  goto_bottom   - Send a robot to bottom_zone"
    echo "  patrol        - Patrol between lobby_waypoint and hub_center"
    echo "  loop          - Loop between charger_main and corridor_north"
}

if [ -z "$1" ]; then
    print_help
    exit 1
fi

case "$1" in
    goto_lobby)
        echo "Dispatching GoTo lobby_waypoint..."
        ros2 run rmf_demos_tasks dispatch_go_to_place -p lobby_waypoint -F $FLEET_NAME
        ;;
    goto_hub)
        echo "Dispatching GoTo hub_center..."
        ros2 run rmf_demos_tasks dispatch_go_to_place -p hub_center -F $FLEET_NAME
        ;;
    goto_room)
        echo "Dispatching GoTo room_entry..."
        ros2 run rmf_demos_tasks dispatch_go_to_place -p room_entry -F $FLEET_NAME
        ;;
    goto_bottom)
        echo "Dispatching GoTo bottom_zone..."
        ros2 run rmf_demos_tasks dispatch_go_to_place -p bottom_zone -F $FLEET_NAME
        ;;
    patrol)
        echo "Dispatching Patrol (lobby_waypoint <-> hub_center x2)..."
        ros2 run rmf_demos_tasks dispatch_patrol -p lobby_waypoint hub_center -n 2 -F $FLEET_NAME
        ;;
    loop)
        echo "Dispatching Loop (charger_main <-> corridor_north x2)..."
        ros2 run rmf_demos_tasks dispatch_loop -s charger_main -f corridor_north -n 2 -F $FLEET_NAME
        ;;
    *)
        echo "Unknown task type: $1"
        print_help
        exit 1
        ;;
esac
