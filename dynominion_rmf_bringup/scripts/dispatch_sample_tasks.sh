#!/bin/bash

# Helper script to dispatch sample RMF tasks for the dynominion fleet

FLEET_NAME="dynominion_fleet"

print_help() {
    echo "Usage: ./dispatch_sample_tasks.sh [task_type]"
    echo "Available task types:"
    echo "  goto_table1  - Send a robot to table_1"
    echo "  goto_table2  - Send a robot to table_2"
    echo "  loop         - Create a loop between charger_1 and table_1"
    echo "  patrol       - Patrol between waiting_zone_1 and waiting_zone_2"
}

if [ -z "$1" ]; then
    print_help
    exit 1
fi

case "$1" in
    goto_table1)
        echo "Dispatching GoToTable1 task..."
        ros2 run rmf_demos_tasks dispatch_go_to_place -p table_1 -F $FLEET_NAME
        ;;
    goto_table2)
        echo "Dispatching GoToTable2 task..."
        ros2 run rmf_demos_tasks dispatch_go_to_place -p table_2 -F $FLEET_NAME
        ;;
    loop)
        echo "Dispatching Loop task (charger_1 <-> table_1)..."
        ros2 run rmf_demos_tasks dispatch_loop -s charger_1 -f table_1 -n 2
        ;;
    patrol)
        echo "Dispatching Patrol task (waiting_zone_1 <-> waiting_zone_2)..."
        ros2 run rmf_demos_tasks dispatch_patrol -p waiting_zone_1 waiting_zone_2 -n 2 -F $FLEET_NAME
        ;;
    *)
        echo "Unknown task type: $1"
        print_help
        exit 1
        ;;
esac
