#pragma once

/*
 * can_node.h - CAN node of the CAN_GAUGE board (see can_node.c)
 */

#include <stdbool.h>

// Starts the CAN node: loads the node ID and the settings from the NVS,
// starts the CAN controller and creates the CAN task.
void can_node_start(void);

// True when the CAN controller is running (not bus-off) and at least one
// frame has been received since start-up. Receiving a frame also means the
// board acknowledged it on the bus, so the bus works in both directions.
bool can_node_bus_ok(void);