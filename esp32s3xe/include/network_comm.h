#ifndef NETWORK_COMM_H
#define NETWORK_COMM_H

#include <Arduino.h>

void init_network();
void update_network();
void broadcast_telemetry();
void send_occupancy_grid();
void setArchitectureProfile(const char* profile);

#endif // NETWORK_COMM_H
