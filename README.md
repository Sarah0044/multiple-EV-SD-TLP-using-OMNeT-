# Centralized Priority Resolution with Dynamic Preemption Transfer for Concurrent Emergency Vehicles in Software-Defined Traffic Networks

## Overview
This project extends the Software-Defined Traffic Light Preemption (SD-TLP) mechanism designed by Bagheri et al. to support **multiple emergency vehicles (EVs)**. The system is implemented using **OMNeT++ (without SUMO/Veins)** and simulates traffic using queue-based modeling.

The proposed extension uses a **centralized controller** that resolves conflicts between multiple EVs based on:
- Priority ( ambulance > fire > police)
- Estimated arrival time (EAT)
- Distance to accident point
- FCFS to Detection Distance (DD) if all the above selection points are equal

## Features
- Multi-EV traffic light preemption
- Conflict resolution between EVs
- Centralized SDN-like controller
- Queue-based traffic modeling (no external simulator)
- Performance evaluation under different traffic conditions:
  - Low
  - Medium
  - High


