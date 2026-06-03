# Semantic RGB-D Navigation System for K12 Mobile Robot

This repository defines a ROS 2 visual semantic navigation system for a K12
mobile robot by combining:

- RGB-D perception
- Semantic segmentation
- RTAB-Map SLAM
- Semantic obstacle mapping
- Nav2 autonomous navigation

## System pipeline

1. RGB-D sensor streams aligned color and depth images into ROS 2 topics.
2. A semantic segmentation node classifies scene elements from RGB frames.
3. RTAB-Map consumes RGB-D odometry and builds a metric map.
4. A semantic obstacle mapping node projects segmented obstacle classes into the
   navigation map/costmap using depth and robot pose.
5. Nav2 plans and executes safe paths using geometric + semantic obstacle data.

## Core integration goal

The stack is intended to let the K12 platform navigate autonomously while
distinguishing obstacle semantics (e.g., traversable vs. non-traversable
classes) instead of relying only on geometry.
