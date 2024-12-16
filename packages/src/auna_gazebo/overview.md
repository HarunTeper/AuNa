# Package Overview

This document provides an overview of the directory structure for the `auna_gazebo` package.

## Directory Structure

```
/home/daes-enzo/AuNa/packages/src/auna_gazebo/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── example.launch
│   └── another_example.launch
├── src/
│   ├── algorithm_name/
│   │   ├── main.cpp
│   │   └── node.cpp
├── include/
│   └── auna_gazebo/
│       └── node.hpp
├── worlds/
│   ├── example.world
│   └── another_example.world
└── models/
    ├── example_model/
    │   ├── model.config
    │   └── model.sdf
    └── another_model/
        ├── model.config
        └── model.sdf
```

## Description

- **CMakeLists.txt**: The CMake build configuration file.
- **package.xml**: The package manifest file.
- **launch/**: Directory containing launch files.
- **src/**: Source code files.
- **include/**: Header files.
- **worlds/**: Gazebo world files.
- **models/**: Gazebo model files.

This structure helps in organizing the package for better maintainability and readability.