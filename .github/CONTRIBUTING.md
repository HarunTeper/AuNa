# Contributing to AuNa

Thank you for your interest in contributing to AuNa! This document provides guidelines for contributing to the project.

## Table of Contents

- [Getting Started](#getting-started)
- [Development Setup](#development-setup)
- [Code Style](#code-style)
- [GitHub Copilot Usage](#github-copilot-usage)
- [Pull Request Process](#pull-request-process)
- [Testing](#testing)
- [Documentation](#documentation)

## Getting Started

### Prerequisites

- Docker with Docker Compose support
- Git
- Basic knowledge of ROS2 and C++/Python
- Familiarity with autonomous vehicle concepts

### Development Setup

1. **Fork and clone the repository:**
   ```bash
   git clone https://github.com/YOUR_USERNAME/AuNa.git
   cd AuNa
   ```

2. **Start the development container:**
   ```bash
   docker compose run development
   ```

   This command automatically builds and sources the workspace upon startup.

## Code Style

### ROS2 C++ Guidelines

- Follow the [ROS2 C++ Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- Use modern C++17 features
- Class names use PascalCase: `CaccControllerNode`
- Function and variable names use snake_case: `calculate_distance()`
- Member variables end with underscore: `publisher_`
- Constants use UPPER_SNAKE_CASE: `MAX_VELOCITY`

### Python Guidelines

- Follow [PEP 8](https://pep8.org/)
- Use type hints where appropriate
- Class names use PascalCase: `TeleopNode`
- Function and variable names use snake_case

### ROS2 Specific Patterns

#### Node Structure
```cpp
class ExampleNode : public rclcpp::Node
{
public:
  ExampleNode() : Node("example_node")
  {
    // Declare parameters
    this->declare_parameter("param_name", default_value);
    
    // Create publishers/subscribers
    publisher_ = this->create_publisher<MsgType>("topic", 10);
    subscription_ = this->create_subscription<MsgType>(
      "input_topic", 10,
      std::bind(&ExampleNode::callback, this, std::placeholders::_1));
    
    // Create timers
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ExampleNode::timer_callback, this));
  }

private:
  void callback(const MsgType::SharedPtr msg);
  void timer_callback();
  
  rclcpp::Publisher<MsgType>::SharedPtr publisher_;
  rclcpp::Subscription<MsgType>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

#### Launch Files
Use Python launch files (.launch.py) with proper parameterization:

```python
import os
from launch import LaunchDescription
from launch.actions import PushRosNamespace
from launch_ros.actions import Node

def generate_launch_description():
    # Get robot index from environment variable
    robot_index = os.environ.get('ROBOT_INDEX', '1')
    namespace = f'robot{robot_index}'
    
    return LaunchDescription([
        PushRosNamespace(namespace),
        
        Node(
            package='package_name',
            executable='node_name',
            parameters=[{
                'param_name': 'param_value'
            }]
        )
    ])
```

## GitHub Copilot Usage

This repository includes comprehensive GitHub Copilot instructions in [`.github/copilot-instructions.md`](copilot-instructions.md). Key points:

- **Project Context**: Copilot understands this is a ROS2-based autonomous vehicle simulation
- **Code Patterns**: Follow established patterns for nodes, launch files, and multi-robot systems
- **Package Structure**: Maintain consistent package organization
- **Multi-Robot Support**: Ensure code works with namespace-based robot separation

### Best Practices with Copilot

1. **Provide Context**: Include relevant comments about what you're trying to achieve
2. **Use Descriptive Names**: Clear function and variable names help Copilot understand intent
3. **Follow Patterns**: Copilot learns from existing code patterns in the repository
4. **Review Suggestions**: Always review and test Copilot-generated code

## Pull Request Process

### Before Submitting

1. **Test Your Changes**: Ensure your code works in simulation
2. **Build Successfully**: Verify all packages build without errors
3. **Follow Style Guidelines**: Use consistent code formatting
4. **Update Documentation**: Update relevant documentation

### PR Guidelines

1. **Clear Title**: Use descriptive titles that explain the change
2. **Detailed Description**: Explain what changes you made and why
3. **Link Issues**: Reference related issues using GitHub keywords
4. **Add Tests**: Include tests for new functionality
5. **Update Changelog**: Add entries to relevant package changelogs

### PR Template

```markdown
## Description
Brief description of changes made.

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Breaking change
- [ ] Documentation update

## Testing
- [ ] Built and tested in simulation
- [ ] Tested with multi-robot scenarios
- [ ] Added/updated tests

## Checklist
- [ ] Code follows style guidelines
- [ ] Self-review completed
- [ ] Documentation updated
- [ ] No new warnings introduced
```

## Testing

### Unit Tests
- Write tests for complex algorithms
- Use `gtest` for C++ and `pytest` for Python
- Test edge cases and error conditions

### Integration Tests
- Test multi-component interactions
- Verify multi-robot functionality
- Test in simulation environment

### Running Tests
```bash
# Run all tests
cd packages && colcon test

# Run tests for specific package
cd packages && colcon test --packages-select package_name

# View test results
cd packages && colcon test-result --verbose
```

## Documentation

### Code Documentation
- Use Doxygen-style comments for C++
- Use docstrings for Python functions
- Document complex algorithms and parameters

### Package Documentation
- Update package.xml with accurate descriptions
- Include README.md for complex packages
- Document configuration parameters

### Launch File Documentation
- Comment launch file parameters
- Provide usage examples
- Document multi-robot considerations

## Issue Reporting

When reporting issues:

1. **Use Templates**: Follow the provided issue templates
2. **Provide Context**: Include system information and steps to reproduce
3. **Minimal Example**: Provide minimal code to reproduce the issue
4. **Logs**: Include relevant ROS2 logs and error messages

## Security

- **Input Validation**: Always validate parameters and message inputs
- **Error Handling**: Implement robust error handling
- **Resource Management**: Properly manage memory and file handles
- **Network Security**: Consider security for distributed deployments

## Community Guidelines

- **Be Respectful**: Treat all contributors with respect
- **Be Collaborative**: Work together to improve the project
- **Be Patient**: Remember that everyone is learning
- **Be Constructive**: Provide helpful feedback and suggestions

## Questions?

- **GitHub Issues**: For bug reports and feature requests
- **GitHub Discussions**: For questions and general discussion
- **Email**: harun.teper@tu-dortmund.de for direct contact

Thank you for contributing to AuNa! 🚗🤖
