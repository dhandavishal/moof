Relevant source files

-   [.github/ISSUE\_TEMPLATE/BUG-REPORT.yml](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/ISSUE_TEMPLATE/BUG-REPORT.yml)
-   [.github/ISSUE\_TEMPLATE/FEATURE-REQUEST.yml](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/ISSUE_TEMPLATE/FEATURE-REQUEST.yml)
-   [.github/PULL\_REQUEST\_TEMPLATE.md](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/PULL_REQUEST_TEMPLATE.md)
-   [.github/PULL\_REQUEST\_TEMPLATE/PULL\_REQUEST\_TEMPLATE.yml](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/PULL_REQUEST_TEMPLATE/PULL_REQUEST_TEMPLATE.yml)
-   [.github/workflows/build-humble.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/build-humble.yaml)
-   [.github/workflows/codecov\_test.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/codecov_test.yaml)
-   [README.md](https://github.com/aerostack2/aerostack2/blob/10200cd9/README.md)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/README.md](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/README.md)
-   [as2\_motion\_controller/.gitignore](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_motion_controller/.gitignore)
-   [as2\_motion\_controller/README.md](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_motion_controller/README.md)
-   [codecov.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/codecov.yaml)

This page provides essential information for developers contributing to the Aerostack2 framework. It covers contribution workflows, CI/CD infrastructure, testing requirements, and development best practices. For installation instructions, see [Getting Started](https://deepwiki.com/aerostack2/aerostack2/1.2-getting-started), and for the architectural overview, see [Architecture](https://deepwiki.com/aerostack2/aerostack2/1.1-architecture).

## Development Workflow

The Aerostack2 project follows a standard GitHub-based development workflow with continuous integration to ensure code quality and maintainability.

```
Identify issue or featureCreate GitHub IssueFork repository or create branchImplement solutionRun tests locallyCreate Pull RequestCI Automated testingCode reviewAddress feedbackMerge to main
```

Sources: [.github/PULL\_REQUEST\_TEMPLATE.md1-37](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/PULL_REQUEST_TEMPLATE.md#L1-L37) [.github/ISSUE\_TEMPLATE/BUG-REPORT.yml1-74](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/ISSUE_TEMPLATE/BUG-REPORT.yml#L1-L74) [.github/ISSUE\_TEMPLATE/FEATURE-REQUEST.yml1-20](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/ISSUE_TEMPLATE/FEATURE-REQUEST.yml#L1-L20)

## Contributing to Aerostack2

### Issue Reporting

Before implementing a new feature or fixing a bug, create an issue to discuss it with the community. Aerostack2 provides templates for:

1.  **Bug Reports**: Include ROS2 version, Aerostack2 version, installation method, bug description, and reproduction steps.
2.  **Feature Requests**: Describe the feature and any implementation considerations.

### Pull Request Process

When submitting changes:

1.  Create a branch for your changes
    
2.  Implement and test your solution
    
3.  Fill out the pull request template with:
    
    -   Issue(s) addressed
    -   ROS2 version tested
    -   Aerial platform tested
    -   Description of the contribution
    -   Documentation updates needed
    -   Future work considerations
4.  Submit the PR for review
    

```
triggersPullRequest+Issue references+ROS2 version+Platform tested+Contribution description+Documentation updates+Future workCISystem+Build packages+Run tests+Code coverage+Report results
```

Sources: [.github/PULL\_REQUEST\_TEMPLATE.md1-37](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/PULL_REQUEST_TEMPLATE.md#L1-L37) [.github/PULL\_REQUEST\_TEMPLATE/PULL\_REQUEST\_TEMPLATE.yml1-56](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/PULL_REQUEST_TEMPLATE/PULL_REQUEST_TEMPLATE.yml#L1-L56)

## Continuous Integration and Testing

Aerostack2 employs GitHub Actions to automate building, testing, and code quality checks for all contributions.

### CI Workflow

```
Platform-Specific TestsPull Request created/updatedGitHub Actions workflow triggeredSetup ROS2 environmentInstall dependenciesBuild packagesRun unit testsGenerate coverage reportReport build statusBuild and test with platform matrix
```

The CI system performs the following:

1.  **Package Building**: All Aerostack2 packages are built to ensure compatibility.
2.  **Unit Testing**: Tests verify functionality across the codebase.
3.  **Code Coverage**: Reports are generated and uploaded to Codecov.
4.  **Platform Testing**: Separate jobs test compatibility with supported drone platforms.

### CI Configuration

The main CI workflow is defined in `.github/workflows/build-humble.yaml` and includes:

| Job | Purpose |
| --- | --- |
| build-and-test-humble | Builds all core packages and runs tests |
| build-platforms | Tests compatibility with various platform implementations (Crazyflie, Tello, DJI OSDK, Pixhawk, DJI PSDK) |

The daily `codecov_test` workflow generates comprehensive code coverage statistics.

Sources: [.github/workflows/build-humble.yaml1-109](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/build-humble.yaml#L1-L109) [.github/workflows/codecov\_test.yaml1-74](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/codecov_test.yaml#L1-L74) [codecov.yaml1-7](https://github.com/aerostack2/aerostack2/blob/10200cd9/codecov.yaml#L1-L7)

## Development Best Practices

### Package Structure

Aerostack2 follows a modular package structure. Each package should:

1.  Have a clear, single responsibility
2.  Include appropriate unit tests
3.  Provide comprehensive documentation
4.  Follow ROS2 package conventions

```
Package Structurepackage.xmlCMakeLists.txtinclude/package_name/src/test/README.mdconfig/launch/
```

### Code Style and Documentation

Aerostack2 maintains consistent code style across the project:

1.  **C++ Code**: Follow ROS2 C++ style guidelines
2.  **Python Code**: Follow PEP 8 style guide
3.  **Documentation**: All public APIs should be documented
4.  **Testing**: New features should include unit tests

### Testing Requirements

All contributions should include tests to verify functionality:

1.  **Unit Tests**: Test individual components
2.  **Integration Tests**: Test interactions between components
3.  **Coverage**: Aim for high test coverage of critical code paths
4.  **Simulation**: Test with appropriate simulators before real hardware

## Platform Support

When adding features that may impact platform compatibility, test with all supported platforms:

1.  **Simulation Platforms**:
    
    -   Gazebo
    -   Multirotor Simulator
2.  **Hardware Platforms**:
    
    -   Crazyflie
    -   Tello
    -   DJI OSDK
    -   Pixhawk
    -   DJI PSDK

```
Aerostack2 Platform Testing MatrixHardwareSimulationCrazyflieNew FeatureTest in SimulationTest on Specific HardwareCreate Pull RequestGazeboMultirotor SimulatorTelloDJI OSDKPixhawkDJI PSDK
```

Sources: [.github/workflows/build-humble.yaml77-108](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/build-humble.yaml#L77-L108) [.github/PULL\_REQUEST\_TEMPLATE.md7-12](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/PULL_REQUEST_TEMPLATE.md#L7-L12)

## Release Process

Aerostack2 follows semantic versioning, with releases managed through GitHub:

1.  **Major Releases**: Significant changes, possibly breaking backward compatibility
2.  **Minor Releases**: New features with backward compatibility
3.  **Patch Releases**: Bug fixes and minor improvements

Current ROS2 distribution support is focused on ROS2 Humble, with previous support for Galactic (versions below 1.0.9) now deprecated.

Sources: [README.md7-11](https://github.com/aerostack2/aerostack2/blob/10200cd9/README.md#L7-L11)

## Debugging and Troubleshooting

When debugging issues:

1.  Check the [Issues](https://github.com/aerostack2/aerostack2/blob/10200cd9/Issues) page for similar problems
2.  Run individual packages with increased verbosity
3.  Use ROS2 tools like `ros2 topic echo` and `ros2 node info`
4.  Check logs and test outputs from CI for failed builds

## Docker Development Environment

For consistent development environments, Aerostack2 provides Docker images:

1.  Images are available on [Dockerhub](https://hub.docker.com/u/aerostack2)
2.  Pre-configured with ROS2 Humble and all dependencies
3.  Useful for development and testing without modifying your local system

Sources: [README.md13-14](https://github.com/aerostack2/aerostack2/blob/10200cd9/README.md#L13-L14)