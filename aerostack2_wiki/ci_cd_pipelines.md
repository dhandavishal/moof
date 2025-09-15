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

This document provides a comprehensive explanation of the Continuous Integration, Continuous Deployment (CI/CD), and testing infrastructure used in the Aerostack2 project. It covers GitHub Actions workflows, code coverage reporting, and best practices for contributing code that passes all automated tests.

## Overview of CI/CD in Aerostack2

Aerostack2 employs a robust CI/CD pipeline to ensure code quality and stability across the framework. The system runs automated builds and tests on multiple platforms whenever changes are proposed, helping maintain a high-quality codebase.

```
CI/CD Pipeline ComponentsDevelopment ProcessCode ChangesPull RequestGitHub Actions CIAutomated TestsCode CoverageCode ReviewMerge to Mainbuild-humble.yamlcodecov_test.yamlROS 2 TestsPlatform-specific TestsCodecov Report
```

Sources: [.github/workflows/build-humble.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/build-humble.yaml) [.github/workflows/codecov\_test.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/codecov_test.yaml) [README.md](https://github.com/aerostack2/aerostack2/blob/10200cd9/README.md)

## GitHub Actions Workflows

Aerostack2 uses GitHub Actions to automate building and testing. Two primary workflows ensure code quality:

### Build and Test Workflow

The `build-humble` workflow runs on pull requests that request review or are marked as ready for review, targeting the main branch. This workflow:

1.  Sets up a ROS 2 Humble environment
2.  Installs dependencies
3.  Builds all packages
4.  Runs tests with code coverage enabled
5.  Reports results to Codecov

```
build-humble.yamlMatrix Build PlatformsCrazyfliePR EventSetup ContainerInstall DependenciesSetup ROSBuild & Test PackagesUpload CoverageTelloDJI OSDKPixhawkDJI PSDK
```

Sources: [.github/workflows/build-humble.yaml3-76](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/build-humble.yaml#L3-L76)

The `build-humble` workflow tests all core packages including:

-   Core system components
-   Behavior system modules
-   Platform interfaces
-   User interfaces
-   Simulation assets

It also separately tests platform-specific implementations through a matrix job, building each platform in isolation with its dependencies.

### Code Coverage Workflow

The `codecov_test` workflow runs daily to track code coverage over time. This workflow:

1.  Builds the codebase with coverage instrumentation
2.  Runs all tests
3.  Generates coverage reports
4.  Uploads results to Codecov

```
codecov_test.yamlDaily Schedule7:07 UTCSetup EnvironmentBuild with CoverageRun TestsGenerate Coverage ReportUpload to Codecov
```

Sources: [.github/workflows/codecov\_test.yaml1-74](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/codecov_test.yaml#L1-L74)

## Testing Infrastructure

### ROS 2 Testing Framework

Aerostack2 leverages ROS 2's standard testing tools:

```
Coverage ToolsROS 2 Testing Frameworkcolcon testgtest (C++)pytest (Python)Test ResultsCoverage Reportslcovtotal_coverage.infocolcon-lcov-resultcolcon-coveragepy-resultCodecov Dashboard
```

Sources: [.github/workflows/build-humble.yaml57-66](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/build-humble.yaml#L57-L66) [.github/workflows/codecov\_test.yaml57-65](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/codecov_test.yaml#L57-L65)

The testing process uses the following tools:

-   `colcon test` to run all package tests
-   Coverage mixins for C++ and Python code
-   `lcov` for generating coverage reports
-   `colcon-lcov-result` for processing coverage data

### Code Coverage Configuration

Aerostack2 uses Codecov to track code coverage metrics. The configuration is defined in `codecov.yaml`:

```
fixes:
  - "ros_ws/src/*/aerostack2/::"
ignore:
  - "ros_ws/build"
  - "ros_ws/*/**/test/*"
  - "ros_ws/*/**/**/test/*"
```

This configuration:

-   Fixes path mappings for reporting
-   Ignores build directories and test files from coverage calculations

Sources: [codecov.yaml1-7](https://github.com/aerostack2/aerostack2/blob/10200cd9/codecov.yaml#L1-L7)

## Contribution Process and Quality Assurance

### Pull Request Workflow

```
Pull Request ProcessFork RepositoryCreate BranchMake ChangesRun Tests LocallySubmit PRCI Runs TestsCode ReviewAddress FeedbackMerge
```

Sources: [.github/PULL\_REQUEST\_TEMPLATE.md1-37](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/PULL_REQUEST_TEMPLATE.md#L1-L37)

When submitting a pull request, contributors must:

1.  Fill out the PR template with information about:
    
    -   Issues addressed
    -   ROS 2 version tested on
    -   Aerial platforms tested on
    -   Description of the contribution
    -   Required documentation updates
    -   Potential future work
2.  Ensure all CI tests pass before requesting review
    

### Issue Templates

Aerostack2 provides structured templates for bug reports and feature requests:

-   **Bug Report Template**: Collects information about the ROS2 version, Aerostack2 version, installation method, and steps to reproduce the issue.
-   **Feature Request Template**: Gathers details about the proposed feature and implementation considerations.

Sources: [.github/ISSUE\_TEMPLATE/BUG-REPORT.yml1-74](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/ISSUE_TEMPLATE/BUG-REPORT.yml#L1-L74) [.github/ISSUE\_TEMPLATE/FEATURE-REQUEST.yml1-20](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/ISSUE_TEMPLATE/FEATURE-REQUEST.yml#L1-L20)

## Continuous Integration Matrix

The CI system tests Aerostack2 across multiple dimensions:

| Component | Test Type | Platform | Tools Used |
| --- | --- | --- | --- |
| Core packages | Build & Unit tests | Ubuntu 22.04 + ROS 2 Humble | colcon, gtest, pytest |
| Core packages | Code coverage | Ubuntu 22.04 + ROS 2 Humble | lcov, Codecov |
| Platform: Crazyflie | Build & Integration | Ubuntu 22.04 + ROS 2 Humble | colcon |
| Platform: Tello | Build & Integration | Ubuntu 22.04 + ROS 2 Humble | colcon |
| Platform: DJI OSDK | Build & Integration | Ubuntu 22.04 + ROS 2 Humble | colcon |
| Platform: Pixhawk | Build & Integration | Ubuntu 22.04 + ROS 2 Humble | colcon |
| Platform: DJI PSDK | Build & Integration | Ubuntu 22.04 + ROS 2 Humble | colcon |

Sources: [.github/workflows/build-humble.yaml77-109](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/build-humble.yaml#L77-L109)

## Repository Structure to CI Mapping

The following diagram shows how different parts of the Aerostack2 repository relate to CI/CD configurations:

```
CI JobsRepository Structureaerostack2/Root RepositoryCore PackagesPlatform PackagesCI/CD Configurationas2_coreas2_behaviors_*as2_motion_*as2_msgsas2_platform_*.github/workflows/codecov.yamlbuild-and-test-humblebuild-platformscodecov_test
```

Sources: [.github/workflows/build-humble.yaml12-76](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/build-humble.yaml#L12-L76) [.github/workflows/codecov\_test.yaml9-74](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/codecov_test.yaml#L9-L74)

## Best Practices for Testing in Aerostack2

### Writing Testable Code

-   Keep classes and methods focused on a single responsibility
-   Use dependency injection to allow mocking of dependencies
-   Separate platform-specific code from core logic
-   Document test requirements and assumptions

### Running Tests Locally

To run the same tests locally that CI will run:

1.  Build with coverage enabled:
    
    ```
    colcon build --mixin coverage-gcc
    ```
    
2.  Run tests:
    
    ```
    colcon test --parallel-workers 1
    ```
    
3.  Generate coverage report:
    
    ```
    colcon lcov-result
    ```
    

Sources: [.github/workflows/build-humble.yaml57-65](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/build-humble.yaml#L57-L65)

## Status Badges

Aerostack2 uses several badges in its README to show the current status of the codebase:

-   Build Status: Shows whether the main branch is currently passing all tests
-   Code Coverage: Shows the percentage of code covered by tests
-   License: Indicates the project's license (BSD 3-Clause)
-   arXiv: Links to the academic paper describing Aerostack2

These badges provide at-a-glance information about the project's health and status.

Sources: [README.md1-11](https://github.com/aerostack2/aerostack2/blob/10200cd9/README.md#L1-L11)