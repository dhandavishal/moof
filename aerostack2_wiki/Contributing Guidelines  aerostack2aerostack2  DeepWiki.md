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

This document provides comprehensive guidelines for contributing to the Aerostack2 project. It covers the development workflow, code standards, testing requirements, and submission processes that all contributors should follow. For information about continuous integration and testing infrastructure, see [CI/CD and Testing](https://deepwiki.com/aerostack2/aerostack2/7.2-cicd-and-testing).

## 1\. Getting Started

Before contributing to Aerostack2, ensure you have the correct development environment set up.

### 1.1 Development Environment

```
Setup OptionsPrerequisitesROS 2 HumbleUbuntu 22.04Gitcolcon build toolsSource InstallationDocker Containerprerequisitessetup
```

**Required Environment**:

-   Ubuntu 22.04
-   ROS 2 Humble
-   Git
-   colcon build tools

**Optional Tools**:

-   Docker (a pre-built image is available at [Aerostack2 Dockerhub](https://hub.docker.com/u/aerostack2))

To set up your development environment, follow the installation instructions in the [Getting Started](https://deepwiki.com/aerostack2/aerostack2/1.2-getting-started) wiki page.

Sources: [README.md9-13](https://github.com/aerostack2/aerostack2/blob/10200cd9/README.md#L9-L13)

### 1.2 Repository Structure

Aerostack2 is organized as a collection of packages following the ROS 2 package structure. Before contributing, familiarize yourself with the overall architecture of the system as described in the [Architecture](https://deepwiki.com/aerostack2/aerostack2/1.1-architecture) page.

## 2\. Development Workflow

### 2.1 Branching Strategy

```
mainfeature-branchmainimplement featureadd testsupdate docsrelease
```

-   The `main` branch contains stable code
-   Create feature branches from `main` for your contributions
-   Branch naming convention: `feature/descriptive-name` or `fix/issue-description`

### 2.2 Development Process

```
Create/Select IssueFork RepositoryCreate Feature BranchImplement ChangesWrite TestsRun Local TestsUpdate DocumentationSubmit Pull RequestAddress Review FeedbackMerge to main
```

1.  **Issue Creation**: Start by creating or selecting an existing issue
2.  **Feature Development**: Implement your changes in a feature branch
3.  **Testing**: Write tests and ensure all tests pass locally
4.  **Documentation**: Update relevant documentation
5.  **Pull Request**: Submit your changes for review
6.  **Review Process**: Address feedback from maintainers
7.  **Merge**: Changes are merged into main after approval

Sources: [.github/PULL\_REQUEST\_TEMPLATE.md1-37](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/PULL_REQUEST_TEMPLATE.md#L1-L37)

## 3\. Code Standards

### 3.1 Code Style and Quality

Aerostack2 follows the standard ROS 2 coding guidelines:

-   C++ code should follow the [ROS 2 C++ Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
-   Python code should follow [PEP 8](https://www.python.org/dev/peps/pep-0008/)
-   Use consistent naming conventions across the codebase
-   Maintain a clean code structure with clear class and function responsibilities
-   Document your code with appropriate comments and documentation strings

### 3.2 Documentation Requirements

All contributions should include:

-   Code documentation (inline comments, function docstrings)
-   Updates to relevant README files
-   Updates to wiki documentation when adding new features
-   Examples of usage when applicable

Sources: [.github/PULL\_REQUEST\_TEMPLATE.md22-27](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/PULL_REQUEST_TEMPLATE.md#L22-L27)

## 4\. Testing Requirements

```
Testing ToolsTesting LevelsUnit TestsIntegration TestsPlatform-specific Testscolcon testCode CoverageCI Verification
```

### 4.1 Writing Tests

-   All new features should include appropriate unit tests
-   Tests should be written using the standard ROS 2 test framework
-   Ensure tests cover both normal operation and edge cases
-   Include integration tests for components that interact with other modules

### 4.2 Running Tests Locally

```
colcon test --packages-select <your_package>
colcon test-result --verbose
```

### 4.3 Code Coverage

Code coverage is monitored through Codecov:

-   Aim for high test coverage for new code
-   Address any coverage issues identified in PR reviews

Sources: [.github/workflows/codecov\_test.yaml1-74](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/codecov_test.yaml#L1-L74) [codecov.yaml1-7](https://github.com/aerostack2/aerostack2/blob/10200cd9/codecov.yaml#L1-L7)

## 5\. Submitting Contributions

### 5.1 Pull Request Process

```
Mark PR readyReviewer assignedReview completedChanges madeAll reviews passCI passesDraftReadyForReviewInReviewNeedsChangesApprovedMerged
```

1.  **Create a Pull Request** using the template provided
    
2.  **Fill in the Template** with:
    
    -   Basic information (issue addressed, ROS version, platform tested)
    -   Description of your contribution in bullet points
    -   Description of required documentation updates
    -   Notes on potential future work
3.  **Address Review Feedback** promptly and thoroughly
    
4.  **Ensure CI Passes** before merging
    

### 5.2 Pull Request Template

The pull request template requires you to provide:

-   Issue(s) the PR addresses
-   ROS 2 version tested on
-   Aerial platform tested on
-   Description of contributions
-   Description of documentation updates
-   Notes on potential future work

Sources: [.github/PULL\_REQUEST\_TEMPLATE.md1-37](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/PULL_REQUEST_TEMPLATE.md#L1-L37) [.github/PULL\_REQUEST\_TEMPLATE/PULL\_REQUEST\_TEMPLATE.yml1-56](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/PULL_REQUEST_TEMPLATE/PULL_REQUEST_TEMPLATE.yml#L1-L56) [.github/workflows/build-humble.yaml1-109](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/build-humble.yaml#L1-L109)

## 6\. Issue Reporting

### 6.1 Bug Reports

When reporting bugs, use the bug report template and include:

-   ROS 2 version
-   Aerostack2 version
-   Installation method
-   Detailed description of the bug
-   Steps to reproduce
-   Relevant log output

### 6.2 Feature Requests

For feature requests, use the feature request template and include:

-   Clear description of the proposed feature
-   Rationale for adding the feature
-   Implementation considerations

Sources: [.github/ISSUE\_TEMPLATE/BUG-REPORT.yml1-74](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/ISSUE_TEMPLATE/BUG-REPORT.yml#L1-L74) [.github/ISSUE\_TEMPLATE/FEATURE-REQUEST.yml1-20](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/ISSUE_TEMPLATE/FEATURE-REQUEST.yml#L1-L20)

## 7\. Continuous Integration

```
CI ProcessPull RequestBuild PackagesRun TestsGenerate Code CoverageTest Supported PlatformsSuccessful CI RunFailed CI Run
```

### 7.1 CI Workflow

The CI workflow is triggered on pull requests:

1.  Builds all Aerostack2 packages
2.  Runs tests for each package
3.  Generates code coverage reports
4.  Tests compatibility with different platforms

### 7.2 Platform Testing

The CI workflow tests compatibility with multiple platforms:

-   Crazyflie
-   Tello
-   DJI OSDK
-   Pixhawk
-   DJI PSDK

Contributors should ensure their changes work with relevant platforms.

Sources: [.github/workflows/build-humble.yaml1-109](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/build-humble.yaml#L1-L109) [.github/workflows/codecov\_test.yaml1-74](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/codecov_test.yaml#L1-L74)

## 8\. Supported Platforms and Configurations

```
Aerial PlatformsROS 2 DistributionsROS 2 Humble (Ubuntu 22.04)Gazebo SimulationCrazyflieTelloPX4/PixhawkDJI OSDKDJI PSDK
```

Aerostack2 officially supports:

-   **ROS 2 Distribution**: Humble (on Ubuntu 22.04)
-   **Aerial Platforms**:
    -   Gazebo simulation
    -   Crazyflie
    -   Tello
    -   PX4/Pixhawk
    -   DJI OSDK
    -   DJI PSDK

When contributing, ensure your changes are compatible with the relevant platforms.

Sources: [README.md7-13](https://github.com/aerostack2/aerostack2/blob/10200cd9/README.md#L7-L13) [.github/workflows/build-humble.yaml77-109](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/build-humble.yaml#L77-L109)

-   **Be Respectful**: Treat all contributors with respect and consideration
-   **Be Constructive**: Provide constructive feedback on pull requests and issues
-   **Be Responsive**: Address feedback and questions promptly
-   **Be Inclusive**: Welcome contributions from contributors of all experience levels

By following these guidelines, you'll help maintain a high-quality codebase and a positive community around the Aerostack2 project.