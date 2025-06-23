# SpotStack

**SpotStack** is a modular Python library designed to simplify perception, control, mapping, and navigation tasks for the Boston Dynamics Spot robot. It provides reusable tools and abstractions for working with Spot’s image streams, general mobility, GraphNav maps, and arm control.

---

## 🚀 Features

SpotStack is divided into focused modules that reflect major capabilities of the Spot platform. Each module contains well-documented components that can be used individually or combined to support complex robotics pipelines in research and development.

---

### 🟢 Power
Utilities for managing the Spot robot’s power states, such as turning the robot on or off and checking power status.

---

### 🟣 Image
Efficient tools for retrieving and decoding camera streams from Spot.

- **`ImageFetcher`**: Unified interface via `get_images()` to retrieve:
  - All 5 raw camera images, or  
  - 4 images with the front view automatically stitched from the left and right fisheye cameras.
- **`StitchingCamera`**: OpenGL-based front image stitching that can be integrated into both online and offline processing pipelines.

Use the following code to inspect available image sources and their supported pixel formats:

```python
sources = self._image_client.list_image_sources()
for src in sources:
    print(src.name, src.pixel_formats)
```

More information on image source fields can be found in the [Boston Dynamics API reference](https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference.html#bosdyn-api-ImageSource).

---

### 🔵 Motion
Utilities for controlling Spot’s mobility using velocity or displacement commands. Designed to support real-time teleoperation and programmatic movement.

---

### 🟠 Graph
Lightweight tools for working with Spot's graph-based navigation system (GraphNav).

- **`GraphCore`**: Basic functionality for loading and reading GraphNav maps.
- **`GraphRecorder`**: Unified interface for logging waypoints and edges.
- **`GraphNavigator`**: Loads pre-recorded maps and executes onboard GraphNav-based navigation.

> **Note:** Graph Nav Anchoring Optimization is **not included** in this library.  
> For more information on anchoring optimization, refer to the official Boston Dynamics example:  
> https://dev.bostondynamics.com/python/examples/graph_nav_anchoring_optimization/readme

---

### 🟡 Arm
Simplified interfaces for Spot arm manipulation.

- **`ArmCore`**: A unified class that integrates arm control, gripper camera image capture, and parameter setting.

---

## 🧪 Example Usage

Each class file includes an example usage block at the bottom to help you quickly understand how to integrate it into your own workflow.

---

## 🤖 Built For

- Research in mobile manipulation  
- Visual navigation and mapping  
- End-to-end learning and perception pipelines

---

## 🛠️ Requirements

- Python 3.8+
- Boston Dynamics Spot SDK
- OpenGL (for image stitching)

---

## 📎 License

MIT License
