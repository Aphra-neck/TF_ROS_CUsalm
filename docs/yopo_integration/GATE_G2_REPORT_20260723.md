# Gate G2 D435 Depth and cuVSLAM Report

Date: 2026-07-23

Result: `PASSED`

This report approves only the D435 depth and cuVSLAM joint-runtime gate. It does
not authorize YOPO control output, PX4 OFFBOARD, arming, or flight.

## Configuration

- cuVSLAM repository revision: `2584606`
- RealSense owner: the single `/camera/camera` node
- Infrared stereo and depth profile: `640x360@90 Hz`
- Depth encoding: `16UC1`, millimetres
- Depth frame: `camera_depth_optical_frame`
- Emitter: disabled (`depth_module.emitter_enabled=0`)
- cuVSLAM input remains IR1/IR2; native depth is a planner input only

## Results

| Measurement | Result |
| --- | --- |
| Depth header rate | `89.860 Hz` |
| Depth maximum header gap | `11.130 ms` |
| Depth nonmonotonic stamps | `0` |
| Full-frame valid fraction, median | `0.879705` |
| Full-frame valid fraction, p05 | `0.873587` |
| Centre valid fraction, median | `0.869253` |
| Centre valid fraction, p05 | `0.854368` |
| Median valid depth | `2.113 m` |
| IR1/IR2 observed rate | approximately `89.4--89.9 Hz` |
| Stable cuVSLAM odometry rate | `89.864 Hz` |
| Stable cuVSLAM maximum interval | `22 ms` |
| cuVSLAM state | `vo_state=1` |
| GPU temperature maximum | approximately `54.6 C` |
| CPU temperature maximum | approximately `57.5 C` |
| RAM | approximately `3.46/15.66 GB`; swap `0` |
| Input power | approximately `7.0--7.9 W` |

The sample window showed no timestamp regression, stream loss, process exit,
CUDA OOM, or sustained thermal throttling. CPU frequencies varied with normal
DVFS while temperatures remained stable. The YOPO ROS 2 wrapper explicitly
converts `16UC1` millimetres to metres before inference.

## Evidence

Evidence directory:
`/workspaces/isaac_ros-dev/evidence/gate_g2_20260723`

| File | SHA-256 |
| --- | --- |
| `depth_quality.txt` | `3de2a756b3bd83db51f0b3e8407e0c430976e7686825ad95cec4039729c4535a` |
| `odometry_hz.txt` | `759b390615f39fadecbefcc79e11c8566c32f7a8a233c11d186abc746fbd9593` |
| `tegrastats.txt` | `e8c4dec0634d641fbaa57928e9562ec629eeb4730243382f32935d1561286f9b` |
| `visual_slam_status.txt` | `2d19ec1bc6193de5b2f03447816f5696028258237b20eb14ee9c120a27531908` |

All four hashes were verified with `sha256sum -c`.

## Decision

Gate G2 and `YP-110` pass. Emitter-off depth is sufficiently dense in the
representative test scene, so `YP-120` is closed as `N/A_DECISION`; no emitter
A/B is required for the first integration round. A materially different scene
or a later sustained depth-quality regression reopens that decision.
