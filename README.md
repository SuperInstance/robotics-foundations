# robotics-foundations

**Robotics foundations in Rust. Kinematics to control, from scratch.**

## What's Inside

- **Spatial transforms** — rotation matrices (SO(3)), unit quaternions, homogeneous transforms (SE(3)), Euler angles, SLERP, axis-angle
- **Denavit-Hartenberg kinematics** — serial robot arms, forward kinematics, all intermediate joint transforms
- **Inverse kinematics** — damped least-squares solver (Levenberg-Marquardt style)
- **Jacobian analysis** — geometric 6×n Jacobian, Yoshikawa manipulability, singular values
- **Path planning** — A* on occupancy grids, RRT in continuous 2D, artificial potential fields
- **PID control** — with output clamping and anti-windup
- **Collision detection** — AABB-AABB, sphere-sphere, sphere-AABB
- **Sensor simulation** — odometry (differential drive), range finder with noise
- **Spatial agents** — autonomous agents with bounding volumes, path following, PID, odometry

## Install

```toml
[dependencies]
robotics-foundations = "0.1.0"
```

## Quick Start

### Forward Kinematics

```rust
use robotics_foundations::kinematics::{DHLink, SerialChain};
use robotics_foundations::transforms::HomogeneousTransform;

let links = vec![
    DHLink::revolute(0.0, 0.0, 1.0, 0.0),
    DHLink::revolute(0.0, 0.0, 1.0, 0.0),
];
let chain = SerialChain::new(links);
let ee = chain.fk(&[std::f64::consts::FRAC_PI_4, std::f64::consts::FRAC_PI_4]);
println!("End-effector: {:?}", ee.translation());
```

### A* Path Planning

```rust
use robotics_foundations::path_planning::astar::{astar_grid, GridCell, euclidean_distance};

let path = astar_grid(
    GridCell::new(0, 0),
    GridCell::new(10, 10),
    |cell| vec![(5, 3), (5, 4), (5, 5)].iter().any(|&(x, y)| cell.x == x && cell.y == y),
    euclidean_distance,
    true,
);
```

### PID Control

```rust
use robotics_foundations::control::PIDController;

let mut pid = PIDController::new(2.0, 0.5, 0.1).with_limits(-1.0, 1.0);
pid.set_setpoint(10.0);
let output = pid.update(0.0, 0.01);
```

## License

MIT OR Apache-2.0
