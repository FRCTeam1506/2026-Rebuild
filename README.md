# Team 1506 FRC 2026 Rebuild

Team 1506's FRC (FIRST Robotics Competition) robot code for the 2026 "Rebuild" season. Built on WPILib's Java command-based framework with physics-based desktop simulation via maple-sim.

## Quick Start

**See [`CLAUDE.md`](CLAUDE.md)** for full project overview, architecture, and working conventions.

### Running the code

```bash
./gradlew build              # Compile and run checks
./gradlew simulateJava       # Run physics-based desktop simulation
./gradlew deploy             # Deploy to the roboRIO
```

### Desktop Simulation

This repo includes **physics-based desktop simulation** (not kinematic). The simulated robot has mass, wheels can slip, and the field has walls that collide. Run `./gradlew simulateJava`, then launch AdvantageScope to see the 3D robot in action.

**See [`docs/simulation.md`](docs/simulation.md)** for detailed simulation setup, tuning parameters, and troubleshooting.

## Structure

- **`src/main/java/frc/robot/`** — robot code entry point and subsystems
- **`src/main/deploy/pathplanner/`** — autonomous paths and autos (edited via PathPlanner GUI)
- **`vendordeps/`** — third-party libraries (CTRE, WPILib, maple-sim, etc.)
- **`docs/`** — supplemental documentation (simulation, etc.)

For detailed architecture and component overview, see [`CLAUDE.md`](CLAUDE.md).
