# Thor Robot - Comprehensive Knowledge Base

## 1. Project Overview

**Thor** is an open source, 3D printed, 6 degrees of freedom robotic arm designed by **Ángel L.M. (AngelLM)**. The project was started in 2015 as a final degree project titled "Design and implementation of an Open Source, 3D printed 6DOF robotic arm" and has continued to develop since then.

### Key Characteristics
- **Purpose**: Affordable and accessible robotic arm ideal for educational settings, makers, and robotics enthusiasts
- **Design Philosophy**: Built entirely using open source tools and designed to be replicable by anyone
- **Target Applications**: Education (robotics courses in schools and universities), hobby projects, research

### Open Source Tools Used
- **FreeCAD**: 3D modeling
- **KiCAD**: PCB design
- **GRBL/RRF**: Firmware
- **ROS2**: Robot Operating System integration

---

## 2. Technical Specifications

### Physical Specifications
- **Degrees of Freedom**: 6 DOF
- **Configuration**: yaw-roll-roll-yaw-roll-yaw (same configuration used by most commercial manipulator robots)
- **Height (Stretched)**: 625mm (without end effector)
- **Payload Capacity**: 750g maximum (including end effector weight)
- **Workspace/Reach**: ~625mm radius

### Accuracy & Repeatability
- **Joint 1-2 (Base)**: < 1° accuracy
- **Joint 3**: ~1° accuracy
- **Joint 4**: ~2° error
- **Joint 5-6 (Wrist)**: 3-4° error
- **Repeatability**: Approximately 0.2mm (community reported, unofficial)

### Build Specifications
- **Total Cost**: ~350€ for all components
- **3D Printing Time**: ~200 hours
- **Assembly Time**: ~6 hours
- **Number of 3D Printed Parts**: 37 different pieces

---

## 3. Hardware Components

### Stepper Motors

**Motor Configuration**:
- **Joints 1-3**: Larger stepper motors with mechanical reduction (gearboxes) for higher torque
- **Joints 4-6**: Smaller stepper motors to reduce overall weight

**Typical Motor Specifications** (from Thor+ builds):
- Base motor: 17HS16-2004S1 (2A) @ 24V
- Geared motors (Joints 2-3): Three 17HS13-0404S-PG5 (400mA) @ 24V with 5:18 to 1 planetary gearboxes
- Wrist motors (Joints 4-6): Three 17HS13-0404S (400mA) @ 24V
- Recommended: NEMA 17 34mm stepper motors with 5:18 planetary gearboxes where needed

### Servo Motors
- **Gripper**: HS-311 servo motor for end effector control
- **Control**: PWM output from ControlPCB STOOL pin

### Sensors
- **Homing System**: Optoisolators and micro-endstops for establishing home position
- **Endstop Configuration**: Up to 8 endstops supported (1 per motor), first five articulations use optoisolators
- **Advantage**: Optoisolators allow >360° rotation without sensor collision

---

## 4. Electronics & Control Board

Thor supports **two main electronics options**: the Thor ControlPCB with Arduino Mega, or the Fly Super8Pro board. Users can choose based on their needs and preferences.

### Option 1: Thor ControlPCB + Arduino Mega

**Main Controller**:
- **Microcontroller**: Arduino Mega 2560
- **Communication**: Serial @ 115200 baud rate
- **Programming Interface**: Arduino IDE
- **Firmware**: GRBL (modified for 8-axis control)

**Thor ControlPCB Specifications**:

**Design Heritage**: Based on RAMPS 1.4 architecture

**Capabilities**:
- Supports up to **8 stepper drivers** (A4988 and compatible)
- Up to **8 endstop inputs** (1 per motor)
- **1 PWM output** for tool control (servo/spindle)
- **Power Rails**: 5V and 12V auxiliary power

**Stepper Drivers**:
- **Type**: Pololu A4988 or compatible
- **Microstepping**: Default 1/16 microstepping (configurable)
- **Configuration**: Set via jumpers on PCB

**Design Files**:
- Designed in KiCAD
- Schematics, source files, and Gerber files available in GitHub repository
- PCB acts as Arduino Mega shield for easy integration
- **Open Source**: Full hardware design files available

**Advantages**:
- Fully open source hardware
- Can be manufactured by anyone
- Lower cost (components + PCB manufacturing)
- Community can modify and improve design
- Future-proof (won't be discontinued)

**Disadvantages**:
- Requires PCB ordering and assembly
- Limited to 8-bit Arduino Mega processor
- GRBL firmware limitations

### Option 2: Fly Super8Pro Board

**Manufacturer**: Mellow 3D

**Processor Options**:
- **STM32H743**: ARM Cortex-M7 @ 550MHz (32-bit)
- **STM32H723**: ARM Cortex-M7 @ 550MHz (32-bit)

**Firmware Support**: RepRapFirmware (RRF), Marlin 2.0, Klipper

**Capabilities**:
- Supports up to **8 stepper motors** simultaneously
- Up to **8 endstop inputs** for motor homing
- Digital outputs/inputs for endstops and servo motor
- Advanced 32-bit processor capabilities
- WiFi connectivity options available
- Web interface support (with RRF)

**Power Specifications**:
- **4 separate power inputs**:
  - Input 1: Powers drivers 0-2 (up to 62V)
  - Input 2: Powers drivers 3-7 (up to 62V)
  - Input 3: Powers boards, heaters, fans (up to 30V)
  - Input 4: Powers heated bed (up to 30V)

**Stepper Drivers**:
- Compatible with various stepper drivers
- Fly-2209 drivers recommended (have diag pin disable switch)
- Supports advanced features like sensorless homing
- Configurable microstepping

**Important Notes**:
- **Diag Pin Issue**: If using endstops (not sensorless homing) with non-Fly-2209 drivers, the diag pin must be bent or removed
- Fuses not pre-installed (supplied with board, must be installed)
- Jumper configuration required for standalone drivers

**Advantages**:
- Ready-to-use board (no PCB manufacturing needed)
- Powerful 32-bit processor
- RepRapFirmware support (advanced features)
- Web interface capability
- WiFi connectivity options
- Better performance for complex calculations
- Cost: ~60€

**Disadvantages**:
- **Not open source** - if manufacturer discontinues, may become unavailable
- Slightly higher cost than DIY ControlPCB
- More complex initial configuration
- Requires understanding of RRF configuration

### Electronics Comparison

| Feature | Thor ControlPCB + Arduino Mega | Fly Super8Pro |
|---------|-------------------------------|---------------|
| **Processor** | 8-bit ATmega2560 @ 16MHz | 32-bit ARM Cortex-M7 @ 550MHz |
| **Firmware** | GRBL (modified) | RRF, Marlin, Klipper |
| **Open Source** | ✓ Yes (fully) | ✗ No (proprietary) |
| **Ready to Use** | ✗ (requires manufacturing) | ✓ Yes |
| **Cost** | ~30-40€ (PCB + components) | ~60€ |
| **Web Interface** | ✗ No | ✓ Yes (with RRF) |
| **WiFi** | ✗ No | ✓ Optional |
| **Motor Capacity** | 8 steppers | 8 steppers |
| **Power Flexibility** | Standard | Multiple isolated inputs |
| **Future Availability** | ✓ Guaranteed (open source) | ? Depends on manufacturer |

### Power Supply Requirements

**Voltage Rails**:
- **24V DC**: Main power for stepper motors
- **12V DC**: Auxiliary power for fans and accessories
- **5V DC**: Logic power for Arduino and control electronics (via DC-DC step-down converter from 24V)

**Power Distribution**:
- Ground/24V/5V distribution across ControlPCB
- DC-to-DC step-down converter: 24V → 5V
- Separate power supply or unified PSU with multiple outputs

---

## 5. Firmware

Thor supports **two firmware options** depending on the electronics board chosen:

### Option 1: GRBL (for Thor ControlPCB + Arduino Mega)

**Firmware Base**: Modified version of GRBL (originally designed for CNC machines)
- **Version**: GRBL 1.0+ (customized for robotic arm control)
- **Platform**: Arduino Mega (grbl-Mega variant)
- **Control Method**: G-code commands (same language as 3D printers and CNC machines)
- **Key Modification**: Extended from 6 to 8 stepper motor support (GRBL 0.9 natively supports up to 6)

**GRBL Characteristics**:
- Originally designed for 8-bit processors
- Lightweight and efficient for Arduino Mega
- Proven stability for CNC-like motion control
- Simple serial communication
- Real-time buffered motion execution

**Installation Procedure**:

1. Download latest firmware from Downloads page (thor.angel-lm.com)
2. Navigate to `grbl-1.0/grbl/examples/grblUpload/`
3. Open `grblUpload.ino` in Arduino IDE
4. Select **Tools > Board > Arduino Mega**
5. Upload firmware to Arduino Mega

**Configuration**:

**Access Configuration**:
- Via serial console at **115200 baud rate**
- Use `$$` command to view all settings
- Modify settings with `$<setting>=<value>` format

**Key Configuration Settings**:
- Steps per degree for each motor
- Maximum feed rates
- Acceleration limits
- Microstepping configuration (default 1/16)
- Endstop behavior
- Homing speeds and directions

**Note**: If using non-standard components (different microstepping, motors with different steps/rev, or different gearbox ratios), the settings must be adjusted accordingly.

### Option 2: RepRapFirmware (RRF) (for Fly Super8Pro)

**Firmware Base**: RepRapFirmware - advanced 32-bit 3D printer/CNC firmware
- **Version**: RRF 3.5.0+ (for STM32 processors)
- **Platform**: Fly Super8Pro (STM32H723 or STM32H743)
- **Control Method**: G-code commands (with extended RRF-specific commands)
- **Architecture**: Modern 32-bit firmware optimized for ARM processors

**RRF Characteristics**:
- **Designed for 32-bit processors** - makes full use of modern ARM capabilities
- Advanced motion planning and look-ahead algorithms
- Web interface for configuration and control (Duet Web Control)
- WiFi connectivity support
- Object model for comprehensive system state access
- CRC error checking for reliable communication
- Quoted strings support in G-code
- Internal command queue with priority management
- More sophisticated acceleration and jerk control

**Installation Procedure**:

1. Download latest RRF firmware for Fly Super8Pro from thor.angel-lm.com or TeamGloomy/Mellow repositories
2. Follow Fly Super8Pro-specific flashing procedure (varies by bootloader method)
3. Upload firmware binary to board
4. Configure WiFi (if using wireless connectivity)
5. Access web interface for further configuration

**Configuration**:

**Configuration Files** (stored on SD card or internal storage):
- **config.g**: Main configuration file (runs at startup)
- **homeall.g**: Homing sequence for all axes
- **homex.g, homey.g**, etc.: Individual axis homing files
- **sys/** folder: System macro files

**Access Configuration**:
- Via **Duet Web Control** (web browser interface)
- Via serial/USB console
- Via Pronterface or other G-code senders
- Direct editing of config files on SD card

**Key Configuration Settings** (in config.g):
- Motor direction and steps per degree (`M569`, `M92`)
- Maximum speeds and accelerations (`M203`, `M201`)
- Motor currents (`M906`)
- Endstop configuration (`M574`)
- Network settings (`M552`, `M587`)
- Kinematics mode (usually Cartesian adapted for robot arm)

**Advantages of RRF vs. GRBL**:
- More powerful processor enables complex calculations
- Web interface for easy control and monitoring
- WiFi connectivity (no USB cable needed)
- Better G-code compatibility with modern 3D printing ecosystem
- Advanced features: restore points, tool changes, conditional G-code
- Real-time configuration changes without recompilation
- Detailed status reporting via object model

**G-Code Compatibility**:
- **Similarity**: Most basic G-codes (G0, G1, M3, M114, etc.) work similarly in both GRBL and RRF
- **Differences**: RRF has extended G-codes and different configuration commands
- **Reference**: Duet3D documentation (https://docs.duet3d.com) for RRF-specific commands
- **Migration**: Users switching from GRBL to RRF need to adapt configuration and some command sequences

**Important Notes**:
- If using non-standard components, the `config.g` file must be adjusted
- RRF configuration is more complex but also more flexible than GRBL
- Community support available through TeamGloomy (RRF on STM32) and Mellow forums

### Firmware Comparison

| Feature | GRBL (Arduino Mega) | RepRapFirmware (Fly Super8Pro) |
|---------|---------------------|--------------------------------|
| **Processor Target** | 8-bit (AVR) | 32-bit (ARM Cortex-M7) |
| **Web Interface** | ✗ No | ✓ Yes (Duet Web Control) |
| **WiFi Support** | ✗ No | ✓ Yes |
| **Configuration Method** | Serial commands | Config files + web interface |
| **Motion Planning** | Basic | Advanced (look-ahead) |
| **Processing Power** | Limited | High |
| **Ease of Setup** | Simpler | More complex |
| **Community Size** | Large (CNC/GRBL) | Large (3D printing/Duet) |
| **Real-time Tuning** | Limited | Extensive |
| **Documentation** | GRBL wiki | Duet3D docs |

### G-Code Commands

#### Movement Commands
- **G0**: Rapid positioning (e.g., `G0 A10 B45 C45 D-45 X15 Y5 Z-5`)
  - Moves articulations to specified angles in degrees
  - A, B, C, D, X, Y, Z represent the 6 joints (non-standard axis naming for robotic arms)

- **G1**: Controlled movement with feed rate (e.g., `G1 A20 X-20 F1000`)
  - F parameter: feed rate in degrees/minute
  - Example: F1000 = 1000°/min movement speed

#### Tool Control Commands
- **M3 S900**: Control servo position (e.g., open gripper)
  - S parameter: PWM value (typically 900-2100 for standard servos)

#### System Commands
- **M115**: Display firmware version information
- **M114**: Display current motor positions
- **$$**: View all GRBL settings
- **$H**: Execute homing cycle

#### Using Universal G-Code Sender
- Type G-code commands in dialog box
- Press **SEND** button to execute
- Monitor robot state (Disconnected, Idle, Run, Alarm)

---

## 6. Kinematics

### Current Implementation

**Forward Kinematics**: Currently implemented
- Calculations performed by GRBL firmware
- User specifies joint angles → firmware calculates end effector position
- Control system uses open kinetic chain starting from home position

**Inverse Kinematics**: Solutions exist but not implemented in standard firmware
- Available in documentation and community projects
- Not currently integrated into GRBL firmware control

### Inverse Kinematics Solution

**Method**: Kinematic decoupling procedure
1. Calculate joint parameters (q1, q2, q3) to place robot tip at desired point in space
2. Calculate joint parameters (q4, q5, q6) for wrist position and orientation

**Current Limitations**:
- No trajectory optimization
- No path planning algorithms
- No collision detection
- Control is point-to-point without interpolated paths

### DH Parameters

Denavit-Hartenberg parameters are documented in:
- GitHub repository technical documentation
- Hackaday.io project logs (Inverse Kinematics section)
- Community-developed kinematic solutions

### Future Development
- Full inverse kinematics integration into firmware
- Trajectory planning and optimization
- Cinematic motion algorithms (Issue #64 on GitHub)
- Movement smoothing and acceleration profiling

---

## 7. Software & Control

### Asgard GUI

**Description**: Graphical User Interface for Thor robotic arm control

**Technology Stack**:
- **Language**: Python
- **GUI Framework**: PyQt5
- **Additional Libraries**: pyserial, numpy, pyqtgraph, pyopengl

**Installation**:
```bash
python -m pip install pyqt5 pyserial numpy pyqtgraph pyopengl
```

**Key Features**:

1. **Serial Communication**
   - Port selection combo box
   - Port list refresh button
   - Baud rate setting (default: 115200)
   - Connection state display (Disconnected, Idle, Run, Alarm)

2. **Robot Control**
   - Home button (executes `$H` command)
   - Articulation angle displays for all 6 joints
   - Manual joint control
   - G-code command input

3. **Kinematics (Version-dependent)**
   - Forward Kinematics implementation (v1.x)
   - Inverse Kinematics (planned for future versions)
   - 3D visualization of robot pose (if implemented)

**Repository**: Available on GitHub (AngelLM/Thor)
**License**: Open source (same as main Thor project)

### ROS2 Integration

**Repository**: AngelLM/Thor-ROS

**Implementation Details**:
- **ROS Distribution**: ROS2 Humble LTS (Long Term Support)
- **Deployment Method**: Dockerized for cross-platform compatibility
- **Advantage**: Runs on any OS (Windows, Linux, macOS) via Docker

**Docker Implementation Features**:
- Abstraction layer for maximum flexibility
- GUI forwarding to local system ports
- Support for RViz and Gazebo visualization
- MoveIt integration (motion planning framework)

**Use Cases**:
- Advanced motion planning
- Integration with computer vision systems
- Multi-robot coordination
- Research and development

**Future Development**:
- Enhanced MoveIt configuration
- Pre-configured manipulation tasks
- ROS2 Control integration
- Simulation environments

---

## 8. 3D Printing & Assembly

### 3D Model Files

**Format Options**:
- **STL files**: Ready-to-print files (37 different pieces)
- **FreeCAD source files**: Editable 3D models for customization
- **Location**: GitHub repository `stl/` and `freecad-src/` directories

**Design Tool**: FreeCAD (open source parametric 3D modeler)

### Printing Recommendations
- Standard FDM 3D printer compatible
- Some parts optimized for smaller print beds (Thor+ modifications)
- Support material may be required for some parts
- Total print time: ~200 hours

### Assembly Resources

**Documentation Types**:
1. **Assembly Videos**: Step-by-step video guides
2. **Interactive Instructions**: Web-based interactive manual
3. **FreeCAD Animation Files**: Exploded assembly views using FreeCAD's Exploded Assembly feature
4. **Written Documentation**: Available at thor.angel-lm.com/documentation/get-started/

**Assembly Process**:
- Estimated assembly time: ~6 hours
- Requires basic tools (screwdrivers, Allen keys, etc.)
- Electronic assembly knowledge helpful but not required
- Clear instructions for cable management

### Bill of Materials (BOM)

**Location**: thor.angel-lm.com/documentation/bom/

**Categories**:
- 3D printed parts (37 pieces)
- Stepper motors (6-7 motors depending on configuration)
- Electronic components (Arduino Mega, ControlPCB, stepper drivers)
- Hardware (bearings, screws, nuts, bolts)
- Power supply components
- Cables and connectors
- Optional: end effector components

**Total Cost**: Approximately 350€ for complete build

**Component Sources**: Listed in BOM with links to common suppliers

---

## 9. End Effectors & Tooling

### Tool Mounting Interface

**Design Philosophy**: Adaptable interface rather than integrated gripper

**Rationale**:
- Many commercial and DIY grippers, vacuum systems, and hooks available
- Flexibility for different applications
- Easy tool changes

### Mounting Options

**Three Attachment Methods**:

1. **Modify the Interface**
   - Add custom mounting holes to standard interface plate (Art56GearPlate)
   - Direct mounting of specialized tools

2. **Modify the Tool**
   - Adapt existing tools to match interface mounting holes
   - Maintain standard interface geometry

3. **Intermediate Adapter**
   - Design adapter piece fitting interface on one side, tool on other
   - Allows use of off-the-shelf end effectors

### Standard Gripper

**Included Design**: 3D-printable gripper
- **Actuator**: HS-311 servo motor
- **Control**: M3 S900 G-code command (PWM control)
- **Mounting**: Direct attachment to Art56GearPlate

### Custom Tool Design

**For 3D-Printed Tools**:
- Design with Art56GearPlate mounting points in mind
- Seamless integration with robot aesthetic
- Tools appear as part of robot rather than add-on

**Community Resources**:
- GitHub Wiki: Grippers section (github.com/AngelLM/Thor/wiki/Grippers)
- Community-shared designs
- Third-party tool adapters

### End Effector Payload
- Maximum payload: 750g (including end effector weight)
- Consider tool weight when designing custom end effectors
- Lighter tools improve accuracy of distal joints

---

## 10. Calibration & Homing

### Homing System

**Purpose**: Establish zero reference position for open kinematic chain control

**Hardware**:
- **Joints 1-5**: Optoisolators for non-contact sensing
- **Micro-endstops**: Backup/alternative sensing method
- **Advantage of Optoisolators**: Allow >360° rotation without physical collision with sensor

**Homing Procedure**:
1. Send `$H` command via serial or Asgard GUI "Home" button
2. Each motor moves to find its home sensor
3. Position is set to zero reference
4. All subsequent movements calculated from this reference

**ControlPCB Support**:
- Up to 8 endstop inputs (1 per motor)
- Configurable trigger logic (normally open/normally closed)
- Integration with GRBL homing cycle

### Control Method

**Open Kinetic Chain**:
- Zero (Home) position set at startup
- Subsequent movements relative to home reference
- Motor position tracked by step counting
- No closed-loop feedback in standard configuration

**Implications**:
- Accuracy depends on proper homing
- Step skipping can cause position drift
- Regular re-homing recommended for precision tasks

### Calibration

**Current State**:
- Manual calibration possible but not simple
- Requires reference measurement system
- Not implemented in standard software

**Accuracy Improvement Potential**:
- Proper calibration could improve stated accuracy values
- Would require:
  - Vision system or coordinate measuring machine
  - Calibration software implementation
  - Joint-by-joint error mapping

**Community Projects** (Issue #32):
- Feedback sensors exploration
- Encoder integration possibilities
- Closed-loop control investigations

### Configuration for Non-Standard Components

**If using different components**:
- Adjust microstepping settings (if not 1/16)
- Modify steps/degree calculations for different motors
- Update gearbox ratios in `config.g` file
- Recalibrate movement limits and feed rates

**Firmware Parameters to Adjust**:
- Steps per degree for each axis
- Maximum velocity
- Acceleration limits
- Homing speeds and pulloff distances

---

## 11. Project History & Versions

### Timeline

**2015**: Project inception
- Started as final degree project
- Initial design and prototype development
- Title: "Design and implementation of an Open Source, 3D printed 6DOF robotic arm"

**2016**: Public release
- First version released to community
- Thingiverse publication
- Initial documentation

**2019**: Three-year milestone
- Active community development
- Multiple builder variations
- Continuous improvements

**Ongoing Development**:
- GitHub repository actively maintained
- Community contributions
- ROS2 integration development
- Documentation expansion

### Thor+ Improvements

**Developer**: DannyvandenHeuvel (community member)

**Key Enhancements**:

**V2.02**:
- Added feedback sensors for position verification
- Initial exploration of closed-loop control

**V2.04**:
- Feedback stability improvements
- Additional cooling fans for extended operation
- Design modifications for reliability
- Print bed size optimization

**Design Modifications**:
- Parts redesigned for smaller 3D printers
- Improved cable management
- Enhanced structural rigidity
- Better motor mounting solutions

### Notable Issues & Development Discussions

**GitHub Issues**:
- Issue #30: Inverse Kinematics implementation discussion
- Issue #32: Feedback sensors exploration
- Issue #64: Cinematic motion algorithm development

**Ongoing Development Areas**:
- Trajectory planning optimization
- Inverse kinematics firmware integration
- Advanced motion control algorithms
- Simulation environment improvements

---

## 12. Community & Resources

### Official Resources

**Main Website**: http://thor.angel-lm.com
- Primary documentation hub
- Downloads section (3D files, PCB files, firmware, software)
- Community forum
- FAQ section

**Documentation**: http://thor.angel-lm.com/documentation/
- Getting started guide
- Assembly instructions
- Electronics setup
- Firmware installation
- Control software guides
- Bill of Materials

### GitHub Repositories

**Main Repository**: https://github.com/AngelLM/Thor
- 3D model source files (FreeCAD and STL)
- Firmware (GRBL)
- Asgard GUI software
- Documentation
- Issue tracking
- Wiki with additional resources

**Related Repositories**:
- **Thor-ROS**: https://github.com/AngelLM/Thor-ROS
  - ROS2 integration
  - Dockerized implementation
  - MoveIt configuration

- **ThorControlPCB**: https://github.com/AngelLM/ThorControlPCB
  - PCB schematics (KiCAD)
  - Gerber manufacturing files
  - PCB documentation

### Third-Party Hardware Resources

**Fly Super8Pro Board** (Mellow 3D):
- **Manufacturer Site**: https://mellow-3d.github.io/
- **Fly Super8Pro H723 Docs**: https://mellow-3d.github.io/fly_super8_pro_h723_general.html
- **GitHub**: https://github.com/Mellow-3D/Fly-Super8Pro

**RepRapFirmware for STM32** (TeamGloomy):
- **TeamGloomy Docs**: https://teamgloomy.github.io/
- **Fly Super8Pro RRF Guide**: https://teamgloomy.github.io/fly_super8pro_h743_general_3_5.html
- **RRF Configuration Tool**: https://configtool.reprapfirmware.org/

**RepRapFirmware** (Duet3D):
- **Duet3D Documentation**: https://docs.duet3d.com
- **G-code Reference**: https://docs.duet3d.com/User_manual/Reference/Gcodes
- **RepRapFirmware Forum**: https://forum.duet3d.com

### Community Platforms

**Hackaday.io**: https://hackaday.io/project/12989-thor
- Project logs and updates
- Build logs from community members
- Technical discussions
- Inverse kinematics documentation

**Thingiverse**: https://www.thingiverse.com/thing:1743075
- Alternative STL file source
- Community remixes
- Build examples and photos

**Google Groups** (Historical): thor-opensource-3d-printable-robotic-arm
- Original community forum (now archived)
- Historical discussions and solutions
- Reference material

### Getting Help

**Recommended Support Channels**:
1. **Current Forum**: thor.angel-lm.com forums
   - "Get Help" section for troubleshooting
   - Active community support
   - Creator participation

2. **GitHub Issues**: For bug reports and feature requests
   - Check existing issues first
   - Detailed problem descriptions appreciated

3. **Hackaday.io**: For build logs and project discussions

### Community Contributions

**List of Builders**: Available on GitHub Wiki
- Community build variations
- Modification examples
- Success stories

**Common Modifications**:
- Different motor configurations
- Alternative control boards (Fly Super8Pro mentioned)
- Custom end effectors
- Enhanced cooling solutions
- Alternative firmware (RRF experimentation)

---

## 13. License & Legal

### Project License

**License Type**: Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0)

**Rights Granted**:
- **Share**: Copy and redistribute material in any medium or format
- **Adapt**: Remix, transform, and build upon the material for any purpose

**Conditions**:
- **Attribution**: Must give appropriate credit to Ángel L.M. and provide link to license
- **ShareAlike**: If you remix, transform, or build upon material, must distribute contributions under same license
- **No Additional Restrictions**: Cannot apply legal terms or technological measures that legally restrict others from doing anything the license permits

### What This Means

**You Can**:
- Build your own Thor robot
- Modify the design
- Use for commercial purposes
- Share your modifications
- Create derivative works

**You Must**:
- Credit the original creator (AngelLM)
- Share modifications under same CC BY-SA 4.0 license
- Provide link to license
- Indicate if changes were made

### Software Components

**Firmware (GRBL)**: Open source (GPL-compatible)
**Asgard GUI**: Open source (same as main project)
**ROS2 Integration**: Open source (Apache 2.0 / BSD depending on components)

---

## 14. Technical Comparison & Context

### Design Philosophy vs. Commercial Robots

**Thor's Approach**:
- Open source and fully documented
- 3D printable with commonly available components
- Affordable (~350€ vs. thousands for commercial alternatives)
- Educational focus with good-enough performance
- Community-driven development

**Trade-offs**:
- Lower accuracy than commercial robots (degrees vs. arcminutes)
- Lower repeatability (~0.2mm vs. <0.05mm for commercial)
- Open-loop control vs. closed-loop feedback
- Manual assembly vs. factory-built

**Sweet Spot**: Education, hobbyist projects, prototyping, robotics learning

### Suitable Applications

**Good For**:
- Educational demonstrations
- Learning robotics concepts
- Pick-and-place tasks (with tolerance)
- Light assembly work
- Camera positioning
- 3D printing/drawing applications
- Hobby projects

**Less Suitable For**:
- High-precision machining
- Medical applications
- Production environments
- Safety-critical tasks
- High-speed applications

---

## 15. Future Development & Roadmap

### Planned Improvements

**Firmware**:
- Inverse kinematics integration
- Trajectory planning algorithms
- Acceleration profiling
- Collision detection

**Software**:
- Enhanced Asgard GUI features
- 3D visualization improvements
- Path recording and playback
- Task-level programming interface

**ROS2 Integration**:
- Complete MoveIt configuration
- Gazebo simulation refinement
- Perception integration (camera systems)
- Grasp planning tools

**Hardware** (Community-driven):
- Closed-loop feedback options
- Improved rigidity designs
- Alternative materials exploration
- Larger/stronger variants

### Open Questions & Research Areas

**From GitHub Issues**:
- Optimal trajectory planning algorithms
- Real-time path modification
- Multi-robot coordination
- Vision-guided manipulation
- Force sensing integration
- Collaborative operation (safe human interaction)

---

## 16. Quick Reference

### Key Specifications Summary
| Parameter | Value |
|-----------|-------|
| DOF | 6 (yaw-roll-roll-yaw-roll-yaw) |
| Reach | 625mm |
| Payload | 750g max |
| Accuracy | 1-4° (varies by joint) |
| Cost | ~350€ |
| Build Time | ~200hrs print + 6hrs assembly |

### Essential Links
- **Website**: http://thor.angel-lm.com
- **GitHub**: https://github.com/AngelLM/Thor
- **Hackaday**: https://hackaday.io/project/12989-thor
- **Documentation**: http://thor.angel-lm.com/documentation/

### Critical Commands

**Common to Both Firmwares**:
| Command | Function |
|---------|----------|
| `M115` | Show firmware version |
| `M114` | Show current position |
| `G0 A10 B45` | Move joints A and B |
| `M3 S900` | Control servo (gripper) |

**GRBL-Specific**:
| Command | Function |
|---------|----------|
| `$H` | Home all axes |
| `$$` | View GRBL settings |
| `$X` | Clear alarm/unlock |

**RRF-Specific**:
| Command | Function |
|---------|----------|
| `G28` | Home all axes |
| `M98 P"config.g"` | Re-run config file |
| `M122` | Diagnostics report |

### Hardware Options Summary
| Component | Option 1 | Option 2 |
|-----------|----------|----------|
| **Board** | Thor ControlPCB + Arduino Mega | Fly Super8Pro |
| **Firmware** | GRBL | RepRapFirmware |
| **Cost** | ~30-40€ | ~60€ |
| **Interface** | Serial/USB | Serial/USB/WiFi/Web |

### Required Software

**For Thor ControlPCB (GRBL)**:
- Arduino IDE (firmware upload)
- Python 3 + PyQt5 (Asgard GUI)
- Universal G-Code Sender (alternative control)

**For Fly Super8Pro (RRF)**:
- Web browser (Duet Web Control)
- Optional: Pronterface, RepRapFirmware Configurator
- Optional: Python 3 + PyQt5 (Asgard GUI - may need adaptation)

**Common/Optional**:
- FreeCAD (for model editing)

---

## Document Information

**Last Updated**: 2025
**Primary Source**: http://thor.angel-lm.com and GitHub (AngelLM/Thor)
**Document Purpose**: Comprehensive technical reference for Thor robotic arm project
**Maintained By**: Project documentation based on official sources and community knowledge

**Note**: This is a living document. Check official sources for latest updates, downloads, and community discussions.
