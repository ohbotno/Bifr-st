# Asgard GUI - RepRapFirmware (RRF) Conversion Notes

## Overview
The Asgard GUI has been updated to support both GRBL and RepRapFirmware (RRF). The firmware type can be switched by changing a single variable at the top of the AsgardGUI class.

## Configuration

To switch between firmware types, edit line 32 in `asgard.py`:

```python
# For RepRapFirmware (Fly Super8Pro board):
self.firmware_type = FIRMWARE_RRF

# For GRBL (Arduino Mega + Thor ControlPCB):
self.firmware_type = FIRMWARE_GRBL
```

## Command Conversions

### Homing Command
- **GRBL**: `$H`
- **RRF**: `G28` (home all axes)

### Kill Alarm / Reset Command
- **GRBL**: `$X` (kill alarm lock)
- **RRF**: `M999` (clear emergency stop / reset)

### Status Query
- **GRBL**: `?` (status query, returns comma-separated format)
- **RRF**: `M408 S0` (status query, returns JSON format)

## Status Parsing Differences

### GRBL Status Format
```
<Idle,MPos:0.000,0.000,0.000,0.000,0.000,0.000,0.000>
```
- Comma-separated values
- States: Idle, Run, Home, Alarm, Hold
- Position data after `MPos:`

### RRF Status Format (M408 S0)
```json
{
  "status": "I",
  "coords": {
    "xyz": [0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
  }
}
```
- JSON format
- Status codes:
  - `I` = Idle
  - `P` = Printing/Running
  - `S` = Stopped
  - `C` = Config
  - `A` = Paused
  - `D` = Pausing
  - `R` = Resuming
  - `B` = Busy
- Axis positions in `coords.xyz` array

## Axis Mapping

Both firmwares use the same axis naming convention for Thor:
- **A**: Articulation 1 (Base rotation)
- **B**: Articulation 2 (Shoulder - motor B)
- **C**: Articulation 2 (Shoulder - motor C, coupled with B)
- **D**: Articulation 3 (Elbow)
- **X**: Articulation 4 (Wrist rotation)
- **Y**: Articulation 5 (Wrist pitch)
- **Z**: Articulation 6 (Wrist roll)

## Movement Commands (Unchanged)

Movement commands remain the same for both firmwares:
- **G0**: Rapid positioning (e.g., `G0 A10 B45`)
- **G1**: Controlled movement with feed rate (e.g., `G1 A20 F1000`)
- **M3**: Servo control for gripper (e.g., `M3 S900`)

## RRF Configuration Requirements

For RRF to work correctly with Thor, your `config.g` must define:

1. **Axis configuration** with A, B, C, D, X, Y, Z axes
2. **Kinematics**: Usually set to Cartesian but adapted for robotic arm
3. **Steps per degree** for each motor (M92 command)
4. **Motor directions** (M569 command)
5. **Speed and acceleration limits** (M203, M201)
6. **Endstop configuration** (M574)

Example RRF config excerpt:
```gcode
; Configure motors (example values - adjust for your Thor)
M569 P0 S1                    ; Motor 0 (A axis) - clockwise
M569 P1 S1                    ; Motor 1 (B axis)
M569 P2 S1                    ; Motor 2 (C axis)
M569 P3 S1                    ; Motor 3 (D axis)
M569 P4 S1                    ; Motor 4 (X axis)
M569 P5 S1                    ; Motor 5 (Y axis)
M569 P6 S1                    ; Motor 6 (Z axis)

; Configure axis mapping
M584 A0 B1 C2 D3 X4 Y5 Z6     ; Map axes to motors

; Steps per degree (example - adjust for your motors and gearing)
M92 A88.9 B88.9 C88.9 D88.9 X88.9 Y88.9 Z88.9

; Speed and acceleration
M203 A6000 B6000 C6000 D6000 X6000 Y6000 Z6000  ; Max speed (deg/min)
M201 A1000 B1000 C1000 D1000 X1000 Y1000 Z1000  ; Acceleration (deg/s^2)
```

## Testing Checklist

After switching to RRF:
- [ ] Test homing command (Home button)
- [ ] Test kill alarm command (Kill Alarm button)
- [ ] Verify position display updates correctly
- [ ] Test individual articulation movements
- [ ] Test "Move All" function
- [ ] Test gripper control (M3 command)
- [ ] Verify console output shows proper responses
- [ ] Check state display (Idle/Run/etc.) updates correctly

## Known Limitations

1. **Coupled articulations**: Art2 (B and C motors) still use simple mirroring, not true coupled kinematics
2. **Art5 and Art6**: Comments in code indicate these may need coupled movement calculations
3. **RRF axis order**: Current code assumes axis positions are in A,B,C,D,X,Y,Z order - verify this matches your RRF configuration

## Troubleshooting

### No position updates
- Check that M408 S0 returns valid JSON
- Verify axis names in RRF config match A,B,C,D,X,Y,Z
- Enable verbose console output to see raw responses

### Homing doesn't work
- RRF requires homeall.g, homea.g, homeb.g, etc. files on SD card
- Check that endstops are properly configured in config.g

### Movement commands don't work
- Verify axis mapping in config.g (M584 command)
- Check steps per degree settings (M92 command)
- Ensure motors are enabled and not in idle hold

## Additional Resources

- **RRF Documentation**: https://docs.duet3d.com
- **M408 Command**: https://docs.duet3d.com/User_manual/Reference/Gcodes#m408-report-json-style-response
- **Fly Super8Pro Setup**: See KNOWLEDGE.md Section 4 & 5
