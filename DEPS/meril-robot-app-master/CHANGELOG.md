# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.6.7] - 2024-11-07

### Changed

- [x] Use J10 for roll in surgical mode, use J6 for handguiding and cameraView position offset (straighten)
- [x] Camera robot can retract when fulcrum is taught
- [x] handcontroller to instrument mapping changed + can be adjusted (reversed)
- [x] add instrument roll offset to align pliers to camera view (tilt more/less)
- [x] added time-out for straighten, calibration and symbolic position moves
- [x] always use roll joint angle in FK calculation
- [x] calculations for straighten to Camera view
- [x] enable the use of J10
- [x] remove min insertion depth for camera
- [x] added lookup table for cartRateLimiter based on insertion depth.
- [x] big admittance updates (use vibration canceling, actively use joint limits, etc)
- [x] updates to symbolic positions
- [x] from sleep to locked directly, not via Init (symbolic positions)

### Fixed

- [x] pinch issue(minimum/maximum pinch position limits removed) resolved.
- [x] camera sync issue resolved
- [x] tool doesn't change pose when camera moves
- [x] J6 always in admittance mode
- [x] enable axesLimiter when entering locked mode

## [1.6.3] - 2024-10-22

### Tests

performed following tests:

- [x] manual mode (with/without fulcrum)
- [x] camera align handcontroller
- [x] camera align instrument
- [x] surgical (no camera offset)
- [x] surgical with camera using haptics
- [x] manual handguiding
- [x] instrument connect and calibration
- [x] instrument straightening
- [x] symbolic positioning
- [x] limits and fulcrum stability

### Changed

- [x] updates in symbolic position module
- [x] handguiding in admittance control is updated with ANC and vitrual Mass
- [x] instrument roll joint is aligned with camera within the patient (inside fulcrum)
- [x] instrument roll joint is aligned with 0.0 outside the patient (outside fulcrum)
- [x] handcontroller is no longer aligned to instrument shaft orientation but kept straight.
- [x] instrument kinematics are decoupled from robot kinematics. (big change)
- [x] limit conditions, stability and softness reaching limits improved.
- [x] camera no longer responds to straightening command

### Added

- [x] instrument calibration sequence added
- [x] instrument makes use of absolute single turn encoders from now
- [x] instrument connect mode to read instrument RFID automatically when instrument is connected. (can be disabled in
  Logic)
- [ ] ...

### Fixed

-

## [1.6.2] - 2024-09-18

### Tests

performed following tests:

- [x] manual mode (with/without fulcrum)
- [x] retract and exchange
- [x] surgical (no camera offset)
- [x] surgical with camera using haptics
- [x] force feedback (trail test)

### Changed

- [x] add surgical is allowed for force feedback
- [x] instrument orientation change shall not lead to robot translation change
- [x] update synchronizing cartManual pose
- [x] reduced max/min settings for insertion depth.
- [x] insertion depth can only be taught in maintenance (engineering mode)

### Fixed

- [x] move iterate from if loop (force feedback)
- [x] hostIn using quaterions and can scale (<3), slow move handcontroller

## [1.6.1] - 2024-09-12

### Changed

- [x] update in admittance control to better stabilize near singular positions using a manipulabilityLookup correction
  for mass and damper settings.
- [x] added positionLimiterLookup that creates a push back force in the admittance controller when nearing joint limits
- [x] corrections in force feedback. Note that changing the scaling in the orientation makes the robot position deviate
  from the haptic device orientation. we need to investigate if this can be improved.
- [x] reduce allow joint upper limit position for j3 to prevent entering extreme poses (near singular)

### Fixed

- [x] corrected estop going to merilMode::sleep
- [x] corrected force disengage going to merilMode::locked
- [x] corrections in merilMode machine

### Note

- [x] for more aggressive moves of the instrument the cart pose rate limiter set at 5 is too slow for the joint
  orientation change however, it prevents the drives from springing into estop

## [1.6.0] - 2024-09-06

### Added

- [x] Instrument roll axis straightening. Any axes (joints) can be straightened with the straightening command.
  Straightening makes use of the symbolicPosition functionality and can be extended.
- [x] Update UDP to straighten on trigger haptic controller.
- [x] Added GUI button inputs to the logic. these are called e.g.:
    - root/Logic/gotoInstrumentStraightenGuiButton for Gui;
    - root/Logic/gotoInstrumentStraightenUdpButton for Udp;
    - root/Logic/gotoInstrumentStraightenButton for HW Button;
- [x] add a maintenance "mode" input to allow overwriting (advanced) of signals like reset fulcrum. furthermore, it
  allows teaching symbolic positions to the actual positions of the joints. functionality will be extended in the
  future.
- [x] add enable Dobot Buttons (disable on button) in logic
- [x] add enable Dobot Handguiding Button (disable on button) in logic
- [x] add initial version for force feedback on the haptic device.

#### IO Updates for cartIsDocked and Z-Actuator

- [x] Add EK1914:In4 - CartIsDocked : add signal as extra condition before teaching fulcrum
- [x] Add EK1914:out4 - DisableZActuator : note that whenever the fulcrum has been taught and the instrument is inside
  the fulcrum the z-actuator should also be prevented to move. only whenever the instrument is outside the fulcrum the
  z-actuator can move.
- [x] Add EK1914:In5 - ZActuatorIsDisabled : confirmation that the Z actuator is disabled.
- [x] Add warnings: cartIsNotDocked when teaching fulcrum
- [x] Add warnings: cartIsNotDocked when fulcrum is active (Notify user as recoverable Warning/Error)
- [x] Add warnings: ZActuatorIsDisabled warning if fulcrumIsValid.

#### Symbolic positions

Symbolic positions mode is added between Sleep and Locked Mode. One enters Symbolic mode when the robot is switched to
ON in the software. There are 3 moves added: movePark, moveStartHigh and moveStartLow.
A move can only be made if the instrument is not connected. If the instrument is connected, this can lead to collision
with the cart or surroundings.
A move is only started when the move button is pressed. upon release of this button the movement will come to a stop.
A move can be bypassed by directly go to locked position when the robot is close to its start position. (recover from
errors)
The order of the symbolic positions can be set in the moveOrder parameter. the value '0' means this joint is not moved.
Values > 0 will have a move, the order starts with 1 (first move). when all joints with moveOrder = 1 have finished
their move, joint with moveOrder=2 will start. etc, etc.
The setting of the movement velocity and acceleration is done in the setpoint generators.

Straighten function will make use of the same function block.

The following action items:

- [x] Change symbolic position mode is the first mode after sleep. PVA is disabled. can be entered from sleep (engage)
  or locked. PVA settings are set in this mode.
- [x] Set Limiter values according to start position.
- [x] disable PVA limiter when traveling to Park Position. (is within the PVA limits. When starting (toEngage), the PVA
  limiter should only become active if the system has reached either Start-High or Start-Low position. From that point
  on the PVA is enabled. Note that the PVA settings for position limits should then change to either start high or start
  low values.)
- [x] Set sequence for Move to Park, Move to High or Move to Low position.
- [x] possibility to Bypass symbolic position moves in the software.
- [x] Remove original jointPositions-upper/lower limit_ and replace links
- [x] When the symbolic move starts time scale factor is 0, time scale factor is
  controlled by the gui, time scale factor 0 <= scale <= 1. can be attached to a button.

### Changed

- [x] Orienation Gain can be set > 1, leading to an amplified movement of the instrument/robot wrt the haptic input;
  disable quaternion input and multiply euler input + correct in udp.link.json
- [x] Move Straighten logic to logic;
- [x] Move exchange logic from manipulator to logic; check instrument exchange
- [x] Move retract logic from manipulator to logic; check instrument retract
- [x] connect fulcrum watchdog (got removed from struct... needs to reattach + warnings)
- [x] entering SleepMode will put the robot to OFF state.
- [x] fulcrum reset is quite difficult and fulcrum is maintained even after restart. This is partially due to the
  kinematic structure of the dobot in combination with the length of the instrument. A maintenance mode was added to
  reset the fulcrum in case the robot goes to singular positions to retract from fulcrum. Note that at teaching the
  fulcrum, the tip of the instrument should set the fulcrum, visibly in the trocart.
- [x] when we are restarting controller fulcrum should be reset (always) -> changed fulcrumPose from parameter to input
- [x] field bus reaching max 90% of utilization almost in every robot app, were logic task is running max 72%. We
  improved/updated some functions in the control3 and math library which were inefficient/consumed significant task
  time. this lead to a decrease inn fieldbus and control task time.
- [x] retract jump resolved
- [x] removed most of the SDOs and save an additional 5% fieldbus task load

### Open issue

- [x] Robot opposing very hard to move when robot hits constraint limits the only solution need to restart: add feedback
  to user. check reset of the force when hitting a constraint. --> reduce torque, wait 1 second for robot to synchronise
  its position to the limit position

## [1.5.2] - 2024-07-05

### Changed

- Retract instrument can now be started from the dobot exchange button (arrow) while 'inside' the fulcrum.
- GUI updates to support homing Module
- General GUI updates for latest GRID release

### Added

- Added homingModule to execute automated homing of the sterile adapter and adjust homing for the instrument.
- Instrument roll axis straightening

## [1.5.0] - 2024-06-03

### Changed

- Use Motorcortex RTOS build 2024-01. Latest RTOS has improvements in EtherLab (EtherCat master), several Bug fixes and
  supports newer hardware.
- Large Logic update: Integrated mode system (meril modes). Mode system conform P0173-2200-02-SPC-R02 MCX-Robot-Modes
  Overview.
  In the Mode system there is a strict sequence. Deviation from the sequence is not allowed.
  Calibration modes are not yet implemented since calibration is pending.
- Mode logic via Dobot buttons (or later defined by meril) and UDP communication only.
- Replaced the insertion depth constraint with a minimum (when not straight) and maximum insertion depth constraint.
  These constraints are implemented as spherical constraint.
- Insertion depth teach by pressing the teach button shortly, reset by pressing 1 sec. the teach button (NOT
  isOutsideFulcrumPort).
- fulcrum pose teach by pressing the teach button shortly, reset by pressing 1 sec. the teach button (
  isOutsideFulcrumPort).
- retract replaced by manual mode. retract now only stores retract position to return to the exact same position when
  disabled.
- unlock_instrument mode respect minimum insertion depth constraint while not straightened. This is to prevent an
  unstraight instrument to be fitted through the canula of the trocar. Retract through trocar is only allowed with
  straightened instrument.
- Straighten process is executed with a single press on the straighten button.
- Calibration can remain in engaged (no longer required to switch drives to Idle) after homing enable.
- Update visual feedback from dobot.
- GUI updates to support modes (disabled inputs from GUI)

### Fixed

- control library update: Required fixes to the actuator control and transducers in control library update
- Several Library updates (core, math, mechanics)
- All code now Compiled with more stringent checks on float exceptions and memory leaks.

### Added

- Initial python script for homing (roll axis and drapery cap only). Please try this initial version and see where we
  can improve.
- Added isOutsideFulcrumPort signal. It depends on the fulcrumPose and the fulcrumPortExternalLength. It is true when
- Added instrument rotation compensation. instrument roll axis compensates the dobot axis 6 rotation. use
  instrumentRotationEnable (true) to enable this.
- homing for axes enabled. Using linking we can now enable homing for complete axis consisting of multiple actuators.
- fulcrum can now be reset. It is only allowed when outside the fulcrum port (isOutsideFulcrumPort).
- insertionDepth can now be reset. It is only allowed when NOT outside the fulcrum port (!isOutsideFulcrumPort).
- Added axesTorqueDetectors to support homing calibration based on force on axis level (multiple actuators)

## [1.4.7] - 2024-05-15

### Changed

- insertionDepth has been replaced with a spherical constraint
- resolved jerk behavior when reaching the limit in constraint handguiding wrt fulcrum.
- this template should be built with Control3::MerilUpdate branch

### Fixed

- admittance only uses low frequency PVA since it is in position mode.

### Added

- indicate if we are in or outside the Port (trocart).

## [1.4.6] - 2024-05-10

### Added

- cameraPoseViewOffset and cameraPoseView (should be linked from camera to instrument robots) added. For this the
  parameter poseSync/poseSyncInSurgicalEnable for the camera(false) and for instrument (true)
- Allow rotations of the local camera frame (allow a tilted cameraviewpoint)
-

### Changed

- externalFeedforward should be turned On since the model torque is ported via feedforward now. Single model of the
  joint mechanisms.
- admittance control removed staticTorqueModel since it comes from feedforward now (complete actuator model)
- handcontroller input poseSync threshold.

### Fixed

- camera view orientation is set by the cameraPoseViewOffset. cameraPoseView is linked to cameraPose of instrument
  robots. this prevents the setting of the phi = pi in mechanism/tool and prevents inverse of the insertiondepth.

### Removed

- Cleaned up code: ActuatorControlLoop and AxesControlLoop part of Control3 library
- Cleaned up code: Subfunctions / feedforward part of Control3 library

## [1.4.5] - 2024-04-19

### Added

- Added torque deadband control to get rid of the disturbance at 0.0 torque target in the drive.

### Changed

- at teach fulcrum, set max insertion depth to default value (0.5 m)
- instrument exchange can only activate/deactivate when in manual joint mode and pause is off
- allow manual joint mode (instrument manual mode) outside the fulcrum
- added pause instrument exchange on return towards fulcrum

### Fixed

- teach maximum insertion depth only when fulcrumValid = true
- backlash compensation pos/neg is replaced by a single value. (to prevent drift)

## [1.4.4] - 2024-04-12

### Added

- added instrument insertion depth teach function
- Backlash compensation in actuator control loop (tested)

### Changed

- instrumentStraightened looks to a parameter now
- Setmaximum insertion depth
-

### Fixed

- instrument depth is not overwritten by the minimum depth.

## [1.4.3] - 2024-04-11

### Added

- Added code for backlash compensation(not implemented yet)
- Added parameter for fulcrumValidRange

### Changed

- Go to Instrument Manual / Straighten via input buttons on dobot.
- Use hand guiding button for both Impedance as admittance (with fulcrum).

### Fixed

## [1.4.2] - 2024-04-04

### Added

- Added stiff axesLimiter settings

## [1.4.1] - 2024-03-29

### Added

- Added camera reference frame. the orientation of the manual input devices is transformed to the camera view making
  instrument moves more intuitive. Camera repositioning resets the hostIn to not jump.
- Added Cartesian pose rate limiter and reset functionality for the hostIn manual inputs. This will prevent the too
  strong movement commanded by the haptic input devices.
- Added feedforward position disturbance compensation using multisine correction of the gearbox and backlash
  compensation.
- Added manual joint mode while respecting the fulcrum. automatically put axes 1,2,3 in admittance mode to do manual
  manipulation. The remaining axes are to compensate for the position to respect the fulcrum.
- Added Homing for axes/actuators. This is applicable for the instrument axes. Robot axes can still be referenced as
  before.
- GUI update with Instrument Control
- GUI update with Homing
- Added Jog mode for actuators (enable inside actuatorControl)

### Changed

- Admittance control updates; now respecting the axesLimiting values and connected feedforward model to the input
  forces. This needs to be tuned again to work with this update. limit the maximum velocity to not break the fulcrum.
- Don't allow going to cart mode (surgical) if depth constraint violated or fulcrum invalid.
- implementation of isOpenLoop for each axis separately. Functionality update.
- jumpDetectors corrected
- linked feedforward to admittance control, to replace the admittance/friction model. Now the friction is modeled only
  in the actuator control loop feedforward module. This feedforward is used in position (joint/cart and admittance) and
  torque (unconstrained handguiding) mode.
- sliding mode for friction compensation added (feedforward). This makes use of the lugre friction model with a sliding
  mode
- added & configured velocity feedback from the drive controller and separated actual velocity filtering from position.
  Decrease position delay by lowpass filtering the position @ 50 Hz.
- Instrument control - axisLimiter updates allow smooth position limiting only. improved opening/closing of the
  instrument. (advice to perform drive tuning)
- GUI update with velocity transformation included in "axes settings" tab general Core library update. (stability
  improvement)
- instrument kinematics update
- instrument straightened added (straightening the instrument before retract)
- instrument retract update, (more robust)

### Deprecated

### Removed

- impedance controller inside compliance. (not used, and not applicable for the impedance control we need)

### Fixed

### Security

## [1.3.3] - 2024-02-02

### Added

- added special versions of the axesControl, actuatorControl and feedforwardControl from control3
- color management
- improved switching in/out torque mode

## [1.3.2] - 2024-01-25

### Added

- instrumentRetractEnable_ = false when siwtching jointMode to cartesian. (prevent jumps)
- respect fulcrum when joints are limited by the axesLimiters
- instrumentAdjust replaces instrumentRetract

## [1.2.0] - 2023-11-03

### Added

- Torque mode Hand Guiding
- Receive Pinch setpoint via UDP

## [1.1.3] - 2023-11-01

### Added

- Instrument motion constraints:
    - minimum distance between instrument tip and fulcrum
    - minimum distance between robot tooltip and fulcum
    - minimum distance between instrument tip and plane parallel to the fucrum xy-plane

## [1.1.2] - 2023-10-25

### Changed

- Refactored Signals names for Target, Actual and Reference coordinates

### Fixed

- Synchronization was using wrong reference signal

## [1.1.1] - 2023-10-24

### Added

- Fulcrum validity checker
- Changelog was not yet being maintained, see [1.1.1]

## [1.1.0] - 2023-10-23

### Added

- Initial version for Demo2
- Changelog was not yet being maintained, see [1.1.0]

## [1.0.2] - 2023-10-19

### Added

- Number of joints for manipulator control loop to support an instrument
- Changelog was not yet being maintained, see [1.0.2]

## [1.0.1] - 2023-09-28

### Added

- Signal and setpoint modules
- Changelog was not yet being maintained, see [1.0.1]

## [1.0.0] - 2023-09-28

### Added

- Initial Commit

[unreleased]: https://git.vectioneer.com/meril/meril-robot-template/-/compare/1.3.2...HEAD

[1.3.2]: https://git.vectioneer.com/meril/meril-robot-template/-/compare/1.2.0...1.3.2

[1.2.0]: https://git.vectioneer.com/meril/meril-robot-template/-/compare/1.1.3...1.2.0

[1.1.3]: https://git.vectioneer.com/meril/meril-robot-template/-/compare/1.1.2...1.1.3

[1.1.2]: https://git.vectioneer.com/meril/meril-robot-template/-/compare/1.1.1...1.1.2

[1.1.1]: https://git.vectioneer.com/meril/meril-robot-template/-/compare/1.1.0...1.1.1

[1.1.0]: https://git.vectioneer.com/meril/meril-robot-template/-/compare/1.0.2...1.1.0

[1.0.2]: https://git.vectioneer.com/meril/meril-robot-template/-/compare/1.0.1...1.0.2

[1.0.1]: https://git.vectioneer.com/meril/meril-robot-template/-/compare/1.0.0...1.0.1

[1.0.0]: https://git.vectioneer.com/meril/meril-robot-template/-/tags/1.0.0
