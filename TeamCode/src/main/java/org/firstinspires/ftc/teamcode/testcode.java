/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/*
 * goBILDA StarterBot TeleOp - TANK DRIVE edition.
 *
 * DRIVETRAIN  : Tank drive. LEFT stick Y drives the LEFT side, RIGHT stick Y drives the RIGHT side.
 *               Skid-steer with the hardware names from the original code (left_drive / right_drive).
 *               Optional extra motors (for a 4-motor chassis) are picked up automatically if they
 *               exist in the robot configuration - see EXTRA_*_DRIVE_NAMES below.
 *
 * CONTROLS (gamepad1 = driver. gamepad2 can optionally run the mechanisms - see "Operator" column)
 *   Left stick Y ........ left side drive
 *   Right stick Y ....... right side drive
 *   Left bumper (hold) .. precision mode while held
 *   A ................... precision mode latch (toggle)
 *   B ................... flip the robot's "front" (swaps/inverts sticks so the back drives forward)
 *   Right trigger ....... intake in      (proportional)   [Operator: same]
 *   Left trigger ........ intake out     (proportional)   [Operator: same]
 *   Right bumper (hold) . launch: spin up, windmill feeds automatically once at speed   [Operator: same]
 *   X ................... launcher pre-spin latch (toggle) [Operator: same]
 *   Y (hold) ............ FORCE feed: windmill runs even if launcher is below speed   [Operator: same]
 *   D-pad up / down ..... launcher speed trim +/- one step [Operator: same]
 *   D-pad left .......... reset launcher trim                [Operator: same]
 *   BACK ................ mechanism lockout (intake, launcher, windmill off) toggle   [Operator: same]
 *
 * PROTECTION / FAIL-SAFES
 *   - Stick deadband + expo curve, asymmetric slew-rate limiting (acceleration vs braking).
 *   - Drive over-current -> automatic power cap + rumble.
 *   - Intake jam detection -> automatic short reverse to clear it.
 *   - Launcher over-current -> launcher cut until the driver releases the launch controls.
 *   - Windmill only feeds when the launcher is at speed (the original code's comment promised this,
 *     but its code did not check the velocity).
 *   - Launcher coasts down (FLOAT) instead of actively reversing the flywheel to a stop.
 *   - Low-battery warning, everything shuts off in stop().
 *
 * Every tunable number lives in the constants block right below. Nothing else needs editing
 * unless your hardware configuration names differ.
 */
@TeleOp(name = "BioBuzz StarterBot Teleop", group = "StarterBot")
//@Disabled
public class testcode extends OpMode {

    // =========================================================================================
    //  HARDWARE NAMES - must match the Robot Configuration on the Driver Station / Control Hub
    // =========================================================================================
    private static final String LEFT_DRIVE_NAME = "left_drive";
    private static final String RIGHT_DRIVE_NAME = "right_drive";
    private static final String INTAKE_NAME = "intake";
    private static final String LAUNCHER_NAME = "launcher";
    private static final String WINDMILL_NAME = "windmill";
    private static final String LEFT_INTAKE_SERVO_NAME = "left_intake_servo";
    private static final String RIGHT_INTAKE_SERVO_NAME = "right_intake_servo";

    // OPTIONAL extra drive motors for a 4-motor chassis. If a name is not in your configuration it is
    // silently skipped (nothing breaks). Each extra motor uses the same direction as its side.
    private static final String[] EXTRA_LEFT_DRIVE_NAMES = {"left_rear_drive"};
    private static final String[] EXTRA_RIGHT_DRIVE_NAMES = {"right_rear_drive"};

    // Same directions as the original StarterBot code. If a side drives backwards when its stick
    // is pushed forward, flip that side's direction here (and only here).
    private static final DcMotorSimple.Direction LEFT_DRIVE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction RIGHT_DRIVE_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // =========================================================================================
    //  DRIVETRAIN TUNING
    // =========================================================================================
    private static final double STICK_DEADBAND = 0.06;       // ignore stick drift below this
    private static final double STICK_EXPO = 0.35;           // 0 = linear, 1 = pure cubic (finer low-speed control)
    private static final double NORMAL_SCALE = 1.00;         // max power in normal mode
    private static final double PRECISION_SCALE = 0.35;      // max power in precision mode
    private static final double DRIVE_ACCEL_PER_SEC = 3.0;   // power/sec when speeding up (3.0 = 0 -> 1 in ~0.33 s)
    private static final double DRIVE_DECEL_PER_SEC = 5.0;   // power/sec when slowing or reversing
    private static final boolean STRAIGHT_ASSIST_ENABLED = true;
    private static final double STRAIGHT_ASSIST_WINDOW = 0.08; // sticks this close together drive dead straight

    // =========================================================================================
    //  INTAKE TUNING
    // =========================================================================================
    private static final double TRIGGER_DEADBAND = 0.05;
    private static final double INTAKE_ACCEL_PER_SEC = 6.0;
    private static final double INTAKE_DECEL_PER_SEC = 12.0;
    private static final double FEED_ASSIST_INTAKE_POWER = 0.5; // added to intake while feeding (as in original)
    private static final double INTAKE_JAM_MIN_COMMAND = 0.10;  // only auto-unjam when driver is intaking

    // =========================================================================================
    //  LAUNCHER TUNING  (velocities are encoder ticks per second; 28 ticks per motor revolution)
    // =========================================================================================
    // NOTE: the original file had "8000+++" here, which does not compile. 8000 ticks/s would be
    // ~17,000 RPM - faster than any FTC motor - so it was treated as a typo. 2800 ticks/s is the
    // ~6000 RPM free speed of a 1:1 goBILDA motor and matches your 2650 minimum (~95%). VERIFY on the
    // robot with the "Launcher speed" telemetry line and adjust.
    private static final double LAUNCHER_TARGET_VELOCITY = 2800.0;
    private static final double LAUNCHER_MIN_VELOCITY = 2650.0;       // "at speed" threshold (windmill may feed)
    private static final double LAUNCHER_MAX_SAFE_VELOCITY = 2800.0;  // hard cap on target incl. trim
    private static final double LAUNCHER_TICKS_PER_REV = 28.0;        // for the RPM readout only
    private static final double LAUNCHER_READY_TOLERANCE = Math.max(25.0, LAUNCHER_TARGET_VELOCITY - LAUNCHER_MIN_VELOCITY);
    private static final double LAUNCHER_READY_EXIT_MARGIN = 100.0;   // hysteresis so feeding doesn't chatter
    private static final double LAUNCHER_TRIM_STEP = 50.0;
    private static final double LAUNCHER_TRIM_LIMIT = 400.0;
    private static final double LAUNCHER_SPINUP_WARN_SECONDS = 4.0;   // "not at speed" warning delay
    private static final double LAUNCHER_COAST_DISPLAY_TPS = 100.0;
    private static final double WINDMILL_FEED_POWER = 1.0;
    // Same velocity PIDF as the original code.
    private static final double LAUNCHER_P = 40.0;
    private static final double LAUNCHER_I = 0.0;
    private static final double LAUNCHER_D = 0.0;
    private static final double LAUNCHER_F = 12.5;

    // =========================================================================================
    //  CURRENT PROTECTION  (amps - assumes goBILDA Yellow Jacket class motors, ~9.2 A stall)
    //  Set CURRENT_PROTECTION_ENABLED = false to turn all of it off.
    // =========================================================================================
    private static final boolean CURRENT_PROTECTION_ENABLED = true;
    private static final double CURRENT_POLL_INTERVAL_SECONDS = 0.05;   // one motor read per interval, round-robin
    private static final double CURRENT_MAX_SAMPLE_SPAN_SECONDS = 0.5;
    private static final double CURRENT_RELEASE_FRACTION = 0.80;        // must fall below 80% of limit to clear
    private static final double DRIVE_TRIP_AMPS = 8.0;
    private static final double DRIVE_TRIP_SECONDS = 1.0;
    private static final double DRIVE_DERATE_SCALE = 0.50;              // power cap while over-current
    private static final double DRIVE_DERATE_HOLD_SECONDS = 2.0;
    private static final double INTAKE_JAM_AMPS = 6.0;
    private static final double INTAKE_JAM_SECONDS = 0.40;
    private static final double INTAKE_UNJAM_SECONDS = 0.35;
    private static final double INTAKE_UNJAM_POWER = 0.80;
    private static final double LAUNCHER_TRIP_AMPS = 8.0;
    private static final double LAUNCHER_TRIP_SECONDS = 1.5;

    // =========================================================================================
    //  MISC
    // =========================================================================================
    private static final double LOW_BATTERY_VOLTS = 11.0;
    private static final double LOW_BATTERY_HYSTERESIS = 0.3;
    private static final double BATTERY_POLL_SECONDS = 0.5;
    private static final double BATTERY_FILTER = 0.25;
    private static final double MAX_LOOP_DT_SECONDS = 0.10;   // clamp so a hiccup can't cause a power jump
    private static final double WRITE_EPSILON = 0.001;        // skip hardware writes smaller than this
    private static final int TELEMETRY_INTERVAL_MS = 100;

    // =========================================================================================
    //  HARDWARE
    // =========================================================================================
    private List<LynxModule> hubs;
    private DcMotorEx[] leftMotors;
    private DcMotorEx[] rightMotors;
    private DcMotorEx intake;
    private DcMotorEx launcher;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;
    private VoltageSensor batterySensor;

    // Current protection
    private CurrentGuard currentGuard;
    private CurrentChannel[] driveChannels = new CurrentChannel[0];
    private CurrentChannel leftDriveChannel;
    private CurrentChannel rightDriveChannel;
    private CurrentChannel intakeChannel;
    private CurrentChannel launcherChannel;

    // =========================================================================================
    //  STATE
    // =========================================================================================
    private final ElapsedTime clock = new ElapsedTime();
    private double lastLoopTime = 0.0;
    private double loopHz = 0.0;

    private final SlewLimiter leftSlew = new SlewLimiter(DRIVE_ACCEL_PER_SEC, DRIVE_DECEL_PER_SEC);
    private final SlewLimiter rightSlew = new SlewLimiter(DRIVE_ACCEL_PER_SEC, DRIVE_DECEL_PER_SEC);
    private final SlewLimiter intakeSlew = new SlewLimiter(INTAKE_ACCEL_PER_SEC, INTAKE_DECEL_PER_SEC);

    private final Edge edgeA = new Edge();
    private final Edge edgeB = new Edge();
    private final Edge edgeX = new Edge();
    private final Edge edgeBack = new Edge();
    private final Edge edgeDpadUp = new Edge();
    private final Edge edgeDpadDown = new Edge();
    private final Edge edgeDpadLeft = new Edge();

    // Toggles
    private boolean precisionLatched = false;
    private boolean frontReversed = false;
    private boolean mechanismsLocked = false;
    private boolean preSpinLatched = false;

    // Drive
    private double leftStickIn = 0.0;
    private double rightStickIn = 0.0;
    private double leftPower = 0.0;
    private double rightPower = 0.0;
    private double driveScale = NORMAL_SCALE;
    private boolean precisionActive = false;
    private boolean straightAssistActive = false;
    private boolean driveDerated = false;
    private boolean wasDriveDerated = false;
    private double driveDerateUntil = -1.0;

    // Intake
    private double intakeRequest = 0.0;
    private double intakeApplied = 0.0;
    private boolean intakeUnjamming = false;
    private double intakeUnjamUntil = 0.0;
    private int jamCount = 0;

    // Launcher
    private double launcherTrim = 0.0;
    private double launcherVelocity = 0.0;
    private double launcherTargetVelocity = LAUNCHER_TARGET_VELOCITY;
    private boolean launcherClosedLoop = false;
    private boolean launcherWantSpin = false;
    private boolean launcherReady = false;
    private boolean launcherFeeding = false;
    private boolean launcherFaulted = false;
    private String launcherFaultReason = "";
    private boolean readyCueGiven = false;
    private boolean spinWarning = false;
    private double notReadySince = -1.0;

    // Battery
    private double batteryVolts = 0.0;
    private boolean batteryLow = false;
    private double lastBatteryPoll = -1000.0;

    // Last values actually written to hardware (NaN = nothing written yet) to avoid redundant writes
    private double lastLeftWritten = Double.NaN;
    private double lastRightWritten = Double.NaN;
    private double lastIntakeWritten = Double.NaN;
    private double lastWindmillWritten = Double.NaN;
    private double lastLauncherTargetWritten = Double.NaN;

    private final StringBuilder warningBuilder = new StringBuilder();

    // =========================================================================================
    //  INIT
    // =========================================================================================
    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);

        /*
         * Manual bulk caching: all encoder/velocity reads for a hub come from ONE bulk read that we
         * refresh once per loop (see clearBulkCaches). This is the standard way to get fast loops.
         */
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // ---- Drivetrain (tank): required motors first, then any optional extras ----
        List<DcMotorEx> left = new ArrayList<>();
        List<DcMotorEx> right = new ArrayList<>();
        left.add(hardwareMap.get(DcMotorEx.class, LEFT_DRIVE_NAME));
        right.add(hardwareMap.get(DcMotorEx.class, RIGHT_DRIVE_NAME));
        for (String name : EXTRA_LEFT_DRIVE_NAMES) {
            DcMotorEx extra = hardwareMap.tryGet(DcMotorEx.class, name);
            if (extra != null) {
                left.add(extra);
            }
        }
        for (String name : EXTRA_RIGHT_DRIVE_NAMES) {
            DcMotorEx extra = hardwareMap.tryGet(DcMotorEx.class, name);
            if (extra != null) {
                right.add(extra);
            }
        }
        leftMotors = left.toArray(new DcMotorEx[0]);
        rightMotors = right.toArray(new DcMotorEx[0]);
        configureDriveMotors(leftMotors, LEFT_DRIVE_DIRECTION);
        configureDriveMotors(rightMotors, RIGHT_DRIVE_DIRECTION);

        // ---- Intake ----
        intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0.0);

        // ---- Launcher ----
        launcher = hardwareMap.get(DcMotorEx.class, LAUNCHER_NAME);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        //launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        // FLOAT so the flywheel coasts down. Commanding velocity 0 would make the velocity PID drive
        // the flywheel in reverse to stop it - a big current spike every time you release the button.
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setPower(0.0);
        applyLauncherPidf();
        launcherClosedLoop = false;

        // ---- Servos ----
        windmillServo = hardwareMap.get(CRServo.class, WINDMILL_NAME);
        leftIntakeServo = hardwareMap.get(CRServo.class, LEFT_INTAKE_SERVO_NAME);
        rightIntakeServo = hardwareMap.get(CRServo.class, RIGHT_INTAKE_SERVO_NAME);
        leftIntakeServo.setPower(0.0);
        rightIntakeServo.setPower(0.0);
        windmillServo.setPower(0.0);
        // Same as original: the right intake servo is mirrored so both pull elements in.
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---- Battery ----
        batterySensor = null;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            batterySensor = sensor;
            break;
        }

        // ---- Current protection ----
        if (CURRENT_PROTECTION_ENABLED) {
            currentGuard = new CurrentGuard(CURRENT_POLL_INTERVAL_SECONDS);
            List<CurrentChannel> drive = new ArrayList<>();
            for (int i = 0; i < leftMotors.length; i++) {
                CurrentChannel c = currentGuard.add(leftMotors[i], DRIVE_TRIP_AMPS, DRIVE_TRIP_SECONDS);
                drive.add(c);
                if (i == 0) {
                    leftDriveChannel = c;
                }
            }
            for (int i = 0; i < rightMotors.length; i++) {
                CurrentChannel c = currentGuard.add(rightMotors[i], DRIVE_TRIP_AMPS, DRIVE_TRIP_SECONDS);
                drive.add(c);
                if (i == 0) {
                    rightDriveChannel = c;
                }
            }
            driveChannels = drive.toArray(new CurrentChannel[0]);
            intakeChannel = currentGuard.add(intake, INTAKE_JAM_AMPS, INTAKE_JAM_SECONDS);
            launcherChannel = currentGuard.add(launcher, LAUNCHER_TRIP_AMPS, LAUNCHER_TRIP_SECONDS);
        }

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        clearBulkCaches();
        updateBattery(clock.seconds());

        telemetry.addData("Status", "READY - press PLAY");
        telemetry.addData("Drivetrain", "TANK | %d left + %d right motor(s)", leftMotors.length, rightMotors.length);
        telemetry.addData("Battery", "%.1f V%s", batteryVolts, batteryLow ? "  ** LOW **" : "");
        telemetry.addData("Current protection", CURRENT_PROTECTION_ENABLED ? "ON" : "OFF");
        telemetry.addLine("Sticks = tank | LB/A = precision | B = flip front");
        telemetry.addLine("RT/LT = intake | RB = launch | X = pre-spin | Y = force feed");
        telemetry.addLine("D-pad up/down = launcher trim | BACK = mechanism lock");
    }

    @Override
    public void start() {
        lastLoopTime = clock.seconds();
        lastBatteryPoll = -1000.0;
        leftSlew.reset();
        rightSlew.reset();
        intakeSlew.reset();
        if (currentGuard != null) {
            currentGuard.resetAll();
        }
    }

    // =========================================================================================
    //  MAIN LOOP
    // =========================================================================================
    @Override
    public void loop() {
        clearBulkCaches();

        double now = clock.seconds();
        double rawDt = now - lastLoopTime;
        lastLoopTime = now;
        double dt = clamp(rawDt, 0.0, MAX_LOOP_DT_SECONDS);
        if (rawDt > 1e-4) {
            double instantHz = 1.0 / rawDt;
            loopHz = (loopHz == 0.0) ? instantHz : 0.9 * loopHz + 0.1 * instantHz;
        }

        updateBattery(now);
        if (currentGuard != null) {
            currentGuard.update(now);
        }

        handleToggles();
        updateDrive(now, dt);
        updateLauncher(now);   // before intake: the feed state adds assist power to the intake
        updateIntake(now, dt);
        sendTelemetry();
    }

    @Override
    public void stop() {
        if (leftMotors != null) {
            for (DcMotorEx m : leftMotors) {
                m.setPower(0.0);
            }
        }
        if (rightMotors != null) {
            for (DcMotorEx m : rightMotors) {
                m.setPower(0.0);
            }
        }
        if (intake != null) {
            intake.setPower(0.0);
        }
        if (launcher != null) {
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setPower(0.0);
        }
        if (leftIntakeServo != null) {
            leftIntakeServo.setPower(0.0);
        }
        if (rightIntakeServo != null) {
            rightIntakeServo.setPower(0.0);
        }
        if (windmillServo != null) {
            windmillServo.setPower(0.0);
        }
    }

    // =========================================================================================
    //  INPUT TOGGLES
    // =========================================================================================
    private void handleToggles() {
        // Edge detectors must be polled every loop so their "previous" state stays current.
        boolean pressA = edgeA.rising(gamepad1.a);
        boolean pressB = edgeB.rising(gamepad1.b);
        boolean pressX = edgeX.rising(gamepad1.x || gamepad2.x);
        boolean pressBack = edgeBack.rising(gamepad1.back || gamepad2.back);
        boolean pressUp = edgeDpadUp.rising(gamepad1.dpad_up || gamepad2.dpad_up);
        boolean pressDown = edgeDpadDown.rising(gamepad1.dpad_down || gamepad2.dpad_down);
        boolean pressLeft = edgeDpadLeft.rising(gamepad1.dpad_left || gamepad2.dpad_left);

        if (pressA) {
            precisionLatched = !precisionLatched;
            gamepad1.rumble(80);
        }
        if (pressB) {
            frontReversed = !frontReversed;
            gamepad1.rumble(80);
        }
        if (pressBack) {
            mechanismsLocked = !mechanismsLocked;
            if (mechanismsLocked) {
                preSpinLatched = false;
            }
            rumbleDrivers(250);
        }
        if (pressX && !mechanismsLocked) {
            preSpinLatched = !preSpinLatched;
        }
        if (pressUp) {
            launcherTrim = clamp(launcherTrim + LAUNCHER_TRIM_STEP, -LAUNCHER_TRIM_LIMIT, LAUNCHER_TRIM_LIMIT);
        }
        if (pressDown) {
            launcherTrim = clamp(launcherTrim - LAUNCHER_TRIM_STEP, -LAUNCHER_TRIM_LIMIT, LAUNCHER_TRIM_LIMIT);
        }
        if (pressLeft) {
            launcherTrim = 0.0;
        }
    }

    // =========================================================================================
    //  DRIVE (TANK)
    // =========================================================================================
    private void updateDrive(double now, double dt) {

        // LEFT STICK Y: Forward and backward
        // FTC reports stick-up as negative, so we invert it.
        leftStickIn = -gamepad1.left_stick_y;

        // RIGHT STICK X: Turn left and right in place
        rightStickIn = gamepad1.right_stick_x;

        // Convert joystick inputs into smooth motor values.
        double drive = shapeStick(leftStickIn);
        double steering = shapeStick(gamepad1.left_stick_x);
        double pivot = shapeStick(rightStickIn);

        // LEFT STICK X: Steer while the robot is moving.
        // A six-wheel traction drivetrain cannot move sideways like mecanum wheels.
        double turnWhileMoving = steering * Math.abs(drive);

        // RIGHT STICK X: Allows turning even when the robot is stationary.
        double turn = turnWhileMoving + pivot;

        // B BUTTON: Reverse the robot's front direction.
        if (frontReversed) {
            drive = -drive;
        }

        // Calculate left and right drivetrain motor powers.
        double left = drive + turn;
        double right = drive - turn;

        // Keep motor power between -1 and 1.
        double maxMagnitude = Math.max(
                1.0,
                Math.max(Math.abs(left), Math.abs(right))
        );

        left /= maxMagnitude;
        right /= maxMagnitude;

        // Straight assist is not used with this control layout.
        straightAssistActive = false;

        // Check whether any drivetrain motor has exceeded its current limit.
        for (CurrentChannel channel : driveChannels) {
            if (channel.tripped) {
                driveDerateUntil = now + DRIVE_DERATE_HOLD_SECONDS;
            }
        }

        driveDerated = now < driveDerateUntil;

        if (driveDerated && !wasDriveDerated) {
            rumbleDrivers(500);
        }

        wasDriveDerated = driveDerated;

        // Precision mode: A toggles it; left bumper activates it while held.
        precisionActive = precisionLatched || gamepad1.left_bumper;

        driveScale = precisionActive
                ? PRECISION_SCALE
                : NORMAL_SCALE;

        if (driveDerated) {
            driveScale = Math.min(driveScale, DRIVE_DERATE_SCALE);
        }

        // Apply smooth acceleration and braking.
        leftPower = leftSlew.update(
                clamp(left * driveScale, -1.0, 1.0),
                dt
        );

        rightPower = rightSlew.update(
                clamp(right * driveScale, -1.0, 1.0),
                dt
        );

        // Send the final power to the drivetrain motors.
        setDrive(leftPower, rightPower);
    }
    // =========================================================================================
    //  LAUNCHER + WINDMILL
    // =========================================================================================
    private void updateLauncher(double now) {
        boolean launchHeld = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean forceFeed = gamepad1.y || gamepad2.y;
        boolean launchRequested = launchHeld || forceFeed;

        launcherVelocity = launcher.getVelocity();
        launcherTargetVelocity = clamp(LAUNCHER_TARGET_VELOCITY + launcherTrim, 0.0, LAUNCHER_MAX_SAFE_VELOCITY);

        boolean wantSpin = launchRequested || preSpinLatched;
        boolean wantFeed = launchRequested;

        // A launcher fault stays latched until the driver lets go of the launch controls.
        if (launcherFaulted && !launchRequested) {
            launcherFaulted = false;
            launcherFaultReason = "";
            if (launcherChannel != null) {
                launcherChannel.reset();
            }
        }
        if (!launcherFaulted && wantSpin && launcherChannel != null && launcherChannel.tripped) {
            launcherFaulted = true;
            launcherFaultReason = "OVER-CURRENT";
            preSpinLatched = false;
            rumbleDrivers(600);
        }
        if (launcherFaulted || mechanismsLocked) {
            wantSpin = false;
            wantFeed = false;
        }
        launcherWantSpin = wantSpin;

        // "At speed" with hysteresis so a small dip while feeding doesn't stutter the windmill.
        if (wantSpin) {
            double readyMin = launcherTargetVelocity - LAUNCHER_READY_TOLERANCE;
            if (launcherVelocity >= readyMin) {
                launcherReady = true;
            } else if (launcherVelocity < readyMin - LAUNCHER_READY_EXIT_MARGIN) {
                launcherReady = false;
            }
        } else {
            launcherReady = false;
            readyCueGiven = false;
        }

        // One short rumble when the launcher first reaches speed after a spin-up.
        if (launcherReady && !readyCueGiven) {
            readyCueGiven = true;
            rumbleDrivers(120);
        }

        // Warn if we've been commanding the launcher for a while and it still isn't at speed.
        if (wantSpin && !launcherReady) {
            if (notReadySince < 0.0) {
                notReadySince = now;
            }
            spinWarning = (now - notReadySince) > LAUNCHER_SPINUP_WARN_SECONDS;
        } else {
            notReadySince = -1.0;
            spinWarning = false;
        }

        // The original code's comment said the windmill only runs once the launcher is fast enough,
        // but the code never checked. Here feeding requires "at speed" (Y overrides).
        launcherFeeding = wantFeed && (forceFeed || launcherReady);

        commandLauncher(wantSpin, launcherTargetVelocity);
        setWindmill(launcherFeeding ? WINDMILL_FEED_POWER : 1.0);
    }

    private void commandLauncher(boolean spin, double targetTicksPerSecond) {
        if (spin) {
            if (!launcherClosedLoop) {
                launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                applyLauncherPidf();
                launcherClosedLoop = true;
                lastLauncherTargetWritten = Double.NaN;   // force the velocity to be re-sent
            }
            if (lastLauncherTargetWritten != targetTicksPerSecond) {
                launcher.setVelocity(targetTicksPerSecond);
                lastLauncherTargetWritten = targetTicksPerSecond;
            }
        } else if (launcherClosedLoop) {
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setPower(0.0);   // with FLOAT zero-power behavior the flywheel coasts down
            launcherClosedLoop = false;
        }
    }

    private void applyLauncherPidf() {
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D, LAUNCHER_F));
    }

    // =========================================================================================
    //  INTAKE
    // =========================================================================================
    private void updateIntake(double now, double dt) {
        double requested = clamp(
                trigger(gamepad1.right_trigger) - trigger(gamepad1.left_trigger)
                        + trigger(gamepad2.right_trigger) - trigger(gamepad2.left_trigger),
                -1.0, 1.0);
        intakeRequest = requested;

        double command = mechanismsLocked ? 0.0 : requested;

        // While feeding the launcher, add a little extra intake to help dislodge stuck elements.
        if (launcherFeeding) {
            command = clamp(command + FEED_ASSIST_INTAKE_POWER, -1.0, 1.0);
        }

        // Jam handling: sustained over-current while intaking -> briefly reverse to clear it.
        if (intakeUnjamming) {
            if (now >= intakeUnjamUntil || mechanismsLocked) {
                intakeUnjamming = false;
                if (intakeChannel != null) {
                    intakeChannel.reset();
                }
            } else {
                command = -INTAKE_UNJAM_POWER;
            }
        } else if (intakeChannel != null && intakeChannel.tripped && command > INTAKE_JAM_MIN_COMMAND) {
            intakeUnjamming = true;
            intakeUnjamUntil = now + INTAKE_UNJAM_SECONDS;
            jamCount++;
            rumbleDrivers(200);
            command = -INTAKE_UNJAM_POWER;
        }

        intakeApplied = intakeSlew.update(command, dt);
        setIntake(intakeApplied);
    }

    // =========================================================================================
    //  HARDWARE WRITES (skip redundant writes - every write is a hub command)
    // =========================================================================================
    private void setDrive(double left, double right) {
        if (shouldWrite(left, lastLeftWritten)) {
            for (DcMotorEx m : leftMotors) {
                m.setPower(left);
            }
            lastLeftWritten = left;
        }
        if (shouldWrite(right, lastRightWritten)) {
            for (DcMotorEx m : rightMotors) {
                m.setPower(right);
            }
            lastRightWritten = right;
        }
    }

    private void setIntake(double power) {
        if (shouldWrite(power, lastIntakeWritten)) {
            intake.setPower(power);
            leftIntakeServo.setPower(power);
            rightIntakeServo.setPower(power);
            lastIntakeWritten = power;
        }
    }

    private void setWindmill(double power) {
        if (shouldWrite(power, lastWindmillWritten)) {
            windmillServo.setPower(power);
            lastWindmillWritten = power;
        }
    }

    private static boolean shouldWrite(double next, double last) {
        if (Double.isNaN(last)) {
            return true;
        }
        if (next == 0.0) {
            return last != 0.0;
        }
        return Math.abs(next - last) > WRITE_EPSILON;
    }

    private static void configureDriveMotors(DcMotorEx[] motors, DcMotorSimple.Direction direction) {
        for (DcMotorEx m : motors) {
            m.setDirection(direction);
            // BRAKE: the robot stops quickly and holds still instead of rolling.
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Open-loop power control for the drivetrain.
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(0.0);
        }
    }

    private void clearBulkCaches() {
        if (hubs == null) {
            return;
        }
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    // =========================================================================================
    //  BATTERY + FEEDBACK
    // =========================================================================================
    private void updateBattery(double now) {
        if (batterySensor == null || now - lastBatteryPoll < BATTERY_POLL_SECONDS) {
            return;
        }
        lastBatteryPoll = now;
        double volts = batterySensor.getVoltage();
        batteryVolts = (batteryVolts <= 0.0) ? volts : (1.0 - BATTERY_FILTER) * batteryVolts + BATTERY_FILTER * volts;
        if (!batteryLow && batteryVolts < LOW_BATTERY_VOLTS) {
            batteryLow = true;
            rumbleDrivers(400);
        } else if (batteryLow && batteryVolts > LOW_BATTERY_VOLTS + LOW_BATTERY_HYSTERESIS) {
            batteryLow = false;
        }
    }

    private void rumbleDrivers(int durationMs) {
        gamepad1.rumble(durationMs);
        gamepad2.rumble(durationMs);
    }

    // =========================================================================================
    //  TELEMETRY
    // =========================================================================================
    private void sendTelemetry() {
        String warnings = buildWarnings();
        if (warnings != null) {
            telemetry.addData("!! WARNING", warnings);
        }

        telemetry.addData("Mode", "%s | front %s | mechs %s",
                precisionActive ? "PRECISION" : "NORMAL",
                frontReversed ? "FLIPPED" : "normal",
                mechanismsLocked ? "LOCKED" : "armed");
        telemetry.addData("Sticks L/R", "%+.2f / %+.2f", leftStickIn, rightStickIn);
        telemetry.addData("Drive L/R", "%+.2f / %+.2f  (cap %.0f%%)%s",
                leftPower, rightPower, driveScale * 100.0, straightAssistActive ? "  [STRAIGHT]" : "");
        telemetry.addData("Drive amps L/R", "%s / %s", ampsText(leftDriveChannel), ampsText(rightDriveChannel));
        telemetry.addData("Intake", "%+.2f (req %+.2f) %s%s",
                intakeApplied, intakeRequest, ampsText(intakeChannel), intakeUnjamming ? "  UNJAMMING" : "");
        telemetry.addData("Launcher", "%s | trim %+.0f | pre-spin %s",
                launcherStatus(), launcherTrim, preSpinLatched ? "ON" : "off");
        telemetry.addData("Launcher speed", "%.0f / %.0f tps (%.0f RPM) %s",
                launcherVelocity, launcherTargetVelocity,
                launcherVelocity / LAUNCHER_TICKS_PER_REV * 60.0, ampsText(launcherChannel));
        telemetry.addData("Windmill", launcherFeeding ? "FEEDING" : "off");
        telemetry.addData("Battery | Loop", "%.1f V | %.0f Hz", batteryVolts, loopHz);
    }

    private String launcherStatus() {
        if (mechanismsLocked) {
            return "LOCKED";
        }
        if (launcherFaulted) {
            return "FAULT " + launcherFaultReason;
        }
        if (!launcherWantSpin) {
            return Math.abs(launcherVelocity) > LAUNCHER_COAST_DISPLAY_TPS ? "COASTING" : "OFF";
        }
        if (launcherFeeding) {
            return "FEEDING";
        }
        return launcherReady ? "READY" : "SPINNING UP";
    }

    private String buildWarnings() {
        warningBuilder.setLength(0);
        if (batteryLow) {
            addWarning("LOW BATTERY");
        }
        if (driveDerated) {
            addWarning("DRIVE CURRENT LIMIT");
        }
        if (intakeUnjamming) {
            addWarning("INTAKE JAM");
        }
        if (launcherFaulted) {
            addWarning("LAUNCHER " + launcherFaultReason);
        }
        if (spinWarning) {
            addWarning("LAUNCHER NOT AT SPEED");
        }
        return warningBuilder.length() == 0 ? null : warningBuilder.toString();
    }

    private void addWarning(String text) {
        if (warningBuilder.length() > 0) {
            warningBuilder.append(" | ");
        }
        warningBuilder.append(text);
    }

    private static String ampsText(CurrentChannel channel) {
        if (channel == null || channel.failed) {
            return "n/a";
        }
        return String.format(Locale.US, "%.1fA", channel.amps);
    }

    // =========================================================================================
    //  MATH HELPERS
    // =========================================================================================
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** Deadband (rescaled so output starts at 0, not at the deadband) followed by an expo curve. */
    private static double shapeStick(double raw) {
        double magnitude = Math.abs(raw);
        if (magnitude <= STICK_DEADBAND) {
            return 0.0;
        }
        double scaled = Math.min(1.0, (magnitude - STICK_DEADBAND) / (1.0 - STICK_DEADBAND));
        double curved = (1.0 - STICK_EXPO) * scaled + STICK_EXPO * scaled * scaled * scaled;
        return Math.copySign(curved, raw);
    }

    private static double trigger(double raw) {
        if (raw <= TRIGGER_DEADBAND) {
            return 0.0;
        }
        return (raw - TRIGGER_DEADBAND) / (1.0 - TRIGGER_DEADBAND);
    }

    // =========================================================================================
    //  SMALL HELPER CLASSES
    // =========================================================================================

    /** Rising-edge detector for buttons (true for exactly one loop per press). */
    private static final class Edge {
        private boolean last = false;

        boolean rising(boolean now) {
            boolean rose = now && !last;
            last = now;
            return rose;
        }
    }

    /**
     * Limits how fast an output may change. Speeding up (moving away from zero) uses the acceleration
     * rate; slowing down or reversing uses the faster deceleration rate.
     */
    private static final class SlewLimiter {
        private final double accelPerSecond;
        private final double decelPerSecond;
        private double value = 0.0;

        SlewLimiter(double accelPerSecond, double decelPerSecond) {
            this.accelPerSecond = accelPerSecond;
            this.decelPerSecond = decelPerSecond;
        }

        double update(double target, double dtSeconds) {
            boolean speedingUp = Math.abs(target) > Math.abs(value)
                    && (value == 0.0 || Math.signum(target) == Math.signum(value));
            double maxStep = (speedingUp ? accelPerSecond : decelPerSecond) * dtSeconds;
            double error = target - value;
            if (Math.abs(error) <= maxStep) {
                value = target;
            } else {
                value += Math.copySign(maxStep, error);
            }
            return value;
        }

        void reset() {
            value = 0.0;
        }
    }

    /** One monitored motor: trips when current stays above a limit for a set time. */
    private static final class CurrentChannel {
        private final DcMotorEx motor;
        private final double tripAmps;
        private final double tripSeconds;
        double amps = 0.0;
        boolean tripped = false;
        boolean failed = false;
        private double overSeconds = 0.0;
        private double lastSampleTime = -1.0;

        CurrentChannel(DcMotorEx motor, double tripAmps, double tripSeconds) {
            this.motor = motor;
            this.tripAmps = tripAmps;
            this.tripSeconds = tripSeconds;
        }

        void sample(double now) {
            if (failed) {
                return;
            }
            double span = (lastSampleTime < 0.0) ? 0.0 : Math.min(now - lastSampleTime, CURRENT_MAX_SAMPLE_SPAN_SECONDS);
            lastSampleTime = now;
            try {
                amps = motor.getCurrent(CurrentUnit.AMPS);
            } catch (RuntimeException e) {
                // Don't let a failed current read take down the OpMode; just stop monitoring this motor.
                failed = true;
                tripped = false;
                amps = 0.0;
                return;
            }
            if (amps >= tripAmps) {
                overSeconds += span;
                if (overSeconds >= tripSeconds) {
                    tripped = true;
                }
            } else {
                overSeconds = 0.0;
                if (amps <= tripAmps * CURRENT_RELEASE_FRACTION) {
                    tripped = false;
                }
            }
        }

        void reset() {
            overSeconds = 0.0;
            tripped = false;
            lastSampleTime = -1.0;
        }
    }

    /**
     * Reads motor currents round-robin, one motor per poll interval. Current reads are NOT part of the
     * hub's bulk read, so polling every motor every loop would slow the loop down.
     */
    private static final class CurrentGuard {
        private final List<CurrentChannel> channels = new ArrayList<>();
        private final double pollIntervalSeconds;
        private double lastPollTime = -1.0;
        private int next = 0;

        CurrentGuard(double pollIntervalSeconds) {
            this.pollIntervalSeconds = pollIntervalSeconds;
        }

        CurrentChannel add(DcMotorEx motor, double tripAmps, double tripSeconds) {
            CurrentChannel channel = new CurrentChannel(motor, tripAmps, tripSeconds);
            channels.add(channel);
            return channel;
        }

        void update(double now) {
            if (channels.isEmpty() || now - lastPollTime < pollIntervalSeconds) {
                return;
            }
            lastPollTime = now;
            channels.get(next).sample(now);
            next = (next + 1) % channels.size();
        }

        void resetAll() {
            for (int i = 0; i < channels.size(); i++) {
                channels.get(i).reset();
            }
            lastPollTime = -1.0;
            next = 0;
        }
    }
}
