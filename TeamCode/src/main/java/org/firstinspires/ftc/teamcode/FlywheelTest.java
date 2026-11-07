package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * BIOBUZZ StarterBot - 6-wheel tank drive (6WD drop-center) autonomous.
 *
 * Hardware (encoders only - no odometry pods, color sensors or cameras):
 *   left_front, left_back, right_front, right_back  - drive motors with encoders
 *   intake_motor   - ground intake roller
 *   feeder_servo   - continuous-rotation "windmill" that pushes pollen into the launcher
 *   launcher_motor - 6,000 RPM motor driving the 96 mm hogback launcher wheel
 *
 * ASSUMPTIONS (change the constants below if your robot or field differs):
 *   1. "Forward" = the end with the intake AND the launcher (positive power drives toward
 *      the intake / toward the Hive goal). The robot starts with that end against the goal
 *      wall, matching the stock StarterBotAuto which backs away from the wall.
 *   2. The pollen piles lie along the side wall, running away from the goal wall.
 *   3. EVERY DISTANCE BELOW IS A PLACEHOLDER. Measure them on your field and tune.
 *
 * ROUTE (all dead reckoning off the drive encoders):
 *   Step 1  shoot the 4 preloaded pollen from the start pose
 *   Step 2  back off the goal wall -> turn to the side wall -> drive to it -> turn to run
 *           along it -> intake running while driving through pile 1
 *   Step 3  retrace the exact same legs in reverse back to the start pose, square up on the
 *           goal wall, shoot
 *   Step 4/5 same again, but run farther along the wall to pile 2
 *   Step 6  park
 *
 * Because every outbound leg is mirrored on the way back, the only things you tune are the
 * leg lengths - there is no separate "return" geometry to calibrate.
 */
@Autonomous(name = "BioBuzz 6WD Auto", group = "StarterBot")
public class FlywheelTest extends LinearOpMode {

    // =====================================================================================
    //                                  TUNABLE CONSTANTS
    // =====================================================================================

    // ---- Field side ----------------------------------------------------------------------
    // true  = the side wall is on the robot's LEFT as it faces the goal at the start.
    // false = it is on the RIGHT. Every turn toward/away from the wall is mirrored by this.
    private static final boolean SIDE_WALL_ON_LEFT = true;
    private static final int WALL = SIDE_WALL_ON_LEFT ? 1 : -1;   // +1 = turn left, -1 = right

    // ---- Match timing --------------------------------------------------------------------
    private static final double AUTO_DURATION_S = 30.0;
    // If less than this many seconds remain after score cycle 1, skip cycle 2 and just park.
    // Cycle 2 is the longest one (farther pile), so tune this after timing a real run.
    private static final double CYCLE_2_MIN_TIME_S = 16.0;

    // ---- Launcher (velocity values taken from the stock StarterBotAuto) ------------------
    // Ticks per second; the motor has 28 ticks/rev, so 1250 is roughly 2,680 RPM at the motor.
    // The motor tops out near 2,800 ticks/s. Velocity control keeps shot distance consistent
    // as the battery sags, which raw full power would not.
    private static final double LAUNCHER_TARGET_VELOCITY = 1250;
    private static final double LAUNCHER_MIN_VELOCITY = 1200;     // "ready to feed" threshold
    private static final double LAUNCHER_SPINUP_TIMEOUT_S = 3.0;
    private static final double LAUNCHER_RECOVERY_TIMEOUT_S = 1.0; // wait between shots

    // ---- Feeder / hopper -----------------------------------------------------------------
    private static final double FEED_POWER = 1.0;
    // One pulse of the windmill should deliver exactly one pollen. TUNE THIS on the robot.
    private static final long FEED_PULSE_MS = 450;
    // The stock auto runs the intake while launching to help push pollen forward. Set to 0
    // if your hopper feeds fine without it.
    private static final double INTAKE_ASSIST_POWER = 0.5;
    private static final long LAST_SHOT_CLEAR_MS = 300;            // let the last pollen leave

    // ---- Intake --------------------------------------------------------------------------
    private static final double INTAKE_POWER = 1.0;
    private static final long INTAKE_DWELL_MS = 600;               // keep rolling after the pile

    // ---- Pollen counts -------------------------------------------------------------------
    private static final int PRELOAD_COUNT = 4;
    private static final int PILE_1_COUNT = 3;   // set to what pile 1 really holds
    private static final int PILE_2_COUNT = 3;   // set to what pile 2 really holds

    // ---- Drivetrain geometry (defaults from the stock StarterBotAuto) --------------------
    private static final double WHEEL_DIAMETER_MM = 96;
    private static final double ENCODER_TICKS_PER_REV = 537.7;     // 312 RPM goBILDA motor
    private static final double DRIVE_MOTOR_RPM = 312;
    // Distance between the left and right wheel centers. A drop-center 6WD scrubs on turns,
    // so tune this empirically: if the robot UNDER-rotates, increase it; over-rotates, decrease.
    private static final double TRACK_WIDTH_MM = 402;

    private static final double TICKS_PER_MM =
            ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);
    // In-place turn: each wheel travels (pi * track / 360) mm per degree of robot rotation.
    private static final double TICKS_PER_DEGREE =
            TICKS_PER_MM * (Math.PI * TRACK_WIDTH_MM / 360.0);
    private static final double MAX_TICKS_PER_SEC = ENCODER_TICKS_PER_REV * DRIVE_MOTOR_RPM / 60.0;

    // ---- Drive speeds (0..1) -------------------------------------------------------------
    private static final double TRAVEL_POWER = 0.6;   // moving between points
    private static final double COLLECT_POWER = 0.35; // slow through the pile so the intake keeps up
    private static final double TURN_POWER = 0.4;
    private static final long SETTLE_MS = 150;        // pause after each move so it stops rolling

    // ---- Route distances in mm (PLACEHOLDERS - MEASURE AND TUNE) -------------------------
    // Reverse straight back from the goal wall so the robot can rotate without touching it.
    private static final double BACK_OFF_MM = 350;
    // Drive from the back-off point to the side wall. TUNE SO THE ROBOT'S CENTER ENDS AT LEAST
    // HALF ITS DIAGONAL FROM THE WALL - otherwise a corner clips the wall during the next turn.
    private static final double TO_SIDE_WALL_MM = 700;
    // Along the wall: fast "approach" up to the pile, then slow "collect" through it.
    private static final double PILE_1_APPROACH_MM = 200;
    private static final double PILE_1_COLLECT_MM = 450;
    private static final double PILE_2_APPROACH_MM = 700;
    private static final double PILE_2_COLLECT_MM = 450;

    // ---- Squaring up on the goal wall ----------------------------------------------------
    // The robot starts against the goal wall, so after each return trip we nudge into it at
    // low power. This removes heading and distance drift before the shot.
    private static final boolean SQUARE_UP_ON_GOAL_WALL = true;
    private static final double SQUARE_UP_POWER = 0.25;
    private static final long SQUARE_UP_MS = 500;

    // ---- Parking (PLACEHOLDERS - take the zone location from the game manual) ------------
    private static final double PARK_BACK_OFF_MM = 450;  // reverse away from the goal wall
    private static final double PARK_TURN_DEG = 0;       // + turns toward the side wall
    private static final double PARK_DRIVE_MM = 0;       // then drive forward this far

    // =====================================================================================
    //                                    HARDWARE
    // =====================================================================================

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor[] driveMotors;
    private DcMotor intake;
    private DcMotorEx launcher;   // DcMotorEx gives us setVelocity()/getVelocity()
    private CRServo feeder;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private String currentStep = "Init";

    // =====================================================================================
    //                                  MAIN SEQUENCE
    // =====================================================================================

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("READY - robot against goal wall, 4 pollen loaded");
        telemetry.addData("Side wall", SIDE_WALL_ON_LEFT ? "LEFT" : "RIGHT");
        telemetry.update();

        waitForStart();                 // nothing moves until the driver presses START
        if (!opModeIsActive()) return;  // STOP pressed during init
        autoTimer.reset();

        try {
            // ---- Step 1: score the 4 preloaded pollen from the start pose ----------------
            setStep("1: Preload scoring");
            shootPollen(PRELOAD_COUNT);

            // ---- Step 2: first pile along the side wall ----------------------------------
            setStep("2: Wall intake 1");
            leaveGoalAndLineUpWithWall();
            collectAlongWall(PILE_1_APPROACH_MM, PILE_1_COLLECT_MM);

            // ---- Step 3: back to the start pose and score --------------------------------
            setStep("3: Score cycle 1");
            returnToShootingPosition(PILE_1_APPROACH_MM + PILE_1_COLLECT_MM);
            shootPollen(PILE_1_COUNT);

            // ---- Steps 4 & 5: second pile, only if there is time to finish AND park ------
            if (timeRemaining() >= CYCLE_2_MIN_TIME_S) {
                setStep("4: Wall intake 2");
                leaveGoalAndLineUpWithWall();
                collectAlongWall(PILE_2_APPROACH_MM, PILE_2_COLLECT_MM);

                setStep("5: Score cycle 2");
                returnToShootingPosition(PILE_2_APPROACH_MM + PILE_2_COLLECT_MM);
                shootPollen(PILE_2_COUNT);
            } else {
                telemetry.addLine("Skipping cycle 2 - not enough time, parking instead");
                telemetry.update();
            }

            // ---- Step 6: park ------------------------------------------------------------
            setStep("6: Park");
            parkRobot();

            setStep("Auto complete");
        } finally {
            // Runs on normal completion AND if STOP is pressed mid-routine: nothing keeps
            // spinning once we leave this OpMode.
            stopEverything();
        }
    }

    // =====================================================================================
    //                                   ROUTE SEGMENTS
    // =====================================================================================

    /**
     * Leaves the start pose and ends up beside the side wall, facing along it (away from the
     * goal wall). Out-and-back legs are exact mirrors of returnToShootingPosition().
     */
    private void leaveGoalAndLineUpWithWall() {
        driveMm(TRAVEL_POWER, -BACK_OFF_MM);        // clear the goal wall
        turnDegrees(TURN_POWER, 90 * WALL);         // face the side wall
        driveMm(TRAVEL_POWER, TO_SIDE_WALL_MM);     // cross to the side wall
        turnDegrees(TURN_POWER, 90 * WALL);         // now facing along the wall, away from goal
    }

    /**
     * Drives along the wall to a pile: fast to the pile, then slow with the intake rolling.
     */
    private void collectAlongWall(double approachMm, double collectMm) {
        driveMm(TRAVEL_POWER, approachMm);          // intake off while just traveling
        intake.setPower(INTAKE_POWER);              // roller on BEFORE touching the pile
        driveMm(COLLECT_POWER, collectMm);          // slow sweep through the pile
        runIntake(INTAKE_POWER, INTAKE_DWELL_MS);   // dwell so the last pollen seats, then stop
    }

    /**
     * Retraces the outbound path in reverse: back along the wall, away from the wall, then
     * forward into the start pose. Spins the launcher up on the last leg so it is ready the
     * moment we arrive (saves ~1 s per cycle).
     */
    private void returnToShootingPosition(double alongWallMm) {
        driveMm(TRAVEL_POWER, -alongWallMm);        // reverse back along the wall
        turnDegrees(TURN_POWER, -90 * WALL);
        driveMm(TRAVEL_POWER, -TO_SIDE_WALL_MM);    // reverse away from the side wall
        turnDegrees(TURN_POWER, -90 * WALL);        // facing the goal again
        spinUpLauncher();                           // overlap spin-up with the final approach
        driveMm(TRAVEL_POWER, BACK_OFF_MM);         // forward to the goal wall
        squareUpOnGoalWall();
    }

    private void parkRobot() {
        driveMm(TRAVEL_POWER, -PARK_BACK_OFF_MM);
        if (PARK_TURN_DEG != 0) turnDegrees(TURN_POWER, PARK_TURN_DEG * WALL);
        if (PARK_DRIVE_MM != 0) driveMm(TRAVEL_POWER, PARK_DRIVE_MM);
        stopEverything();
    }

    // =====================================================================================
    //                                  SCORING / INTAKE
    // =====================================================================================

    /**
     * Fires 'count' pollen: spin up, feed one pollen per windmill pulse (waiting for the
     * launcher to recover between shots), then spin the launcher down.
     */
    private void shootPollen(int count) {
        if (count <= 0) return;

        spinUpLauncher();                                      // no-op if already spinning
        waitForLauncherReady(LAUNCHER_SPINUP_TIMEOUT_S);

        for (int shot = 1; shot <= count && opModeIsActive(); shot++) {
            // A shot drops the wheel speed; wait for it to recover so every pollen flies the
            // same distance. The timeout guarantees we never stall here forever.
            waitForLauncherReady(LAUNCHER_RECOVERY_TIMEOUT_S);

            feeder.setPower(FEED_POWER);
            intake.setPower(INTAKE_ASSIST_POWER);
            pause(FEED_PULSE_MS);
            feeder.setPower(0);
            intake.setPower(0);

            telemetry.addData("Step", currentStep);
            telemetry.addData("Shots fired", "%d / %d", shot, count);
            telemetry.update();
        }

        pause(LAST_SHOT_CLEAR_MS);   // don't cut the wheel while the last pollen is still in it
        spinDownLauncher();
    }

    /** Runs the intake at 'power' for 'durationMs', then stops it. */
    private void runIntake(double power, long durationMs) {
        intake.setPower(power);
        pause(durationMs);
        intake.setPower(0);
    }

    private void spinUpLauncher() {
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
    }

    private void spinDownLauncher() {
        launcher.setVelocity(0);
    }

    /** Blocks until the launcher reaches shooting speed or the timeout expires. */
    private void waitForLauncherReady(double timeoutS) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()
                && launcher.getVelocity() < LAUNCHER_MIN_VELOCITY
                && timer.seconds() < timeoutS) {
            telemetry.addData("Step", currentStep);
            telemetry.addData("Launcher velocity", "%.0f / %.0f",
                    launcher.getVelocity(), LAUNCHER_TARGET_VELOCITY);
            telemetry.update();
            idle();
        }
    }

    // =====================================================================================
    //                                  MOVEMENT HELPERS
    // =====================================================================================

    /** Drives straight. Positive ticks = forward, negative = reverse. */
    private void driveStraight(double power, int encoderTicks) {
        moveByTicks(power, encoderTicks, encoderTicks);
    }

    /**
     * Rotates in place. Positive ticks = counter-clockwise (left turn): the left side runs
     * backward while the right side runs forward. Negative = clockwise (right turn).
     */
    private void turn(double power, int encoderTicks) {
        moveByTicks(power, -encoderTicks, encoderTicks);
    }

    private void driveMm(double power, double mm) {
        driveStraight(power, (int) Math.round(mm * TICKS_PER_MM));
    }

    /** Positive degrees = left (counter-clockwise). */
    private void turnDegrees(double power, double degrees) {
        turn(power, (int) Math.round(degrees * TICKS_PER_DEGREE));
    }

    /**
     * Core RUN_TO_POSITION move. Targets are RELATIVE to each motor's current position, so we
     * never need to reset encoders between moves. 'power' is a magnitude - the motor
     * controller picks the direction from the sign of the target.
     *
     * Safety: loops only while the OpMode is active, and a timeout derived from the move size
     * ends any move that stalls (e.g. wedged against a wall) instead of hanging the auto.
     */
    private void moveByTicks(double power, int leftTicks, int rightTicks) {
        if (!opModeIsActive()) return;

        power = Math.min(1.0, Math.abs(power));
        int largestMove = Math.max(Math.abs(leftTicks), Math.abs(rightTicks));
        double timeoutS = 1.0 + 2.0 * largestMove / (MAX_TICKS_PER_SEC * Math.max(power, 0.1));

        // Set targets BEFORE switching to RUN_TO_POSITION (the SDK requires this order).
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + leftTicks);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + leftTicks);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + rightTicks);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + rightTicks);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(power);

        ElapsedTime moveTimer = new ElapsedTime();
        while (opModeIsActive() && anyDriveBusy() && moveTimer.seconds() < timeoutS) {
            telemetry.addData("Step", currentStep);
            telemetry.addData("Left  now/target", "%d / %d",
                    leftFront.getCurrentPosition(), leftFront.getTargetPosition());
            telemetry.addData("Right now/target", "%d / %d",
                    rightFront.getCurrentPosition(), rightFront.getTargetPosition());
            telemetry.addData("Time left", "%.1f s", timeRemaining());
            telemetry.update();
            idle();
        }

        setDrivePower(0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pause(SETTLE_MS);
    }

    /** Low-power timed push into the goal wall to square up (see SQUARE_UP_* constants). */
    private void squareUpOnGoalWall() {
        if (!SQUARE_UP_ON_GOAL_WALL || !opModeIsActive()) return;
        // Open-loop on purpose: velocity regulation would ramp power up against a stalled wheel.
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setDrivePower(SQUARE_UP_POWER);
        pause(SQUARE_UP_MS);
        setDrivePower(0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean anyDriveBusy() {
        for (DcMotor m : driveMotors) {
            if (m.isBusy()) return true;
        }
        return false;
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        for (DcMotor m : driveMotors) m.setMode(mode);
    }

    private void setDrivePower(double power) {
        for (DcMotor m : driveMotors) m.setPower(power);
    }

    // =====================================================================================
    //                                 INIT / UTILITIES
    // =====================================================================================

    private void initHardware() {
        // The strings must match the names in the Robot Controller configuration exactly.
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        feeder = hardwareMap.get(CRServo.class, "feeder_servo");

        driveMotors = new DcMotor[]{leftFront, leftBack, rightFront, rightBack};

        // The two sides face opposite ways, so one side is reversed to make positive power
        // drive the robot forward. Same convention as the stock StarterBot. Verify on your
        // first test drive - if the robot goes backward, swap FORWARD/REVERSE on both sides.
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Zero the encoders once, then run closed-loop. BRAKE makes the robot stop sharply,
        // which keeps dead reckoning repeatable.
        for (DcMotor m : driveMotors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Launcher uses velocity control. PIDF values are the stock StarterBot's.
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        // Stock StarterBot reverses the windmill servo; flip if yours pushes pollen backward.
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setPower(0);   // also wakes the servo controller
    }

    /** Stops every mechanism. Safe to call repeatedly. */
    private void stopEverything() {
        setDrivePower(0);
        intake.setPower(0);
        feeder.setPower(0);
        launcher.setVelocity(0);
    }

    private double timeRemaining() {
        return AUTO_DURATION_S - autoTimer.seconds();
    }

    private void setStep(String step) {
        currentStep = step;
        telemetry.addData("Step", step);
        telemetry.update();
    }

    /** Non-blocking-friendly wait: exits immediately if the OpMode is stopped. */
    private void pause(long ms) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < ms) {
            idle();
        }
    }
}