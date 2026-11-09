package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Launcher PIDF Tuner", group = "Tuning")
public class LauncherTuner extends OpMode {

    private DcMotorEx launcher;
    private double p = 10.0, i = 0.0, d = 0.0, f = 12.5;
    private double target = 2000;
    private double maxSeen = 0;
    private boolean running = false;
    private boolean prevA, prevX, prevUp, prevDown, prevLeft, prevRight, prevRB, prevLB;

    @Override
    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorSimple.Direction.FORWARD); // match your teleop
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyPidf();
    }

    @Override
    public void loop() {
        double vel = launcher.getVelocity();

        // B (held): full power, open loop, to measure max velocity
        if (gamepad1.b) {
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setPower(1.0);
            maxSeen = Math.max(maxSeen, vel);
            running = false;
        } else {
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (running) launcher.setVelocity(target);
            else launcher.setPower(0);
        }

        // A: toggle closed-loop run at target
        if (gamepad1.a && !prevA) { running = !running; applyPidf(); }
        // X: set F from the measured max velocity
        if (gamepad1.x && !prevX && maxSeen > 0) { f = 32767.0 / maxSeen; applyPidf(); }
        // D-pad up/down: P +/- 1.  D-pad right/left: F +/- 0.1
        if (gamepad1.dpad_up && !prevUp)       { p += 1;   applyPidf(); }
        if (gamepad1.dpad_down && !prevDown)   { p = Math.max(0, p - 1); applyPidf(); }
        if (gamepad1.dpad_right && !prevRight) { f += 0.1; applyPidf(); }
        if (gamepad1.dpad_left && !prevLeft)   { f = Math.max(0, f - 0.1); applyPidf(); }
        // Bumpers: target +/- 100 tps
        if (gamepad1.right_bumper && !prevRB) target += 100;
        if (gamepad1.left_bumper && !prevLB)  target = Math.max(0, target - 100);
        // Y: reset max seen
        if (gamepad1.y) maxSeen = 0;

        prevA = gamepad1.a; prevX = gamepad1.x;
        prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;
        prevLeft = gamepad1.dpad_left; prevRight = gamepad1.dpad_right;
        prevRB = gamepad1.right_bumper; prevLB = gamepad1.left_bumper;

        telemetry.addData("Running (A)", running);
        telemetry.addData("Target", "%.0f", target);
        telemetry.addData("Velocity", "%.0f", vel);
        telemetry.addData("Error", "%.0f", target - vel);
        telemetry.addData("Max seen (B held, Y reset)", "%.0f", maxSeen);
        telemetry.addData("P (dpad up/down)", "%.1f", p);
        telemetry.addData("F (dpad left/right, X=auto)", "%.2f", f);
        telemetry.update();
    }

    @Override
    public void stop() {
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setPower(0);
    }

    private void applyPidf() {
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(p, i, d, f));
    }
}