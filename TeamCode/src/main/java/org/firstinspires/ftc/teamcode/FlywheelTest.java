package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Flywheel Test", group = "Test")
public class FlywheelTest extends LinearOpMode {

    private DcMotor launcher;

    @Override
    public void runOpMode() {

        launcher = hardwareMap.get(
                DcMotor.class, "launcher"
        );

        launcher.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        launcher.setPower(0);

        telemetry.addLine("Flywheel Test Ready");
        telemetry.addLine("Hold A to run the flywheel");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                launcher.setPower(0.6);
            } else {
                launcher.setPower(0);
            }

            telemetry.addData(
                    "Flywheel Power",
                    gamepad1.a ? 0.6 : 0.0
            );
            telemetry.update();
        }

        launcher.setPower(0);
    }
}


