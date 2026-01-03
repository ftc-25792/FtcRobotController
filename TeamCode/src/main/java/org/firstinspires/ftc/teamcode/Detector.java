package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "AprilTag Quick Test", group = "Test")
public class Detector extends LinearOpMode {

    private AprilTagHelper aprilTagHelper;

    @Override
    public void runOpMode() {

        // get the helper
        aprilTagHelper = new AprilTagHelper(hardwareMap);

        telemetry.addLine("AprilTag Helper Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            aprilTagHelper.telemetryAprilTag(telemetry);
            telemetry.update();
        }
        aprilTagHelper.stop();
    }
}
