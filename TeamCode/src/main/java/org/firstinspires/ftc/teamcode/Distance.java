package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Swyft Mecanum Distance Safety", group = "Sensor")
public class SwyftMecanumDistanceSafety extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map motors — make sure names match your Control Hub configuration
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Map distance sensor (Swyft or REV 2m)
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        // Motor direction (reverse left side if needed)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Gamepad control inputs
            double y = -gamepad1.left_stick_y;  // forward/back
            double x = gamepad1.left_stick_x;   // strafe
            double rx = gamepad1.right_stick_x; // rotation

            // Read distance in cm (with safety check)
            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);
            if (Double.isNaN(distanceCM)) distanceCM = 999; // if no reading, treat as far away

            // Default speed modifier
            double speedModifier = 1.0;

            // Deceleration + stop logic
            if (distanceCM <= 70 && distanceCM > 10 && y > 0) {
                speedModifier = (distanceCM - 10) / 60.0;
            } else if (distanceCM <= 10 && y > 0) {
                speedModifier = 0.0;
            }

            // Apply modifier only to forward motion
            double adjustedY = y * speedModifier;

            // Mecanum drive calculations
            double frontLeftPower  = adjustedY + x + rx;
            double backLeftPower   = adjustedY - x + rx;
            double frontRightPower = adjustedY - x - rx;
            double backRightPower  = adjustedY + x - rx;

            // Normalize powers if any exceed 1.0
            double max = Math.max(1.0,
                    Math.max(Math.abs(frontLeftPower),
                            Math.max(Math.abs(frontRightPower),
                                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

            frontLeft.setPower(frontLeftPower / max);
            frontRight.setPower(frontRightPower / max);
            backLeft.setPower(backLeftPower / max);
            backRight.setPower(backRightPower / max);

            // Telemetry
            telemetry.addData("Distance (cm)", "%.1f", distanceCM);
            telemetry.addData("Speed Modifier", "%.2f", speedModifier);
            telemetry.addData("Forward Input", "%.2f", y);
            telemetry.addData("Adjusted Forward", "%.2f", adjustedY);
            telemetry.update();
        }
    }
}
