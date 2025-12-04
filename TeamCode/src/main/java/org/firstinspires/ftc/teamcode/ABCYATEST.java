package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DistanceStopGamepad", group = "Linear Opmode")
public class ABCYATEST extends LinearOpMode {

    DcMotor frontRight, frontLeft, backRight, backLeft;
    DistanceSensor sensorDistance;

    @Override
    public void runOpMode() {

        // Hardware mapping
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        // Reverse left motors if needed
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double distance = sensorDistance.getDistance(DistanceUnit.CM);

            // Gamepad inputs
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;


            double leftPower = drive + turn;
            double rightPower = drive - turn;


            if (distance < 70 && drive > 0) {

                leftPower = 0;
                rightPower = 0;
            }


            leftPower = Math.max(-1, Math.min(1, leftPower));
            rightPower = Math.max(-1, Math.min(1, rightPower));

            // Apply motor power
            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);

            // Telemetry
            telemetry.addData("Distance (cm)", "%.1f", distance);
            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);
            telemetry.update();
        }
    }
}
