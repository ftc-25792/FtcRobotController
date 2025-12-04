package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldCentricTest", group = "Drive")
public class FieldCentricTest extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(params);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double fieldX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
        double fieldY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);

        double frontLeftPower = fieldY + fieldX + turn;
        double frontRightPower = fieldY - fieldX - turn;
        double backLeftPower = fieldY - fieldX + turn;
        double backRightPower = fieldY + fieldX - turn;

        double max = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (max > 0.7) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.addData("Field X", "%.2f", fieldX);
        telemetry.addData("Field Y", "%.2f", fieldY);
        telemetry.update();
    }
}