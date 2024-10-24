package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name="BasicTeleOp", group="Linear Opmode")
public class BasicTeleOpTest extends OpMode {
    // Declare motors
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor armMotor;
    DcMotor viperMotor; // For the Viper slide
    IMU imu;

    private static final double FORWARD_DISTANCE = 0.5; // Adjust distance in meters
    private static final double DRIVE_SPEED = 0.5; // Speed for driving

    @Override
    public void init() {
        // Initialize motors
        motorFrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        motorFrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        motorBackLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        motorBackRight = hardwareMap.get(DcMotor.class, "Right_rear");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        viperMotor = hardwareMap.get(DcMotor.class, "viperMotor");

        // Set motor directions (adjust as needed)
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
        // Set gamepad inputs
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;
        boolean triangle = gamepad1.dpad_up;
        boolean square = gamepad1.dpad_down;

        // Calculate wheel powers
        double frontLeftPower = leftStickY + leftStickX;
        double frontRightPower = leftStickY - leftStickX;
        double backLeftPower = leftStickY + rightStickX;
        double backRightPower = leftStickY - rightStickX;

        // Set motor powers
        motorFrontLeft.setPower(frontLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackLeft.setPower(backLeftPower);
        motorBackRight.setPower(backRightPower);

        // Control arm with triangle button
        if (triangle) {
            armMotor.setPower(1.0); // Move arm up
        } else {
            armMotor.setPower(0.0); // Stop arm
        }


        if (square) { // Move the Viper slide up when square button is pressed
            viperMotor.setPower(1.0);
        } else {
            viperMotor.setPower(0.0); // Stop Viper slide when not pressed
        }
    }
}
