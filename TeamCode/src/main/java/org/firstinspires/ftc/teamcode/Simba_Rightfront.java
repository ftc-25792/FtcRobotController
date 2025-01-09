package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name = "SimbaRightFront", group = "Autonomous")
    public class Simba_Rightfront extends LinearOpMode {

    private DcMotor RightFront; // Only one motor is needed
    private IMU imu;

    private static final double FORWARD_DISTANCE = 0.5; // Adjust distance in meters
    private static final double DRIVE_SPEED = 0.5; // Speed for driving

    @Override
    public void runOpMode() {
        // Initialize motor
        RightFront = hardwareMap.get(DcMotor.class, "Right_front");

        // Set motor direction
        RightFront.setDirection(DcMotor.Direction.FORWARD);

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();

        // Drive forward
        driveForward(FORWARD_DISTANCE);
    }

    private void driveForward(double distance) {
        setMotorPower(DRIVE_SPEED);
        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        setMotorPower(0); // Stop motor
        sleep(500); // Pause briefly after moving forward
    }

    private void setMotorPower(double power) {
        RightFront.setPower(power);
    }
}
