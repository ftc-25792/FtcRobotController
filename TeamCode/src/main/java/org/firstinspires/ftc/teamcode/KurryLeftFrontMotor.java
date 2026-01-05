package org.firstinspires.ftc.teamcode.KurrysLibrary;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name = "KurryLeftFrontMotor", group = "Autonomous")
public class KurryLeftFrontMotor extends LinearOpMode {
    private DcMotor frontLeft; // Only one motor is needed
    private IMU imu;

    private static final double FORWARD_DISTANCE = 0.5; // Adjust distance in meters
    private static final double DRIVE_SPEED = 0.8; // Speed for driving

    @Override
    public void runOpMode() {
        // Initialize motor
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        // Set motor direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

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
        frontLeft.setPower(DRIVE_SPEED);
        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        frontLeft.setPower(0); // Stop motor
        sleep(500); // Pause briefly after moving forward
    }
}
