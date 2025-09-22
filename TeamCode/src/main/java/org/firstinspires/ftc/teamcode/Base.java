package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Base {

    // Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    HardwareMap hwMap;

    // Constructor
    public void BaseRobot() {
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Map motors
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");

        // Set directions (adjust if needed)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set to zero power when stopped
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Drive Methods
    public void setPower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void stop() {
        setPower(0, 0, 0, 0);
    }

    // Basic movements
    public void driveForward(double power) {
        setPower(power, power, power, power);
    }

    public void driveBackward(double power) {
        setPower(-power, -power, -power, -power);
    }

    public void strafeRight(double power) {
        setPower(power, -power, -power, power);
    }

    public void strafeLeft(double power) {
        setPower(-power, power, power, -power);
    }

    public void turnRight(double power) {
        setPower(power, -power, power, -power);
    }

    public void turnLeft(double power) {
        setPower(-power, power, -power, power);
    }
}
