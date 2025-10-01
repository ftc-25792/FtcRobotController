package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "KammanderAuto", group = "Autonomous")
public class ColorSensor_Testing extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private IMU imu;
    private DcMotor armMotor;
    private DcMotor viperMotor;
    private ColorSensor colorSensor;

    double armPosition = 0;
    double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.4;
    static final double HEADING_THRESHOLD = 1.0;

    @Override
    public void runOpMode() {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "Left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "Right_front");
        leftRearMotor = hardwareMap.get(DcMotor.class, "Left_rear");
        rightRearMotor = hardwareMap.get(DcMotor.class, "Right_rear");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();

        driveStraight(0.5, true);
        turn(42, true); // This will now stop if red is detected
    }

    private void driveStraight(double distance, boolean isForward) {
        setMotorPower(isForward ? DRIVE_SPEED : -DRIVE_SPEED);
        sleep((long) (distance * 1000 / DRIVE_SPEED));
        setMotorPower(0);
        sleep(500);
    }

    private void turn(double angle, boolean isLeft) {
        imu.resetYaw();
        double startAngle = getHeading();
        double targetAngle = startAngle + angle;

        while (targetAngle >= 180) targetAngle -= 360;
        while (targetAngle < -180) targetAngle += 360;

        if (isLeft) {
            leftFrontMotor.setPower(-TURN_SPEED);
            leftRearMotor.setPower(-TURN_SPEED);
            rightFrontMotor.setPower(TURN_SPEED);
            rightRearMotor.setPower(TURN_SPEED);
        } else {
            leftFrontMotor.setPower(TURN_SPEED);
            leftRearMotor.setPower(TURN_SPEED);
            rightFrontMotor.setPower(-TURN_SPEED);
            rightRearMotor.setPower(-TURN_SPEED);
        }
        // While turning the robot will check for the any color IN THIS CASE RED
        while (opModeIsActive() && !isAngleReached(targetAngle)) {
            // Check for red detection
            int red = colorSensor.red();
            int blue = colorSensor.blue();
            int green = colorSensor.green();

            telemetry.addData("Red", red);
            telemetry.addData("Color Check", red > blue + 30 && red > green + 30 ? "RED DETECTED" : "No red");
            telemetry.addData("Current Angle", getHeading());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.update();

            if (red > blue + 30 && red > green + 30) { // You can tune this threshold
                telemetry.addLine("Stopping turn: RED DETECTED");
                telemetry.update();
                break;
            }
        }

        setMotorPower(0);
        sleep(500);
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private boolean isAngleReached(double targetAngle) {
        double currentAngle = getHeading();
        while (currentAngle >= 180) currentAngle -= 360;
        while (currentAngle < -180) currentAngle += 360;
        return Math.abs(currentAngle - targetAngle) < HEADING_THRESHOLD;
    }

    private void setMotorPower(double power) {
        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);
    }
}
