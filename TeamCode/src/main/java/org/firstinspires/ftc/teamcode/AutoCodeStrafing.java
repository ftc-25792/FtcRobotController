package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "AutoCodeStrafing", group = "Autonomous")
public class AutoCodeStrafing  extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private IMU imu;
    CRServo intake;
    Servo wrist;

    private static final double FORWARD_DISTANCE = 0.5; // Adjust distance in meters

    private static final double DRIVE_SPEED = 0.6 ;
    private static final double OFFSET_SPEED = 0.2;
    // Speed for driving

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "Left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "Right_front");
        leftRearMotor = hardwareMap.get(DcMotor.class, "Left_rear");
        rightRearMotor = hardwareMap.get(DcMotor.class, "Right_rear");

        // Set motor directions
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        // Initialize the IMU
        waitForStart();
        driveStraight(0.05, true);
        strafing(1.2,true);

        driveStraight(0.7, true);
        strafing(0.10,true);
        driveStraight(1.2,false);

        driveStraight(0.8, true);
        strafing(0.10,true);
        driveStraight(1.2,false);

        driveStraight(0.8, true);
        strafing(0.10,true);
        driveStraight(1.2,false);


    }
    private void driveStraight(double distance, boolean isForward) {

        if(isForward){
            setMotorPower(DRIVE_SPEED);
        }else {
            setMotorPower(-DRIVE_SPEED + OFFSET_SPEED);
        }

        if(DRIVE_SPEED > 0 && distance > 0.0) {
            sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        }else {
            sleep(500);
        }
        setMotorPower(0); // Stop all motors
        sleep(1500); // Pause briefly after moving forward
    }

    private void strafing(double distance, boolean isleft ){
        if (isleft == true) {
            rightFrontMotor.setPower(DRIVE_SPEED-OFFSET_SPEED);
            rightRearMotor.setPower(-DRIVE_SPEED+OFFSET_SPEED);
            leftFrontMotor.setPower(-DRIVE_SPEED+OFFSET_SPEED);
            leftRearMotor.setPower(DRIVE_SPEED-OFFSET_SPEED);

        }else{
            rightFrontMotor.setPower(-DRIVE_SPEED);
            rightRearMotor.setPower(DRIVE_SPEED);
            leftFrontMotor.setPower(DRIVE_SPEED);
            leftRearMotor.setPower(-DRIVE_SPEED);
        }
        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        setMotorPower(0); // Stop all motors
        sleep(500); // Pause briefly after moving forward

    }
    private void UpDiagonalStraf(double distance, boolean IsUpperLeft){
        if (IsUpperLeft == true) {
            rightFrontMotor.setPower(DRIVE_SPEED);
            leftRearMotor.setPower(DRIVE_SPEED);

        }else{
            rightRearMotor.setPower(DRIVE_SPEED);
            leftFrontMotor.setPower(DRIVE_SPEED);
        }
        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        setMotorPower(0); // Stop all motors
        sleep(500); // Pause briefly after moving forward

    }
    private void DownDiagonalStraf(double distance, boolean IsLowerLeft){
        if (IsLowerLeft== true) {
            rightRearMotor.setPower(-DRIVE_SPEED);
            leftFrontMotor.setPower(-DRIVE_SPEED);


        }else{
            rightFrontMotor.setPower(-DRIVE_SPEED);
            leftRearMotor.setPower(-DRIVE_SPEED);
        }
        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        setMotorPower(0); // Stop all motors
        sleep(500); // Pause briefly after moving forward
    }



    private void turn(double angle, boolean isright) {
        double startAngle = getHeading();
        double targetAngle = startAngle + angle;

        // Normalize the angle to keep it within -180 to 180 degrees
        while (targetAngle >= 180) targetAngle -= 360;
        while (targetAngle < -180) targetAngle += 360;


        double power = 0.3;

        // Set motors for turning
        if (isright == true){
            leftFrontMotor.setPower(-power);
            leftRearMotor.setPower(-power);
            rightFrontMotor.setPower(power);
            rightRearMotor.setPower(power);
        } else {
            leftFrontMotor.setPower(power);
            leftRearMotor.setPower(power);
            rightFrontMotor.setPower(-power);
            rightRearMotor.setPower(-power);
        }
        // Continue turning until the target angle is reached
        while (opModeIsActive() && !isAngleReached(targetAngle)) {
            telemetry.addData("Current Angle", getHeading());
            telemetry.addData("Start ANgele",startAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.update();
            // sleep(1000);
        }

        setMotorPower(0); // Stop all motors
        sleep(1800); // Pause briefly after turning
    }

    private double getHeading() {
        // Get the IMU's heading
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES); // Return the Z-axis angle
    }

    private boolean isAngleReached(double targetAngle) {
        double currentAngle = getHeading();

        // Normalize the angle to keep it within -180 to 180 degrees
        while (currentAngle >= 180) currentAngle -= 360;
        while (currentAngle < -180) currentAngle += 360;
        double error = currentAngle - targetAngle;




        return Math.abs(error) < 1.0; // Tolerance can be adjusted
    }

    private void setMotorPower(double power) {
        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);
    }

    }