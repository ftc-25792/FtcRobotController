package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="SimbaAutonomous", group="Autonomous")
public class SimbaAutonomous extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private DcMotor leftVertMotor;
    private DcMotor rightVertMotor;
    private Servo linearServo;
    private Servo rightClaw;
    private IMU imu;
    double DRIVE_SPEED = 0.5;
    private double getHeading() {
        return 0;
    }
     final double viperPower = 1.0;
     final double High_Basket = -800;
     final double Extend = -800;
     final double Claw_Off = 0;
    final double Claw_On = 0.5;


    private void setMotorPower(double power) {
        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);
    }

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "Left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "Right_front");
        leftRearMotor = hardwareMap.get(DcMotor.class, "Left_rear");
        rightRearMotor = hardwareMap.get(DcMotor.class, "Right_rear");

        leftVertMotor = hardwareMap.get(DcMotor.class, "Left_vert");
        rightVertMotor = hardwareMap.get(DcMotor.class, "Right_vert");

        linearServo = hardwareMap.get(Servo.class, "linearServo");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        rightVertMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        strafing(0.5, false);
        driveStraight(1.5, true);
        turn(45,true);



    }



   private void High_Basket(double distance, boolean isUp) {
        if(isUp) {
   }

    private void strafing(double distance, boolean isLeft) {
        // Set powers for strafing
        if (isLeft) {
            leftFrontMotor.setPower(-DRIVE_SPEED);
            rightFrontMotor.setPower(DRIVE_SPEED);
            leftRearMotor.setPower(DRIVE_SPEED);
            rightRearMotor.setPower(-DRIVE_SPEED);


        } else {
            leftFrontMotor.setPower(DRIVE_SPEED);
            rightFrontMotor.setPower(-DRIVE_SPEED);
            leftRearMotor.setPower(-DRIVE_SPEED);
            rightRearMotor.setPower(DRIVE_SPEED);

        }
    }

    void turn(double angle, boolean isLeft){
        imu.resetYaw();
        double startAngle = getHeading();
        double targetAngle = startAngle + angle;

    }

    private void driveStraight(double distance, boolean isForward) {
//        // Start moving forward or backward based on direction
//        double initialHeading = getHeading();
        if (isForward == true) {
            setMotorPower(DRIVE_SPEED);
        } else {
            setMotorPower(-DRIVE_SPEED);
        }

        // Calculate the distance (in meters)
        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        setMotorPower(0); // Stop all motors

        // loop to maintain heading while driving
//        while (opModeIsActive() && (System.currentTimeMillis() < endTime)) {
//            double correction = getSteeringCorrection(initialHeading, 0.03);
//            setMotorPower(DRIVE_SPEED - correction);
//        }

        setMotorPower(0);
        sleep(500);


            }
        }




