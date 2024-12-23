package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Kammander'sStrafingAllWays", group = "Autonomous")
public class KammanderStrafing extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private DcMotor armMotor;
    private IMU imu;
    CRServo intake;
    Servo wrist;
    double viperposition = 0.0;
    final double ARM_TICKS_PER_DEGREE = 7125.16 / 360;
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 10 * ARM_TICKS_PER_DEGREE;//250
    final double ARM_SCORE_SAMPLE_IN_LOW = 30 * ARM_TICKS_PER_DEGREE; //160
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE; // Allow for slight adjustments
    final double ARM_SCORE_IN_HIGH = 70 * ARM_TICKS_PER_DEGREE;


    //Viper Slide Control Variables
    final double VIPERSLIDE_TICKS_PER_DEGREE = 2500 / 360;
    final double VIPER_COLLECT = 5 * VIPERSLIDE_TICKS_PER_DEGREE;
    //final double Maxposition = 1800 * VIPERSLIDE_TICKS_PER_DEGREE;
    final double Maxposition = 26 * VIPERSLIDE_TICKS_PER_DEGREE;



    double armPosition = ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;
    final double WRIST_FOLDED_MIN = 0.2;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 50 * ARM_TICKS_PER_DEGREE;//160
    final double ARM_ATTACH_HANGING_HOOK = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 0 * ARM_TICKS_PER_DEGREE;//15  //8
    final double INTAKE_COLLECT = 1;
    final double INTAKE_OFF = 0;
    final double INTAKE_DEPOSIT = -1;

    private static final double FORWARD_DISTANCE = 0.5; // Adjust distance in meters
    private static final double TURN_ANGLE = -90.0; // Degrees to turn
    private static final double DRIVE_SPEED = 0.9; // Speed for driving

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "Left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "Right_front");
        leftRearMotor = hardwareMap.get(DcMotor.class, "Left_rear");
        rightRearMotor = hardwareMap.get(DcMotor.class, "Right_rear");
        //armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        //intake = hardwareMap.get(CRServo.class, "intake");
        //wrist = hardwareMap.get(Servo.class, "wrist");

        // Set motor directions
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);

//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        ((DcMotorEx) armMotor).setCurrentAlert(2,CurrentUnit.AMPS);

//         Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD; //BACK
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();



//      This code allows the robot to score a sample AND PARK

       /* telemetry.update();
        driveForward(0.15);
        turn(-TURN_ANGLE);
        driveForward(0.45);

        */
        strafing(0.8,true);
        sleep(1000);
        strafing(0.8,false);
        sleep(1000);
        UpDiagonalStraf(0.6,true);
        sleep(1001);
        DownDiagonalStraf(0.6,false );
        sleep(1000);
        UpDiagonalStraf(0.6,false);
        sleep(1001);
        DownDiagonalStraf(0.6, true);



        /*
        armPosition = ARM_COLLECT;
        wrist.setPosition(WRIST_FOLDED_OUT);
        sleep(200);
        intake.setPower(1);
        sleep(292);
        intake.setPower(0);
        turn(TURN_ANGLE);
        turn(TURN_ANGLE);
        driveForward(1.75);

*/

    }

    private void driveForward(double distance) {
        setMotorPower(DRIVE_SPEED);
        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        setMotorPower(0); // Stop all motors
        sleep(1500); // Pause briefly after moving forward
    }
    private void strafing(double distance, boolean isleft ){
        if (isleft == true) {
            rightFrontMotor.setPower(DRIVE_SPEED);
            rightRearMotor.setPower(-DRIVE_SPEED);
            leftFrontMotor.setPower(-DRIVE_SPEED);
            leftRearMotor.setPower(DRIVE_SPEED);

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



    private void turn(double angle) {
        double startAngle = getHeading();
        double targetAngle = startAngle + angle;

        // Normalize the angle to keep it within -180 to 180 degrees
        while (targetAngle >= 180) targetAngle -= 360;
        while (targetAngle < -180) targetAngle += 360;


        double power = 0.3;

        // Set motors for turning
        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        rightRearMotor.setPower(-power);

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