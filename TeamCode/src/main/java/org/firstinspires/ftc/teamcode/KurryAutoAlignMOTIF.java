package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Autonomous(name = "--KurryAuto Align MOTIF v2", group = "Linear Opmode")
public class KurryAutoAlignMOTIF extends LinearOpMode {
    enum KurryState{
        eFind_MOTIF,
        eConfirm_MOTIF,
        eAlign_MOTIF,
        eDone
  

    };
    enum Alliance {
        eBlue,
        eRed
    };

    Alliance alliance = Alliance.eRed;

    static double SIGN_Alliance= 1;

    private KurryState CurrentState = KurryState.eFind_MOTIF;
    private int MotifID = 21;
    private boolean findMotif = false, findPost = false;
    private org.firstinspires.ftc.vision.apriltag.AprilTagDetection MotifTag,PostTag;

    private final int RedPost = 24, BluePost = 20;

    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean targetHeadingInit = false;
    private double targetHeading = 0;

    private double fallbackLaunchHeading = 0.0;

    static final double Align_POST_Timeout = 2000;// milliseconds before we give up
    static final double fing_Post_Timeout = 2000;
    private DcMotor frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private IMU imu;
    private Servo flapperRight, flapperLeft;
    private CRServo divider;

    private ElapsedTime runtime = new ElapsedTime();
    private AprilTagHelper aprilTagHelper;
    private String pattern = "UNKNOWN";

    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.5;
    private static final double HEADING_THRESHOLD = 3.0; // init was 1 checking 3 now
    private static final double TICKS_PER_INCH = 537.7 / (Math.PI * 4); // 4" wheels, 537.7 ticks/rev

    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    // Telemetry variables

    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private double headingError = 0;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.

    static final double MAX_TURN_SPEED = 0.2;
    static boolean findMOTIFStrafing = true;
    static boolean findPostOneTime = true;
    static final double TIMEOUT_SECONDS = 5.0;

    @Override
    public void runOpMode() {

        InitializeMotorServosEverything();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b) {
                alliance = Alliance.eRed;
            }

            if (gamepad1.x) {
                alliance = Alliance.eBlue;
            }

            telemetry.addData("Alliance ", alliance);
            telemetry.update();
        }

        waitForStart();

        if (alliance == Alliance.eBlue){
            SIGN_Alliance = -1;
        }

        while(opModeIsActive())
        {
           // TelemetryRobotState();
            switch(CurrentState){
                case eFind_MOTIF:
                {
                    if(findMOTIFStrafing) {
                        driveStraight(1,false);
//                        if(alliance == Alliance.eRed) {
//                            strafing(24, true);
//                        }
//                        else {
//                            strafing(24, false);
//                        }
                        findMOTIFStrafing = false;
                    }
                    List detections = aprilTagHelper.getDetections();
                    //while(detections.isEmpty()){
                    aprilTagHelper.telemetryAprilTag(telemetry);
                    telemetry.update();
                    detections = aprilTagHelper.getDetections();
                    //}
                    if(!detections.isEmpty())
                    {
                        MotifID= ((org.firstinspires.ftc.vision.apriltag.AprilTagDetection) detections.get(0)).id;
                        MotifTag = ((org.firstinspires.ftc.vision.apriltag.AprilTagDetection) detections.get(0));
                        CurrentState = KurryState.eConfirm_MOTIF;//.eConfirm_MOTIF;
                        break;
                    }
                    driveStraight(1,false);
                }

                break;
                case eConfirm_MOTIF:
                    sleep(500);
                    List detections;
                    aprilTagHelper.telemetryAprilTag(telemetry);
                    telemetry.update();
                    detections = aprilTagHelper.getDetections();
                    if(((org.firstinspires.ftc.vision.apriltag.AprilTagDetection) detections.get(0)).id == MotifID) {
                        CurrentState = KurryState.eAlign_MOTIF;
                        findMotif = true;
                    }
                    else {
                        CurrentState = KurryState.eFind_MOTIF;
                    }

                    telemetryIMU();

                    telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                    telemetry.addData("Found", "ID %d (%s)", MotifTag.id, MotifTag.metadata.name);
                    telemetry.addData("Range",  "%5.1f inches", MotifTag.ftcPose.range);
                    telemetry.addData("Bearing","%3.0f degrees", MotifTag.ftcPose.bearing);
                    telemetry.addData("Yaw","%3.0f degrees", MotifTag.ftcPose.yaw);
                    telemetry.update();
                    sleep(10000);
                    break;

                case eAlign_MOTIF:
                    AlignMOTIF();
                    break;
            }
        }
        sleep(500);

        // Stop camera
        aprilTagHelper.stop();
    }
    private void AlignMOTIF() {

        double drive = 1, turn = 1 , strafe  = 1;
        if(!targetHeadingInit)
        {
            // Reset timer so we can timeout if tag doesnt align in time
            stateTimer.reset();
            targetHeadingInit = true;

            // Compute Absolute Heading target once
            double currentHeading = getHeading();
            if(MotifTag != null) {
                double  rangeError      = (MotifTag.ftcPose.range - 10);
                double  headingError    = MotifTag.ftcPose.bearing;
                double  yawError        = MotifTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                sleep(10000);
            }
            else {
                // fallback heading if no Post Tag
                telemetry.addLine("fallback ");
                telemetry.update();
                sleep(500);
                targetHeading = fallbackLaunchHeading;

                drive = 1;
                turn = 1;
                strafe = 1;
            }
            setMotorsNOTUsingEncoders();
        }


        moveRobot(drive, strafe, turn);
        sleep(10);

       sendTelemetry(false);

       CurrentState= KurryState.eDone;

    }

    private void TelemetryRobotState() {
        telemetry.addData("current state is ",CurrentState);

        if(findMotif)
            telemetry.addLine("Seen Motif Yes") ;
        else
            telemetry.addLine("Seen Motif NO");
        if(findPost)
            telemetry.addLine("Seen Post Yes");
        else
            telemetry.addLine("Seen Post NO");

        telemetry.addData("Pattern ", MotifID);
        telemetry.update();
    }

    private void InitializeMotorServosEverything() {
        // ===== Initialize Motors & Servos =====


        findMOTIFStrafing = true;
        findPostOneTime = true;
        targetHeadingInit = false;

       // frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        launcherLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");

        flapperRight = hardwareMap.get(Servo.class, "fr");
        flapperLeft = hardwareMap.get(Servo.class, "fl");
        flapperRight.setDirection(Servo.Direction.FORWARD);
        flapperLeft.setDirection(Servo.Direction.REVERSE);

        divider = hardwareMap.get(CRServo.class, "sw");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
       // frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        setMotorsUsingEncoders();

        aprilTagHelper = new AprilTagHelper(hardwareMap);
        telemetry.addLine("AprilTag Helper initialized...");
        telemetry.update();
    }

    private void telemetryIMU() {
        telemetry.addData("imu Head  Pitch, Roll" , "%5.2f %5.2f  %5.2f", getHeading(),getPitch(),getRoll());
        telemetry.update();
        sleep(2000);
    }

    private void launch(int pattern) {

        launcherLeft.setPower(0.7);
        launcherRight.setPower(0.7);
        sleep(1000);

        divider.setPower(1);
        intake.setPower(0.8);
        sleep(500);

        switch (pattern) {
            case 23:
                rightLaunch();
                divide();
                rightLaunch();
                leftLaunch();

                break;
            case 22:
//                sleep(1000);
                rightLaunch();
                leftLaunch();
                divide();
                rightLaunch();
                break;
            case 21:
//                sleep(1000);
                leftLaunch();
                rightLaunch();
                divide();
                rightLaunch();
                break;
            default: // fallback
                leftLaunch();
                rightLaunch();
                break;
        }
        strafing(15,false);
        divider.setPower(0);
        intake.setPower(0);
        launcherLeft.setPower(0);
        launcherRight.setPower(0);
    }
    private void divide(){
        sleep(500);
        divider.setPower(1);//move to the right
        sleep(1000);
    }

    private void rightLaunch() {
        flapperRight.setPosition(0.58);
        sleep(1000);
        flapperRight.setPosition(0.71);
        sleep(1000);
    }

    private void leftLaunch() {
        flapperLeft.setPosition(0.14);
        sleep(1000);
        flapperLeft.setPosition(0.3);
        sleep(1000);
    }

    private void setMotorsUsingEncoders() {

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setMotorsNOTUsingEncoders(){

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setAllPower(double p) {

        frontRight.setPower(p);
        backLeft.setPower(p);
        backRight.setPower(p);
    }

    private void stopAll() {
        setAllPower(0);
    }

    private void driveStraight(double inches, boolean forward) {
        int move = (int) (inches * TICKS_PER_INCH);
        if (!forward) move = -move;

        setMotorsUsingEncoders();

        frontRight.setTargetPosition(move);
        backLeft.setTargetPosition(move);
        backRight.setTargetPosition(move);


        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllPower(DRIVE_SPEED);
        runtime.reset();
        while (opModeIsActive() &&
                (frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) &&
                runtime.seconds() < 3) {
            telemetry.addData("Driving", inches);
            telemetry.update();
        }
        stopAll();
        sleep(500);
    }

    private void strafing(double inches, boolean left) {
        double correction = 1.12;
        int move = (int) (inches * TICKS_PER_INCH * correction);

        setMotorsUsingEncoders();

        if (left) {

            frontRight.setTargetPosition(move);
            backLeft.setTargetPosition(move);
            backRight.setTargetPosition(-move);
        } else {

            frontRight.setTargetPosition(-move);
            backLeft.setTargetPosition(-move);
            backRight.setTargetPosition(move);
        }

       // frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllPower(DRIVE_SPEED);
        runtime.reset();
        while (opModeIsActive() &&
                (frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) &&
                runtime.seconds() < 5) {
            telemetry.addData("Strafing", inches);
            telemetry.update();
        }
        stopAll();
        sleep(100);
    }

    public void turnRelative(double maxTurnSpeed, double deltaAngle, double timeoutInMS) {
        double targetHeading = AngleUnit.normalizeDegrees(getHeading() + deltaAngle);
        turnToHeading(maxTurnSpeed, targetHeading, timeoutInMS);
    }

    /**

     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param timeoutInMS
     */
    public void turnToHeading(double maxTurnSpeed, double heading, double timeoutInMS) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive()) {

            if ((Math.abs(headingError) <= HEADING_THRESHOLD))
            {
                telemetry.addLine("Heading error small exit");
                telemetry.update();
                sleep(500);
                break;
            }
            if(runtime.milliseconds() > timeoutInMS)
            {
                telemetry.addLine("turn timeout");
                telemetry.update();
                sleep(500);
                break;
            }
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            if(Math.abs(turnSpeed) < 0.05){
                telemetry.addLine("turnSpeed low exit");
                telemetry.update();
                sleep(500);
                break;
            }

            // Pivot in place by applying the turning correction
            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);

        }

        // Stop all motion;
        moveRobot(0, 0);
    }
    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive; // Save for telemetry
        turnSpeed  = turn;  // Save for telemetry

        // Mecanum Kinematics for Y-Drive and T-Turn (assuming no strafe X=0):
        // All motors on the left side spin the same way for turning.
        // All motors on the right side spin the opposite way for turning.
        double frontLeftPower = drive + turn;
        double frontRightPower = drive - turn;
        double backLeftPower = drive + turn;
        double backRightPower = drive - turn;

        // Scale speeds down if any one exceeds +/- 1.0;
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0)
        {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Apply power to all four motors!
       // frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // Retain leftSpeed/rightSpeed for existing telemetry
        leftSpeed = frontLeftPower;
        rightSpeed = frontRightPower;
    }
    /**
     *    // Apply desired axes motions to the drivetrain.
     *             moveRobot(drive, strafe, turn);
     *             sleep(10);
     *             Check RobotAutoDriveToAprilTagOmni.java
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        //frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /**
     * Obtain & hold a heading for a finite amount of time
     * <p>
     * Move will stop once the requested time has elapsed
     * <p>
     * This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     * 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     * If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // ********** LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double currentHeading = getHeading();

        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error

        double error = AngleUnit.normalizeDegrees(desiredHeading-currentHeading);
        headingError =  error;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        double steer =  Range.clip(headingError * proportionalGain, -1, 1);
        return steer;
    }

    /**
     * Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if(!straight){
            telemetry.addData("Motion", "Turning");
        }


        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  ",  "%5.1f", headingError);
        telemetry.addData("Turn SPeed Power ", "%5.1f",turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }


    public double getPitch(){

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        return orientation.getPitch(AngleUnit.DEGREES);

    }

    public double getRoll()

    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        return orientation.getRoll(AngleUnit.DEGREES);

    }
}