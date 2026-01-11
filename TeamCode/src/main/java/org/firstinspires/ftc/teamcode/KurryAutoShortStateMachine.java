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

@Autonomous(name = "--KurryAutoShortStateMachineTTTT", group = "Linear Opmode")
public class KurryAutoShortStateMachine extends LinearOpMode {
    enum KurryState{
        eFind_MOTIF,
        eConfirm_MOTIF,
        eFind_POST,
        eAlign_POST,
        eLaunch,
        eFind_More_Artifacts,
        ePICKUP,
        ePark
    };
    private KurryState CurrentState = KurryState.eFind_MOTIF;
    private int MotifID = 21;
    private boolean findMotif = false, findPost = false;
    private org.firstinspires.ftc.vision.apriltag.AprilTagDetection MotifTag,PostTag;

    private final int RedPost = 24, BluePost = 20;

    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean targetHeadingInit = false;
    private double targetHeading = 0;

    private double fallbackLaunchHeading = 0.0;

    static final double Align_POST_Timeout = 3000; // milliseconds before we give up

    private DcMotor frontLeft, frontRight, backLeft, backRight;
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

        waitForStart();

        while(opModeIsActive())
        {
            TelemetryRobotState();
            switch(CurrentState){
                case eFind_MOTIF:
                {
                    if(findMOTIFStrafing) {
                        driveStraight(4,false);
                        strafing(24, true);
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
                        CurrentState = KurryState.eFind_POST;//.eConfirm_MOTIF;
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
                        CurrentState = KurryState.eFind_POST;
                        findMotif = true;

                    }
                    else {
                        CurrentState = KurryState.eFind_MOTIF;
                    }
                    break;

                case eFind_POST:


                    if (findPostOneTime )
                    {
                        driveStraight(15, false);
                        setMotorsNOTUsingEncoders();
                        turnRelative(0.2, 45, 1000);
                        findPostOneTime = false;
                    }
                    turnRelative(0.1,4,1000);
                    detections = aprilTagHelper.getDetections();
                    aprilTagHelper.telemetryAprilTag(telemetry);
                    telemetry.update();
                    if(!detections.isEmpty()  && ((org.firstinspires.ftc.vision.apriltag.AprilTagDetection) detections.get(0)).id  == RedPost){
                        org.firstinspires.ftc.vision.apriltag.AprilTagDetection tag = (org.firstinspires.ftc.vision.apriltag.AprilTagDetection) detections.get(0);

                        PostTag = ((org.firstinspires.ftc.vision.apriltag.AprilTagDetection) detections.get(0));
                        CurrentState = KurryState.eAlign_POST;
                        findPost = true;
                        break;
                    }
                    break;
                case eAlign_POST:
                    AlignPost();

                    break;
                case eLaunch:
                    launch(MotifID);
                    CurrentState = KurryState.eFind_More_Artifacts;
                    break;
                case ePICKUP:
                    break;
                case ePark:
                    break;
                case eFind_More_Artifacts:
                    break;
            }
        }
        sleep(500);

        // Stop camera
        aprilTagHelper.stop();
    }

    private void AlignPost() {
        if(!targetHeadingInit)
        {
            // Reset timer so we can timeout if tag doesnt align in time
            stateTimer.reset();
            targetHeadingInit = true;

            // Compute Absolute Heading target once
            double currentHeading = getHeading();
            if(PostTag != null) {
                double tagYAWDeg = Math.toDegrees(PostTag.ftcPose.yaw);
                double offset = 10+  90; // left side of the tag ??
                // double offset = 0 +10 -90 ; // right side of the tag
                targetHeading = AngleUnit.normalizeDegrees(currentHeading + tagYAWDeg + offset);

                telemetry.addData("Target , TagyAW , current heading ","%5.2f %5.3f %5.2f",targetHeading,tagYAWDeg,currentHeading);
                telemetry.update();
                sleep(3000);
            }
            else {
                // fallback heading if no Post Tag
                telemetry.addLine("fallback ");
                telemetry.update();
                sleep(3000);
                targetHeading = fallbackLaunchHeading;
            }
            setMotorsNOTUsingEncoders();
        }
        double currentHeading = getHeading();
        double headingError = AngleUnit.normalizeDegrees(targetHeading-currentHeading);

        // if within threshold, move to next state
        if(Math.abs(headingError) <= HEADING_THRESHOLD){
            moveRobot(0,0);
            targetHeadingInit = false;
            CurrentState = KurryState.eLaunch;
            telemetry.addLine("heading reached");
            telemetry.update();
            sleep(3000);
            return;
        }
        // if we gave taken too long,fallback
        if(stateTimer.milliseconds() > Align_POST_Timeout)
        {
            moveRobot(0,0);
            targetHeadingInit = false;
            CurrentState = KurryState.eLaunch;
            telemetry.addLine("timeout ");
            telemetry.update();
            sleep(3000);
            return;
        }
        // Lets turn and align
        double turnSpeed = Range.clip(headingError * P_TURN_GAIN, -MAX_TURN_SPEED, MAX_TURN_SPEED);
        moveRobot(0.2,turnSpeed);

        sendTelemetry(false);

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

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        launcherLeft = hardwareMap.get(DcMotor.class, "launcherRight"); // swapped intentionally
        launcherRight = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherLeft");
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
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
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

        launcherLeft.setPower(0.5);
        launcherRight.setPower(0.5);
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
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setMotorsNOTUsingEncoders(){
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setAllPower(double p) {
        frontLeft.setPower(p);
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
        frontLeft.setTargetPosition(move);
        frontRight.setTargetPosition(move);
        backLeft.setTargetPosition(move);
        backRight.setTargetPosition(move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllPower(DRIVE_SPEED);
        runtime.reset();
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) &&
                runtime.seconds() < 5) {
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
            frontLeft.setTargetPosition(-move);
            frontRight.setTargetPosition(move);
            backLeft.setTargetPosition(move);
            backRight.setTargetPosition(-move);
        } else {
            frontLeft.setTargetPosition(move);
            frontRight.setTargetPosition(-move);
            backLeft.setTargetPosition(-move);
            backRight.setTargetPosition(move);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllPower(DRIVE_SPEED);
        runtime.reset();
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) &&
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
     * Spin on the central axis to point in a new direction.
     * <p>
     * Move will stop if either of these conditions occur:
     * <p>
     * 1) Move gets to the heading (angle)
     * <p>
     * 2) Driver stops the OpMode running.
     *
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
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // Retain leftSpeed/rightSpeed for existing telemetry
        leftSpeed = frontLeftPower;
        rightSpeed = frontRightPower;
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