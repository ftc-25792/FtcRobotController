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

@Autonomous(name = "--Kurry AutoShort StateMachine", group = "Linear Opmode")
public class KurryAutoShortStateMachine extends LinearOpMode {


    public static final double Red_In = 0.45;
    public static final double Red_Out = 0.5;
    public static final double Blue_IN = 0.47;
    public static final double Blue_Out = 0.40;
    public static final double POST_DISTANCE = 1.5;
    double diff = 2;
    enum KurryState{
        eFind_MOTIF,
        eConfirm_MOTIF,
        eFind_POST,
        eAlign_POST,
        eLaunch,
        eFind_More_Artifacts,
        ePICKUP,
        ePark,
        GetBackToSho,
        eDone;

    };
    enum Alliance {
        eBlue,
        eRed
    };

    Alliance alliance = Alliance.eRed;
    static double Find_Post_Dist_Moved;
    static double SIGN_Alliance= -1;
    static final double POST_RANGE_INCHES = 36;
    static final double POST_RANGE_TOL = 1;
    static final double POST_BEARING_TOL = 3;
    static final double POST_YAW_TOL = 10;
    final double SPEED_GAIN = 0.03;
    final double STRAFE_GAIN = 0.02;
    final double TURN_GAIN = 0.01;
    static final double MAX_AUTO_SPEED = 0.5;
    static final double MAX_AUTO_STRAFE = 0.5;
    static final double MAX_AUTO_TURN = 0.3;
    private KurryState CurrentState = KurryState.eFind_MOTIF;
    private int MotifID = 21;
    private boolean findMotif = false, findPost = false;
    private static boolean PrepDone = false;
    private ElapsedTime prepTimer = new ElapsedTime();
    private org.firstinspires.ftc.vision.apriltag.AprilTagDetection MotifTag = null,PostTag = null;

    private final int RedPost = 24, BluePost = 20;

    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean targetHeadingInit = false;
    private double targetHeading = 0;

    private double fallbackLaunchHeading = 0.0;

    static final double Align_POST_Timeout = 2000;// milliseconds before we give up
    static final double fing_Post_Timeout = 2000;
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

    private double LATEST_Range, LATEST_Bearing ;

    private static final double MOTIF_STRAF = 20;
    private static final double MOTIF_DRIVE = 1.5;

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

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera

    static final double MAX_TURN_SPEED = 0.2;
    static boolean findMOTIFStrafing = true;
    static double TOTAL_STAF = 0.0;
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

            if (alliance == Alliance.eBlue){
                SIGN_Alliance = 1;
            }
            else{
                SIGN_Alliance = -1;
            }
            telemetry.addData("Alliance ", alliance);
            telemetry.addData("Sign", SIGN_Alliance);
            telemetry.update();

        }

        waitForStart();


        while(opModeIsActive())
        {
            TelemetryRobotState();
            switch(CurrentState){
                case eFind_MOTIF:
                {

                    if(findMOTIFStrafing) {
                        //driveStraight(1,false);

                        PrepForLaunch();
                        if(alliance == Alliance.eRed) {
                            strafing(MOTIF_STRAF, true);
                        }
                        else {
                            strafing(MOTIF_STRAF, false);
                        }
                        TOTAL_STAF = 0.0;
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
                        MotifID = ((org.firstinspires.ftc.vision.apriltag.AprilTagDetection) detections.get(0)).id;
                        MotifTag = ((org.firstinspires.ftc.vision.apriltag.AprilTagDetection) detections.get(0));
                        CurrentState = KurryState.eFind_POST;//.eConfirm_MOTIF; //eFind_POST
                        findMotif = true;
                        break;
                    }
                    driveStraight(MOTIF_DRIVE,false);
                    TOTAL_STAF += MOTIF_DRIVE;
                }

                break;
                case eConfirm_MOTIF: {
                    List detections;
                    aprilTagHelper.telemetryAprilTag(telemetry);
                    telemetry.update();
                    detections = aprilTagHelper.getDetections();
                    if (!detections.isEmpty()) {
                        if (((org.firstinspires.ftc.vision.apriltag.AprilTagDetection) detections.get(0)).id == MotifID) {
                            aprilTagHelper.telemetryAprilTag(telemetry);
                            telemetry.update();
                            CurrentState = KurryState.eFind_POST;
                            findMotif = true;
                        }
                    } else {
                        CurrentState = KurryState.eFind_MOTIF;
                    }
                }
                    break;

                case eFind_POST:
                    FindPost();
                    break;
                case eAlign_POST:
                    AlignPost();
                    break;
                case eLaunch:
                    if (!PrepDone){
                        PrepForLaunch();
                    }
                    while(prepTimer.milliseconds() <= 4000) {
                    telemetry.addLine("Waiting");
                    telemetry.update();
                    }
                    telemetry.addLine("Launching");
                    telemetry.update();
                    launch(MotifID);
                    CurrentState = KurryState.ePark;
                    break;
                case ePICKUP:
                    Sort(1);
                    CurrentState = KurryState.GetBackToSho;
                    break;
                case GetBackToSho:
                    BackToShoot();
                    CurrentState = KurryState.eLaunch;

                    break;
                case ePark:
                    if(alliance == Alliance.eRed) {
                        strafing(20, false);
                    }
                    else {
                        strafing(20, true);
                    }
                    CurrentState = KurryState.eDone;
                    break;
                case eFind_More_Artifacts:
                    turnRelative(0.3,SIGN_Alliance * 45,1000);
                    if(alliance == Alliance.eRed) {
                        strafing(20, false);
                    }
                    else {
                        strafing(20, true);
                    }

                    break;
            }
        }
        // Stop camera
        aprilTagHelper.stop();
    }

    private void FindPost() {
        if (findPostOneTime )
        {
            double angle = TurnToPost();
            telemetry.addData("Angle rotated is ",+ angle);
            telemetry.update();
            PrepForLaunch();
            PrepDone = true;

            findPostOneTime = false;
            stateTimer.reset();
        }


        List detections = aprilTagHelper.getDetections();
        aprilTagHelper.telemetryAprilTag(telemetry);
        telemetry.update();
        int post = RedPost;
        if(Alliance.eBlue == alliance)
        {
            post = BluePost;
        }
        if(!detections.isEmpty()  && ((org.firstinspires.ftc.vision.apriltag.AprilTagDetection) detections.get(0)).id  == post){
            org.firstinspires.ftc.vision.apriltag.AprilTagDetection tag = (org.firstinspires.ftc.vision.apriltag.AprilTagDetection) detections.get(0);

            PostTag = ((org.firstinspires.ftc.vision.apriltag.AprilTagDetection) detections.get(0));
            CurrentState = KurryState.eAlign_POST;
            findPostOneTime = true;
            findPost = true;

            TelemetryRobotState();
            telemetry.addData("Range",  "%5.1f inches", PostTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", PostTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", PostTag.ftcPose.yaw);
            telemetry.update();
            return;
        }
        driveStraight(POST_DISTANCE,false);
        Find_Post_Dist_Moved =+ POST_DISTANCE;
    if(stateTimer.milliseconds() >= fing_Post_Timeout)
        {
            driveStraight(Find_Post_Dist_Moved,true);
            CurrentState = KurryState.eLaunch;
            findPost = false;
            findPostOneTime = true;
            return;
        }
    }

    private double TurnToPost() {
        setMotorsNOTUsingEncoders();
        double angle = SIGN_Alliance * 90;
        turnRelative(0.3, angle, 1500); //SIGN_Alliance *
        telemetryIMU();
        return angle;
    }

    private void BackToShoot(){
        driveStraight(21,false);
        if (alliance == Alliance.eRed){
            strafing(20,true);
        }else {
            strafing(20,false);
        }
        turnRelative(0.3,45*SIGN_Alliance,2000);
    }
    private void Sort(double order){
        intake.setPower(0.767);
        if(order == 1){
            driveStraight(10,true);
            divider.setPower(1);
            driveStraight(5,true);
            divider.setPower(-1);
            driveStraight(6,true);
            divider.setPower(0);
        }
    }
private void AlignPost() {

    double drive = 1, turn = 1 , strafe  = 1;

    if(!targetHeadingInit) {
        // Reset timer so we can timeout if tag doesnt align in time
        stateTimer.reset();
        setMotorsNOTUsingEncoders();
        targetHeadingInit = true;
    }
    List detections = aprilTagHelper.getDetections();
    if(!detections.isEmpty()) {
        PostTag = (org.firstinspires.ftc.vision.apriltag.AprilTagDetection) detections.get(0);
        if(alliance == Alliance.eBlue){
             diff = 2;
        } else{
            diff = -2;

        }
        if (PostTag != null) {
            double rangeError = (PostTag.ftcPose.range - 40);
            double headingError = PostTag.ftcPose.bearing - diff;
            double yawError = PostTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Range",  "%5.1f inches", PostTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", PostTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", PostTag.ftcPose.yaw);
            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            telemetry.addData("Range","Error %5.2f, Tolerance %5.2f",rangeError,POST_RANGE_TOL);
            telemetry.addData("Yaw","Error %5.2f, Tolerance %5.2f",yawError, POST_YAW_TOL);
            telemetry.addData("Bearing","Heading error %5.2f, Bearing tol %5.2f", headingError,POST_BEARING_TOL);

            telemetry.update();

            //sleep(200);
            moveRobotForTurn(drive,strafe,turn);
            sleep(100); // configure
            moveRobot(0,0);


            if(Math.abs(rangeError)<POST_RANGE_TOL && Math.abs(headingError) < POST_BEARING_TOL && Math.abs(yawError)<POST_YAW_TOL){
                telemetry.addLine("POST aligned");
                telemetry.update();
                moveRobot(0,0);
                targetHeadingInit = false;
                Post_AlignTelemetry();
                CurrentState = KurryState.eLaunch;
                return;

            }
        }
        else {
            // fallback heading if no  Tag
            telemetry.addLine("fallback ");
            telemetry.update();
            sleep(500);
            targetHeading = fallbackLaunchHeading;
        }
    }
    else {
        telemetry.addLine("Detection is empty");
        telemetry.update();
        Post_AlignTelemetry();
        CurrentState = KurryState.eLaunch;
    }

}

    private void Post_AlignTelemetry() {
        if(PostTag != null) {
            LATEST_Bearing = PostTag.ftcPose.bearing;
            LATEST_Range = PostTag.ftcPose.range;
        }
        TelemetryRobotState();
        //sleep(2000);
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

        telemetry.addData("Latest Range " , +LATEST_Range);
        telemetry.addData("LATEST bearing", +LATEST_Bearing);
        telemetry.update();
    }

    private void InitializeMotorServosEverything() {
        // ===== Initialize Motors & Servos =====


        findMOTIFStrafing = true;
        findPostOneTime = true;
        targetHeadingInit = false;
        PrepDone = false;
        TOTAL_STAF = 0.0;
        Find_Post_Dist_Moved = 0.0;

        SIGN_Alliance = -1;
        alliance = Alliance.eRed;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
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

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        setMotorsUsingEncoders();

        aprilTagHelper = new AprilTagHelper(hardwareMap);
        telemetry.addLine("AprilTag Helper initialized...");
        telemetry.update();
    }

    private void telemetryIMU() {
        telemetry.addData("imu Head  Pitch, Roll" , "%5.2f %5.2f  %5.2f", getHeading(),getPitch(),getRoll());
        telemetry.update();
        //sleep(2000);
    }

    private void launch(int pattern) {

        switch (pattern) {
            case 23:

                rightLaunch();
                divide(false);
                rightLaunch();
                divide(true);
                leftLaunch();

                break;
            case 22:

                rightLaunch();

                leftLaunch();
                divide(false);
                rightLaunch();
                break;
            case 21:
            default:
                leftLaunch();
                rightLaunch();
                divide(false);
                rightLaunch();
                break;

        }

        divider.setPower(0);
        intake.setPower(0);
        launcherLeft.setPower(0);
        launcherRight.setPower(0);
    }

    private void PrepForLaunch() {

        if(Alliance.eBlue == alliance){
            launcherLeft.setPower(Blue_Out);
            launcherRight.setPower(Blue_IN);
            telemetry.addData("Launch Left Power", + Blue_Out);
            telemetry.addData("Launch Right Power" , Blue_IN);
            telemetry.update();

        }else {
            launcherLeft.setPower(Red_In);
            launcherRight.setPower(Red_Out);
            telemetry.addData("Launch Left Power", + Red_In);
            telemetry.addData("Launch Right Power" ,+ Red_Out);
            telemetry.update();
        }

        divider.setPower(0);
        intake.setPower(1);
        prepTimer.reset();

    }

    private void divide(boolean isLeft){

        if(isLeft){
            divider.setPower(-1);//move to the left

        } else{
            divider.setPower(1);
        }
        sleep(3000);
    }

    private void rightLaunch() {

        flapperRight.setPosition(0.65);
        sleep(1500);
        flapperRight.setPosition(0.81);
        sleep(1500);

    }

    private void leftLaunch() {

        if(alliance == Alliance.eRed)
        {
            strafing(2,false);
        }
        flapperLeft.setPosition(0.4);
        sleep(1000);
        flapperLeft.setPosition(0.55);
        sleep(1000);
        if(alliance == Alliance.eRed)
        {
            strafing(2, true);
        }

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
                runtime.seconds() < 3) {
            telemetry.addData("Driving", inches);
            telemetry.update();
        }
        stopAll();
        sleep(50);
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
      //  double targetHeading = AngleUnit.normalizeDegrees(getHeading() + deltaAngle);
        //turnToHeading(maxTurnSpeed, targetHeading, timeoutInMS);
        setMotorsNOTUsingEncoders();

        double startHeading = getHeading();
        double direction = Math.signum(deltaAngle); // +CCW, -CW
        double targetAngle = Math.abs(deltaAngle);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.milliseconds() < timeoutInMS) {

            double currentHeading = getHeading();

            // How far have we rotated since start
            double delta = AngleUnit.normalizeDegrees(currentHeading - startHeading);

            double rotated = Math.abs(delta);

            if (rotated >= targetAngle - HEADING_THRESHOLD) {
                break;
            }

            double turn = direction * maxTurnSpeed;
            moveRobotForTurn(0, 0, turn);

            sendTelemetry(false);
        }

        moveRobot(0, 0);
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
             //   sleep(500);
                break;
            }
            if(runtime.milliseconds() > timeoutInMS)
            {
                telemetry.addLine("turn timeout");
                telemetry.update();
               // sleep(500);
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

    public void moveRobotForTurn(double x, double y, double yaw) {
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
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
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