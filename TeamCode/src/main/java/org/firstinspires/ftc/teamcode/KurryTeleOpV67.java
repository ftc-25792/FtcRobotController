diff --git a/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/KurryTeleOpFinal.java b/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/KurryTeleOpFinal.java
index 6f2b159..c076731 100644
--- a/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/KurryTeleOpFinal.java
+++ b/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/KurryTeleOpFinal.java
@@ -23,31 +23,50 @@ public class KurryTeleOpFinal extends LinearOpMode {
     private Servo flapperLeft, flapperRight;
     private CRServo servoWheel;
     private IMU imu;
-    private ElapsedTime stateTimer = new ElapsedTime();
-    private boolean targetHeadingInit = false;
-    private KurryStateDrive CurrentState = KurryStateDrive.eDrive;
-    private double diff = -8;
+
+    private AprilTagHelper aprilTagHelper;
+    private AprilTagDetection lockedTag = null;
+
+    private ElapsedTime alignTimer = new ElapsedTime();
+    private ElapsedTime lostTagTimer = new ElapsedTime();
+
+    private boolean alignInit = false;
 
     private static final double SPEED_FACTOR = 0.7;
-    private Alliance alliance = Alliance.eRed;
-    private static double SIGN_Alliance = -1;
 
-    private static final double SPEED_GAIN = 0.03;
-    private static final double TURN_GAIN  = 0.01;
+    private static final double SPEED_GAIN  = 0.03;
     private static final double STRAFE_GAIN = 0.02;
-    private static final double MAX_AUTO_SPEED = 0.5;
-    private static final double MAX_AUTO_TURN  = 0.3;
-    private static final double MAX_AUTO_STRAFE = 0.5;
+    private static final double TURN_GAIN   = 0.01;
 
-    private static final double RANGE_TOLERANCE = 1;
-    private static final double BEARING_TOLERANCE = 1.0;
-    private static final double YAW_TOLERANCE = 1.0;
+    private static final double MAX_DRIVE  = 0.5;
+    private static final double MAX_STRAFE = 0.5;
+    private static final double MAX_TURN   = 0.3;
 
-    private AprilTagHelper aprilTagHelper;
-    private AprilTagDetection firstTag = null;
+    private static final double TARGET_RANGE = 60.0;
+    private static final double RANGE_TOL = 1.0;
+    private static final double BEARING_TOL = 1.0;
+    private static final double YAW_TOL = 1.0;
+
+    enum DriveState { DRIVE, ALIGN }
+    enum Alliance { RED, BLUE }
+
+    private DriveState state = DriveState.DRIVE;
+    private Alliance alliance = Alliance.RED;
+
+    private double allianceSign = -1;
+
+    // ================= PID VARIABLES (ADDED) =================
+    private double kP = 0.0008;
+    private double kI = 0.0000008;
+    private double kD = 0.00015;
 
-    enum KurryStateDrive { eDrive, eAlign_POST }
-    enum Alliance { eRed, eBlue }
+    private double leftIntegral = 0, rightIntegral = 0;
+    private double leftLastError = 0, rightLastError = 0;
+
+    private double targetVelocityLeft = 0;
+    private double targetVelocityRight = 0;
+
+    private ElapsedTime pidTimer = new ElapsedTime();
 
     @Override
     public void runOpMode() {
@@ -56,29 +75,21 @@ public class KurryTeleOpFinal extends LinearOpMode {
         frontRight  = hardwareMap.get(DcMotor.class, "frontRight");
         backLeft    = hardwareMap.get(DcMotor.class, "backLeft");
         backRight   = hardwareMap.get(DcMotor.class, "backRight");
-        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
         launcherLeft  = hardwareMap.get(DcMotorEx.class, "launcherLeft");
-        intake      = hardwareMap.get(DcMotor.class, "intake");
+        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
+        intake = hardwareMap.get(DcMotor.class, "intake");
         flapperLeft = hardwareMap.get(Servo.class, "fl");
         flapperRight = hardwareMap.get(Servo.class, "fr");
-        servoWheel  = hardwareMap.get(CRServo.class, "sw");
+        servoWheel = hardwareMap.get(CRServo.class, "sw");
         imu = hardwareMap.get(IMU.class, "imu");
 
         frontLeft.setDirection(DcMotor.Direction.REVERSE);
         backLeft.setDirection(DcMotor.Direction.REVERSE);
-        frontRight.setDirection(DcMotor.Direction.FORWARD);
-        backRight.setDirection(DcMotor.Direction.FORWARD);
 
         launcherLeft.setDirection(DcMotor.Direction.REVERSE);
-        launcherRight.setDirection(DcMotor.Direction.FORWARD);
-        flapperLeft.setDirection(Servo.Direction.REVERSE);
-        flapperRight.setDirection(Servo.Direction.FORWARD);
 
-        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
-        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
-        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
-        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
-        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
+        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
+        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
 
         imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                 RevHubOrientationOnRobot.LogoFacingDirection.UP,
@@ -86,144 +97,108 @@ public class KurryTeleOpFinal extends LinearOpMode {
 
         aprilTagHelper = new AprilTagHelper(hardwareMap);
 
-        telemetry.addLine("Ready to Start");
-        telemetry.update();
-
-        while (!isStarted() && !isStopRequested()) {
-            if (gamepad1.b) alliance = Alliance.eRed;
-            if (gamepad1.x) alliance = Alliance.eBlue;
-
-            SIGN_Alliance = (alliance == Alliance.eBlue) ? 1 : -1;
-
+        while (!isStarted()) {
+            if (gamepad1.b) alliance = Alliance.RED;
+            if (gamepad1.x) alliance = Alliance.BLUE;
+            allianceSign = (alliance == Alliance.BLUE) ? 1 : -1;
             telemetry.addData("Alliance", alliance);
-            telemetry.addData("Sign", SIGN_Alliance);
             telemetry.update();
         }
 
         waitForStart();
+        pidTimer.reset();
 
         while (opModeIsActive()) {
-            switch (CurrentState) {
-                case eDrive:
-                    driveControls();
-                    if (gamepad1.a) CurrentState = KurryStateDrive.eAlign_POST;
-                    break;
-                case eAlign_POST:
-                    alignPostTeleOp();
-                    if (gamepad1.b) { stopAll(); CurrentState = KurryStateDrive.eDrive; firstTag = null; }
-                    break;
+
+            updateLaunchers(); // ===== PID RUNS HERE (ADDED) =====
+
+            if (state == DriveState.DRIVE) {
+                driveControls();
+                if (gamepad1.a) {
+                    state = DriveState.ALIGN;
+                    alignInit = false;
+                }
+            } else {
+                alignToPost();
+                if (gamepad1.b) exitAlign();
             }
+
             telemetry.update();
         }
     }
 
     private void driveControls() {
-
-        double y  = -gamepad1.left_stick_y;
-        double x  =  gamepad1.left_stick_x;
-        double rx =  gamepad1.right_stick_x;
-        moveRobot(y * SPEED_FACTOR, x * SPEED_FACTOR, rx * SPEED_FACTOR);
-
-        double pL = 0, pR = 0;
-        if (gamepad2.left_trigger > 0.2) pL = 0.43;
-        else if (gamepad2.left_bumper)  pL = 0.48;
-        else if (gamepad2.dpad_up)      pL = 0.75;
-
-        if (gamepad2.right_trigger > 0.2) pR = 0.467;
-        else if (gamepad2.right_bumper) pR = 0.55;
-        else if (gamepad2.y) pR = 0.75;
-
-        launcherLeft.setPower(pL);
-        launcherRight.setPower(pR);
-
-        if (gamepad1.right_trigger > 0.2) {
-            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
-            intake.setPower(0.8);
-        } else if (gamepad1.left_trigger > 0.2) {
-            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
-            intake.setPower(-0.8);
-        } else {
-            if (intake.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
-                intake.setTargetPosition(intake.getCurrentPosition());
-                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
-                intake.setPower(0.5);
-            }
-        }
-
-        flapperLeft.setPosition(gamepad2.b ? 0.4 : 0.55);
-        flapperRight.setPosition(gamepad2.dpad_left ? 0.65 : 0.82);
-
-        if (gamepad2.left_stick_button) servoWheel.setPower(1.0);
-        else if (gamepad2.right_stick_button) servoWheel.setPower(-1.0);
-        else if (gamepad2.dpad_down) servoWheel.setPower(0);
+        moveRobot(
+                -gamepad1.left_stick_y * SPEED_FACTOR,
+                gamepad1.left_stick_x * SPEED_FACTOR,
+                gamepad1.right_stick_x * SPEED_FACTOR
+        );
     }
 
-    private void alignPostTeleOp() {
-        if(!targetHeadingInit) {
-            // Reset timer so we can timeout if tag doesnt align in time
-            stateTimer.reset();
-            setMotorsNOTUsingEncoders();
-            targetHeadingInit = true;
+    private void alignToPost() {
+
+        if (!alignInit) {
+            alignTimer.reset();
+            lostTagTimer.reset();
+            setRunWithoutEncoders();
+            lockedTag = null;
+            alignInit = true;
         }
 
-        // Get latest tag detection continuously
         List<AprilTagDetection> detections = aprilTagHelper.getDetections();
+
         if (detections != null && !detections.isEmpty()) {
-            firstTag = detections.get(0); // always use the latest
+            lockedTag = getClosestTag(detections);
+            lostTagTimer.reset();
         }
 
-        if (firstTag == null) {
-            telemetry.addLine("No POST detected - Searching...");
-            stopAll();
+        if (lockedTag == null && lostTagTimer.seconds() > 0.4) {
+            moveRobot(0, 0, 0.2 * allianceSign);
             return;
         }
 
-        // Calculate errors
-        double rangeError = firstTag.ftcPose.range - 60;
-        double bearingError = firstTag.ftcPose.bearing - (4*SIGN_Alliance);
-        double yawError = firstTag.ftcPose.yaw;
+        if (alignTimer.seconds() > 3.0) {
+            exitAlign();
+            return;
+        }
 
-        // Check if aligned
-        boolean isDistanceAligned = Math.abs(rangeError) < RANGE_TOLERANCE;
-        boolean isBearingAligned  = Math.abs(bearingError) < BEARING_TOLERANCE;
-        boolean isYawAligned      = Math.abs(yawError) < YAW_TOLERANCE;
+        double rangeError = lockedTag.ftcPose.range - TARGET_RANGE;
+        double bearingError = lockedTag.ftcPose.bearing;
+        double yawError = lockedTag.ftcPose.yaw * allianceSign;
 
-        if (isDistanceAligned && isBearingAligned && isYawAligned) {
-            stopAll();
+        if (Math.abs(rangeError) < RANGE_TOL &&
+                Math.abs(bearingError) < BEARING_TOL &&
+                Math.abs(yawError) < YAW_TOL) {
             gamepad1.rumble(250);
-            CurrentState = KurryStateDrive.eDrive;
-            firstTag = null;
-            return;
-        }
-        if(stateTimer.milliseconds() >= 1000)
-        {
-            CurrentState = KurryStateDrive.eDrive;
+            exitAlign();
             return;
         }
 
-        // Apply gains with clamping to prevent spinning
-        double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
-        double strafe = Range.clip(-bearingError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
-        double turn   = Range.clip(-yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
-
-        if (Math.abs(drive)  < 0.02) drive = 0;
-        if (Math.abs(strafe) < 0.02) strafe = 0;
-        if (Math.abs(turn)   < 0.02) turn = 0;
+        double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_DRIVE, MAX_DRIVE);
+        double strafe = Range.clip(-bearingError * STRAFE_GAIN, -MAX_STRAFE, MAX_STRAFE);
+        double turn   = Range.clip(-yawError * TURN_GAIN, -MAX_TURN, MAX_TURN);
 
-        // Move the robot
         moveRobot(drive, strafe, turn);
+    }
 
-        // Telemetry for debugging
-        telemetry.addLine("AutoAlign: Active");
-        telemetry.addData("Errors (R/B/Y)", "%5.2f / %5.2f / %5.2f", rangeError, bearingError, yawError);
-        telemetry.addData("Outputs (D/S/T)", "%5.2f / %5.2f / %5.2f", drive, strafe, turn);
+    private AprilTagDetection getClosestTag(List<AprilTagDetection> detections) {
+        AprilTagDetection best = null;
+        double bestRange = Double.MAX_VALUE;
+        for (AprilTagDetection tag : detections) {
+            if (tag.ftcPose.range < bestRange) {
+                bestRange = tag.ftcPose.range;
+                best = tag;
+            }
+        }
+        return best;
     }
 
-    private void setMotorsNOTUsingEncoders(){
-        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
-        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
-        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
-        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
+    private void exitAlign() {
+        stopAll();
+        setRunUsingEncoders();
+        state = DriveState.DRIVE;
+        alignInit = false;
+        lockedTag = null;
     }
 
     private void moveRobot(double y, double x, double rx) {
@@ -231,10 +206,9 @@ public class KurryTeleOpFinal extends LinearOpMode {
         double fr = y - x - rx;
         double bl = y - x + rx;
         double br = y + x - rx;
-
-        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
-        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }
-
+        double max = Math.max(Math.abs(fl),
+                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
+        if (max > 1) { fl /= max; fr /= max; bl /= max; br /= max; }
         frontLeft.setPower(fl);
         frontRight.setPower(fr);
         backLeft.setPower(bl);
@@ -247,4 +221,72 @@ public class KurryTeleOpFinal extends LinearOpMode {
         backLeft.setPower(0);
         backRight.setPower(0);
     }
+
+    private void setRunWithoutEncoders() {
+        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
+        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
+        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
+        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
+    }
+
+    private void setRunUsingEncoders() {
+        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
+        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
+        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
+        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
+    }
+
+    // ================= LAUNCHER PID (ADDED AT BOTTOM) =================
+
+    private void updateLaunchers() {
+
+        targetVelocityLeft = 0;
+        targetVelocityRight = 0;
+
+        if (gamepad2.left_trigger > 0.2)  targetVelocityLeft  = 1900;
+        if (gamepad2.right_trigger > 0.2) targetVelocityRight = 2000;
+
+        if (gamepad2.left_bumper)  targetVelocityLeft  = 1900;
+        if (gamepad2.right_bumper) targetVelocityRight = 2000;
+
+        if (gamepad2.dpad_up) targetVelocityLeft  = 4500;
+        if (gamepad2.y)       targetVelocityRight = 4500;
+
+        double dt = pidTimer.seconds();
+        pidTimer.reset();
+
+        double leftPower = launcherPID(
+                targetVelocityLeft,
+                launcherLeft.getVelocity(),
+                dt,
+                true
+        );
+
+        double rightPower = launcherPID(
+                targetVelocityRight,
+                launcherRight.getVelocity(),
+                dt,
+                false
+        );
+
+        launcherLeft.setPower(Range.clip(leftPower, -1, 1));
+        launcherRight.setPower(Range.clip(rightPower, -1, 1));
+    }
+
+    private double launcherPID(double target, double current, double dt, boolean isLeft) {
+
+        double error = target - current;
+
+        if (isLeft) {
+            leftIntegral += error * dt;
+            double derivative = (error - leftLastError) / dt;
+            leftLastError = error;
+            return (kP * error) + (kI * leftIntegral) + (kD * derivative);
+        } else {
+            rightIntegral += error * dt;
+            double derivative = (error - rightLastError) / dt;
+            rightLastError = error;
+            return (kP * error) + (kI * rightIntegral) + (kD * derivative);
+        }
+    }
 }
