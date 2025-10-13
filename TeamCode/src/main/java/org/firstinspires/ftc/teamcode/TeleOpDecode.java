package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp(name = "TeleOp2025", group = "Linear Opmode")
public class TeleOp2025 extends LinearOpMode {

    // --- Drive motors ---
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- Mechanism motors ---
    public DcMotor intakeMotor;
    public DcMotor launcherLeftMotor;
    public DcMotor launcherRightMotor;

    // --- Servos ---
    public Servo flapServo;
    public CRServo launcherLeftServo;
    public CRServo launcherRightServo;

    // --- Distance sensor ---
    private DistanceSensor distanceSensor;
    private Rev2mDistanceSensor rev2mSensor;

    // --- Vision (AprilTag) ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // --- Targeting constants ---
    public static final double DESIRED_LAUNCH_DISTANCE_CM = 90.0;
    public static final double DISTANCE_TOLERANCE_CM = 5.0;

    @Override
    public void runOpMode() {

        // ---- Drive Motors ----
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- Mechanism Motors ----
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launcherLeftMotor = hardwareMap.get(DcMotor.class, "launcherLeftMotor");
        launcherRightMotor = hardwareMap.get(DcMotor.class, "launcherRightMotor");

        launcherLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- Servos ----
        flapServo = hardwareMap.get(Servo.class, "flapServo");
        launcherLeftServo = hardwareMap.get(CRServo.class, "launcherLeftServo");
        launcherRightServo = hardwareMap.get(CRServo.class, "launcherRightServo");

        // ---- Distance Sensor ----
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        rev2mSensor = (Rev2mDistanceSensor) distanceSensor;

        // ---- AprilTag Vision ----
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.getAll(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class).get(0))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("🔵 Initialized — Press PLAY to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------------- DRIVE ----------------
            double driveY = -gamepad1.left_stick_y;  // forward/back
            double driveX =  gamepad1.left_stick_x;  // strafe
            double turn   =  gamepad1.right_stick_x; // rotate

            double fl = driveY + driveX + turn;
            double fr = driveY - driveX - turn;
            double bl = driveY - driveX + turn;
            double br = driveY + driveX - turn;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) {
                fl /= max; fr /= max; bl /= max; br /= max;
            }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // ---------------- INTAKE CONTROL ----------------
            if (gamepad2.left_bumper) {
                intakeMotor.setPower(1.0); // intake in
            } else if (gamepad2.right_bumper) {
                intakeMotor.setPower(-1.0); // reverse out
            } else {
                intakeMotor.setPower(0.0);
            }

            // ---------------- FLAP SERVO ----------------
            if (gamepad2.dpad_left) {
                flapServo.setPosition(0.0);  // push left
            } else if (gamepad2.dpad_right) {
                flapServo.setPosition(1.0);  // push right
            } else if (gamepad2.dpad_up) {
                flapServo.setPosition(0.5);  // center
            }

            // ---------------- LAUNCHER MOTORS ----------------
            if (gamepad2.a) {
                launcherLeftMotor.setPower(1.0);
            } else {
                launcherLeftMotor.setPower(0.0);
            }

            if (gamepad2.b) {
                launcherRightMotor.setPower(1.0);
            } else {
                launcherRightMotor.setPower(0.0);
            }

            // ---------------- CR SERVOS ----------------
            double leftServoPower = gamepad2.left_trigger;
            double rightServoPower = gamepad2.right_trigger;

            launcherLeftServo.setPower(leftServoPower);
            launcherRightServo.setPower(rightServoPower);

            // ---------------- DISTANCE SENSOR ----------------
            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addLine("📏 Distance Sensor Data:");
            telemetry.addData("• Range (cm)", String.format("%.1f", distanceCM));
            telemetry.addData("• Timeout?", rev2mSensor.didTimeoutOccur());

            // ---------------- APRILTAG DETECTION ----------------
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (detections.size() > 0) {
                AprilTagDetection tag = detections.get(0);
                double tagDistanceCM = tag.ftcPose.range * 100.0;

                telemetry.addLine("\n🟣 AprilTag Detected:");
                telemetry.addData("• Tag ID", tag.id);
                telemetry.addData("• Tag Distance (cm)", String.format("%.1f", tagDistanceCM));

                double delta = distanceCM - DESIRED_LAUNCH_DISTANCE_CM;
                if (Math.abs(delta) <= DISTANCE_TOLERANCE_CM) {
                    telemetry.addLine("✅ PERFECT SPOT! Ready to launch!");
                } else if (delta > 0) {
                    telemetry.addData("Status", "Too FAR by %.1f cm", delta);
                } else {
                    telemetry.addData("Status", "Too CLOSE by %.1f cm", -delta);
                }

            } else {
                telemetry.addLine("❌ No AprilTag detected — align robot toward tag");
            }

            // ---------------- TELEMETRY ----------------
            telemetry.addData("Intake Power", intakeMotor.getPower());
            telemetry.addData("Launcher Left Power", launcherLeftMotor.getPower());
            telemetry.addData("Launcher Right Power", launcherRightMotor.getPower());
            telemetry.addData("CRServos L/R", "%.2f / %.2f", launcherLeftServo.getPower(), launcherRightServo.getPower());
            telemetry.addData("Flap Servo Pos", "%.2f", flapServo.getPosition());
            telemetry.update();
        }

        visionPortal.close();
    }
}
