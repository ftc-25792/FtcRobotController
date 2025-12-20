package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="KurryAutoShortScore", group="Auto")
public class KurryAutoShortScore extends LinearOpMode {

    DcMotor fl, fr, bl, br;
    DcMotor intake, launcherL, launcherR;
    Servo flapperL, flapperR;
    CRServo divider;
    IMU imu;


    BallOrderDetector detector;
    String order = "UNKNOWN";


    static final double DRIVE_SPEED = 0.5;
    static final double TURN_KP = 0.012;
    static final double TURN_MIN = 0.08;
    static final double HEADING_TOLERANCE = 1.0;

    static final double COUNTS_PER_REV = 537.7;
    static final double WHEEL_DIAMETER = 4.0;
    static final double COUNTS_PER_INCH =
            COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER);


    @Override
    public void runOpMode() {

        initHardware();
        initIMU();

        detector = new BallOrderDetector(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            order = detector.getBallOrder();
            telemetry.addData("Detected Order", order);
            telemetry.update();
        }

        detector.stop();
        waitForStart();

        driveStraight(-18);
        turnTo(0);

        runIntakeSort(order);
        launchSequence();


    }

    void initHardware() {
        fl = hardwareMap.get(DcMotor.class,"frontLeft");
        fr = hardwareMap.get(DcMotor.class,"frontRight");
        bl = hardwareMap.get(DcMotor.class,"backLeft");
        br = hardwareMap.get(DcMotor.class,"backRight");

        intake = hardwareMap.get(DcMotor.class,"intake");
        launcherL = hardwareMap.get(DcMotor.class,"launcherLeft");
        launcherR = hardwareMap.get(DcMotor.class,"launcherRight");

        flapperL = hardwareMap.get(Servo.class,"fl");
        flapperR = hardwareMap.get(Servo.class,"fr");
        divider = hardwareMap.get(CRServo.class,"sw");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        launcherL.setDirection(DcMotorSimple.Direction.REVERSE);

        flapperL.setDirection(Servo.Direction.REVERSE);
        flapperR.setPosition(0.71);
        flapperL.setPosition(0.3);
    }

    void initIMU() {
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        imu.resetYaw();
    }


    void driveStraight(double inches) {
        int move = (int)(inches * COUNTS_PER_INCH);

        resetEncoders();
        fl.setTargetPosition(move);
        fr.setTargetPosition(move);
        bl.setTargetPosition(move);
        br.setTargetPosition(move);

        setRunToPosition(DRIVE_SPEED);

        while (opModeIsActive() &&
                (fl.isBusy() || fr.isBusy())) {}

        stopDrive();
    }


    void turnTo(double targetAngle) {

        while (opModeIsActive()) {

            double current = getHeading();
            double error = AngleUnit.normalizeDegrees(targetAngle - current);

            if (Math.abs(error) <= HEADING_TOLERANCE) break;

            double turnPower = Range.clip(error * TURN_KP, -0.4, 0.4);

            if (Math.abs(turnPower) < TURN_MIN)
                turnPower = TURN_MIN * Math.signum(turnPower);

            fl.setPower(turnPower);
            bl.setPower(turnPower);
            fr.setPower(-turnPower);
            br.setPower(-turnPower);
        }

        stopDrive();
        sleep(100);
    }

    double getHeading() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        return ypr.getYaw(AngleUnit.DEGREES);
    }


    void runIntakeSort(String order) {

        intake.setPower(0.8);

        switch (order) {
            case "PGP":
                pulseDivider(true);
                pulseDivider(false);
                break;

            case "PPG":
                pulseDivider(false);
                pulseDivider(false);
                break;

            case "GPP":
                pulseDivider(true);
                pulseDivider(true);
                break;

            default:
                pulseDivider(true);
                pulseDivider(false);
        }

        intake.setPower(0);
    }

    void pulseDivider(boolean right) {
        divider.setPower(right ? 1 : -1);
        sleep(450);
        divider.setPower(0);
        sleep(120);
    }


    void launchSequence() {

        launcherL.setPower(0.42);
        launcherR.setPower(0.52);
        sleep(2000);

        rightLaunch();
        leftLaunch();

        launcherL.setPower(0);
        launcherR.setPower(0);
    }

    void rightLaunch() {
        flapperR.setPosition(0.58);
        sleep(700);
        flapperR.setPosition(0.71);
        sleep(700);
    }

    void leftLaunch() {
        flapperL.setPosition(0.14);
        sleep(700);
        flapperL.setPosition(0.3);
        sleep(700);
    }


    
    void resetEncoders() {
        for (DcMotor m : new DcMotor[]{fl,fr,bl,br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    void setRunToPosition(double pwr) {
        for (DcMotor m : new DcMotor[]{fl,fr,bl,br}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(pwr);
        }
    }

    void strafing(double inches, boolean isLeft) {
        double correction = 1.12;
        int move = (int)(inches * COUNTS_PER_INCH * correction);

        resetEncoders();

        if (isLeft) {
            fl.setTargetPosition(-move);
            fr.setTargetPosition(move);
            bl.setTargetPosition(move);
            br.setTargetPosition(-move);
        } else {
            fl.setTargetPosition(move);
            fr.setTargetPosition(-move);
            bl.setTargetPosition(-move);
            br.setTargetPosition(move);
        }

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllPower(DRIVE_SPEED);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() &&
                (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()) &&
                runtime.seconds() < 5) {
            telemetry.addData("Strafing", inches);
            telemetry.update();
        }

        stopDrive();
        sleep(100);
    }

    void setAllPower(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }


    void stopDrive() {
        for (DcMotor m : new DcMotor[]{fl,fr,bl,br})
            m.setPower(0);
    }
}
