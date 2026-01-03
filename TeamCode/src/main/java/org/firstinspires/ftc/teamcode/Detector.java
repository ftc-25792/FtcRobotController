package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name = "Ball Order Test", group = "Testing")
public class Detector extends LinearOpMode {

    private BallOrderDetector detector;

    @Override
    public void runOpMode() {

        detector = new BallOrderDetector(hardwareMap);

        sleep(1000);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Ball Order", detector.getBallOrder());
            telemetry.update();
        }

        waitForStart();

        telemetry.addData("FINAL Ball Order", detector.getBallOrder());
        telemetry.update();

        detector.stop();
    }
}
