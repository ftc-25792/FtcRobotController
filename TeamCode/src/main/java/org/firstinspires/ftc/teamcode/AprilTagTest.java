

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Distance Auto Drive)", group = "Sensor")
public class AprilTagTest extends LinearOpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private DistanceSensor sensorDistance;
    private Rev2mDistanceSensor revSensor;

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        revSensor = (Rev2mDistanceSensor) sensorDistance;

        telemetry.addLine("Initialized and Ready. Press Start.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            double distance = sensorDistance.getDistance(DistanceUnit.CM);

            while (opModeIsActive() && distance > 10) {
                if (revSensor.didTimeoutOccur()) {
                    telemetry.addLine("Sensor timeout! Stopping.");
                    setAllPower(0);
                    break;
                }
                double power;
                if (distance > 40) {
                    power = 0.5;
                } else if (distance > 20) {
                    power = 0.3;
                } else {
                    power = 0.15;
                }
                setAllPower(power);

                distance = sensorDistance.getDistance(DistanceUnit.CM);

                // Krithik just used REV2mDistanceSensor logic for telemetry
                telemetry.addData("Device Name", sensorDistance.getDeviceName());
                telemetry.addData("Distance (mm)", "%.1f", sensorDistance.getDistance(DistanceUnit.MM));
                telemetry.addData("Distance (cm)", "%.1f", distance);
                telemetry.addData("Distance (m)", "%.2f", sensorDistance.getDistance(DistanceUnit.METER));
                telemetry.addData("Distance (in)", "%.2f", sensorDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("REV2m ID", String.format("%x", revSensor.getModelID()));
                telemetry.addData("Timeout occurred", revSensor.didTimeoutOccur());
                telemetry.addData("Motor Power", "%.2f", power);
                telemetry.addLine(distance <= 10 ? "Target reached" : "Approaching target...");
                telemetry.update();
            }

            // Stop all motors
            setAllPower(0);
            telemetry.addLine("Stopped. Final distance: " + String.format("%.1f cm", distance));
            telemetry.update();
            sleep(1000);
        }
    }
    private void setAllPower(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }
}
