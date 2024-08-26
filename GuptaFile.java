package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotElectronicP extends LinearOpMode {
   private DcMotor frontLeft = null;
   private DcMotor frontRight = null;
   private DcMotor backLeft = null;
   private DcMotor backRight = null;
   double speed = 0.75;
   long moveDuration = 700; // Move duration in milliseconds. Change this number to adjust the distance traveled per side
   long turnDuration = 325; // Turn duration in milliseconds

   @Override
   public void runOpMode() {
       // Initialize the hardware variables.
       frontLeft = hardwareMap.get(DcMotor.class, "Left_front");
       frontRight = hardwareMap.get(DcMotor.class, "Right_front");
       backLeft = hardwareMap.get(DcMotor.class, "Left_rear");
       backRight = hardwareMap.get(DcMotor.class, "Right_rear");
       backRight.setDirection(DcMotor.Direction.REVERSE);
       frontRight.setDirection(DcMotor.Direction.REVERSE);

       waitForStart();

       if (opModeIsActive()) {
           for (int i = 0; i < 4; i++) {
               // Move forward 20 inches
               frontLeft.setPower(0.75);
               frontRight.setPower(0.75);
               backLeft.setPower(0.75);
               backRight.setPower(0.75);
               sleep(800);

               // Stop moving forward
               frontLeft.setPower(0);
               frontRight.setPower(0);
               backLeft.setPower(0);
               backRight.setPower(0);
               sleep(500); // Short pause before turning

            // Repeat this loop 3 times
            for (int i = 0; i < 3; i++) {
               // Turn right 90 degrees
               frontLeft.setPower(0.4);
               frontRight.setPower(-0.4);
               backLeft.setPower(0.4);
               backRight.setPower(-0.4);
               sleep(500);

               // Stop turning
               frontLeft.setPower(0);
               frontRight.setPower(0);
               backLeft.setPower(0);
               backRight.setPower(0);
               sleep(500);

               // Move forward 6 inches
               frontLeft.setPower(0.25);
               frontRight.setPower(0.25);
               backLeft.setPower(0.25);
               backRight.setPower(0.25);
               sleep(400);


           }
       }
   }
}
