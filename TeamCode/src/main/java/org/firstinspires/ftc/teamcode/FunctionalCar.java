/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static java.util.logging.Logger.global;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Functional Car", group="Linear OpMode")
//@Disabled
public class FunctionalCar extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    //    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor leftBackDrive = null;

    private DcMotor Arm1 = null;
    //private DcMotor Arm2 = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        Arm1 = hardwareMap.get(DcMotor.class, "ArmMotor1");
        ///Arm2 = hardwareMap.get(DcMotor.class, "ArmMotor2");
        double powerTele = 0;
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        Arm1.setTargetPosition(0);
        ///Arm2.setTargetPosition(0);
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        ///Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        int zeroPositionArm1 = Arm1.getCurrentPosition();
        //int zeroPositionArm2 = Arm2.getCurrentPosition();




        //this is to set where you want the arms final position to be
        int targetPosition = 30;
        int holdPositionArm1 = 0;

        ///int holdPositionArm2 = 0;

//        int differenceInArms = Arm1.getCurrentPosition() - Arm2.getCurrentPosition();
//        int targetPositionArm2 = targetPosition - differenceInArms;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            //this just updates the distance between the two arms
            //int differenceInArms = Arm1.getCurrentPosition() - Arm2.getCurrentPosition();
            //this syncs up the final postition for arm2 so that it doesnt jitter
            //int targetPositionArm2 = targetPosition - differenceInArms;

            if (gamepad2.dpad_left) {
                double powerArm1 = 0.5;
                while (Arm1.getCurrentPosition()>-130) {
                    Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    Arm1.setPower(powerArm1);
                    telemetry.addData("Arm1 Test", Arm1.getCurrentPosition());
                }
                Arm1.setPower(0);
            }

            if (gamepad2.dpad_down) {
                ///int currentPositionArm1 = Arm1.getCurrentPosition();
                //int currentPositionArm2 = Arm2.getCurrentPosition();


                //This is for power scaling of the arm mototsbut it doesnt work as it currently doesnt work
                //with even full power which means that this code wouldnt work either
//                int errorArm1 = targetPosition - currentPositionArm1;
//                int errorArm2 = targetPositionArm2 - currentPositionArm2;
//
//                // this code was just a test but it does the same as the powerscaling thing below
//                // its faster at the start but it takes longer for it to find the final spot
//                //they do the same thing but the say that the power slows down is different.
//                double distanceArm1 = Math.abs(errorArm1);
//                double distanceArm2 = Math.abs(errorArm2);
//
//                //you could play around with this value a little bit more
//                //im sure what the best value would be but 100 seems to work fine.
//                double decelerationThreshold = 100;
//
//                double maxPower = 0.9;
//                double minPower = 0.7;
//
//                double powerArm1 = minPower + (maxPower - minPower) * Math.min(1, distanceArm1 / decelerationThreshold);
//                double powerArm2 = minPower + (maxPower - minPower) * Math.min(1, distanceArm2 / decelerationThreshold);
//
//                powerArm1 *= Math.signum(errorArm1);
//                powerArm2 *= Math.signum(errorArm2);

                double powerArm1 = 0.5;
                //double powerArm2 = 0.6;

                ////////////////
//
//
//                // Smoothly move upwards
//                int currentPositionArm1 = Arm1.getCurrentPosition();
//                int currentPositionArm2 = Arm2.getCurrentPosition();
//
////                 this is the inital final version, its a bit slow
////                int errorArm1 = targetPosition - currentPositionArm1;
////                int errorArm2 = targetPositionArm2 - currentPositionArm2;
////
////                double kP = 0.02;
////                double maxPower = 1;
////                double minPower = 0.5;
////
////                double powerArm1 = Math.max(minPower, Math.min(maxPower, kP * Math.abs(errorArm1)));
////                double powerArm2 = Math.max(minPower, Math.min(maxPower, kP * Math.abs(errorArm2)));
////
////                powerArm1 *= Math.signum(errorArm1); // Ensure correct direction
////                powerArm2 *= Math.signum(errorArm2);
////
//////                int powerArm1 = 1;
//////                int powerArm2 = 1;

                ///////////////////////


                ///Arm1.setTargetPosition(targetPosition);
                //Arm2.setTargetPosition(targetPositionArm2);

                //you can use RUN_WITHOUT_ENCODER and it seems to run MUCH faster but
                //it doesnt work with this code.
                Arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //Arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                Arm1.setPower(powerArm1);
                //Arm2.setPower(powerArm2);

                ///holdPositionArm1 = currentPositionArm1; // Update hold position
                //holdPositionArm2 = currentPositionArm2;

            }
            else if (gamepad2.dpad_up) {

                    // Set the target position to 130 for the upward movement
                int targetPositionNew = -130;

                    // Set the motor mode to RUN_TO_POSITION so the motor moves to the target position
                Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // Set the target position for the motor
                Arm1.setTargetPosition(targetPositionNew);

                    // Set the power to move the motor towards the target position
                Arm1.setPower(-0.5);  // Full speed to move to the target position
                    // Check if the motor's current position is within the range of -125 to -135
                int currentPosition = Arm1.getCurrentPosition();
                if (currentPosition >= -140 && currentPosition <= -120) {
                        // Once the motor is within the range, stop it and hold
                    Arm1.setPower(0);  // Stop the motor
                    Arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Hold the position

                        // Set the motor's zero-power behavior to BRAKE to ensure it holds the position
                    Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }


                /*
                if (Arm1.getCurrentPosition() >= -140 && Arm1.getCurrentPosition() <= -120) {
                    Arm1.setTargetPosition(Arm1.getCurrentPosition());
                }

                int currentPositionArm1 = Arm1.getCurrentPosition();
                Arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Arm1.setPower(-1);
                */




                //int currentPositionArm2 = Arm2.getCurrentPosition();

//                int errorArm1 = (targetPosition - 100) - currentPositionArm1;
//                //int errorArm2 = (targetPositionArm2 - 100) - currentPositionArm2;
//
//                double kP = 0.01;
//                double maxPower = 0.05;
//                double minPower = 0.005;
//
//                double powerArm1 = Math.max(minPower, Math.min(maxPower, kP * Math.abs(errorArm1)));
//                //double powerArm2 = Math.max(minPower, Math.min(maxPower, kP * Math.abs(errorArm2)));
//
//                powerArm1 *= Math.signum(errorArm1);
                //powerArm2 *= Math.signum(errorArm2);



                // (100-current)/100
//                double powerArm1 = 0.1;

//                if ((Arm1.getCurrentPosition() > -120) || (Arm1.getCurrentPosition() < -140));
                //double powerArm1 = (-((double) (130 + Arm1.getCurrentPosition()) / 130));
//                double powerArm1 = 0.1;
                //////////Math.min(1, distanceArm1 / decelerationThreshold);

                // double powerArm1 = -0.5;
                //Arm1.setTargetPosition(-100);
                //Arm2.setTargetPosition(zeroPositionArm2);
//RUN_TO_POSITION
                //RUN_WITHOUT_ENCODER

                ////Arm1.setMode(DcMotor.ZeroPowerBehavior.BRAKE);
                ///Arm1.setTargetPosition(-130);
                //Arm1.setMode(DcMotor.);
                //Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Arm2.setPower(powerArm2);

                ///holdPositionArm1 = currentPositionArm1; // Update hold position
                //holdPositionArm2 = currentPositionArm2;
            }

            else {
//                // Hold position when Dpad is released
//                Arm1.setTargetPosition(holdPositionArm1);
//                //Arm2.setTargetPosition(holdPositionArm2);
//
//                Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                //Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                Arm1.setPower(0.5);
//                //Arm2.setPower(0.5);
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("hold position Test", holdPositionArm1);

            //telemetry.addData("difference Test", differenceInArms);
            telemetry.addData("targetPosition Test", targetPosition);
            //telemetry.addData("targetPositionArm2 Test", targetPositionArm2);
            telemetry.addData("zeroPositionArm1 Test", zeroPositionArm1);
            //telemetry.addData("zeroPositionArm2 Test", zeroPositionArm2);

            telemetry.addData("Arm1 speed",  (-((double) (130 + Arm1.getCurrentPosition())) / 130));

            // Show the elapsed game time and wheel power.
            telemetry.addData("Arm1 Test", Arm1.getCurrentPosition());
            //telemetry.addData("Arm2 Test", Arm2.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}
