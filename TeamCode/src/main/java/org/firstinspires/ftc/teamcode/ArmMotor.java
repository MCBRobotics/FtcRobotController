package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="ArmMotorTesting", group="Linear OpMode")
public class ArmMotor extends LinearOpMode {

    private DcMotor Arm1 = null;
//    private DcMotor Arm2 = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 4; // originally 288
    static final double     GEAR_REDUCTION    = 72; // originally 2.7778
    static final double     COUNTS_PER_GEAR_REV    = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    static final double     COUNTS_PER_DEGREE    = COUNTS_PER_GEAR_REV/360;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm1 = hardwareMap.get(DcMotor.class, "ArmMotor1");
//        Arm2 = hardwareMap.get(DcMotor.class, "ArmMotor2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();
        Arm1.setTargetPosition(0);
//        Arm2.setTargetPosition(1000);


        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int minPosition = 0;
        int maxPosition = 90;


        while (opModeIsActive()) {
//            if(gamepad1.dpad_up) {
//                Arm1.setTargetPosition(40);
//                Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Arm1.setPower(0.5);
////                Arm2.setTargetPosition(40);
////                Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                Arm2.setPower(0.5);
//            }
            //Arm1.setPower(-0.1);
//            if(gamepad1.dpad_down) {
//                Arm1.setTargetPosition(0);
//                Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Arm1.setPower(0.5);
////                Arm2.setTargetPosition(0);
////                Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                Arm2.setPower(0.5);
//            }
   /*        if (gamepad1.dpad_up && Arm1.getCurrentPosition() < maxPosition) {
      //          Arm1.setPower(0.5);
      //      } else if (gamepad1.dpad_down && Arm1.getCurrentPosition() > minPosition) {
                Arm1.setPower(-0.5);
            } else {
                Arm1.setPower(0);
            }*/
            if(gamepad1.dpad_up){
                Arm1.setTargetPosition(0);
                Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm1.setPower(0.2);
                //wait(1000);
                //Arm1.setPower(0);
            }
            else if (gamepad1.dpad_down){
                Arm1.setTargetPosition(40);
                Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm1.setPower(0.2);
                //wait(1000);
                //Arm1.setPower(0);
            }else{
                Arm1.setPower(0);

            }

            telemetry.addData("Arm Test", Arm1.getCurrentPosition());
            telemetry.update();

            // while(!gamepad1.dpad_up & !gamepad1.dpad_down)
            //   Arm.setPower(0);
            // gamepad1.rumble(10000);static final double     COUNTS_PER_MOTOR_REV    = 288;
            //    static final double     GEAR_REDUCTION    = 2.7778;
            //    static final double     COUNTS_PER_GEAR_REV    = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
            //    static final double     COUNTS_PER_DEGREE    = COUNTS_PER_GEAR_REV/360;
            //sleep(1000);
        }
    }
}
