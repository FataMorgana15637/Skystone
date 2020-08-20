package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "TeleOpTest", group = "teamcode")
public class TeleOpTest extends Robot {

    private int grandFlag = 0;

    private boolean upStep1 = true;
    private boolean upStep2;
    private boolean upStep3;
    private boolean upStep4;

    private int num = 0;
    private int stayingPosition = 0;
    private double power = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        runtime.reset();

        SkyStone.setPosition(SkyStoneUP);
        Foundation.setPosition(DontCatchFoundation);
        waitForStart();
        while (opModeIsActive()) {
            double time=runtime.seconds();

            angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double heading = angles.firstAngle;

            if (gamepad1.b) MyDriveTrain.setMode("arcade");
            else if (gamepad1.x) MyDriveTrain.setMode("Oriented");

            telemetry.addData("Mode: ", MyDriveTrain.Mode);
            telemetry.addData("Position:",MyElevator.getCurrentPosition());
            telemetry.update();

            if (gamepad1.right_trigger>0){
                MyDriveTrain.fieldOriented(gamepad1.left_stick_y*0.4, gamepad1.left_stick_x*0.4, gamepad1.right_stick_x*0.4, heading+1.5707963267948 );
                // MyDriveTrain.arcade(gamepad1.left_stick_y*0.4, gamepad1.left_stick_x*0.4, gamepad1.right_stick_x*0.4);
            }

            else{
                if (MyDriveTrain.getMode().equals("Oriented")) {
                    MyDriveTrain.fieldOriented(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, heading+1.5707963267948 );
                } else {
                    MyDriveTrain.arcade(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                }
            }
            telemetry.addLine("Power is" + String.valueOf(LB.getPower()));
//            if (gamepad1.x) {
//                Arm.setPosition(0);
//            } else if (gamepad1.b) {
//                Arm.setPosition(1);
//            }
//            telemetry.addData("Arm Position", Arm.getPosition());
//            telemetry.update();

//            if (gamepad1.a) {
//                Output.setPosition(1);
//            } else if (gamepad1.y) {
//                Output.setPosition(0);
//            }

        /*   if (gamepad1.right_trigger > 0) {
               MyElevator.dontMoveElevator(0.5,0.02);
           }*/
           /*else if (gamepad1.left_trigger > 0) {
               MyElevator.setPower(-1, -1);
           } else {
               MyElevator.setPower(0, 0);
           }*/

            if (gamepad1.dpad_left && Base==0){
                Foundation.setPosition(DontCatchFoundation);
                Base=1.0;
                sleep(600);
            }
            else if (gamepad1.dpad_left && Base==1){
                Foundation.setPosition(CatchFoundation);
                Arm.setPosition(ArmUp);
                Base=0.0;
                sleep(600);
            }

            stayingPosition = stayingPosition;

//            telemetry.addData("leftEncodersLinear", leftLinearMotor.getCurrentPosition());
//            telemetry.addData("rightEncodersLinear", rightLinearMotor.getCurrentPosition());
//            telemetry.addData("stayingPosition:", stayingPosition);
//            telemetry.update();
            //  SkyStone.setPosition(SkyStoneUP);

            if (gamepad2.dpad_right){
                SkyStone.setPosition(SkyStoneDOWN);
            }
            else if (gamepad2.dpad_left){
                SkyStone.setPosition(SkyStoneUP);
            }

            if (gamepad2.a){
                Finger.setPosition(FingerCatch);
            }
            else if (gamepad2.y){
                Finger.setPosition(FingerDontCatch);
            }

            if (gamepad1.left_trigger>0 && gamepad1.right_trigger>0 && gamepad1.y){
                Meter.setPower(0.8);
            }

            else if (gamepad1.left_trigger>0 && gamepad1.right_trigger>0 && gamepad1.a){
                Meter.setPower(-0.8);
            }


            else if (gamepad2.right_trigger>0 && gamepad1.left_trigger==0){

                Meter.setPower(gamepad2.right_trigger*0.8);
            }
            else if (gamepad2.left_trigger>0 && gamepad1.left_trigger==0){

                Meter.setPower(-gamepad2.left_trigger*0.7);
            }
            else {

                Meter.setPower(0);
            }


            while (gamepad1.y){
                ArmMid=ArmMid+0.001;
                if (ArmMid>=ArmUp){
                    ArmMid=ArmUp;
                }
                Arm.setPosition(ArmMid);
            }
            while (gamepad1.a){
                ArmMid=ArmMid-0.001;
                if (ArmMid<=ArmDown){
                    ArmMid=ArmDown;
                }
                Arm.setPosition(ArmMid);
            }


/*
           if (gamepad1.y){
               Arm.setPosition(ArmUp);

           }
           else if (gamepad1.a){

               Arm.setPosition(ArmDown);
           }
*/




//            if ((gamepad2.right_trigger > 0 )) {
//                power = 0.2;
//                MyElevator.ElevateWithEncoder(gamepad2.right_trigger,1,1000);
//                stayingPosition = Elevator.getCurrentPosition();
//            }
            telemetry.addData("Magnet:", MagnetDigitalTouch.getState());
            telemetry.update();

            if (gamepad1.dpad_up && gamepad1.right_trigger>0 ){
                power = 0.2;
                MyElevator.ElevateWithEncoder(1,1,1000);
                stayingPosition = Elevator.getCurrentPosition();
            }
            else if (gamepad1.dpad_down && (digitalTouch.getState() == true || MagnetDigitalTouch.getState()==true)/* && gamepad1.right_trigger>0*/){
                power = 0.2;
                MyElevator.ElevateWithEncoder(0.6,-1,300);
                stayingPosition = Elevator.getCurrentPosition();
            }



          /* else if (((gamepad2.left_trigger > 0 && digitalTouch.getState() == false && gamepad1.right_trigger==0))){
               power = 0.2;
               MyElevator.ElevateWithEncoder(gamepad2.left_trigger,-1,1000);
               stayingPosition = Elevator.getCurrentPosition();
           }
*/
            else if (gamepad1.left_trigger>0 && (digitalTouch.getState() == true || MagnetDigitalTouch.getState()==true)){
                power = 1;
                stayingPosition = -100;
            }
            else if (digitalTouch.getState() == false || MagnetDigitalTouch.getState()==false){
                power = 0.2;
                Elevator.setPower(0);
            }
            else {
                Elevator.setTargetPosition(stayingPosition);
                Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Elevator.setPower(power);
                // MyElevator.ElevateWithEncoder(0,0,0);
            }

            //LED.setPower(1);

            //if (digitalTouch.getState() == true){
            if (digitalTouch.getState()==false || MagnetDigitalTouch.getState()==false){
                LED.setPower(1);
            }
            else {
                LED.setPower(0);
            }
          /* if (Elevator.getCurrentPosition()<=200){
              Elevator.setPower(0);
           }
*/
            if (gamepad1.dpad_right && BaseCap==0 && gamepad1.left_trigger>0 && ((MagnetDigitalTouch.getState()==false)||(digitalTouch.getState()==false))){
                SkyStone.setPosition(SkyStoneUP -0.1);
                Arm.setPosition(ArmDown);
                sleep(500);
                CapStone.setPosition(CapStoneDown);
                sleep(850);
                BlockCap.setPosition(DontBlockingCap);
                sleep(700);
                CapStone.setPosition(CapStoneUp);
                sleep(500);
                SkyStone.setPosition(SkyStoneUP);
                BaseCap = 1.0;
            }






            if ((gamepad1.right_bumper)) {
                FingerArm.setPosition(FingerClose);
            }
            else if (gamepad1.left_bumper) {
                FingerArm.setPosition(FingerOpen);
            }
//              else if ((gamepad2.right_bumper)&&(digitalTouch.getState() == false)) {
//                FingerArm.setPosition(FingerClose);
//            }

          /* if (gamepad2.right_bumper) {
               MyIntake.maxIntake();
           } else if (gamepad2.left_bumper) {
               MyIntake.maxOuttake();
           } else {
               MyIntake.ShutDown();
           }
*/
            if (gamepad1.right_stick_button){
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
                parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parameters.loggingEnabled      = true;
                parameters.loggingTag          = "IMU";
                parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
                IMU.initialize(parameters);
            }
            if (!isStopRequested() && !IMU.isGyroCalibrated()){
                idle();
                telemetry.addLine("imu is not calibrated");
            }

//            if (gamepad2.x){
//
//                Output.setPosition(0.75);
//                if(-time+runtime.seconds()>2) {
//                    MyElevator.ElevateWithEncoder(300, 1);
//                    if (rightLinearMotor.getCurrentPosition()>300|| leftLinearMotor.getCurrentPosition()>300)
//                        MyElevator.dontMoveElevator(1,300);
//                        Arm.setPosition(0.75);
//                }
        }
/////////////////////////////////////////////////////////////////////////////////////////////
        //dont press on thes points/*
          /* if (gamepad1.dpad_up){
               grandFlag = 1;
               //סוגר על הקובייה
               if (upStep1 == true){
                   Output.setPosition(0.75);
               }
               //משנה את הסמן
               if (upStep1 == true) {
                   upStep2 = false;
               }
               //מרים את המעלית
               else if (upStep2 == false){
                   MyElevator.setPower(1 , 1);
               }
               if (upStep2 == false){
                   upStep3 = false;
               }
               //מסובב את הזרוע ב180 מעלות
               else if (upStep3 == false){
                   Arm.setPosition(0.75);
               }

               telemetry.addData("upStep1" , upStep1);
               telemetry.addData("upStep2" , upStep2);
               telemetry.addData("upStep3" , upStep3);
               telemetry.addData("Output" , Output.getPosition());
               telemetry.update();
           }*/

           /*if (gamepad1.dpad_down) {
               double flag = 0;
               //מסובב בחזרה את הזרוע
               if (flag == 0){
                   Arm.setPosition(0);
                   Output.setPosition(0);
               }
               //משנה את הסמן
               if (Arm.getPosition() == 1 && Output.getPosition() == 1) flag =1;
               //מוריד את המעלית עד למעלה
               if (flag == 1 && MyElevator.stateDownMagnet()){
                   MyElevator.setPower(-0.5,-0.5);
               }
               if (downMagnetElevator.getState() == true) flag = 2;
               telemetry.addLine("its working DOWN");
               telemetry.update();
           }*/
/////////////////////////////////////////////////////////////////////////////////////////////

//            autoOpenWitheStone(gamepad1.dpad_left);

           /*if (gamepad1.right_trigger > 0) {
               LeftServo.setPosition(0.75);
               RightServo.setPosition(0.25);
           } else if (gamepad1.left_trigger > 0) {
               LeftServo.setPosition(0.4);
               RightServo.setPosition(0.5);
           }*/
    }
}
