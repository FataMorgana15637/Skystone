package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.doge_cv.SkystoneDetector;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraFactoryImpl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class basicAuto extends Robot {

    public OpenCvCamera webcam;
    public SkystoneDetector skystoneDetector;

    public double Mikum = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

//        TODO: Webcam

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactoryImpl.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();

        skystoneDetector = new SkystoneDetector(2,50,70);
        webcam.setPipeline(skystoneDetector);

        webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);

//      TODO: Anothers
        Arm.setPosition(ArmUp);
        FingerArm.setPosition(FingerClose);
        CapStone.setPosition(CapStoneUp);

//        TODO: IMU

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMU.initialize(parameters);
        // make sure the imu gyro is calibrated before continuing.
        while (!isStarted() && !isStopRequested() && !IMU.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addLine("IMU_IS_READY");
        telemetry.update();

    }

//    /*
//
//    public void RotateP(int degrees, double power, double timeoutR,double KP) {
//        runtime.reset();
//
//
//        double PNumber = 0.0108;
//        double INumber = 0;
//        double SumErrors = 0;
//        double oldSumErors = 0;
//        double NewAngleTarget = 0;
//
//
//        if (MyDriveTrain.getAngle() < degrees) {
//            while (MyDriveTrain.getAngle() < degrees && runtime.seconds() < timeoutR && opModeIsActive()) {
//                //SumErrors = SumErrors + (getAngle() + degrees);
//                double error = MyDriveTrain.getAngle() - degrees;
//
//                LF.setPower(power * error * KP /* * (((getAngle() -  degrees) * PNumber) + SumErrors * INumber)*/);
//                LB.setPower(power * error * KP/* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
//                RF.setPower(-power * error * KP/* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
//                RB.setPower(-power * error * KP/* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
//
//            }
//        } else if (MyDriveTrain.getAngle() > degrees) {
//            while (MyDriveTrain.getAngle() > degrees && runtime.seconds() < timeoutR && opModeIsActive()) {
//                double error = degrees - MyDriveTrain.getAngle();
//
//                LF.setPower(-power * error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
//                LB.setPower(-power * error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
//                RF.setPower(power * error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
//                RB.setPower(power * error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
//
//
//            }
//        } else return;
//        // turn the motors off.
//        LF.setPower(0);
//        LB.setPower(0);
//        RF.setPower(0);
//        RB.setPower(0);
//
////        LastDegrees = degrees;
//
//        // reset -angle tracking on new heading
//    }
//
//
//    //TODO
//
//    public void Rotate(int degrees, double power, double timeoutR) {
//        runtime.reset();
//
//
//        double PNumber = 0.0108;
//        double INumber = 0;
//        double SumErrors = 0;
//        double oldSumErors = 0;
//        double NewAngleTarget = 0;
//
////        if (getAngle()-degrees > 110) {
////
////            PNumber = 0.0088;
////        }
////        if (getAngle()-degrees>30){
////            PNumber = 0.4;
////        }
//        //         else  if(getAngle()-degrees < 30) {
//        //            PNumber = 0.5;
//        //
//        //        }
////        degrees =  degrees-LastDegrees;
////        resetAngle();
//        // restart imu movement tracking.
//
//        if (MyDriveTrain.getAngle() < degrees) {
//            while (MyDriveTrain.getAngle() < degrees && runtime.seconds() < timeoutR) {
//                //SumErrors = SumErrors + (getAngle() + degrees);
//                double error = degrees - MyDriveTrain.getAngle();
//                LF.setPower(-power /* error * KP /* * (((getAngle() -  degrees) * PNumber) + SumErrors * INumber)*/);
//                LB.setPower(-power /* error * KP/* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
//                RF.setPower(power /* error * KP/* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
//                RB.setPower(power /* error * KP/* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
//
//            }
//        } else if (MyDriveTrain.getAngle() > degrees) {
//            while (MyDriveTrain.getAngle() > degrees && runtime.seconds() < timeoutR) {
//                double error = MyDriveTrain.getAngle() - degrees;
//                LF.setPower(power /* error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
//                LB.setPower(power /* error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
//                RF.setPower(-power /* error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
//                RB.setPower(-power /* error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
//
//
//            }
//        } else return;
//        // turn the motors off.
//        LF.setPower(0);
//        LB.setPower(0);
//        RF.setPower(0);
//        RB.setPower(0);
//
////        LastDegrees = degrees;
//
//        // reset -angle tracking on new heading
//    }
//
//    //TODO
//
//    public void driveWithGyro (double speed, double LeftFrontCM, double LeftBackCM, double RightFrontCM, double RightBackCM,int degrees,double KP, double TimeOut){
//        runtime.reset();
//
//        double oldEncoders = LF.getCurrentPosition();
//        double errorGyro =  (degrees-MyDriveTrain.getAngle());
//        double error = LeftFrontCM - LF.getCurrentPosition() ;
//        int newLeftFrontTarget = 0;
//        int newLeftBackTarget = 0;
//        int newRightFrontTarget = 0;
//        int newRightBackTarget = 0;
//        double Pnumber = 0.019;
//        // Ensure that the opmode is still active
//
//        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        // Determine new target position, and pass to motor controller
//        newLeftFrontTarget = LeftFront.getCurrentPosition() + (int) (LeftFrontCM * COUNTS_PER_CM);
//        newLeftBackTarget = LeftBack.getCurrentPosition() + (int) (LeftBackCM * COUNTS_PER_CM);
//        newRightFrontTarget = RightFront.getCurrentPosition() + (int) (RightFrontCM * COUNTS_PER_CM);
//        newRightBackTarget = RightBack.getCurrentPosition() + (int) (RightBackCM * COUNTS_PER_CM);
//
//        LeftFront.setTargetPosition(newLeftFrontTarget);
//        LeftBack.setTargetPosition(newLeftBackTarget);
//        RightFront.setTargetPosition(newRightFrontTarget);
//        RightBack.setTargetPosition(newRightBackTarget);
//
//        // Turn On RUN_TO_POSITION
//        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // reset the timeout time and start motion.
//
//
////Strafe
//        while ((((newLeftFrontTarget) <= LeftFront.getCurrentPosition()&& runtime.seconds() < TimeOut )&&((newRightBackTarget) <= RightBack.getCurrentPosition()&& runtime.seconds() < TimeOut))&&(((newRightFrontTarget) >= RightFront.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newLeftBackTarget) >= LeftBack.getCurrentPosition()&& runtime.seconds() < TimeOut))) {
//
//            error = newLeftFrontTarget - LeftFront.getCurrentPosition();
//            errorGyro = -(degrees - getAngle());
//
//
//          /*  LeftFront.setPower(Math.abs(speed - errorGyro * KP));
//            LeftBack.setPower(Math.abs(speed +  errorGyro * KP));
//            RightFront.setPower(Math.abs(speed - errorGyro * KP));
//            RightBack.setPower(Math.abs(speed  + errorGyro * KP));
//*/
//            LeftFront.setPower(Math.abs(speed - errorGyro * KP));
//            LeftBack.setPower(Math.abs(speed +  errorGyro * KP));
//            RightFront.setPower(Math.abs(speed - errorGyro * KP));
//            RightBack.setPower(Math.abs(speed  + errorGyro * KP));
//
//        }
//
////Strafe
//        while ((((newLeftFrontTarget) >= LF.getCurrentPosition()&& runtime.seconds() < TimeOut )&&((newRightBackTarget) >= RB.getCurrentPosition()&& runtime.seconds() < TimeOut))&&(((newRightFrontTarget) <= RightFront.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newLeftBackTarget) <= LeftBack.getCurrentPosition()&& runtime.seconds() < TimeOut))) {
//
//            error = newLeftFrontTarget - LF.getCurrentPosition();
//            errorGyro = (degrees - MyDriveTrain.getAngle());
//
//
//          /*  LeftFront.setPower(Math.abs(speed - errorGyro * KP));
//            LeftBack.setPower(Math.abs(speed +  errorGyro * KP));
//            RightFront.setPower(Math.abs(speed - errorGyro * KP));
//            RightBack.setPower(Math.abs(speed  + errorGyro * KP));
//*/
//            LF.setPower(Math.abs(speed - errorGyro * KP));
//            LB.setPower(Math.abs(speed +  errorGyro * KP));
//            RF.setPower(Math.abs(speed - errorGyro * KP));
//            RB.setPower(Math.abs(speed  + errorGyro * KP));
//
//        }
////forward/back
//        while (((newLeftFrontTarget) >= LF.getCurrentPosition()&& runtime.seconds() < TimeOut )&&((newLeftBackTarget) >= LB.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newRightFrontTarget) >= RightFront.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newRightBackTarget) >= RightBack.getCurrentPosition()&& runtime.seconds() < TimeOut)) {
//
//            error = newLeftFrontTarget - LF.getCurrentPosition();
//            errorGyro = (degrees - MyDriveTrain.getAngle());
//
//
//            LF.setPower(Math.abs(speed - errorGyro * KP));
//            LB.setPower(Math.abs(speed - errorGyro * KP));
//            RF.setPower(Math.abs(speed  + errorGyro * KP));
//            RB.setPower(Math.abs(speed  + errorGyro * KP));
//
//        }
//
////forward/back
//        while (((newLeftFrontTarget) < LF.getCurrentPosition()&& runtime.seconds() < TimeOut )&&((newLeftBackTarget) < LeftBack.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newRightFrontTarget) < RightFront.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newRightBackTarget) < RightBack.getCurrentPosition()&& runtime.seconds() < TimeOut)) {
//
//            error = newLeftFrontTarget - LF.getCurrentPosition();
//            errorGyro = -(degrees - MyDriveTrain.getAngle());
//
//
//            LF.setPower(Math.abs(speed - errorGyro * KP));
//            LB.setPower(Math.abs(speed - errorGyro * KP));
//            RF.setPower(Math.abs(speed  + errorGyro * KP));
//            RB.setPower(Math.abs(speed  + errorGyro * KP));
//
//        }
//
//
//
//
//
//        // Stop all motion;
//        LF.setPower(0);
//        LB.setPower(0);
//        RF.setPower(0);
//        RB.setPower(0);
//
//        // Turn off RUN_TO_POSITION
//        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//        //  sleep(250);   // optional pause after each move
//    }
//
//


}


