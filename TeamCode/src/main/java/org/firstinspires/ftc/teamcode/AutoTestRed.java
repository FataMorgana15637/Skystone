package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.doge_cv.SkystoneDetector;


@Autonomous(name = "AutoTestRed", group = "teamcode")
public class AutoTestRed extends basicAuto {

    /* Declare OpMode members. */



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //  LeftServo.setPosition(0.4);
        //   RightServo.setPosition(0.5);

//        SkyStone.setPosition(SkyStoneUP);
//        Finger.setPosition(FingerCatch);
        while (!isStarted()){
            Mikum = skystoneDetector.getScreenPosition().y;
            telemetry.addData("Mikum:", Mikum);
            telemetry.update();
        }

        waitForStart();
        runtime.reset();
        runtimestuck.reset();
        SkyStone.setPosition(SkyStoneDOWN);
        Finger.setPosition(FingerDontCatch);

        //sleep(200);
        //  MyDriveTrain.Rotate(0, 0.2, 2);
        //LED.setPower(1);
        //sleep(1500);

        //LED.setPower(0);

////        MyDriveTrain.encoderDrive(0.4,12,12,12,12);
////        sleep(400);
////        // MyDriveTrain.Rotate(0,0.1,10,0.03);


        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Mikum >= 220 ) {
            telemetry.addLine("you're on the right");
            telemetry.update();
            telemetry.addData("Time",runtimestuck);
            telemetry.update();
            // MyDriveTrain.Rotate(145,0.8,10,0.03);
////            MyDriveTrain.encoderDrive(0.4,7,7,7,7);
            MyDriveTrain.driveWithGyro(0.65,55,-55,-55,55,0,0.026,3);
            MyDriveTrain.encoderDrive(0.8,-27,-27,-27,-27); //TODO Big Change
            MyDriveTrain.Rotate(0, 0.2, 2);
            //sleep(200);
            MyDriveTrain.encoderDrive(0.65, 25, -25, -25, 25);
            MyDriveTrain.Rotate(0,0.22,1.5);

            Finger.setPosition(FingerCatch);
            sleep(400);
            SkyStone.setPosition(SkyStoneUPAUTO);
            LED.setPower(1);
            //MyDriveTrain.encoderDrive(0.7, 10, 10, 10, 10);
            //MyDriveTrain.encoderDrive(0.5, -7, 7, 7, -7);
            //sleep(500);
            MyDriveTrain.encoderDrivediagonal(1,-50,0,0,-50);
            MyDriveTrain.Rotate(0, 0.2, 1.5);
            // sleep(150);
            MyDriveTrain.driveWithGyro(1,-124,-124,-124,-124,0,0.025,3);
            MyDriveTrain.encoderDrivediagonal(1,0,-50,-50,0); //TODO Big Change
//            MyDriveTrain.encoderDrive(0.8,48,-48,-48,48);
            SkyStone.setPosition(SkyStoneDownAUTO);

            SkyStone.setPosition(SkyStoneDownAUTO);
            sleep(100);
            Finger.setPosition(FingerDontCatch);
            sleep(250);

            LED.setPower(0);
            SkyStone.setPosition(SkyStoneUP);
            Finger.setPosition(FingerCatch);
            MyDriveTrain.Rotate(0,0.25,2);
            MyDriveTrain.encoderDrivediagonal(1,0,50,50,0);

            //MyDriveTrain.encoderDrive(0.7, -33, 33, 33, -33);
            MyDriveTrain.Rotate(0,0.2,2);
//            sleep(300);
            //TODO Way to the second SkyStone
            MyDriveTrain.driveWithGyro(0.9,192,192,192,192,0,0.025,3); //TODO Big Change
//            MyDriveTrain.driveWithGyroAndServo(0.9,190,190,190,190,0,0.025,150,3);
            SkyStone.setPosition(SkyStoneDOWN);
            Finger.setPosition(FingerDontCatch);
            sleep(100);

//          MyDriveTrain.driveWithGyro(1,73,73,73,73,0,0.025,5);
            MyDriveTrain.Rotate(0,0.2,2);
            sleep(100);
            MyDriveTrain.encoderDrive(0.65,19,-19,-19,19); //TODO Big Change

            Finger.setPosition(FingerCatch);
            sleep(400);
            SkyStone.setPosition(SkyStoneUPAUTO);
            LED.setPower(1);
            MyDriveTrain.Rotate(0,0.25,2);
            MyDriveTrain.encoderDrivediagonal(1,-45,0,0,-45);
            MyDriveTrain.Rotate(0,0.2,2);
            MyDriveTrain.driveWithGyro(1,-150,-150,-150,-150,0,0.025,7);
            Foundation.setPosition(MidFoundation);
            MyDriveTrain.encoderDrivediagonal(1,0,-55,-55,0);
            SkyStone.setPosition(SkyStoneDownAUTO);

            SkyStone.setPosition(SkyStoneDownAUTO);
            sleep(100);
            Finger.setPosition(FingerDontCatch);
            sleep(250);
            LED.setPower(0);
            SkyStone.setPosition(SkyStoneUP);
            MyDriveTrain.Rotate(0,0.25,2);
            //MyDriveTrain.encoderDriveP(1,-15,-15,-15,-15,0.0015,2);
            MyDriveTrain.encoderDrive(1, -15, 15, 15, -15);
            Finger.setPosition(FingerCatch);
            MyDriveTrain.RotateP(90,1,1.5,0.013);
            MyDriveTrain.Rotate(90,0.25,1.5);
            MyDriveTrain.encoderDrive(0.3,-19,-19,-19,-19);
            Foundation.setPosition(CatchFoundation);
            sleep(900);
            MyDriveTrain.encoderDrive(0.3,-7,-7,-7,-7);
            Foundation.setPosition(CatchFoundation);
            MyDriveTrain.encoderDrive(1,20,20,20,20);
            MyDriveTrain.Rotate(80,0.7,2);
            MyDriveTrain.encoderDrive(0.8,55,55,55,55);
            MyDriveTrain.Rotate(0,0.7,1.5);
            Foundation.setPosition(DontCatchFoundation);

            MyDriveTrain.encoderDrive(1,-40,-40,-40,-40);
            MyDriveTrain.encoderDrive(1,7,7,7,7);
            MyDriveTrain.RotateP(160,1,1.5,0.0125);
            MyDriveTrain.Rotate(160,0.2,1.5);
            if (MyDriveTrain.runtimestuck.seconds()<=MyDriveTrain.T-0.5) {
                Meter.setPower(1);
                sleep(700);
            }

        }

        else if (Mikum > 110 && Mikum < 220) {
            telemetry.addLine("You are on the center!");
            telemetry.update();
            //  MyDriveTrain.Rotate(145,1,10);
            //          MyDriveTrain.encoderDrive(0.4,7,7,7,7);
            MyDriveTrain.driveWithGyro(0.65,55,-55,-55,55,0,0.026,3);
            MyDriveTrain.encoderDrive(0.3, -12, -12, -12, -12); //TODO Big Change
            //sleep(200);
            MyDriveTrain.encoderDrive(0.65, 25, -25, -25, 25);
            MyDriveTrain.Rotate(0, 0.2, 2);


            //SkyStone.setPosition(SkyStoneDOWN);
            //sleep(400);
            Finger.setPosition(FingerCatch);
            sleep(400);
            SkyStone.setPosition(SkyStoneUPAUTO);
            LED.setPower(1);
            //MyDriveTrain.encoderDrive(0.7, 10, 10, 10, 10);
            //MyDriveTrain.encoderDrive(0.5, -7, 7, 7, -7);
            //sleep(500);
            MyDriveTrain.encoderDrivediagonal(1,-50,0,0,-50);
            MyDriveTrain.Rotate(0, 0.2, 1.5);
            // sleep(150);
            MyDriveTrain.driveWithGyro(1,-143,-143,-143,-143,0,0.025,3);
            MyDriveTrain.encoderDrivediagonal(1,0,-50,-50,0); //TODO Big Change
//            MyDriveTrain.encoderDrive(0.8,48,-48,-48,48);
            SkyStone.setPosition(SkyStoneDownAUTO);

            SkyStone.setPosition(SkyStoneDownAUTO);
            sleep(100);
            Finger.setPosition(FingerDontCatch);
            sleep(250);

            LED.setPower(0);
            SkyStone.setPosition(SkyStoneUP);
            Finger.setPosition(FingerCatch);
            MyDriveTrain.Rotate(0,0.25,2);
            MyDriveTrain.encoderDrivediagonal(1,0,50,50,0);

            //MyDriveTrain.encoderDrive(0.7, -33, 33, 33, -33);
            MyDriveTrain.Rotate(0,0.2,2);
//            sleep(300);
            //TODO Way to the second SkyStone
            MyDriveTrain.driveWithGyro(0.9,214,214,214,214,0,0.025,3); //TODO Big Change
//            MyDriveTrain.driveWithGyroAndServo(0.9,190,190,190,190,0,0.025,150,3);
            SkyStone.setPosition(SkyStoneDOWN);
            Finger.setPosition(FingerDontCatch);
            sleep(100);

//          MyDriveTrain.driveWithGyro(1,73,73,73,73,0,0.025,5);
            MyDriveTrain.Rotate(0,0.2,2);
            sleep(100);
            MyDriveTrain.encoderDrive(0.65,19,-19,-19,19); //TODO Big Change

            Finger.setPosition(FingerCatch);
            sleep(400);
            SkyStone.setPosition(SkyStoneUPAUTO);
            LED.setPower(1);
            MyDriveTrain.Rotate(0,0.25,2);
            MyDriveTrain.encoderDrivediagonal(1,-45,0,0,-45);
            MyDriveTrain.Rotate(0,0.2,2);
            MyDriveTrain.driveWithGyro(1,-168,-168,-168,-168,0,0.025,7);
            Foundation.setPosition(MidFoundation);
            MyDriveTrain.encoderDrivediagonal(1,0,-55,-55,0);
            SkyStone.setPosition(SkyStoneDownAUTO);

            SkyStone.setPosition(SkyStoneDownAUTO);
            sleep(100);
            Finger.setPosition(FingerDontCatch);
            sleep(250);
            LED.setPower(0);
            SkyStone.setPosition(SkyStoneUP);
            MyDriveTrain.Rotate(0,0.25,2);
            //MyDriveTrain.encoderDriveP(1,-15,-15,-15,-15,0.0015,2);
            MyDriveTrain.encoderDrive(1, -15, 15, 15, -15);
            Finger.setPosition(FingerCatch);
            MyDriveTrain.RotateP(90,1,1.5,0.013);
            MyDriveTrain.Rotate(90,0.25,1.5);
            MyDriveTrain.encoderDrive(0.3,-19,-19,-19,-19);
            Foundation.setPosition(CatchFoundation);
            sleep(900);
            MyDriveTrain.encoderDrive(0.3,-7,-7,-7,-7);
            Foundation.setPosition(CatchFoundation);
            MyDriveTrain.encoderDrive(1,20,20,20,20);
            MyDriveTrain.Rotate(80,0.7,2);
            MyDriveTrain.encoderDrive(0.8,55,55,55,55);
            MyDriveTrain.Rotate(0,0.7,1.5);
            Foundation.setPosition(DontCatchFoundation);

            MyDriveTrain.encoderDrive(1,-40,-40,-40,-40);
            MyDriveTrain.encoderDrive(1,7,7,7,7);
            MyDriveTrain.RotateP(160,1,1.5,0.0125);
            MyDriveTrain.Rotate(160,0.2,1.5);
            if (MyDriveTrain.runtimestuck.seconds()<=MyDriveTrain.T-0.5) {
                Meter.setPower(1);
                sleep(700);
            }
        }
        else  {
            telemetry.addLine("You are on the left!");
            telemetry.update();
            //  MyDriveTrain.Rotate(145,1,10);
            //          MyDriveTrain.encoderDrive(0.4,7,7,7,7);
            MyDriveTrain.driveWithGyro(0.65,55,-55,-55,55,0,0.026,3);
            MyDriveTrain.encoderDrive(0.3, 12, 12, 12, 12); //TODO Big Change
            //sleep(200);
            MyDriveTrain.encoderDrive(0.65, 25, -25, -25, 25);
            MyDriveTrain.Rotate(0, 0.2, 2);


            Finger.setPosition(FingerCatch);
            sleep(400);
            SkyStone.setPosition(SkyStoneUPAUTO);
            LED.setPower(1);
            //MyDriveTrain.encoderDrive(0.7, 10, 10, 10, 10);
            //MyDriveTrain.encoderDrive(0.5, -7, 7, 7, -7);
            //sleep(500);
            MyDriveTrain.encoderDrivediagonal(1,-50,0,0,-50);
            MyDriveTrain.Rotate(0, 0.2, 1.5);
            // sleep(150);
            MyDriveTrain.driveWithGyro(1,-160,-160,-160,-160,0,0.025,3);
            MyDriveTrain.encoderDrivediagonal(1,0,-50,-50,0); //TODO Big Change
//            MyDriveTrain.encoderDrive(0.8,48,-48,-48,48);
            SkyStone.setPosition(SkyStoneDownAUTO);

            SkyStone.setPosition(SkyStoneDownAUTO);
            sleep(100);
            Finger.setPosition(FingerDontCatch);
            sleep(250);

            LED.setPower(0);
            SkyStone.setPosition(SkyStoneUP);
            Finger.setPosition(FingerCatch);
            MyDriveTrain.Rotate(0,0.25,2);
            MyDriveTrain.encoderDrivediagonal(1,0,50,50,0);

            //MyDriveTrain.encoderDrive(0.7, -33, 33, 33, -33);
            MyDriveTrain.Rotate(0,0.2,2);
//            sleep(300);
            //TODO Way to the second SkyStone
            MyDriveTrain.driveWithGyro(0.9,228,228,228,228,0,0.025,3); //TODO Big Change
//            MyDriveTrain.driveWithGyroAndServo(0.9,190,190,190,190,0,0.025,150,3);
            SkyStone.setPosition(SkyStoneDOWN);
            Finger.setPosition(FingerDontCatch);
            sleep(100);

//          MyDriveTrain.driveWithGyro(1,73,73,73,73,0,0.025,5);
            MyDriveTrain.Rotate(0,0.2,2);
            sleep(100);
            MyDriveTrain.encoderDrive(0.65,19,-19,-19,19); //TODO Big Change

            Finger.setPosition(FingerCatch);
            sleep(400);
            SkyStone.setPosition(SkyStoneUPAUTO);
            LED.setPower(1);
            MyDriveTrain.Rotate(0,0.25,2);
            MyDriveTrain.encoderDrivediagonal(1,-45,0,0,-45);
            MyDriveTrain.Rotate(0,0.2,2);
            MyDriveTrain.driveWithGyro(1,-188,-188,-188,-188,0,0.025,7);
            Foundation.setPosition(MidFoundation);
            MyDriveTrain.encoderDrivediagonal(1,0,-55,-55,0);
            SkyStone.setPosition(SkyStoneDownAUTO);

            SkyStone.setPosition(SkyStoneDownAUTO);
            sleep(100);
            Finger.setPosition(FingerDontCatch);
            sleep(250);
            LED.setPower(0);
            SkyStone.setPosition(SkyStoneUP);
            MyDriveTrain.Rotate(0,0.25,2);
            //MyDriveTrain.encoderDriveP(1,-15,-15,-15,-15,0.0015,2);
            MyDriveTrain.encoderDrive(1, -15, 15, 15, -15);
            Finger.setPosition(FingerCatch);
            MyDriveTrain.RotateP(90,1,1.5,0.013);
            MyDriveTrain.Rotate(90,0.25,1.5);
            MyDriveTrain.encoderDrive(0.3,-19,-19,-19,-19);
            Foundation.setPosition(CatchFoundation);
            sleep(900);
            MyDriveTrain.encoderDrive(0.3,-7,-7,-7,-7);
            Foundation.setPosition(CatchFoundation);
            MyDriveTrain.encoderDrive(1,20,20,20,20);
            MyDriveTrain.Rotate(80,0.7,2);
            MyDriveTrain.encoderDrive(0.8,55,55,55,55);
            MyDriveTrain.Rotate(0,0.7,1.5);
            Foundation.setPosition(DontCatchFoundation);

            MyDriveTrain.encoderDrive(1,-40,-40,-40,-40);
            MyDriveTrain.encoderDrive(1,7,7,7,7);
            MyDriveTrain.RotateP(160,1,1.5,0.0125);
            MyDriveTrain.Rotate(160,0.2,1.5);
            if (MyDriveTrain.runtimestuck.seconds()<=MyDriveTrain.T-0.5) {
                Meter.setPower(1);
                sleep(700);
            }


        }

    }
}





