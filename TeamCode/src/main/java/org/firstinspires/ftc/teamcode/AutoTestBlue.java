package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LegacyModule;


@Autonomous(name = "AutoTestBlue", group = "teamcode")
public class AutoTestBlue extends basicAuto {

    /* Declare OpMode members. */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //  LeftServo.setPosition(0.4);
        //   RightServo.setPosition(0.5);
        while (!isStarted()){
            Mikum = skystoneDetector.getScreenPosition().y;
            telemetry.addData("Mikum:", Mikum);
            telemetry.update();
        }

        SkyStone.setPosition(SkyStoneDOWN);
        Finger.setPosition(FingerDontCatch);


        waitForStart();
        runtime.reset();
        runtimestuck.reset();

        SkyStone.setPosition(SkyStoneDOWN);
        Finger.setPosition(FingerDontCatch);


        // Mikum = MyVuforiaStone.ConceptVuforiaSkyStoneNavigationWebcam();
        //LED.setPower(0);
        // telemetry.addData("Mikum:", Mikum);

////        MyDriveTrain.encoderDrive(0.4,12,12,12,12);
////        sleep(400);
////        // MyDriveTrain.Rotate(0,0.1,10,0.03);


        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Mikum <180 ) {
            telemetry.addLine("you're on the left");
            telemetry.update();
            // MyDriveTrain.Rotate(145,0.8,10,0.03);
////            MyDriveTrain.encoderDrive(0.4,7,7,7,7);
            MyDriveTrain.driveWithGyro(0.65,55,-55,-55,55,0,0.026,3);
            MyDriveTrain.encoderDrive(0.8,8,8,8,8); //TODO Big Change
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
            MyDriveTrain.encoderDrivediagonal(1,0,48,48,0);//Left Forward
            MyDriveTrain.Rotate(0, 0.2, 1.5);
            // sleep(150);
            MyDriveTrain.driveWithGyro(1,117,117,117,117,0,0.025,3);
            MyDriveTrain.encoderDrivediagonal(1,60,0,0,60);//RIGHT Forward //TODO Big Change
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
            MyDriveTrain.encoderDrivediagonal(1,-50,0,0,-50);//Left Back

            //MyDriveTrain.encoderDrive(0.7, -33, 33, 33, -33);
            MyDriveTrain.Rotate(0,0.2,2);
//            sleep(300);
            //TODO Way to the second SkyStone
            MyDriveTrain.driveWithGyro(0.9,-195,-195,-195,-195,0,0.025,3); //TODO Big Change
//            MyDriveTrain.driveWithGyroAndServo(0.9,190,190,190,190,0,0.025,150,3);
            SkyStone.setPosition(SkyStoneDOWN);
            Finger.setPosition(FingerDontCatch);
            sleep(100);

//          MyDriveTrain.driveWithGyro(1,73,73,73,73,0,0.025,5);
            MyDriveTrain.Rotate(0,0.2,2);
            sleep(100);
            MyDriveTrain.encoderDrive(0.65,22,-22,-22,22);

            Finger.setPosition(FingerCatch);
            sleep(400);
            SkyStone.setPosition(SkyStoneUPAUTO);
            LED.setPower(1);
            MyDriveTrain.Rotate(0,0.25,2);
            MyDriveTrain.encoderDrivediagonal(1,0,47,47,0);//Left Forward
            MyDriveTrain.Rotate(0,0.2,2);
            MyDriveTrain.driveWithGyro(1,147,147,147,147,0,0.025,7);

            // MyDriveTrain.Rotate(0,0.2,2);
            Foundation.setPosition(MidFoundation);
            MyDriveTrain.encoderDrivediagonal(1,54,0,0,54);//RIGHT Forward
            MyDriveTrain.Rotate(0,0.21,2);
            SkyStone.setPosition(SkyStoneDownAUTO);

            SkyStone.setPosition(SkyStoneDownAUTO);
            sleep(100);
            Finger.setPosition(FingerDontCatch);
            sleep(250);
            LED.setPower(0);
            SkyStone.setPosition(SkyStoneUP);
            MyDriveTrain.Rotate(0,0.25,2);
            //MyDriveTrain.encoderDriveP(1,-15,-15,-15,-15,0.0015,2);

            /** Foundation **/
            MyDriveTrain.encoderDrivediagonal(1,10,47,47,10);//Left Forward
            Finger.setPosition(FingerCatch);
            MyDriveTrain.RotateP(90,1,3,0.013);
            MyDriveTrain.Rotate(90,0.25,1.5);
            MyDriveTrain.encoderDrive(0.3,-19,-19,-19,-19);
            Foundation.setPosition(CatchFoundation);
            sleep(900);
            MyDriveTrain.encoderDrive(0.3,-7,-7,-7,-7);
            Foundation.setPosition(CatchFoundation);
            MyDriveTrain.encoderDrive(1,20,20,20,20);
            MyDriveTrain.Rotate(100,0.7,2);
            MyDriveTrain.encoderDrive(0.8,55,55,55,55);
            MyDriveTrain.Rotate(180,0.7,1.5);
            Foundation.setPosition(DontCatchFoundation);

            MyDriveTrain.encoderDrive(1,-40,-40,-40,-40);
            MyDriveTrain.encoderDrive(1,7,7,7,7);
            MyDriveTrain.RotateP(25,1,1.5,0.0125);
            MyDriveTrain.Rotate(25,0.2,1.5);
            if (MyDriveTrain.runtimestuck.seconds()<=MyDriveTrain.T-0.5) {
                Meter.setPower(1);
                sleep(700);
            }




        }
        else if (Mikum < 250 && Mikum >180) {
            telemetry.addLine("You are on the center!");
            telemetry.update();
            //  MyDriveTrain.Rotate(145,1,10);
            //          MyDriveTrain.encoderDrive(0.4,7,7,7,7);
            MyDriveTrain.driveWithGyro(0.65,55,-55,-55,55,0,0.026,3);
            MyDriveTrain.encoderDrive(0.8,-16.5,-16.5,-16.5,-16.5);
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
            MyDriveTrain.encoderDrivediagonal(1,0,48,48,0);//Left Forward
            MyDriveTrain.Rotate(0, 0.2, 1.5);
            // sleep(150);
            MyDriveTrain.driveWithGyro(1,132,132,132,132,0,0.025,3);
            MyDriveTrain.encoderDrivediagonal(1,60,0,0,60);//RIGHT Forward
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
            MyDriveTrain.encoderDrivediagonal(1,-50,0,0,-50);//Left Back

            //MyDriveTrain.encoderDrive(0.7, -33, 33, 33, -33);
            MyDriveTrain.Rotate(0,0.2,2);
//            sleep(300);
            //TODO Way to the second SkyStone
            MyDriveTrain.driveWithGyro(0.9,-216,-216,-216,-216,0,0.025,3);
//            MyDriveTrain.driveWithGyroAndServo(0.9,190,190,190,190,0,0.025,150,3);
            SkyStone.setPosition(SkyStoneDOWN);
            Finger.setPosition(FingerDontCatch);
            sleep(100);

//          MyDriveTrain.driveWithGyro(1,73,73,73,73,0,0.025,5);
            MyDriveTrain.Rotate(0,0.2,2);
            sleep(100);
            MyDriveTrain.encoderDrive(0.65,22,-22,-22,22);

            Finger.setPosition(FingerCatch);
            sleep(400);
            SkyStone.setPosition(SkyStoneUPAUTO);
            LED.setPower(1);
            MyDriveTrain.Rotate(0,0.25,2);
            MyDriveTrain.encoderDrivediagonal(1,0,47,47,0);//Left Forward
            MyDriveTrain.Rotate(0,0.2,2);
            MyDriveTrain.driveWithGyro(1,155,155,155,155,0,0.025,7);
            // MyDriveTrain.Rotate(0,0.2,2);
            Foundation.setPosition(MidFoundation);
            MyDriveTrain.encoderDrivediagonal(1,54,0,0,54);//RIGHT Forward
            SkyStone.setPosition(SkyStoneDownAUTO);

            SkyStone.setPosition(SkyStoneDownAUTO);
            sleep(100);
            Finger.setPosition(FingerDontCatch);
            sleep(250);
            LED.setPower(0);
            SkyStone.setPosition(SkyStoneUP);
            MyDriveTrain.Rotate(0,0.25,2);
            //MyDriveTrain.encoderDriveP(1,-15,-15,-15,-15,0.0015,2);

            /** Foundation **/
            MyDriveTrain.encoderDrivediagonal(1,10,47,47,10);//Left Forward
            Finger.setPosition(FingerCatch);
            MyDriveTrain.RotateP(90,1,3,0.013);
            MyDriveTrain.Rotate(90,0.25,1.5);
            MyDriveTrain.encoderDrive(0.3,-22,-22,-22,-22);
            Foundation.setPosition(CatchFoundation);
            sleep(900);
            MyDriveTrain.encoderDrive(0.3,-7,-7,-7,-7);
            Foundation.setPosition(CatchFoundation);
            MyDriveTrain.encoderDrive(1,20,20,20,20);
            MyDriveTrain.Rotate(100,0.7,2);
            MyDriveTrain.encoderDrive(0.8,55,55,55,55);
            MyDriveTrain.Rotate(180,0.7,1.5);
            Foundation.setPosition(DontCatchFoundation);

            MyDriveTrain.encoderDrive(1,-40,-40,-40,-40);
            MyDriveTrain.encoderDrive(1,7,7,7,7);
            MyDriveTrain.RotateP(25,1,1.5,0.0125);
            MyDriveTrain.Rotate(25,0.2,1.5);
            if (MyDriveTrain.runtimestuck.seconds()<=MyDriveTrain.T-0.5) {
                Meter.setPower(1);
                sleep(700);
            }

        } else {
            telemetry.addLine("You are on the right!");
            telemetry.update();
            MyDriveTrain.driveWithGyro(0.65,55,-55,-55,55,0,0.026,3);
            MyDriveTrain.encoderDrive(0.8,-33,-33,-33,-33);
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
            MyDriveTrain.encoderDrivediagonal(1,0,48,48,0);//Left Forward
            MyDriveTrain.Rotate(0, 0.2, 1.5);
            // sleep(150);
            MyDriveTrain.driveWithGyro(1,148,148,148,148,0,0.025,3);
            MyDriveTrain.encoderDrivediagonal(1,60,0,0,60);//RIGHT Forward
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
            MyDriveTrain.encoderDrivediagonal(1,-50,0,0,-50);//Left Back

            //MyDriveTrain.encoderDrive(0.7, -33, 33, 33, -33);
            MyDriveTrain.Rotate(0,0.2,2);
//            sleep(300);
            //TODO Way to the second SkyStone
            MyDriveTrain.driveWithGyro(0.9,-141,-141,-141,-141,0,0.025,3);
//            MyDriveTrain.driveWithGyroAndServo(0.9,190,190,190,190,0,0.025,150,3);
            SkyStone.setPosition(SkyStoneDOWN);
            Finger.setPosition(FingerDontCatch);
            sleep(100);

//          MyDriveTrain.driveWithGyro(1,73,73,73,73,0,0.025,5);
            MyDriveTrain.Rotate(0,0.2,2);
            sleep(100);
            MyDriveTrain.encoderDrive(0.65,22,-22,-22,22);

            Finger.setPosition(FingerCatch);
            sleep(400);
            SkyStone.setPosition(SkyStoneUPAUTO);
            LED.setPower(1);
            MyDriveTrain.Rotate(0,0.25,2);
            MyDriveTrain.encoderDrivediagonal(1,0,47,47,0);//Left Forward
            MyDriveTrain.Rotate(0,0.2,2);
            MyDriveTrain.driveWithGyro(1,98,98,98,98,0,0.025,7);
            // MyDriveTrain.Rotate(0,0.2,2);
            Foundation.setPosition(MidFoundation);
            MyDriveTrain.encoderDrivediagonal(1,54,0,0,54);//RIGHT Forward
            SkyStone.setPosition(SkyStoneDownAUTO);

            SkyStone.setPosition(SkyStoneDownAUTO);
            sleep(100);
            Finger.setPosition(FingerDontCatch);
            sleep(250);
            LED.setPower(0);
            SkyStone.setPosition(SkyStoneUP);
            MyDriveTrain.Rotate(0,0.25,2);

            // MyDriveTrain.encoderDrivediagonal(0.8,-43,0,0,-43);//Left Back
            Finger.setPosition(FingerCatch);
            MyDriveTrain.Rotate(0,0.2,2);


            /** Foundation **/
            MyDriveTrain.encoderDrivediagonal(1,10,47,47,10);//Left Forward
            Finger.setPosition(FingerCatch);
            MyDriveTrain.RotateP(90,1,3,0.013);
            MyDriveTrain.Rotate(90,0.25,1.5);
            MyDriveTrain.encoderDrive(0.3,-19,-19,-19,-19);
            Foundation.setPosition(CatchFoundation);
            sleep(900);
            MyDriveTrain.encoderDrive(0.3,-7,-7,-7,-7);
            Foundation.setPosition(CatchFoundation);
            MyDriveTrain.encoderDrive(1,20,20,20,20);
            MyDriveTrain.Rotate(100,0.7,2);
            MyDriveTrain.encoderDrive(0.8,55,55,55,55);
            MyDriveTrain.Rotate(180,0.7,1.5);
            Foundation.setPosition(DontCatchFoundation);

            MyDriveTrain.encoderDrive(1,-40,-40,-40,-40);
            MyDriveTrain.encoderDrive(1,7,7,7,7);
            MyDriveTrain.RotateP(25,1,1.5,0.0125);
            MyDriveTrain.Rotate(25,0.2,1.5);
            if (MyDriveTrain.runtimestuck.seconds()<=MyDriveTrain.T-0.5) {
                Meter.setPower(1);
                sleep(700);
            }
        }
    }

}








