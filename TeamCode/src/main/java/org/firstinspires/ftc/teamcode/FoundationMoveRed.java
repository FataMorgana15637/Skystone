package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "FoundationMoveRed", group = "teamcode")
public class FoundationMoveRed extends Robot {

    /* Declare OpMode members. */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //  LeftServo.setPosition(0.4);
        //   RightServo.setPosition(0.5);
//        while (!isStarted())
//        Mikum = MyVuforiaStone.ConceptVuforiaSkyStoneNavigationWebcam();
//
        Arm.setPosition(ArmUp);
        FingerArm.setPosition(FingerClose);
        CapStone.setPosition(CapStoneUp);
//        SkyStone.setPosition(SkyStoneUP);
//        Finger.setPosition(FingerCatch);

        waitForStart();

        MyDriveTrain.encoderDrive(1,10,-10,-10,10);
        MyDriveTrain.driveWithGyro(1,-20,-20,-20,-20,0,0.025,2);
        MyDriveTrain.driveWithGyro(1,60,-60,-60,60,0,0.15,5);

        MyDriveTrain.RotateP(90,1,1.5,0.02);
        MyDriveTrain.Rotate(90,0.25,1.5);
        MyDriveTrain.encoderDrive(0.4,-20,-20,-20,-20);
        Foundation.setPosition(CatchFoundation);
        sleep(1100);
        MyDriveTrain.encoderDrive(0.3,-7,-7,-7,-7);
        Foundation.setPosition(CatchFoundation);
        MyDriveTrain.Rotate(80,0.8,2);
        MyDriveTrain.encoderDrive(0.8,75,75,75,75);
        MyDriveTrain.Rotate(0,0.8,1.5);
        Foundation.setPosition(DontCatchFoundation);

        MyDriveTrain.encoderDrive(1,-40,-40,-40,-40);
        MyDriveTrain.encoderDrive(1,10,10,10,10);
        MyDriveTrain.RotateP(150,1,1.5,0.015);
        MyDriveTrain.Rotate(150,0.2,1.5);
        Meter.setPower(1);
        sleep(50);
    }

}



