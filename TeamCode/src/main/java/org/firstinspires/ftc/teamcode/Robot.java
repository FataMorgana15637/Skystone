package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.doge_cv.SkystoneDetector;
import org.openftc.easyopencv.OpenCvWebcam;

public class Robot extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();
    protected ElapsedTime runtimestuck = new ElapsedTime();


    /* Drive Train Motor */
    public DcMotor LB = null;
    public DcMotor LF = null;
    public DcMotor RF = null;
    public DcMotor RB = null;
    /*Sensors*/
    DigitalChannel digitalTouch;  // Hardware Device Object
    DigitalChannel MagnetDigitalTouch;  // Hardware Device Object

    /* Elevator */
    // public DcMotor Elevator = null;
    public DcMotor Elevator = null;

    public DcMotor Meter = null;

    public DcMotor LED = null;


    /*Systems Motor and Servo*/
//    public DcMotor leftLinearMotor = null;
//    public DcMotor rightLinearMotor = null;
//    public DcMotor IntakeL = null;
//    public DcMotor IntakeR = null;
    //    public Servo Arm = null;
//    public Servo Output = null;
    public Servo Foundation = null;
    public Servo SkyStone = null;
    public Servo Finger = null;
    public Servo CapStone = null;
    public Servo Arm = null;
    public Servo FingerArm = null;
    public Servo BlockCap = null;


    /*IMU Fileds*/
    protected BNO055IMU IMU = null;
    protected Orientation angles = null;

    //digitalChannels
    //  public DigitalChannel downMagnetElevator = null;
    //  public DigitalChannel upMagnetElevator = null;

    /*Mechanisms*/
    protected DriveTrain MyDriveTrain = null;
    protected elevator MyElevator = null;

    //public double servoPosition = 0.005;
    //protected int fixedPosition = 0;
    public double SkyStoneUP = 0.83;
    public double SkyStoneDOWN = 0.47;
    public double SkyStoneUPAUTO = 0.83;
    public double SkyStoneDownAUTO = 0.48;
    public double FingerCatch = 0.04;
    public double FingerDontCatch = 0.43;
    public double MidFoundation = 0.25;
    public double CatchFoundation = 0.0;
    public double Base = 0.0;
    public double BaseCap = 0.0;
    public double DontCatchFoundation = 0.56;
    public double ArmDown = 0.57;
    public double ArmMid = 0.58;
    public double ArmUp = 0.86;
    public double FingerClose = 0.47;
    public double FingerOpen = 0.06;
    public double CapStoneDown = 0.013;
    public double CapStoneUp = 0.72;
    public double BlockingCap = 0.4;
    public double DontBlockingCap = 0.82;
    public double DeltaX = 0;  //minus to go more back, plus to more forward
    public double DeltaDiagonal = 0;  //plus to go closer to thr foundation
    //plus to go closer to thr foundation


    @Override
    public void runOpMode() throws InterruptedException {

        runtime.reset();
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        // Define and Initialize Motors Of Drive Train
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        // Define and Initialize Systems Motors and Servo
        //  leftLinearMotor = hardwareMap.get(DcMotor.class,"leftLinearMotor");
        // rightLinearMotor = hardwareMap.get(DcMotor.class,"rightLinearMotor");
//        Arm  = hardwareMap.get(Servo.class, "Arm");
//        Output  = hardwareMap.get(Servo.class, "OutPut");
        Foundation = hardwareMap.get(Servo.class, "Foundation");
        SkyStone = hardwareMap.get(Servo.class, "SkyStone");
        // SkyStoneBack = hardwareMap.get(Servo.class, "SkyStoneBack");
        Finger = hardwareMap.get(Servo.class, "Finger");
        //  FingerBack = hardwareMap.get(Servo.class, "FingerBack");
        CapStone = hardwareMap.get(Servo.class, "CapStone");
        Arm = hardwareMap.get(Servo.class, "Arm");
        FingerArm = hardwareMap.get(Servo.class, "FingerArm");
        BlockCap = hardwareMap.get(Servo.class, "BlockCap");
        Elevator = hardwareMap.get(DcMotor.class, "Elevator");
        Meter = hardwareMap.get(DcMotor.class, "Meter");
        LED = hardwareMap.get(DcMotor.class, "LED");
        //  LeftServo  = hardwareMap.get(Servo.class, "LeftServo");
        //  RightServo  = hardwareMap.get(Servo.class, "RightServo");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        MagnetDigitalTouch = hardwareMap.get(DigitalChannel.class, "MagnetDigitalTouch");


        //upMagnetElevator = hardwareMap.get(DigitalChannel.class,"upMagnetELevator");
        //downMagnetElevator = hardwareMap.get(DigitalChannel.class,"downMagnetELevator");

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        IntakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        IntakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //leftLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        LB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        LED.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        Elevator.setDirection(DcMotor.Direction.FORWARD);
        Meter.setDirection(DcMotor.Direction.REVERSE);
        //   IntakeL.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE the intake System
//        leftLinearMotor.setDirection(DcMotor.Direction.REVERSE);//set to rverse the elevator system

        Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        MagnetDigitalTouch.setMode(DigitalChannel.Mode.INPUT);
        //rightLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to zero power
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
        //  linear_motor.setPower(0);

        LED.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LED.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
      /* LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);/*

       RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       /*Define and Initialize Of IMU*/

        angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        // Define and initialize ALL installed servos.

        // Define Mechanisms:
        MyDriveTrain = new DriveTrain(LB, LF, RF, RB, IMU, SkyStone, Finger, runtimestuck);//TODO
        // MyIntake = new IntakeTrain(IntakeL, IntakeR);
        MyElevator = new elevator(Elevator);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        /**  IMU.initialize(parameters);
         // make sure the imu gyro is calibrated before continuing.
         while (!isStarted() && !isStopRequested() && !IMU.isGyroCalibrated()) {
         sleep(50);
         idle();
         }
         **/
    }
}