package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Arrays;

public class DriveTrain {

    /* Fileds */

    public double T = 29.7;

    public DcMotor LeftBack = null;
    public DcMotor LeftFront = null;
    public DcMotor RightFront = null;
    public DcMotor RightBack = null;

    public Servo SkyStone = null;
    public Servo Finger = null;

    public ElapsedTime runtimestuck = null;


    double RF = 0;
    double RB = 0;
    double LF = 0;
    double LB = 0;



    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: ANDYMARK Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CM = 10.16;     // For figuring circumference
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415926);


    Orientation lastAngles = new Orientation();
    Orientation angle = new Orientation();
    double globalAngle = 0, power = .30, correction;

    protected BNO055IMU IMU = null;

    public String Mode = "Oriented";

    public String telemetry[];

    private ElapsedTime runtime = new ElapsedTime();
    private HardwareMap hardwareMap;
    private VuforiaLocalizer vuforia;


    /* Constructor */
    public DriveTrain(DcMotor LB, DcMotor LF, DcMotor RF, DcMotor RB, BNO055IMU imu, Servo SkyStone, Servo Finger, ElapsedTime runtimestuck) {
        this.LeftBack = LB;
        this.LeftFront = LF;
        this.RightFront = RF;
        this.RightBack = RB;
        this.SkyStone = SkyStone;
        this.Finger = Finger;
        IMU = imu;
        this.runtimestuck = runtimestuck;
    }

    public void setPower(int LB, int RB, int LF, int RF) {
        LeftFront.setPower(LF);
        LeftBack.setPower(LB);
        RightFront.setPower(RF);
        RightBack.setPower(RB);

    }



    /* Methodes */

    public void arcade(double y, double x, double c) {
        double leftFrontVal = -y + x + c;
        double rightFrontVal = -y - x - c;
        double leftBackVal = -y - x + c;
        double rightBackVal = -y + x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        LeftFront.setPower(leftFrontVal);
        RightFront.setPower(rightFrontVal);
        RightBack.setPower(rightBackVal);
        LeftBack.setPower(leftBackVal);
    }

    public void fieldOriented(double y, double x, double c, double gyroheading) {
        double cosA = Math.cos(gyroheading);
        double sinA = Math.sin(gyroheading);
        double xOut = x * cosA - y * sinA;
        double yOut = x * sinA + y * cosA;
        arcade(yOut, xOut, c);
    }

    public String getMode() {
        return Mode;
    }

    public void setMode(String mode) {
        Mode = mode;
    }


    public void encoderDrivediagonal(double speed, double LeftFrontCM, double LeftBackCM, double RightFrontCM, double RightBackCM) {
        runtime.reset();
        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;
        double Pnumber = 0.019;
        // Ensure that the opmode is still active

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = LeftFront.getCurrentPosition() + (int) (LeftFrontCM * COUNTS_PER_CM);
        newLeftBackTarget = LeftBack.getCurrentPosition() + (int) (LeftBackCM * COUNTS_PER_CM);
        newRightFrontTarget = RightFront.getCurrentPosition() + (int) (RightFrontCM * COUNTS_PER_CM);
        newRightBackTarget = RightBack.getCurrentPosition() + (int) (RightBackCM * COUNTS_PER_CM);

        LeftFront.setTargetPosition(newLeftFrontTarget);
        LeftBack.setTargetPosition(newLeftBackTarget);
        RightFront.setTargetPosition(newRightFrontTarget);
        RightBack.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        LeftFront.setPower(Math.abs(speed));
        LeftBack.setPower(Math.abs(speed));
        RightFront.setPower(Math.abs(speed));
        RightBack.setPower(Math.abs(speed));

        while (((RightBack.isBusy()) && LeftFront.isBusy() && runtime.seconds() < Math.abs(LeftFrontCM / 35) || (LeftBack.isBusy()) && RightFront.isBusy() && runtime.seconds() < Math.abs(LeftBackCM / 35))&& runtimestuck.seconds()<=T) {

            /**
             Direction of diagonal
             MyDriveTrain.encoderDrivediagonal(0.8,0,50,50,0);//Left Forward
             MyDriveTrain.encoderDrivediagonal(0.8,0,-50,-50,0);//RIGHT BACK
             MyDriveTrain.encoderDrivediagonal(0.8,50,0,0,50);//RIGHT Forward
             MyDriveTrain.encoderDrivediagonal(0.8,-50,0,0,-50);//Left Back
             **/
        }


        // Stop all motion;
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    } //  sleep(250);   // optional pause after each move


    public void encoderDrive(double speed, double LeftFrontCM, double LeftBackCM, double RightFrontCM, double RightBackCM) {
        runtime.reset();
        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;
        double Pnumber = 0.019;
        // Ensure that the opmode is still active

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = LeftFront.getCurrentPosition() + (int) (LeftFrontCM * COUNTS_PER_CM);
        newLeftBackTarget = LeftBack.getCurrentPosition() + (int) (LeftBackCM * COUNTS_PER_CM);
        newRightFrontTarget = RightFront.getCurrentPosition() + (int) (RightFrontCM * COUNTS_PER_CM);
        newRightBackTarget = RightBack.getCurrentPosition() + (int) (RightBackCM * COUNTS_PER_CM);

        LeftFront.setTargetPosition(newLeftFrontTarget);
        LeftBack.setTargetPosition(newLeftBackTarget);
        RightFront.setTargetPosition(newRightFrontTarget);
        RightBack.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        LeftFront.setPower(Math.abs(speed));
        LeftBack.setPower(Math.abs(speed));
        RightFront.setPower(Math.abs(speed));
        RightBack.setPower(Math.abs(speed));

        while ((LeftFront.isBusy() && LeftBack.isBusy() && RightFront.isBusy()) && RightBack.isBusy() && runtime.seconds() < Math.abs(LeftFrontCM / 35)&& runtimestuck.seconds()<=T) {

        }

        // Stop all motion;
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }  //  sleep(250);   // optional pause after each move


    public void encoderDriveAlahson(double speed, double LeftFrontCM, double RightBackCM) {
        runtime.reset();
        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;
        double Pnumber = 0.019;
        // Ensure that the opmode is still active

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = LeftFront.getCurrentPosition() + (int) (LeftFrontCM * COUNTS_PER_CM);
        newRightBackTarget = RightBack.getCurrentPosition() + (int) (RightBackCM * COUNTS_PER_CM);

        LeftFront.setTargetPosition(newLeftFrontTarget);
        LeftBack.setTargetPosition(newLeftBackTarget);
        RightFront.setTargetPosition(newRightFrontTarget);
        RightBack.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.


        while ((LeftFront.isBusy() && RightBack.isBusy()  && runtime.seconds() < Math.abs(LeftFrontCM / 35))) {

            LeftFront.setPower(Math.abs(speed));
            RightBack.setPower(Math.abs(speed));
        }

        // Stop all motion;
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //  sleep(250);   // optional pause after each move
    }




    public void encoderDriveP( double speed, double LeftFrontCM, double LeftBackCM, double RightFrontCM, double RightBackCM, double KP,double TimeOut) {
        runtime.reset();
        double oldEncoders = LeftFront.getCurrentPosition();

        double error = LeftFrontCM - LeftFront.getCurrentPosition();
        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;
        double Pnumber = 0.019;

        // Ensure that the opmode is still active

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = LeftFront.getCurrentPosition() + (int) (LeftFrontCM * COUNTS_PER_CM);
        newLeftBackTarget = LeftBack.getCurrentPosition() + (int) (LeftBackCM * COUNTS_PER_CM);
        newRightFrontTarget = RightFront.getCurrentPosition() + (int) (RightFrontCM * COUNTS_PER_CM);
        newRightBackTarget = RightBack.getCurrentPosition() + (int) (RightBackCM * COUNTS_PER_CM);

        LeftFront.setTargetPosition(newLeftFrontTarget);
        LeftBack.setTargetPosition(newLeftBackTarget);
        RightFront.setTargetPosition(newRightFrontTarget);
        RightBack.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.

        while ((newLeftFrontTarget >= LeftFront.getCurrentPosition() && runtime.seconds() < TimeOut) && (newLeftBackTarget >= LeftBack.getCurrentPosition() && runtime.seconds() < TimeOut) && (newRightFrontTarget >= RightFront.getCurrentPosition() && runtime.seconds() < TimeOut) && (newRightBackTarget >= RightBack.getCurrentPosition() && runtime.seconds() < TimeOut) || (newLeftFrontTarget <= LeftFront.getCurrentPosition() && runtime.seconds() < TimeOut) && (newLeftBackTarget <= LeftBack.getCurrentPosition() && runtime.seconds() < TimeOut) && (newRightFrontTarget <= RightFront.getCurrentPosition() && runtime.seconds() < TimeOut) && (newRightBackTarget <= RightBack.getCurrentPosition() && runtime.seconds() < TimeOut)&& runtimestuck.seconds()<=T) {

            error = newLeftFrontTarget - LeftFront.getCurrentPosition();

           /*LeftFront.setPower(Math.abs(speed * error * KP));
           LeftBack.setPower(Math.abs(speed*error * KP));
           RightFront.setPower(Math.abs(speed * error * KP));
           RightBack.setPower(Math.abs(speed*error * KP));
*/

            LeftFront.setPower(Math.abs(speed * error * KP));
            LeftBack.setPower(Math.abs(speed * error * KP));
            RightFront.setPower(Math.abs(speed * error * KP));
            RightBack.setPower(Math.abs(speed * error * KP));


        }

        // Stop all motion;
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }




    public void driveWithGyro (double speed, double LeftFrontCM, double LeftBackCM, double RightFrontCM, double RightBackCM,int degrees,double KP, double TimeOut){
        runtime.reset();

        double oldEncoders = LeftFront.getCurrentPosition();
        double errorGyro = (degrees - getAngle());
        double error = LeftFrontCM - LeftFront.getCurrentPosition();
        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;
        double Pnumber = 0.019;
        // Ensure that the opmode is still active

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = LeftFront.getCurrentPosition() + (int) (LeftFrontCM * COUNTS_PER_CM);
        newLeftBackTarget = LeftBack.getCurrentPosition() + (int) (LeftBackCM * COUNTS_PER_CM);
        newRightFrontTarget = RightFront.getCurrentPosition() + (int) (RightFrontCM * COUNTS_PER_CM);
        newRightBackTarget = RightBack.getCurrentPosition() + (int) (RightBackCM * COUNTS_PER_CM);

        LeftFront.setTargetPosition(newLeftFrontTarget);
        LeftBack.setTargetPosition(newLeftBackTarget);
        RightFront.setTargetPosition(newRightFrontTarget);
        RightBack.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.


//Strafe
        while ((((newLeftFrontTarget) <= LeftFront.getCurrentPosition() && runtime.seconds() < TimeOut) && ((newRightBackTarget) <= RightBack.getCurrentPosition() && runtime.seconds() < TimeOut)) && (((newRightFrontTarget) >= RightFront.getCurrentPosition() && runtime.seconds() < TimeOut) && ((newLeftBackTarget) >= LeftBack.getCurrentPosition() && runtime.seconds() < TimeOut))&& runtimestuck.seconds()<=T) {

            error = newLeftFrontTarget - LeftFront.getCurrentPosition();
            errorGyro = -(degrees - getAngle());


         /*  LeftFront.setPower(Math.abs(speed - errorGyro * KP));
           LeftBack.setPower(Math.abs(speed +  errorGyro * KP));
           RightFront.setPower(Math.abs(speed - errorGyro * KP));
           RightBack.setPower(Math.abs(speed  + errorGyro * KP));
*/
            LeftFront.setPower(Math.abs(speed - errorGyro * KP));
            LeftBack.setPower(Math.abs(speed + errorGyro * KP));
            RightFront.setPower(Math.abs(speed - errorGyro * KP));
            RightBack.setPower(Math.abs(speed + errorGyro * KP));

        }

//Strafe
        while ((((newLeftFrontTarget) >= LeftFront.getCurrentPosition() && runtime.seconds() < TimeOut) && ((newRightBackTarget) >= RightBack.getCurrentPosition() && runtime.seconds() < TimeOut)) && (((newRightFrontTarget) <= RightFront.getCurrentPosition() && runtime.seconds() < TimeOut) && ((newLeftBackTarget) <= LeftBack.getCurrentPosition() && runtime.seconds() < TimeOut))&& runtimestuck.seconds()<=T) {

            error = newLeftFrontTarget - LeftFront.getCurrentPosition();
            errorGyro = (degrees - getAngle());


         /*  LeftFront.setPower(Math.abs(speed - errorGyro * KP));
           LeftBack.setPower(Math.abs(speed +  errorGyro * KP));
           RightFront.setPower(Math.abs(speed - errorGyro * KP));
           RightBack.setPower(Math.abs(speed  + errorGyro * KP));
*/
            LeftFront.setPower(Math.abs(speed - errorGyro * KP));
            LeftBack.setPower(Math.abs(speed + errorGyro * KP));
            RightFront.setPower(Math.abs(speed - errorGyro * KP));
            RightBack.setPower(Math.abs(speed + errorGyro * KP));

        }
//forward/back
        while (((newLeftFrontTarget) >= LeftFront.getCurrentPosition() && runtime.seconds() < TimeOut) && ((newLeftBackTarget) >= LeftBack.getCurrentPosition() && runtime.seconds() < TimeOut) && ((newRightFrontTarget) >= RightFront.getCurrentPosition() && runtime.seconds() < TimeOut) && ((newRightBackTarget) >= RightBack.getCurrentPosition() && runtime.seconds() < TimeOut)&& runtimestuck.seconds()<=T) {

            error = newLeftFrontTarget - LeftFront.getCurrentPosition();
            errorGyro = (degrees - getAngle());


            LeftFront.setPower(Math.abs(speed - errorGyro * KP));
            LeftBack.setPower(Math.abs(speed - errorGyro * KP));
            RightFront.setPower(Math.abs(speed + errorGyro * KP));
            RightBack.setPower(Math.abs(speed + errorGyro * KP));

        }

//forward/back
        while (((newLeftFrontTarget) < LeftFront.getCurrentPosition() && runtime.seconds() < TimeOut) && ((newLeftBackTarget) < LeftBack.getCurrentPosition() && runtime.seconds() < TimeOut) && ((newRightFrontTarget) < RightFront.getCurrentPosition() && runtime.seconds() < TimeOut) && ((newRightBackTarget) < RightBack.getCurrentPosition() && runtime.seconds() < TimeOut)&& runtimestuck.seconds()<=T) {

            error = newLeftFrontTarget - LeftFront.getCurrentPosition();
            errorGyro = -(degrees - getAngle());


            LeftFront.setPower(Math.abs(speed - errorGyro * KP));
            LeftBack.setPower(Math.abs(speed - errorGyro * KP));
            RightFront.setPower(Math.abs(speed + errorGyro * KP));
            RightBack.setPower(Math.abs(speed + errorGyro * KP));

        }


        // Stop all motion;
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }//  sleep(250);   // optional pause after each move



    public void driveWithGyroAndServo (double speed, double LeftFrontCM, double LeftBackCM, double RightFrontCM, double RightBackCM,int degrees,double KP,double GoSkyStone ,double TimeOut){
        runtime.reset();

        double oldEncoders = LeftFront.getCurrentPosition();
        double errorGyro =  (degrees-getAngle());
        double error = LeftFrontCM - LeftFront.getCurrentPosition() ;
        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;
        double Pnumber = 0.019;
        // Ensure that the opmode is still active




        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = LeftFront.getCurrentPosition() + (int) (LeftFrontCM * COUNTS_PER_CM);
        newLeftBackTarget = LeftBack.getCurrentPosition() + (int) (LeftBackCM * COUNTS_PER_CM);
        newRightFrontTarget = RightFront.getCurrentPosition() + (int) (RightFrontCM * COUNTS_PER_CM);
        newRightBackTarget = RightBack.getCurrentPosition() + (int) (RightBackCM * COUNTS_PER_CM);

        LeftFront.setTargetPosition(newLeftFrontTarget);
        LeftBack.setTargetPosition(newLeftBackTarget);
        RightFront.setTargetPosition(newRightFrontTarget);
        RightBack.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        GoSkyStone=GoSkyStone*COUNTS_PER_CM;


        while ((((newLeftFrontTarget) <= LeftFront.getCurrentPosition()&& runtime.seconds() < TimeOut )&&((newRightBackTarget) <= RightBack.getCurrentPosition()&& runtime.seconds() < TimeOut))&&(((newRightFrontTarget) >= RightFront.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newLeftBackTarget) >= LeftBack.getCurrentPosition()&& runtime.seconds() < TimeOut))) {

            if ((LeftFront.getCurrentPosition() >= GoSkyStone)||(LeftBack.getCurrentPosition() >= GoSkyStone)){
                SkyStone.setPosition(0.47);
                Finger.setPosition(0.44);
            }

            // error = newLeftFrontTarget - LeftFront.getCurrentPosition();
            errorGyro = -(degrees - getAngle());



         /*  LeftFront.setPower(Math.abs(speed - errorGyro * KP));
           LeftBack.setPower(Math.abs(speed +  errorGyro * KP));
           RightFront.setPower(Math.abs(speed - errorGyro * KP));
           RightBack.setPower(Math.abs(speed  + errorGyro * KP));
*/

            LeftFront.setPower(Math.abs(speed - errorGyro * KP));
            LeftBack.setPower(Math.abs(speed +  errorGyro * KP));
            RightFront.setPower(Math.abs(speed - errorGyro * KP));
            RightBack.setPower(Math.abs(speed  + errorGyro * KP));



        }

//Strafe
        while ((((newLeftFrontTarget) >= LeftFront.getCurrentPosition()&& runtime.seconds() < TimeOut )&&((newRightBackTarget) >= RightBack.getCurrentPosition()&& runtime.seconds() < TimeOut))&&(((newRightFrontTarget) <= RightFront.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newLeftBackTarget) <= LeftBack.getCurrentPosition()&& runtime.seconds() < TimeOut))) {

            if ((LeftFront.getCurrentPosition() >= GoSkyStone)||(LeftBack.getCurrentPosition() >= GoSkyStone)){
                SkyStone.setPosition(0.47);
                Finger.setPosition(0.44);
            }

            //  error = newLeftFrontTarget - LeftFront.getCurrentPosition();
            errorGyro = (degrees - getAngle());


         /*  LeftFront.setPower(Math.abs(speed - errorGyro * KP));
           LeftBack.setPower(Math.abs(speed +  errorGyro * KP));
           RightFront.setPower(Math.abs(speed - errorGyro * KP));
           RightBack.setPower(Math.abs(speed  + errorGyro * KP));
*/

            LeftFront.setPower(Math.abs(speed - errorGyro * KP));
            LeftBack.setPower(Math.abs(speed +  errorGyro * KP));
            RightFront.setPower(Math.abs(speed - errorGyro * KP));
            RightBack.setPower(Math.abs(speed  + errorGyro * KP));



        }

        while (((newLeftFrontTarget) >= LeftFront.getCurrentPosition()&& runtime.seconds() < TimeOut )&&((newLeftBackTarget) >= LeftBack.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newRightFrontTarget) >= RightFront.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newRightBackTarget) >= RightBack.getCurrentPosition()&& runtime.seconds() < TimeOut)) {

            if ((LeftFront.getCurrentPosition() >= GoSkyStone)||(LeftBack.getCurrentPosition() >= GoSkyStone)){
                SkyStone.setPosition(0.47);
                Finger.setPosition(0.44);
            }

            //  error = newLeftFrontTarget - LeftFront.getCurrentPosition();
            errorGyro = (degrees - getAngle());



            LeftFront.setPower(Math.abs(speed - errorGyro * KP));
            LeftBack.setPower(Math.abs(speed - errorGyro * KP));
            RightFront.setPower(Math.abs(speed  + errorGyro * KP));
            RightBack.setPower(Math.abs(speed  + errorGyro * KP));



        }


        while (((newLeftFrontTarget) < LeftFront.getCurrentPosition()&& runtime.seconds() < TimeOut )&&((newLeftBackTarget) < LeftBack.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newRightFrontTarget) < RightFront.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newRightBackTarget) < RightBack.getCurrentPosition()&& runtime.seconds() < TimeOut)) {

            if ((LeftFront.getCurrentPosition() >= GoSkyStone)||(LeftBack.getCurrentPosition() >= GoSkyStone)){
                SkyStone.setPosition(0.47);
                Finger.setPosition(0.44);
            }

            //  error = newLeftFrontTarget - LeftFront.getCurrentPosition();
            errorGyro = -(degrees - getAngle());



            LeftFront.setPower(Math.abs(speed - errorGyro * KP));
            LeftBack.setPower(Math.abs(speed - errorGyro * KP));
            RightFront.setPower(Math.abs(speed  + errorGyro * KP));
            RightBack.setPower(Math.abs(speed  + errorGyro * KP));



        }




        // Stop all motion;
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //  sleep(250);   // optional pause after each move
    }



    //TODO***********************************************************************
    public void GyroWithDriveP (double speed, double LeftFrontCM, double LeftBackCM, double RightFrontCM, double RightBackCM,int degrees,double KPGyro,double KPDrive, double TimeOut){
        runtime.reset();

        double oldEncoders = LeftFront.getCurrentPosition();
        double errorGyro =  (degrees-getAngle());
        double error = LeftFrontCM - LeftFront.getCurrentPosition() ;
        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;
        double Pnumber = 0.019;
        // Ensure that the opmode is still active

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = LeftFront.getCurrentPosition() + (int) (LeftFrontCM * COUNTS_PER_CM);
        newLeftBackTarget = LeftBack.getCurrentPosition() + (int) (LeftBackCM * COUNTS_PER_CM);
        newRightFrontTarget = RightFront.getCurrentPosition() + (int) (RightFrontCM * COUNTS_PER_CM);
        newRightBackTarget = RightBack.getCurrentPosition() + (int) (RightBackCM * COUNTS_PER_CM);

        LeftFront.setTargetPosition(newLeftFrontTarget);
        LeftBack.setTargetPosition(newLeftBackTarget);
        RightFront.setTargetPosition(newRightFrontTarget);
        RightBack.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.

        while ((((newLeftFrontTarget) <= LeftFront.getCurrentPosition()&& runtime.seconds() < TimeOut )&&((newRightBackTarget) <= RightBack.getCurrentPosition()&& runtime.seconds() < TimeOut))&&(((newRightFrontTarget) >= RightFront.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newLeftBackTarget) >= LeftBack.getCurrentPosition()&& runtime.seconds() < TimeOut))) {

            error = newLeftFrontTarget - LeftFront.getCurrentPosition();
            errorGyro = -(degrees - getAngle());


         /*  LeftFront.setPower(Math.abs(speed - errorGyro * KP));
           LeftBack.setPower(Math.abs(speed +  errorGyro * KP));
           RightFront.setPower(Math.abs(speed - errorGyro * KP));
           RightBack.setPower(Math.abs(speed  + errorGyro * KP));
*/
            LeftFront.setPower(Math.abs(speed - errorGyro * KPGyro));
            LeftBack.setPower(Math.abs(speed +  errorGyro * KPGyro));
            RightFront.setPower(Math.abs(speed - errorGyro * KPGyro));
            RightBack.setPower(Math.abs(speed  + errorGyro * KPGyro));

        }

//Strafe
        while ((((newLeftFrontTarget) >= LeftFront.getCurrentPosition()&& runtime.seconds() < TimeOut )&&((newRightBackTarget) >= RightBack.getCurrentPosition()&& runtime.seconds() < TimeOut))&&(((newRightFrontTarget) <= RightFront.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newLeftBackTarget) <= LeftBack.getCurrentPosition()&& runtime.seconds() < TimeOut))) {

            error = newLeftFrontTarget - LeftFront.getCurrentPosition();
            errorGyro = (degrees - getAngle());


         /*  LeftFront.setPower(Math.abs(speed - errorGyro * KP));
           LeftBack.setPower(Math.abs(speed +  errorGyro * KP));
           RightFront.setPower(Math.abs(speed - errorGyro * KP));
           RightBack.setPower(Math.abs(speed  + errorGyro * KP));
*/
            LeftFront.setPower(Math.abs(speed - errorGyro * KPGyro));
            LeftBack.setPower(Math.abs(speed +  errorGyro * KPGyro));
            RightFront.setPower(Math.abs(speed - errorGyro * KPGyro));
            RightBack.setPower(Math.abs(speed  + errorGyro * KPGyro));

        }

        while (((newLeftFrontTarget) >= LeftFront.getCurrentPosition()&& runtime.seconds() < TimeOut )&&((newLeftBackTarget) >= LeftBack.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newRightFrontTarget) >= RightFront.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newRightBackTarget) >= RightBack.getCurrentPosition()&& runtime.seconds() < TimeOut)) {

            error = newLeftFrontTarget - LeftFront.getCurrentPosition();
            errorGyro = (degrees - getAngle());


            LeftFront.setPower(Math.abs(speed - errorGyro * KPGyro));
            LeftBack.setPower(Math.abs(speed - errorGyro * KPGyro));
            RightFront.setPower(Math.abs(speed  + errorGyro * KPGyro));
            RightBack.setPower(Math.abs(speed  + errorGyro * KPGyro));

        }


        while (((newLeftFrontTarget) < LeftFront.getCurrentPosition()&& runtime.seconds() < TimeOut )&&((newLeftBackTarget) < LeftBack.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newRightFrontTarget) < RightFront.getCurrentPosition()&& runtime.seconds() < TimeOut)&&((newRightBackTarget) < RightBack.getCurrentPosition()&& runtime.seconds() < TimeOut)) {

            error = newLeftFrontTarget - LeftFront.getCurrentPosition();
            errorGyro = -(degrees - getAngle());


            LeftFront.setPower(Math.abs(speed - errorGyro * KPGyro));
            LeftBack.setPower(Math.abs(speed - errorGyro * KPGyro));
            RightFront.setPower(Math.abs(speed  + errorGyro * KPGyro));
            RightBack.setPower(Math.abs(speed  + errorGyro * KPGyro
            ));

        }


        // Stop all motion;
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //  sleep(250);   // optional pause after each move
    }


    //TODO***********************************************************************

    public void encoderDriveInLoop(double speed, double LeftFrontCM, double LeftBackCM, double RightFrontCM, double RightBackCM) {
        runtime.reset();
        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;

        // Ensure that the opmode is still active


        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = LeftFront.getCurrentPosition() + (int) (LeftFrontCM * COUNTS_PER_CM);
        newLeftBackTarget = LeftBack.getCurrentPosition() + (int) (LeftBackCM * COUNTS_PER_CM);
        newRightFrontTarget = RightFront.getCurrentPosition() + (int) (RightFrontCM * COUNTS_PER_CM);
        newRightBackTarget = RightBack.getCurrentPosition() + (int) (RightBackCM * COUNTS_PER_CM);

        LeftFront.setTargetPosition(newLeftFrontTarget);
        LeftBack.setTargetPosition(newLeftBackTarget);
        RightFront.setTargetPosition(newRightFrontTarget);
        RightBack.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        /*runtime.reset();*/
        LeftFront.setPower(Math.abs(speed));
        LeftBack.setPower(Math.abs(speed));
        RightFront.setPower(Math.abs(speed));
        RightBack.setPower(Math.abs(speed));

        // Turn off RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //  sleep(250);   // optional pause after each move
    }

    public void startAndResetEncoders() {

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    private void resetAngle() {
        lastAngles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }


    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = Math.toDegrees(angles.firstAngle - lastAngles.firstAngle);

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    public void RotateP(int degrees, double power, double timeoutR,double KP) {
        runtime.reset();

        double PNumber = 0.0108;
        double INumber = 0;
        double SumErrors = 0;
        double oldSumErors = 0;
        double NewAngleTarget = 0;

//        if (getAngle()-degrees > 110) {
//
//            PNumber = 0.0088;
//        }
//        if (getAngle()-degrees>30){
//            PNumber = 0.4;
//        }
        //         else  if(getAngle()-degrees < 30) {
        //            PNumber = 0.5;
        //
        //        }
//        degrees =  degrees-LastDegrees;
//        resetAngle();
        // restart imu movement tracking.

        if (getAngle() < degrees) {
            while (getAngle() < degrees && runtime.seconds() < timeoutR && runtimestuck.seconds()<=T) {
                //SumErrors = SumErrors + (getAngle() + degrees);
                double error = getAngle() - degrees;

                LeftFront.setPower(power * error * KP /* * (((getAngle() -  degrees) * PNumber) + SumErrors * INumber)*/);
                LeftBack.setPower(power * error * KP/* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
                RightFront.setPower(-power * error * KP/* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
                RightBack.setPower(-power * error * KP/* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);

            }
        } else if (getAngle() > degrees) {
            while (getAngle() > degrees && runtime.seconds() < timeoutR && runtimestuck.seconds()<=T) {
                double error = degrees - getAngle();

                LeftFront.setPower(-power * error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
                LeftBack.setPower(-power * error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
                RightFront.setPower(power * error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
                RightBack.setPower(power * error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);


            }
        } else return;
        // turn the motors off.
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

//        LastDegrees = degrees;

    }// reset -angle tracking on new heading


    public void Rotate(int degrees, double power, double timeoutR) {
        runtime.reset();


        double PNumber = 0.0108;
        double INumber = 0;
        double SumErrors = 0;
        double oldSumErors = 0;
        double NewAngleTarget = 0;

//        if (getAngle()-degrees > 110) {
//
//            PNumber = 0.0088;
//        }
//        if (getAngle()-degrees>30){
//            PNumber = 0.4;
//        }
        //         else  if(getAngle()-degrees < 30) {
        //            PNumber = 0.5;
        //
        //        }
//        degrees =  degrees-LastDegrees;
//        resetAngle();
        // restart imu movement tracking.

        if (getAngle() < degrees) {
            while (getAngle() < degrees && runtime.seconds() < timeoutR && runtimestuck.seconds()<=T) {
                //SumErrors = SumErrors + (getAngle() + degrees);
                double error = degrees - getAngle();
                LeftFront.setPower(-power /* error * KP /* * (((getAngle() -  degrees) * PNumber) + SumErrors * INumber)*/);
                LeftBack.setPower(-power /* error * KP/* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
                RightFront.setPower(power /* error * KP/* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
                RightBack.setPower(power /* error * KP/* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);

            }
        } else if (getAngle() > degrees) {
            while (getAngle() > degrees && runtime.seconds() < timeoutR && runtimestuck.seconds()<=T) {
                double error = getAngle() - degrees;
                LeftFront.setPower(power /* error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
                LeftBack.setPower(power /* error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
                RightFront.setPower(-power /* error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);
                RightBack.setPower(-power /* error * KP  /* * (((getAngle() - degrees) * PNumber) + SumErrors * INumber)*/);


            }
        } else return;
        // turn the motors off.
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

//        LastDegrees = degrees;

    } // reset -angle tracking on new heading
}