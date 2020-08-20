package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class elevator {

    // private DcMotor leftEle = null;
    private DcMotor Elevator = null;
    private DigitalChannel upMagnet = null;
    private DigitalChannel downMagnet = null;
    public int fixedPosition = 0;


    public elevator(DcMotor Elevator) {
        // this.leftEle = leftEle;
        this.Elevator = Elevator;
//        this.upMagnet = upMagnet;
//        this.downMagnet = downMagnet;
////        this.fixedPosition = fixedPosition;

        //this.leftEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getCurrentPosition() {
        return Elevator.getCurrentPosition();
    }

    public void moveElevator(double leftPower, double rightPower) {
        //leftEle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //leftEle.setPower(leftPower);
        Elevator.setPower(rightPower);
    }

    public void ElevateWithEncoder(double power, int direction, int Jump) {

        // int newrightTargetPositin = Elevator.getCurrentPosition() + pos;
        Elevator.setTargetPosition(Elevator.getCurrentPosition() + (Jump * direction));
        Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Elevator.setPower(power);
        Elevator.getCurrentPosition();


    }

    public void dontMoveElevator(double power, int stayingPosition/*,double PN,int leftCurrent, int rightCurrent*/) {
        double error = fixedPosition - Elevator.getCurrentPosition();
        Elevator.setTargetPosition(stayingPosition);
        Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Elevator.setPower(power);
    }


}