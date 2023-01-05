package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Slide {

    private final MotorEx motor;

    private int target = 0;

    public Slide(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "slides", Motor.GoBILDA.RPM_1620);
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setInverted(false);
        motor.setPositionCoefficient(0.04);
        motor.setPositionTolerance(30);
        motor.setTargetPosition(0);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    private void runTo(int t){
        motor.setInverted(false);
        motor.setTargetPosition(t);
        target = t;
    }
    public void goDown() {
        runTo(motor.getCurrentPosition()-20);
    }

    public void goUp()
    {
        runTo(motor.getCurrentPosition()+40);
    }

    public void runToTop(){
        runTo(730);
        if (motor.getCurrentPosition() < (target-100)){
            motor.set(0.4);
        }
    }

    public void runToMiddle(){
        runTo(500);
    }

    public void runToLow(){
        runTo(300);
    }

    public void runToBottom(){
        runTo(0);
        if (motor.getCurrentPosition() < (target+150)){
            motor.set(0.2);
        }
    }
    public void runPower(double speed){
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.set(speed);
    }

    public void stopManual(){
        motor.setRunMode(Motor.RunMode.PositionControl);
        runTo(motor.getCurrentPosition());
    }
    public void runToCone(){
        runTo(110);
    }
    public void periodic(){
        if(motor.atTargetPosition()){
            motor.set(0.4);
        }else if(motor.getCurrentPosition() < target){
            motor.set(0.6);
        }else{
            motor.set(0.001);
        }
    }
    public double isHigh(){
        return (double)motor.getCurrentPosition()/1000;
    }

}
