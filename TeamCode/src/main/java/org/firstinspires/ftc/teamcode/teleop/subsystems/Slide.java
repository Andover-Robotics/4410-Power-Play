package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Slide {

    private final MotorEx motor;

    private int target = 0;

    public Slide(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "slides", Motor.GoBILDA.RPM_312);
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setInverted(false);
        motor.setPositionCoefficient(0.04);
        motor.setPositionTolerance(100);
        motor.setTargetPosition(0);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void runTo(int t){
        motor.setInverted(false);
        motor.setTargetPosition(t);
        target = t;
    }
    public void goDown() {
        runTo(target-100);
    }

    public void goUp()
    {
        runTo(target+100);
    }

    public void runToTop(){
        runTo(3700);
    }

    public void runToMiddle(){
        runTo(2900);
    }

    public void runToLow(){
        runTo(1700);
    }

    public void runToBottom(){
        runTo(0);
    }
    public void runPower(double speed){
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.set(speed);
    }

    public void stopManual(){
        motor.setRunMode(Motor.RunMode.PositionControl);
        runTo(motor.getCurrentPosition());
    }

//    public void runToCone(){
//        runTo(110);
//    }

    public void periodic(){
        if(motor.atTargetPosition()){
            motor.set(0.1);
        }else if(motor.getCurrentPosition() < target){
            motor.set(0.4);
        }else{
            motor.set(0.1);
        }
    }
    public double isHigh(){
        return (double)motor.getCurrentPosition()/5000;
    }

    public double getCurrent(){
        return motor.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }

}
