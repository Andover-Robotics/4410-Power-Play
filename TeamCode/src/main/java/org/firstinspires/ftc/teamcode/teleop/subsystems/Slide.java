package org.firstinspires.ftc.teamcode.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

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

    public void runToTop(){
        runTo(730);
    }

    public void runToMiddle(){
        runTo(500);
    }

    public void runToLow(){
        runTo(300);
    }

    public void runToBottom(){
        runTo(0);
    }

    public void periodic(){
        if(motor.atTargetPosition()){
            motor.set(0.4);
        }else if(motor.getCurrentPosition() < target){
            motor.set(0.9);
        }else{
            motor.set(0.04);
        }
    }

}