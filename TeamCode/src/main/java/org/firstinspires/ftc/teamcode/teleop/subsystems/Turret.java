package org.firstinspires.ftc.teamcode.teleop.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class Turret {
    private final MotorEx motor;
    private PIDFController controller;

    public static double p = 0.07, i = 0, d = 0.003, f = 0;//TODO experiment with increasing P and D to get more precision
    private double tolerance = 5, powerUp = 0.1, manualDivide = 1.5, manualPower = 0, powerMin = 0.05;
    public static double tickToAngle = 3300.0/360, fullRotation = 3300.0;
    public static int saveState = 0, turretAutoOuttakeRight = -420, turretAutoIntakeRight = 830, turretAutoOuttakeLeft = 420, turretAutoIntakeLeft = -838, limit = 5400,
            turretAutoOuttakeMidRight = -1260, turretAutoOuttakeMidLeft = 1260;
    public Turret(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void runTo(int t){
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(t);
    }


    public void runToAutoOuttakeRight(double imu){
        runTo(turretAutoOuttakeRight + (int)(imu*tickToAngle));
    }
    public void runToAutoIntakeRight(double imu){
        runTo(turretAutoIntakeRight + (int)(imu*tickToAngle));
    }
    public void runToAutoOuttakeLeft(double imu){
        runTo(turretAutoOuttakeLeft + (int)(imu*tickToAngle));
    }
    public void runToAutoIntakeLeft(double imu){
        runTo(turretAutoIntakeLeft + (int)(imu*tickToAngle));
    }
    public void runToTeleOpOuttakeRight(double imu){
        runTo(turretAutoOuttakeRight + (int)fullRotation + (int)(imu*tickToAngle));
    }

    public void runToIntake(double imu){
        int target = (int)tickToAngle*360/2 + (int)(imu*tickToAngle);
        runTo(target);
    }

    public void runToFront(){
        runTo(0);
    }



    public void runToSaveState(){
        runTo(saveState);
    }

    public void runManual(double manual){
        if(manual > powerMin || manual < -powerMin){
            manualPower = manual;
        }else{
            manualPower = 0;
        }
    }

    public void runToAngle(double angle, double imu){
        int target = (int)((angle + imu)*tickToAngle);
        while(Math.abs(target - motor.getCurrentPosition()) > tickToAngle*360*3/5){
            if(target < motor.getCurrentPosition()){
                target += tickToAngle*360;
            }else{
                target -= tickToAngle*360;
            }
            if(target > limit){
                target -= tickToAngle*360;
            }else if(target < -limit){
                target += tickToAngle*360;
            }
        }
        runTo(target);
    }

    public void savePosition(){
        saveState = motor.getCurrentPosition();
    }

    public void periodic(){
        motor.setInverted(false);
        controller.setPIDF(p, i, d, f);
        if(Math.abs(motor.getCurrentPosition()) > limit){
            if(motor.getCurrentPosition() > 0){
                controller.setSetPoint(limit-100);
            }else{
                controller.setSetPoint(100-limit);
            }
            motor.set(powerUp * controller.calculate(motor.getCurrentPosition()));
        }else {
            if (manualPower != 0) {
                motor.set(manualPower / manualDivide);
                controller.setSetPoint(motor.getCurrentPosition());
            } else {
                motor.set(powerUp * controller.calculate(motor.getCurrentPosition()));
            }
        }
    }

    public void resetEncoder(){
        motor.resetEncoder();
    }

    public int getPosition(){
        return motor.getCurrentPosition();
    }

}
