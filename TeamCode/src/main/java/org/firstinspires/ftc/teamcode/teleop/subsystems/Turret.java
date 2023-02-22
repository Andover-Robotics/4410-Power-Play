package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class Turret {
    private final MotorEx motor;
    private final PIDFController controller;

    public static double p = 0.07, i = 0, d = 0.003, f = 0;
    private double tolerance = 5, powerUp = 0.1, manualDivide = 1.5, manualPower = 0, powerMin = 0.1;
    public static double tickToAngle = 3200/Math.PI/2;
    public static int saveState = 0, turretAutoOuttakeRight = -420, turretAutoIntakeRight = 830, turretAutoOuttakeLeft = 420, turretAutoIntakeLeft = -830, limit = 5400;
    private int target = 0;

    public Turret(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(target);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void runTo(int t){
        controller.setSetPoint(t);
        target = t;
    }

    public void runToAutoOuttakeRight(){
        runTo(turretAutoOuttakeRight);
    }

    public void runToAutoIntakeRight(){
        runTo(turretAutoIntakeRight);
    }
    public void runToAutoOuttakeLeft(){runTo(turretAutoOuttakeLeft);}

    public void runToAutoIntakeLeft(){runTo(turretAutoIntakeLeft);}

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
        int target = (int)((angle - imu)*tickToAngle);
        while(Math.abs(target - motor.getCurrentPosition()) > tickToAngle*3/5){
            if(target < motor.getCurrentPosition()){
                target += tickToAngle*2*Math.PI;
            }else{
                target -= tickToAngle*2*Math.PI;
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
