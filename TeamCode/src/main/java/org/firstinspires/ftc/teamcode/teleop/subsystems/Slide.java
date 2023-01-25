package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.checkerframework.checker.signedness.qual.Constant;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Slide {

    private final MotorEx motors;
//    private final MotorEx motorRight;
//    private final MotorGroup motors;
    private final PIDFController controller;
    public static double p = 0.04, d = 0, f = 0, staticF = 0.1;
    public static double tolerance = 100, powerUp = 0.4;
    public static int MAXHEIGHT = 5000, top = 3700, mid = 2900, low = 1700, ground = 0, inc = 100, dec = 100;

    private int target = 0;

    public Slide(OpMode opMode){
        motors = new MotorEx(opMode.hardwareMap, "slides", Motor.GoBILDA.RPM_312);
//        motorRight = new MotorEx(opMode.hardwareMap, "slidesRight", Motor.GoBILDA.RPM_435);
//        motorRight.setInverted(true);
        motors.setInverted(false);
        controller = new PIDFController(p, 0, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(target);
        motors.setRunMode(Motor.RunMode.RawPower);
        motors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void runTo(int t){
//        motorRight.setInverted(true);
        motors.setInverted(false);
        controller.setSetPoint(t);
        target = t;
    }
    public void goDown() {
        runTo(target-dec);
    }

    public void goUp()
    {
        runTo(target+inc);
    }

    public void runToTop(){
        runTo(top);
    }

    public void runToMiddle(){
        runTo(mid);
    }

    public void runToLow(){
        runTo(low);
    }

    public void runToBottom(){
        runTo(ground);
    }

    public void periodic(){
        controller.setPIDF(p, 0, d , f);
        if(controller.atSetPoint()){
            motors.set(staticF);
        }else{
            motors.set(powerUp * controller.calculate(motors.getCurrentPosition()));
        }
    }
    public double isHigh(){
        return (double) motors.getCurrentPosition()/MAXHEIGHT;
    }

    public double getCurrent(){
        return motors.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }

}
