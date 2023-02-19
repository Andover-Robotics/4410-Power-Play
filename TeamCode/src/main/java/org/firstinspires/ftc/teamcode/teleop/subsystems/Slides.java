package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.MotionProfiler;

@Config
public class Slides {

    private final MotorEx motorLeft;
    private final MotorEx motorRight;
    private final PIDFController controller;
    public static double p = 0, i = 0, d = 0, f = 0, staticF = 0;
    public static double tolerance = 0, powerUp = 0, powerDown = 0, manualDivide = 1, manualPower = 0, powerMin = 0.1;
    public static int MAXHEIGHT = 5000, top = 3700, mid = 2900, low = 1700, ground = 0, inc = 100, dec = 100;

    private int target = 0;

    private final OpMode opMode;
    private double profile_init_time = 0;
    private static MotionProfiler profiler = new MotionProfiler(1, 1);

    public Slides(OpMode opMode){
        motorLeft = new MotorEx(opMode.hardwareMap, "slidesLeft", Motor.GoBILDA.RPM_435);
        motorRight = new MotorEx(opMode.hardwareMap, "slidesRight", Motor.GoBILDA.RPM_435);
        motorRight.setInverted(true);
        motorLeft.setInverted(false);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(target);
        motorLeft.setRunMode(Motor.RunMode.RawPower);
        motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRight.setRunMode(Motor.RunMode.RawPower);
        motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.opMode = opMode;
    }

    public void runTo(int t){
        profiler.init_new_profile(motorLeft.getCurrentPosition(), t);
        profile_init_time = opMode.time;
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

    public void runManual(double manual){
        if(manual > powerMin || manual < -powerMin){
            manualPower = manual;
        }else{
            manualPower = 0;
        }
    }

    public void periodic(){
        motorRight.setInverted(false);
        motorLeft.setInverted(true);
        controller.setPIDF(p, i, d, f);
        double dt = opMode.time - profile_init_time;
        if(manualPower == 0 || dt < profiler.getEntire_dt()) {
            controller.setSetPoint(profiler.motion_profile_pos(dt));
            if(dt < profiler.getEntire_dt()){
                double power = powerUp * controller.calculate(motorLeft.getCurrentPosition());
                motorLeft.set(power);
                motorRight.set(power);
            }else{
                motorLeft.set(staticF);
                motorRight.set(staticF);
            }
        }else {
            controller.setSetPoint(motorLeft.getCurrentPosition());
            motorLeft.set(manualPower / manualDivide);
            motorRight.set(manualPower / manualDivide);
        }
    }
    public double isHigh(){
        return (double) motorLeft.getCurrentPosition()/MAXHEIGHT;
    }

    public double getCurrent(){
        return motorLeft.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + motorRight.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public void resetEncoder(){
        motorLeft.resetEncoder();
    }

    public int getPosition(){
        return motorLeft.getCurrentPosition();
    }

}
