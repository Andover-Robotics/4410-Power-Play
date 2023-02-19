package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.MotionProfiler;

@Config
public class Turret {
    private final MotorEx motor;
    private final PIDFController controller;

    public static double slideOut = 0, slideIn = 1;
    public static double p = 0, i = 0, d = 0, f = 0, staticF = 0;
    public static double tolerance = 0, powerUp = 0,    powerDown = 0, manualDivide = 1.5, manualPower = 0, powerMin = 0.1;
    public static double tickToAngle = 1000;
    public static int MAXHEIGHT = 5000, fullOut = 3700, fullIn = 2900, outtake = 1700, saveState = 0;
    private int target = 0;
    private final OpMode opMode;
    private double profile_init_time = 0;
    private static MotionProfiler profiler = new MotionProfiler(1, 1);

    public Turret(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(target);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.opMode = opMode;
    }

    public void runTo(int t){
        profiler.init_new_profile(motor.getCurrentPosition(), t);
        profile_init_time = opMode.time;
        target = t;
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
        //TODO calculate ticksTOAngle
    }

    public void savePosition(){
        saveState = motor.getCurrentPosition();
    }

    public void periodic(){
        motor.setInverted(false);
        controller.setPIDF(p, i, d, f);
        double dt = opMode.time - profile_init_time;
        if(manualPower == 0 || dt < profiler.getEntire_dt()) {
            controller.setSetPoint(profiler.motion_profile_pos(dt));
            motor.set(powerUp * controller.calculate(motor.getCurrentPosition()));
        }else{
            controller.setSetPoint(motor.getCurrentPosition());
            motor.set(manualPower/manualDivide);
        }
    }

    public void resetEncoder(){
        motor.resetEncoder();
    }

    public int getPosition(){
        return motor.getCurrentPosition();
    }

}
