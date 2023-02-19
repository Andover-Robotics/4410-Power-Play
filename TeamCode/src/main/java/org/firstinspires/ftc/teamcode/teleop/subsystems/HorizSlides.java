package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.MotionProfiler;

@Config
public class HorizSlides {

    private final MotorEx motor;
    private final PIDFController controller;
    private final OpMode opMode;
    public static double p = 0.04, i = 0, d = 0, f = 0;
    public static double tolerance = 10, powerUp = 0.1, manualDivide = 1.5, manualPower = 0, powerMin = 0.1;
    public static int fullOut = 580, fullIn = 0, outtake = 0;
    private int target = 0;
    private double profile_init_time = 0;

    private static MotionProfiler profiler = new MotionProfiler(1, 1);

    public HorizSlides(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "slidesHoriz", Motor.GoBILDA.RPM_1150);
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

    public void runToFullIn(){
        runTo(fullIn);
    }
    public void runToFullOut(){
        runTo(fullOut);
    }
    public void runToOuttake(){
        runTo(outtake);
    }

    public void runManual(double manual){
        if(manual > powerMin || manual < -powerMin){
            manualPower = manual;
        }else{
            manualPower = 0;
        }
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
