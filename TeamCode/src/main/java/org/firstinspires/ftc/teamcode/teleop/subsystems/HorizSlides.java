package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.MotionProfiler;

@Config
public class HorizSlides {

    public final MotorEx motor;
    private PIDFController controller;
    private final OpMode opMode;
    public static double p = 0.04, i = 0, d = 0, f = 0;
    private final double tolerance = 5, powerUp = 0.1, manualDivide = 1.5, powerMin = 0.1, powerHold = 0.2;
    private double manualPower = 0;
    public static int fullOut = 540, fullIn = 0, autoIntake = 540, teleOpIntake = 540, autoOuttake = 100;
    private double profile_init_time = 0;

    private MotionProfiler profiler = new MotionProfiler(8000, 8000);

    public HorizSlides(OpMode opMode) {
        motor = new MotorEx(opMode.hardwareMap, "slidesHoriz", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.opMode = opMode;
    }

    public void runTo(int t) {
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        resetProfiler();
        profiler.init_new_profile(motor.getCurrentPosition(), t);
        profile_init_time = opMode.time;
    }

    public void saveTeleOpIntake() {
        teleOpIntake = motor.getCurrentPosition();
    }

    public void saveAutoIntake() {
        autoIntake = motor.getCurrentPosition();
    }

    public void runToFullIn() {
        runTo(fullIn);
    }

    public void runToFullOut() {
        runTo(fullOut);
    }

    public void runToAutoOuttake() {
        runTo(autoOuttake);
    }

    public void runToAutoIntake() {
        runTo(autoIntake);
    }

    public void runToTeleOpIntake() {
        runTo(teleOpIntake);
    }

    public void runManual(double power) {
        if (power > powerMin || power < -powerMin) {
            manualPower = power;
        } else {
            manualPower = 0;
        }
    }

    public void periodic() {
        motor.setInverted(false);
        controller.setPIDF(p, i, d, f);
        double dt = opMode.time - profile_init_time;
        if (!profiler.isOver()) {
            controller.setSetPoint(profiler.motion_profile_pos(dt));
            motor.set(powerUp * controller.calculate(motor.getCurrentPosition()));
        } else {
            if (profiler.isDone()) {
                profiler = new MotionProfiler(3000, 6000);
            }
            if (manualPower != 0) {
                controller.setSetPoint(motor.getCurrentPosition());
                motor.set(manualPower / manualDivide);
            } else {
                motor.set(powerHold * controller.calculate(motor.getCurrentPosition()));
            }
        }
    }

    public void resetEncoder() {
        motor.resetEncoder();
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public double getPercent(){
        return motor.getCurrentPosition() / 580.0;
    }

    public void resetProfiler() {
        profiler = new MotionProfiler(3000, 6000);
    }

}
