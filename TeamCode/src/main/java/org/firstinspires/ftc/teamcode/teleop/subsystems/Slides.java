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

    public final MotorEx motorLeft;
    public final MotorEx motorRight;
    private PIDFController controller;

    enum Position {
        HIGH,
        HIGH_DEC,
        MID,
        MID_DEC,
        LOW,
        GROUND
    }

    private Position position = Position.GROUND;
    public static double p = 0.015, i = 0, d = 0, f = 0, staticF = 0.25;
    private final double tolerance = 20, powerUp = 0.1, powerDown = 0.05, manualDivide = 1, powerMin = 0.1;
    private double manualPower = 0;

    public static int MAXHEIGHT = -1800, top = -1700, topTeleOp = -1650, mid = -1030, low = -200, ground = 0, inc = 100, dec = 300;


    private final OpMode opMode;
    private double target = 0;
    private boolean goingDown = false;
    private double profile_init_time = 0;
    private MotionProfiler profiler = new MotionProfiler(30000, 20000);

    public Slides(OpMode opMode) {
        motorLeft = new MotorEx(opMode.hardwareMap, "slidesLeft", Motor.GoBILDA.RPM_435);
        motorRight = new MotorEx(opMode.hardwareMap, "slidesRight", Motor.GoBILDA.RPM_435);
        motorRight.setInverted(true);
        motorLeft.setInverted(false);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);
        motorLeft.setRunMode(Motor.RunMode.RawPower);
        motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRight.setRunMode(Motor.RunMode.RawPower);
        motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.opMode = opMode;
    }

    public void runTo(int t) {
        motorLeft.setRunMode(Motor.RunMode.RawPower);
        motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRight.setRunMode(Motor.RunMode.RawPower);
        motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        resetProfiler();
        profiler.init_new_profile(motorLeft.getCurrentPosition(), t);
        profile_init_time = opMode.time;
        if (t > target) {
            goingDown = true;
        } else {
            goingDown = false;
        }
        target = t;
    }

    public void runToTop() {
        runTo(top);
    }

    public void runToTopTeleOp() {
        runTo(topTeleOp);
        position = Position.HIGH;
    }

    public void runToTopDec() {
        runTo(topTeleOp + dec);
        position = Position.HIGH_DEC;
    }

    public void runToMiddle() {
        runTo(mid);
        position = Position.MID;
    }

    public void runToMiddleDec() {
        runTo(mid + dec);
        position = Position.MID_DEC;
    }

    public void runToLow() {
        runTo(low);
        position = Position.LOW;
    }

    public void runToBottom() {
        runTo(ground);
        position = Position.GROUND;
    }

    public void runManual(double manual) {
        if (manual > powerMin || manual < -powerMin) {
            manualPower = manual;
        } else {
            manualPower = 0;
        }
    }

    public void periodic() {
        motorRight.setInverted(false);
        motorLeft.setInverted(true);
        controller.setPIDF(p, i, d, f);
        double dt = opMode.time - profile_init_time;
        if (!profiler.isOver()) {
            controller.setSetPoint(profiler.motion_profile_pos(dt));
            double power = powerUp * controller.calculate(motorLeft.getCurrentPosition());
            if (goingDown) {
                power = powerDown * controller.calculate(motorLeft.getCurrentPosition());
            }
            motorLeft.set(power);
            motorRight.set(power);
        } else {
            if (profiler.isDone()) {
                profiler = new MotionProfiler(30000, 20000);
            }
            if (manualPower != 0) {
                controller.setSetPoint(motorLeft.getCurrentPosition());
                motorLeft.set(manualPower / manualDivide);
                motorRight.set(manualPower / manualDivide);
            } else {
                double power = staticF * controller.calculate(motorLeft.getCurrentPosition());
                motorLeft.set(power);
                motorRight.set(power);
//                if(motorLeft.getCurrentPosition() < -20) {
//
//                }else{
//                    motorLeft.set(0);
//                    motorRight.set(0);
//                }
            }
        }
    }

    public double isHigh() {
        return (double) motorLeft.getCurrentPosition() / MAXHEIGHT;
    }

    public double getCurrent() {
        return motorLeft.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + motorRight.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public void resetEncoder() {
        motorLeft.resetEncoder();
    }

    public int getPosition() {
        return motorLeft.getCurrentPosition();
    }

    public void resetProfiler() {
        profiler = new MotionProfiler(30000, 20000);
    }

    public Position getState() {
        return position;
    }
}
