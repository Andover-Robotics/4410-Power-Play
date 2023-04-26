package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class Turret {
    private final MotorEx motor;
    private PIDFController controller;

    public static double p = 0.07, i = 0, d = 0.003, f = 0;//TODO experiment with increasing P and D to get more precision
    private double tolerance = 5, powerUp = 0.1, manualDivide = 1.5, manualPower = 0, powerMin = 0.05, veloMin = 200;
    public static double tickToAngle = 3300.0 / 360, fullRotation = 3300.0;

    private boolean isManual = false;

    enum Side {
        RIGHT, LEFT, CENTER;
    }

    private Side side = Side.CENTER;
    public int intake = 1650, turretAutoOuttakeRight = -470, turretAutoIntakeRight = 830, turretAutoOuttakeLeft = 420, turretAutoIntakeLeft = -838, limit = 5400;

    public Turret(OpMode opMode) {
        motor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void runTo(int t) {
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(t);
    }

    public void alignjunction() {
        while (JunctionDetectionPipeline.junctionVal == JunctionDetectionPipeline.JunctionVal.ONLEFT){
            runManual(-0.07);
        }
        while (JunctionDetectionPipeline.junctionVal == JunctionDetectionPipeline.JunctionVal.ONRIGHT){
            runManual(0.07);
        }
    }

    public void runToAutoOuttakeRight(double imu) {
        runTo(turretAutoOuttakeRight + (int) (imu * tickToAngle));
    }

    public void runToAutoIntakeRight(double imu) {
        runTo(turretAutoIntakeRight + (int) (imu * tickToAngle));
    }

    public void runToAutoOuttakeLeft(double imu) {
        runTo(turretAutoOuttakeLeft + (int) (imu * tickToAngle));
    }

    public void runToAutoIntakeLeft(double imu) {
        runTo(turretAutoIntakeLeft + (int) (imu * tickToAngle));
    }

    public void runToTeleOpOuttakeRight(double imu) {
        side = Side.RIGHT;
        runTo(turretAutoOuttakeRight + (int) fullRotation + (int) (imu * tickToAngle));
//        if (side == Side.LEFT) {
//            runTo(turretAutoOuttakeRight + (int) (imu * tickToAngle));
//        } else {
//            side = Side.RIGHT;
//            runTo(turretAutoOuttakeRight + (int) fullRotation + (int) (imu * tickToAngle));
//        }
    }

    public void runToTeleOpOuttakeLeft(double imu) {

        runTo(turretAutoOuttakeLeft + (int) (imu * tickToAngle));
        side = Side.LEFT;
//        if (side == Side.RIGHT) {
//            runTo(turretAutoOuttakeLeft + (int) fullRotation + (int) (imu * tickToAngle));
//        } else {
//            side = Side.LEFT;
//              runTo(turretAutoOuttakeLeft + (int) (imu * tickToAngle));
//        }
    }

    public void runToIntake(double imu) {
        int target = intake + (int) (imu * tickToAngle);
        runTo(target);
        side = Side.CENTER;
    }

    public void runToFront() {
        runTo(0);
    }

    public void saveOuttakePosition(double imu) {
        if (side == Side.RIGHT) {
            turretAutoOuttakeRight = motor.getCurrentPosition() - (int) (imu * tickToAngle) - (int)fullRotation;
        } else if (side == Side.LEFT) {
            turretAutoOuttakeLeft = motor.getCurrentPosition() - (int) (imu * tickToAngle);
        }
    }

    public void saveIntakePosition(double imu) {
        intake = motor.getCurrentPosition() - (int) (imu * tickToAngle);
    }

    public void runManual(double manual) {
        if (manual > powerMin || manual < -powerMin) {
//            isManual = true;
            manualPower = manual;
        } else {
            manualPower = 0;
        }
    }

    public void saveAutoIntake(boolean isRight, double imu) {
        if (isRight) {
            turretAutoIntakeRight = motor.getCurrentPosition() - (int) (imu * tickToAngle);
        } else {
            turretAutoIntakeLeft = motor.getCurrentPosition() - (int) (imu * tickToAngle);
        }
    }

    public void saveAutoOuttake(boolean isRight, double imu) {
        if (isRight) {
            turretAutoOuttakeRight = motor.getCurrentPosition() - (int) (imu * tickToAngle);
        } else {
            turretAutoOuttakeLeft = motor.getCurrentPosition() - (int) (imu * tickToAngle);
        }
    }

    public void runToAngle(double angle, double imu) {
        int target = (int) ((angle + imu) * tickToAngle);
        while (Math.abs(target - motor.getCurrentPosition()) > tickToAngle * 360 * 3 / 5) {
            if (target < motor.getCurrentPosition()) {
                target += tickToAngle * 360;
            } else {
                target -= tickToAngle * 360;
            }
            if (target > limit) {
                target -= tickToAngle * 360;
            } else if (target < -limit) {
                target += tickToAngle * 360;
            }
        }
        runTo(target);
    }


    public void periodic() {
        motor.setInverted(false);
        controller.setPIDF(p, i, d, f);
        if (Math.abs(motor.getCurrentPosition()) > limit) {
            if (motor.getCurrentPosition() > 0) {
                controller.setSetPoint(limit - 100);
            } else {
                controller.setSetPoint(100 - limit);
            }
            motor.set(powerUp * controller.calculate(motor.getCurrentPosition()));
        } else {
            if (manualPower != 0) {
                motor.set(manualPower / manualDivide);
                controller.setSetPoint(motor.getCurrentPosition());
            } else {
                motor.set(powerUp * controller.calculate(motor.getCurrentPosition()));
//                if(!isManual) {
//                    motor.set(powerUp * controller.calculate(motor.getCurrentPosition()));
//                }else{
//                    if(Math.abs(motor.getCorrectedVelocity()) > veloMin){
//                        if(motor.getCorrectedVelocity() > 0) {
//                            motor.set(-0.5);
//                        }else{
//                            motor.set(0.5);
//                        }
//                    }else{
//                        isManual = false;
//                    }
//                }
            }
        }
    }

    public void resetEncoder() {
        motor.resetEncoder();
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

}
