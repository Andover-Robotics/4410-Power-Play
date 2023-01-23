package org.firstinspires.ftc.teamcode.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.auto.SampleMecanumDrive;

public class Bot {

    public enum BotState{
        INTAKE_OUT, // linkage fully extended, ready to pick up cone
        INTAKE, // ready to intake, but linkage in
        STORAGE, // arm up, linkage+rail in, used when moving around field
        OUTTAKE, // ready to outtake
        SECURE, // cone secured on junction but not let go
        FRONTOUTTAKE, // ready to outtake from front side
        FRONTSECURE // cone secured on junction from front side but not let go
    }

    public static Bot instance;

    public final Slide slide;
    public final Claw claw;
    public final Linkage linkage;
    public final Arm arm;

    private final MotorEx fl, fr, bl, br;
    public final SampleMecanumDrive rr;
    public BotState state = BotState.STORAGE;

    public OpMode opMode;

    public static Bot getInstance() {
        if (instance == null) {
            throw new IllegalStateException("tried to getInstance of Bot when uninitialized");
        }
        return instance;
    }

    public static Bot getInstance(OpMode opMode){
        if (instance == null) {
            return instance = new Bot(opMode);
        }
        instance.opMode = opMode;
        return instance;
    }

    private Bot(OpMode opMode){
        this.opMode = opMode;
        enableAutoBulkRead();


        fl = new MotorEx(opMode.hardwareMap, "motorFL");
        fr = new MotorEx(opMode.hardwareMap, "motorFR");
        bl = new MotorEx(opMode.hardwareMap, "motorBL");
        br = new MotorEx(opMode.hardwareMap, "motorBR");

        //required subsystems

        this.slide = new Slide(opMode);
        this.claw = new Claw(opMode);
        this.arm = new Arm(opMode);
        this.linkage = new Linkage(opMode);

        this.rr = new SampleMecanumDrive(opMode.hardwareMap);
    }

    public void intakeOut(){
        state = BotState.INTAKE_OUT;
        slide.runToBottom();
        claw.intake();
        arm.intake();
        linkage.fullOut();
    }

    public void intakeIn(){
        state = BotState.INTAKE;
        slide.runToBottom();
        claw.intake();
        arm.intake();
        linkage.intake();
    }
    public void storage(){
        state = BotState.STORAGE;
        slide.runToBottom();
        claw.flipOuttake();
        arm.storage();
        linkage.outtake();
    }

    public void outtake(){ // must be combined with bot.slide.run___() in MainTeleOp
        state = BotState.OUTTAKE;
        claw.close();
        claw.flipOuttake();
        arm.outtake();
        linkage.outtake();
    }

    public void secure(){
        state = BotState.SECURE;
        claw.close();
        claw.flipOuttake();
        arm.secure();
        linkage.outtake();
    }

    public void frontOuttake(){
        state = BotState.FRONTOUTTAKE;
        claw.close();
        claw.flipIntake();
        arm.frontOuttake();
        linkage.intake();
    }

    public void frontSecure(){
        state = BotState.FRONTSECURE;
        claw.close();
        claw.flipIntake();
        arm.frontSecure();
        linkage.intake();
    }



    public void fixMotors(){
        fl.setInverted(false);
        fr.setInverted(true);
        bl.setInverted(false);
        br.setInverted(true);
        
        fl.setRunMode(Motor.RunMode.RawPower);
        fr.setRunMode(Motor.RunMode.RawPower);
        bl.setRunMode(Motor.RunMode.RawPower);
        br.setRunMode(Motor.RunMode.RawPower);
        
        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double strafeSpeed, double forwardBackSpeed, double turnSpeed){
        double[] speeds = {
                forwardBackSpeed-strafeSpeed-turnSpeed,
                forwardBackSpeed+strafeSpeed+turnSpeed,
                forwardBackSpeed+strafeSpeed-turnSpeed,
                forwardBackSpeed-strafeSpeed+turnSpeed
        };
        double maxSpeed = 0;
        for(int i = 0; i < 4; i++){
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }
        if(maxSpeed > 1) {
            for (int i = 0; i < 4; i++){
                speeds[i] /= maxSpeed;
            }
        }
        fl.set(speeds[0]);
        fr.set(speeds[1]);
        bl.set(speeds[2]);
        br.set(speeds[3]);
    }

    private void enableAutoBulkRead() {
        for (LynxModule mod : opMode.hardwareMap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public double getCurrent(){
        return fl.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }
}