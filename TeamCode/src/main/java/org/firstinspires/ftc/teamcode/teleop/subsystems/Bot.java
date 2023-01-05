package org.firstinspires.ftc.teamcode.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Bot {

    public static Bot instance;

    public final Intake intake;
    public final Slide slide;
    public final Claw claw;
    public Turret turret;

    private MotorEx fl, fr, bl, br;
    public final MecanumDrive drive;

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
        this.drive = new MecanumDrive(fl, fr, bl, br);

        this.intake = new Intake(opMode);
        this.slide = new Slide(opMode);
        this.claw = new Claw(opMode);
    }

    public void fixMotors(){
        drive.setRightSideInverted(true);

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
        double speeds[] = {
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
}
// non needed
//        try {
////      this.hubs = Pair.create(opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1"), // TODO: check if revextensions2 works with sdk7.0 and control hubs
////          opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"));
//        } catch (Exception e) {
//            // Avoid catastrophic errors if RevExtensions don't behave as expected. Limited trust of stability
//            e.printStackTrace();
//        }