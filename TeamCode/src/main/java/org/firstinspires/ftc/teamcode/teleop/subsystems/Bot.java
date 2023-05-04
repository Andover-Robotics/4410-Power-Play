package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.JunctionDetectionTest;

public class Bot {

    public enum BotState {
        INTAKE_OUT, // linkage fully extended, ready to pick up cone
        INTAKE, // ready to intake, but linkage in
        STORAGE, // arm up, linkage+rail in, used when moving around field
        OUTTAKE, // ready to outtake
        SECURE, // cone secured on junction but not let go
        BRACE_OUTTAKE,
        BRACE_SECURE
    }

    public static Bot instance;

    public final Slides slides;
    public final Claw claw;
    public final HorizSlides horizSlides;
    public final Arm arm;
    public final Turret turret;

    private final MotorEx fl, fr, bl, br;
    //    public final SampleMecanumDrive rr;
    public BotState state = BotState.STORAGE;

    public OpMode opMode;

    public BNO055IMU imu0;
    public boolean fieldCentricRunMode = false;

    private double imuOffset = 0;

    public static Bot getInstance() {
        if (instance == null) {
            throw new IllegalStateException("tried to getInstance of Bot when uninitialized");
        }
        return instance;
    }

    public static Bot getInstance(OpMode opMode) {
        if (instance == null) {
            return instance = new Bot(opMode);
        }
        instance.opMode = opMode;
        return instance;
    }

    private Bot(OpMode opMode) {
        this.opMode = opMode;
        enableAutoBulkRead();

        try {
            this.initializeImus();
            fieldCentricRunMode = false;
        } catch (Exception e) {
            imu0 = null;
            fieldCentricRunMode = false;

        }


        fl = new MotorEx(opMode.hardwareMap, "motorFL");
        fr = new MotorEx(opMode.hardwareMap, "motorFR");
        bl = new MotorEx(opMode.hardwareMap, "motorBL");
        br = new MotorEx(opMode.hardwareMap, "motorBR");

        //required subsystems

        this.slides = new Slides(opMode);
        this.claw = new Claw(opMode);
        this.arm = new Arm(opMode);
        this.horizSlides = new HorizSlides(opMode);
        this.turret = new Turret(opMode);

//        this.rr = new SampleMecanumDrive(opMode.hardwareMap);


    }




    public void slidesalignjunction() {
        while (horizSlides.getCurrent() > horizSlides.currentthres){
            horizSlides.runManual(0.35);
        }
    }
    public void turretalignjunction() {
        if (JunctionDetectionPipeline.junctionVal == JunctionDetectionPipeline.JunctionVal.ONLEFT){
            turret.runRawPower(-0.3);
        }
        if (JunctionDetectionPipeline.junctionVal == JunctionDetectionPipeline.JunctionVal.ONRIGHT){
            turret.runRawPower(0.3);
        }
        if (JunctionDetectionPipeline.junctionVal == JunctionDetectionPipeline.JunctionVal.NOTDETECTED || JunctionDetectionPipeline.junctionVal == JunctionDetectionPipeline.JunctionVal.ATJUNCTION) {
            turret.runRawPower(0);
        }
    }


    public void intakeFallen() {
        state = BotState.INTAKE;
        slides.runToBottom();
        arm.fallenintake();
        horizSlides.runToFullIn();
        claw.open();
    }

    public void intakeOut() {
        state = BotState.INTAKE_OUT;
        slides.runToBottom();
        arm.intake();
        horizSlides.runToTeleOpIntake();
        claw.open();
    }

    public void sideStackIntake(int i) {
        state = BotState.INTAKE_OUT;
        slides.runToBottom();
        arm.intakeAuto(i);
        horizSlides.runToAutoIntake();
        claw.open();
    }

    public void intakeIn() {
        state = BotState.INTAKE;
        slides.runToBottom();
        arm.intake();
        horizSlides.runToFullIn();
        claw.open();
    }

    public void storage() {
        state = BotState.STORAGE;
        slides.runToBottom();
        arm.storage();
        horizSlides.runToFullIn();
    }

    public void outtake() { // must be combined with bot.slide.run___() in MainTeleOp
        state = BotState.OUTTAKE;
        claw.close();
        arm.outtake();
    }

    public void secure() {
        state = BotState.SECURE;
        claw.close();
        arm.secure();
    }

    public void braceOuttake() {
        state = BotState.BRACE_OUTTAKE;
        claw.close();
        arm.brace();
    }

    public void bringSlidesDown() {
        if (slides.getState() == Slides.Position.HIGH) {
            slides.runToTopDec();
        } else if (slides.getState() == Slides.Position.MID) {
            slides.runToMiddleDec();
        }
        state = BotState.BRACE_SECURE;
    }

    public void bringSlidesUp() {
        if (slides.getState() == Slides.Position.HIGH_DEC) {
            slides.runToTopTeleOp();
        } else if (slides.getState() == Slides.Position.MID_DEC) {
            slides.runToMiddle();
        }
        state = BotState.BRACE_OUTTAKE;
    }

    public void initializeImus() {
        imu0 = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu0.initialize(parameters);
        resetIMU();
    }


    public void fixMotors() {
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

    public void driveRobotCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
        double[] speeds = {
                forwardBackSpeed - strafeSpeed - turnSpeed,
                forwardBackSpeed + strafeSpeed + turnSpeed,
                forwardBackSpeed + strafeSpeed - turnSpeed,
                forwardBackSpeed - strafeSpeed + turnSpeed
        };
        double maxSpeed = 0;
        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }
        if (maxSpeed > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }
        fl.set(speeds[0]);
        fr.set(speeds[1]);
        bl.set(speeds[2]);
        br.set(speeds[3]);
    }

    public void driveFieldCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed, double heading) {
        double magnitude = Math.sqrt(strafeSpeed * strafeSpeed + forwardBackSpeed * forwardBackSpeed);
        double theta = (Math.atan2(forwardBackSpeed, strafeSpeed) - heading) % (2 * Math.PI);
        double[] speeds = {
                magnitude * Math.sin(theta + Math.PI / 4) + turnSpeed,
                magnitude * Math.sin(theta - Math.PI / 4) - turnSpeed,
                magnitude * Math.sin(theta - Math.PI / 4) + turnSpeed,
                magnitude * Math.sin(theta + Math.PI / 4) - turnSpeed
        };

        double maxSpeed = 0;

        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }

        if (maxSpeed > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }

        //        for (int i = 0; i < 4; i++) {
        //            driveTrainMotors[i].set(speeds[i]);
        //        }
        // manually invert the left side

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

    public double getCurrent() {
        return fl.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public void resetEncoder() {
        fl.resetEncoder();
        fr.resetEncoder();
        bl.resetEncoder();
        br.resetEncoder();
        horizSlides.resetEncoder();
        slides.resetEncoder();
        turret.resetEncoder();
    }

    public void setImuOffset(double offset) {
        imuOffset += offset;
    }

    public void resetIMU() {
        imuOffset += getIMU();
    }

    public double getIMU() {
        double angle = (imu0.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle - imuOffset) % 360;
        if (angle > 180) {
            angle = angle - 360;
        }
        return angle;
    }

    public void resetProfiler() {
        slides.resetProfiler();
        horizSlides.resetProfiler();
    }
}