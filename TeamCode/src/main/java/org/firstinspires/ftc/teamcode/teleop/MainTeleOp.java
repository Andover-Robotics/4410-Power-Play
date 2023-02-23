package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

import java.util.Map;


@TeleOp(name = "MainTeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1, cycleTime = 1;

    private boolean debugMode = false;
    private boolean cancelPrevAction = false;
    private boolean autoMode = false;
    private double outtakeFinalHzSlides;//TODO find presets
    private double outtakeFinalVSlides;
    private double outtakeFinalTurret;

    private int index = 4;



    private GamepadEx gp1, gp2;


    @Override
    public void runOpMode() throws InterruptedException {
        //TODO EXPERIMENTAL CODE =================
        Bot.instance = null;
        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
            entry.getValue().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            while (!isStopRequested() && Math.abs(entry.getValue().getCurrentPosition()) > 1) {
//                idle();
//            }
        }
        //End experimental code ===================

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot = Bot.getInstance(this);

        Thread periodic = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                bot.slides.periodic();
                bot.turret.periodic();
                bot.horizSlides.periodic();
                drive();
            }
        });
        Thread checkForCycleMode = new Thread(() -> {
            if(gp2.getButton(GamepadKeys.Button.BACK)){
                autoMode = false;
            }
        });

        gp2 = new GamepadEx(gamepad2);
        gp1 = new GamepadEx(gamepad1);

//        bot.initializeImus(); TODO see if this is required

        bot.state = Bot.BotState.STORAGE;
        bot.arm.storage();
        bot.claw.open();

        waitForStart();
        periodic.start();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("cycle", time - cycleTime);
            cycleTime = time;

            gp1.readButtons();
            gp2.readButtons();

            if(gp2.wasJustPressed(GamepadKeys.Button.START)){
                debugMode = !debugMode;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.START)){
                bot.fieldCentricRunMode = !bot.fieldCentricRunMode;
            }

            if(!debugMode) {//finite state
                if (bot.state == Bot.BotState.INTAKE || bot.state == Bot.BotState.INTAKE_OUT) {
                    if(gp2.wasJustPressed(GamepadKeys.Button.A)){
                        bot.claw.close();
                    }
                    if(gp2.wasJustPressed(GamepadKeys.Button.X)){
                        bot.claw.open();
                        cancelPrevAction = true;
                    }
                    if(gp2.wasJustReleased(GamepadKeys.Button.A)){
                        if(!cancelPrevAction){
                            bot.storage();
                        }
                        cancelPrevAction = false;
                    }
                } else if (bot.state == Bot.BotState.STORAGE) {
                    if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                        bot.intakeIn();
                        cancelPrevAction = true;
                    } else if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                        bot.intakeOut();
                    } else if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                        bot.outtake();
                    }
                    if(gp2.wasJustPressed(GamepadKeys.Button.B)){
                        bot.state = Bot.BotState.OUTTAKE;
                        autoMode = true;
                        goToOuttake();
                    }
                } else if (bot.state == Bot.BotState.OUTTAKE || bot.state == Bot.BotState.SECURE) {
                    if(gp2.wasJustPressed(GamepadKeys.Button.X) && bot.state == Bot.BotState.OUTTAKE){
                        bot.storage();
                    }
                    if(gp2.wasJustPressed(GamepadKeys.Button.X) && bot.state == Bot.BotState.SECURE){
                        bot.outtake();
                        cancelPrevAction = true;
                    }
                    if(gp2.getButton(GamepadKeys.Button.Y)){
                        if(!cancelPrevAction) {
                            bot.secure();
                        }
                    }
                    if(gp2.wasJustReleased(GamepadKeys.Button.Y)){
                        if(!cancelPrevAction){
                            bot.claw.open();
                            bot.storage();
                            if(autoMode) {
                                bot.turret.runToFront();//TODO switch with values
                            }
                            autoMode = false;
                        }
                        cancelPrevAction = false;
                    }
                }
//                Vector2d stickVector = new Vector2d(gp2.getRightX(), gp2.getRightY()); // TODO get this working
//                double angle = stickVector.angle() * 180/Math.PI;
//                if(stickVector.norm() > 0.5) {
//                    bot.turret.runToAngle(angle, bot.getIMU());
//                }
                bot.turret.runManual(gp2.getLeftX());
                bot.horizSlides.runManual(-gp2.getRightY());
//                bot.slides.runManual(-gp2.getLeftY());
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    bot.slides.runToTop();
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    bot.slides.runToMiddle();
                }else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    bot.slides.runToLow();
                }else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    bot.slides.runToBottom();
                }
            }else{//debug mode
                bot.horizSlides.runManual(-gp2.getRightY());
                bot.slides.runManual(-gp2.getLeftY());
                bot.turret.runManual(gp2.getLeftX());

                if(gp2.wasJustPressed(GamepadKeys.Button.START)){
                    bot.resetEncoder();
                }

                if (gp2.getButton(GamepadKeys.Button.A)) {
                    bot.claw.open();
                } else {
                    bot.claw.close();
                }


                if (gp2.getButton(GamepadKeys.Button.X)) {
                    bot.arm.intakeAuto(index);
                } else if (gp2.getButton(GamepadKeys.Button.Y)) {
                    bot.arm.outtake();
                } else if (gp2.getButton(GamepadKeys.Button.B)) {
                    bot.arm.secure();
                } else {
                    bot.arm.storage();
                }

                if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    index++;
                    if (index > 5) {
                        index = 5;
                    }
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    index--;
                    if (index < 0) {
                        index = 0;
                    }
                }
            }

            telemetry.addData("drive current", bot.getCurrent());
            telemetry.addData("slide current", bot.slides.getCurrent());
            telemetry.addData("getIMU", bot.getIMU());
            telemetry.addData("debug", debugMode);
            telemetry.addData("fieldCentric", bot.fieldCentricRunMode);
            telemetry.addData("state", bot.state.toString());
            telemetry.update();
        }
    }

    private void storeValues() {
        outtakeFinalHzSlides = bot.horizSlides.getPosition();
        outtakeFinalVSlides = bot.slides.getPosition();
        outtakeFinalTurret = bot.turret.getPosition();
    }


    //copied from auto
    public static int driveTime = 2200, timeSlidesUp = 850, timeSlidesDown = 550, timeOuttake = 350, timeConeDrop = 150, timeIntakeDown = 200, timeIntakeOut = 700, timeIntakeClose = 150, timeIntakeUp = 500, timeIntakeIn = 400;//old 400


    private void goToOuttake() {//TODO change values to use the stored values

        bot.turret.runToAutoOuttakeRight(bot.getIMU());//this line
        bot.slides.runToTop();//this line
        sleep(timeSlidesUp);
        bot.arm.autoOuttake();
        sleep(timeOuttake);
        bot.arm.autoSecure();
    }

    private void drive() {
        if (gp1.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            bot.resetIMU();
        }
        driveSpeed = 1;
//        driveSpeed -= bot.slides.isHigh()/3;
        driveSpeed *= 1 - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);
        bot.fixMotors();
        Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                turnVector = new Vector2d(
                        gp1.getRightX() , 0);
        if (bot.fieldCentricRunMode) {
            bot.driveFieldCentric(
                    driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed/1.7,
                    (bot.getIMU() + 360) % 360
            );
        } else {
            bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed/1.7
            );
        }
    }

}
