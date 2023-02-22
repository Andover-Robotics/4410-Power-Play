package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

import java.util.Timer;
import java.util.TimerTask;


//TODO:
// Slides(Vertical)
// Slides(Horizontal)
// Toggle Claw
// Intake Position = Ground/Storage values
// Turret
// Outtake Position


// Later Job:
// Debug
// Auto mode

@TeleOp(name = "MainTeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {
    private Bot bot;
    private double driveSpeed = 1, fieldCentricOffset = 0;

    private boolean debugMode = false;
    private boolean autoMode = false;
    private boolean clicked = false;
    private boolean cancelPrevAction = false;
    private boolean lastStep = false; // true - Storage
                                    // false - Intake

    private GamepadEx gp1, gp2;


    private double outtakeFinalHzSlides;
    private double outtakeFinalVSlidesLeft;
    private double outtakeFinalVSlidesRight;
    private double outtakeFinalTurret;

    int index = 4;

    public void storeValues() {
        outtakeFinalHzSlides = bot.horizSlides.motor.getCurrentPosition();
        outtakeFinalVSlidesLeft = bot.slides.motorLeft.getCurrentPosition();
        outtakeFinalVSlidesRight = bot.slides.motorRight.getCurrentPosition();
        outtakeFinalTurret = bot.turret.getPosition();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot = Bot.getInstance(this);
        gp2 = new GamepadEx(gamepad2);
        gp1 = new GamepadEx(gamepad1);

        bot.initializeImus();
        storeValues();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons(); // read buttons pressed
            gp2.readButtons();

            if(gp2.wasJustPressed(GamepadKeys.Button.START)){
                debugMode = !debugMode;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                bot.fieldCentricRunMode = !bot.fieldCentricRunMode;
            }
            
            
            if (!debugMode) { // Driver 2 mode
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { // Slides DPAD
                    bot.slides.runToTop();
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    bot.slides.runToMiddle();
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    bot.slides.runToLow();
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    bot.slides.runToBottom();
                }

                if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    if (lastStep) {
                        bot.slides.runToBottom();
                        bot.arm.intake();
                        bot.claw.open();
                    } else {
                        bot.arm.storage();
                    }
                } else if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                    bot.arm.secure();
                    if (bot.claw.isOpen) {
                        bot.claw.close();
                    } else {
                        bot.claw.open();
                    }
                } else if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    bot.arm.outtake();
                } else if (gp2.wasJustPressed(GamepadKeys.Button.B)) {
                    autoMode = !autoMode;
                } else if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                    storeValues();
                }

                Vector2d hzSlideVector = new Vector2d(gp2.getLeftX(), gp2.getLeftY());
                bot.horizSlides.runManual(hzSlideVector.getY());

                Vector2d turretVector = new Vector2d(gp2.getRightX(), gp2.getRightY());
                bot.turret.runManual(turretVector.getX());





            } else { // debug mode

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

                if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    index++;
                    if (index > 5) {
                        index = 5;
                    }
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    index--;
                    if (index < 0) {
                        index = 0;
                    }
                }

                bot.arm.updateIntakeAuto();

//            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
////                bot.slides.runToTop();
//            }else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
////                bot.slides.runToMiddle();
//            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
////                bot.slides.runToLow();
//            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
////                bot.slides.runToBottom();
//            }
//
//            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//                bot.slides.goDown();
//            }
//
//            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
//
//            }

//
//            if(gp2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)){
//                bot.turret.runToFront();
//            }

                bot.horizSlides.runManual(-gp2.getRightY());
                bot.slides.runManual(-gp2.getLeftY());
                bot.turret.runManual(gp2.getLeftX());
                if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                    bot.resetEncoder();
                }

            }

            while (autoMode) {
                bot.claw.open();
                bot.arm.intake();

                //checkpoint # 1
                Timer timer = new Timer();
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        if (gp2.wasJustPressed(GamepadKeys.Button.BACK)) {
                            clicked = true;
                            timer.cancel();
                        }
                    }
                }, 0, 1000);

                try {
                    Thread.sleep(2000); // Wait for 2 seconds
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                timer.cancel();

                if (clicked) {
                    bot.claw.close();
                    bot.arm.storage();
                    bot.turret.runTo((int) -outtakeFinalTurret);
                    bot.slides.runTo((int) outtakeFinalVSlidesLeft);
                    bot.arm.secure();

                    //checkpoint
                    clicked = false;
                    Timer timer2 = new Timer();
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            if (gp2.wasJustPressed(GamepadKeys.Button.BACK)) {
                                clicked = true;
                                timer2.cancel();
                            }
                        }
                    }, 0, 1000);

                    try {
                        Thread.sleep(2000); // Wait for 2 seconds
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    timer2.cancel();

                    if (clicked) {
                        bot.claw.open();
                        bot.arm.storage();
                        bot.slides.runToBottom();
                        bot.turret.runTo((int) outtakeFinalTurret);
                    }
                }



                if (gp2.wasJustPressed(GamepadKeys.Button.B)) {
                    autoMode = !autoMode;
                }
            }
        }
    }
}
