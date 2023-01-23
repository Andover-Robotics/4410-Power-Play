package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Claw;

public class AllPositionTester extends LinearOpMode {

    private Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot = Bot.getInstance(this);
        GamepadEx gp2 = new GamepadEx(gamepad2);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
            gp2.readButtons();

            if (gp2.getButton(GamepadKeys.Button.A)) {
                bot.claw.open();
            } else {
                bot.claw.close();
            }

            if (gp2.getButton(GamepadKeys.Button.B)) {
                bot.claw.flipIntake();
            } else {
                bot.claw.flipOuttake();
            }

            if (gp2.getButton(GamepadKeys.Button.Y)) {
                bot.linkage.fullOut();
            } else {
                bot.linkage.outtake();
            }

            if (gp2.getButton(GamepadKeys.Button.X)){
                bot.arm.storage();
            }else if(gp2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                bot.arm.outtake();
            }else if(gp2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)){
                bot.arm.secure();
            }else if(gp2.getButton(GamepadKeys.Button.START)){
                bot.arm.frontSecure();
            }else if(gp2.getButton(GamepadKeys.Button.BACK)){
                bot.arm.frontOuttake();
            }else{
                bot.arm.intake();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.slide.runToTop();
            }else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.slide.runToMiddle();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.slide.runToLow();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.slide.runToBottom();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                bot.slide.goDown();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.slide.goUp();
            }


            bot.slide.periodic();
        }
    }
}
