package org.firstinspires.ftc.teamcode.opmodes.auto;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test", group="Practice")
public class PurePursuitTrial extends LinearOpMode{

    public static final double WHEEL_DIAMETER= 4.0;
    //change diameter
    public static final double TICKS_PER_REV=8192;
    //change to reflects accuracy
    public static final double DISTANCE_PER_PULSE= Math.PI* WHEEL_DIAMETER/TICKS_PER_REV;

    //lateral distance between the left and right odometers- determining angle for turning approximations
    public static final double TRACKWIDTH=14.7;

    //distance between the center of rotation of the robot and the center odometer
    //A negative offset means the odometer is closer to the back
    //a positive offset means its closer to the front
    public static final double CENTER_WHEEL_OFFSET= -2.1;

    @Override
    public void runOpMode() throws InterruptedException {
        Waypoint start= new StartWaypoint(0,0);
        Waypoint middle= new GeneralWaypoint(2,2);
        Waypoint middle2= new GeneralWaypoint(3,2);
        Waypoint end= new EndWaypoint(3,2,0,0,0,0,0,0);
        Path path = new Path(start, middle, middle2, end);
        MotorEx frontLeft= new MotorEx(hardwareMap, "motorFL");
        MotorEx backLeft= new MotorEx(hardwareMap, "motorBL");
        MotorEx backRight= new MotorEx(hardwareMap, "motorBR");
        MotorEx frontRight= new MotorEx(hardwareMap, "motorFR");
        MecanumDrive mecanumDrive = new MecanumDrive(frontLeft, frontRight,
                backLeft, backRight);
        Motor.Encoder leftOdometer= frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        //front left motor
        Motor.Encoder frontOdometer= frontRight.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        //front right motor
        Motor.Encoder rightOdometer= backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        //back left motor
        HolonomicOdometry odometry= new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer:: getDistance,
                frontOdometer:: getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET);

        waitForStart();

    /*    while (!path.isFinished()) {
            if(isStopRequested()) return;
            if (path.timedOut())
                throw new InterruptedException("Timed out");

            // return the motor speeds
            double speeds[] = path.loop(odometry.getPose().getX(), odometry.getPose().getY(),
                    odometry.getPose().getHeading());

            mecanumDrive.driveWithMotorPowers(speeds[0], speeds[1], speeds[2], speeds[3]);
            odometry.updatePose();
        }
     */
        mecanumDrive.stop();

    }
}
