package org.firstinspires.ftc.teamcode.opmodes.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.RoadRunnerTrials.drive.SampleMecanumDrive;

import java.util.ArrayList;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RoadRunnerPathTrial extends LinearOpMode {

    public static Vector2d returnParkingSpot(int parkingZone){
        ArrayList<Vector2d> parkingZones = new ArrayList<>();
        parkingZones.add(new Vector2d(35.25, -58.375));
        parkingZones.add(new Vector2d(35.25, -35.25));
        parkingZones.add(new Vector2d(35.25, -11.75));
        return parkingZones.get(parkingZone-1);
    }

    public static Vector2d getCoordinates(int x, int y){
        double finalX, finalY = 0;
        if (x != 3 && x != -3) {
            finalX = x*23.5;
        } else{
            finalX = x*23.5 - ((x/3) * 0.75);
        }
        if (y != 3 && y != -3) {
            finalY = y*23.5;
        } else {
            finalY = y*23.5 - ((y/3) * 0.75);
        }

        return new Vector2d(finalX, finalY);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(58.375, -35.25, Math.toRadians(270));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Vector2d parkingSpot = returnParkingSpot(1);
        Vector2d D2 = getCoordinates(1, -1);
        Vector2d D1 = getCoordinates(1, -2);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(35.25, -35.25), Math.toRadians(270)) // Parking sensor 3
                .splineTo(D2, Math.toRadians(315)) // D2 medium tower
                .addDisplacementMarker(() -> {
                    // Put cone on the pole
                    System.out.println("Reached D2 medium tower");
                })
                .splineTo(new Vector2d(11.75, -69.5), Math.toRadians(194.48))
                .addDisplacementMarker(() -> {
                    // Pick up cone
                    System.out.println("Reached Cone pick up location");
                })// Cones red #2
                .splineTo(D1, Math.toRadians(27.32)) // D1 Low tower
                .addDisplacementMarker(() -> {
                    // Put cone on pole
                    System.out.println("Reached D1 Low tower");
                })
                .splineTo(parkingSpot, Math.toRadians(90)) // Park
                .build();
        waitForStart();
        drive.followTrajectory(traj);
    }
}
