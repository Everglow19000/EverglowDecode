package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class FarAutonomous_red extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startingPlace = new Pose2d(63, 9, Math.toRadians(180));
        boolean isBlue = false; //this will be defined by the camera
        int isBlueValue = isBlue ? 1 : -1;

        MecanumDrive drive = new MecanumDrive(hardwareMap,startingPlace);


        TrajectoryActionBuilder b_MoveToArtifact1 = drive.actionBuilder(startingPlace)
                .splineTo(new Vector2d(36,-40*isBlueValue), Math.toRadians(-90*isBlueValue))
                ;

        TrajectoryActionBuilder b_MoveToShootingPlace = b_MoveToArtifact1 .endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(52,-15*isBlueValue, Math.toRadians(180)), Math.toRadians(-180))

                ;

        TrajectoryActionBuilder b_MoveToArtifact2 = b_MoveToShootingPlace .endTrajectory().fresh()
                .splineTo(new Vector2d(12,40), Math.toRadians(90))
                ;

        Action MoveToArtifact1 = b_MoveToArtifact1.build();
        Action MoveToShootingPlace = b_MoveToShootingPlace.build();
        Action MoveToArtifact2 = b_MoveToArtifact2.build();
        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        MoveToArtifact1,
                        MoveToShootingPlace,
                        MoveToArtifact2
                )
        );

    }
}
