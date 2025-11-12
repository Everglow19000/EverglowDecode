package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
public class CloseAutonomous extends LinearOpMode{

    public boolean isBlue = false;
    @Override
    public void runOpMode() throws InterruptedException {
        blackboard.put("isBlue", isBlue);
        int isBlueValue = isBlue ? 1 : -1;
        Pose2d startingPlace = new Pose2d(-50, -48*isBlueValue, Math.toRadians(225*isBlueValue));

        Robot robot = new Robot(hardwareMap);
        MecanumDrive drive = robot.drive;
        drive.localizer.setPose(startingPlace);

        TrajectoryActionBuilder b_MoveToArtifact1 = drive.actionBuilder(startingPlace)
                .setTangent(Math.toRadians(45*isBlueValue))
                .splineToSplineHeading(new Pose2d(-12,-30*isBlueValue, Math.toRadians(-90*isBlueValue)) ,Math.toRadians(-90))
                .splineTo(new Vector2d(-12,-40*isBlueValue) ,Math.toRadians(-90*isBlueValue))
                ;

        TrajectoryActionBuilder b_MoveToShootingPlace = b_MoveToArtifact1 .endTrajectory().fresh()
                .splineTo(new Vector2d(-30,-28*isBlueValue), Math.toRadians(225*isBlueValue))
                ;

        TrajectoryActionBuilder b_MoveToOutOfLine = b_MoveToShootingPlace .endTrajectory().fresh()
                .strafeTo(new Vector2d(0,-28*isBlueValue))
                ;



        Action MoveToArtifact1 = b_MoveToArtifact1.build();
        Action MoveToShootingPlace = b_MoveToShootingPlace.build();
        Action MoveToOutOfLine = b_MoveToOutOfLine.build();

        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        MoveToArtifact1,
                        MoveToShootingPlace,
                        MoveToOutOfLine


                )
        );
    }

}
