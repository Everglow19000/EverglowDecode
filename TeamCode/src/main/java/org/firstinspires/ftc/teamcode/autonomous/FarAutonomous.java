package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;

public class FarAutonomous extends LinearOpMode {
    public boolean isBlue = false;
    @Override
    public void runOpMode() throws InterruptedException {
        blackboard.put("isBlue", isBlue);
        int isBlueValue = isBlue ? 1 : -1;
        Pose2d startingPlace = new Pose2d(63, -9 * isBlueValue, Math.toRadians(180));

        Robot robot = new Robot(hardwareMap, isBlue);
        MecanumDrive drive = robot.drive;
        drive.localizer.setPose(startingPlace);

        TrajectoryActionBuilder b_MoveToArtifact1 = drive.actionBuilder(startingPlace)
                .splineTo(new Vector2d(36, -40 * isBlueValue), Math.toRadians(-90 * isBlueValue));

        TrajectoryActionBuilder b_MoveToShootingPlace = b_MoveToArtifact1.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(52, -15 * isBlueValue, Math.toRadians(180)), Math.toRadians(-180));

        TrajectoryActionBuilder b_MoveToArtifact2 = b_MoveToShootingPlace.endTrajectory().fresh()
                .splineTo(new Vector2d(12, 40), Math.toRadians(90));

        TrajectoryActionBuilder b_MoveToOutOfLine = b_MoveToArtifact2.endTrajectory().fresh()
                .splineTo(new Vector2d(12, 40), Math.toRadians(90))
                ;


        Action MoveToArtifact1 = b_MoveToArtifact1.build();
        Action MoveToShootingPlace = b_MoveToShootingPlace.build();
        Action MoveToArtifact2 = b_MoveToArtifact2.build();
        Action MoveToOutOfLine = b_MoveToOutOfLine.build();
        waitForStart();
        Action currentAction = new SequentialAction(

                robot.getOrientRobotForShootAction(),
                robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),

                robot.getLaunchSingleArtifactAction(),
                robot.getLaunchSingleArtifactAction(),
                robot.getLaunchSingleArtifactAction(),

                robot.getStopShooterAction(),

                robot.getStartIntakeAction(),
                MoveToArtifact1,
                robot.getStopIntakeAction(),

                MoveToShootingPlace
        );
        while (getRuntime() < 28 && !currentAction.run(new TelemetryPacket())) {
            // do nothing cuz this just runs the action
        }
        currentAction = new SequentialAction(
                robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),

                robot.getLaunchSingleArtifactAction(),
                robot.getLaunchSingleArtifactAction(),
                robot.getLaunchSingleArtifactAction(),

                robot.getStartIntakeAction(),
                MoveToArtifact2,
                robot.getStopIntakeAction(),

                MoveToShootingPlace
        );
        while (getRuntime() < 28 && !currentAction.run(new TelemetryPacket())) {

        }
        currentAction = new SequentialAction(
                robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),

                robot.getLaunchSingleArtifactAction(),
                robot.getLaunchSingleArtifactAction(),
                robot.getLaunchSingleArtifactAction()
        );
        while (getRuntime() < 28 && !currentAction.run(new TelemetryPacket())) {

        }
        currentAction = new ParallelAction(
                robot.getStopShooterAction(),
                MoveToOutOfLine
        );

        while (!currentAction.run(new TelemetryPacket())) {
            // do nothing cuz this just runs the action
        }
    }
}
