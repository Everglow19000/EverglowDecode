package org.firstinspires.ftc.teamcode.autonomous;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;
import org.firstinspires.ftc.teamcode.subsystems.FeedingMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

import java.util.Arrays;

public class CloseAutonomous {
    LinearOpMode opMode;
    public boolean isBlue;

    public CloseAutonomous(LinearOpMode opMode, boolean isBlue) {
        this.opMode = opMode;
        this.isBlue = isBlue;
    }
    public void run() throws InterruptedException {
        blackboard.put("isBlue", isBlue);
        int isBlueValue = isBlue ? 1 : -1;
        Robot robot = new Robot(opMode.hardwareMap, isBlue, true, Motif.NONE);

        while (opMode.opModeInInit() && !opMode.isStopRequested()) {
            robot.camera.start();
            robot.camera.setPipeline(1);
            opMode.telemetry.addData("current camera reading", robot.camera.status());
            opMode.telemetry.update();
        }

        opMode.waitForStart();

        double[] position = new double[3];

        Actions.runBlocking(
                new SequentialAction(
                        robot.getLocalizeWithApriltagAction(position),
                        robot.getScanArtifactColorsAction()
                )
        );

        opMode.telemetry.addData("stored", robot.getFeedingMechanismContents());
        opMode.telemetry.update();

        Pose2d startingPlace = new Pose2d(position[0], position[1], position[2]);

        robot.drive.localizer.setPose(startingPlace);
        MecanumDrive drive = robot.drive;

        TrajectoryActionBuilder b_MoveToScanObelisk = drive.actionBuilder(startingPlace)
                .setTangent(startingPlace.heading)
                .splineToSplineHeading(new Pose2d(-36, -18 * isBlueValue, Math.toRadians(135 * isBlueValue)), (Math.PI/2.0) * isBlueValue);

        TrajectoryActionBuilder b_MoveToArtifact1 = b_MoveToScanObelisk.endTrajectory().fresh()
                .setTangent(Math.toRadians(45 * isBlueValue))
                .splineToSplineHeading(new Pose2d(-12, -30 * isBlueValue, Math.toRadians(-90 * isBlueValue)), Math.toRadians(-90))
                .splineTo(new Vector2d(-12, -40 * isBlueValue), Math.toRadians(-90 * isBlueValue));

        TrajectoryActionBuilder b_MoveToShootingPlace = b_MoveToArtifact1.endTrajectory().fresh()
                .splineTo(new Vector2d(-30, -28 * isBlueValue), Math.toRadians(225 * isBlueValue));

        TrajectoryActionBuilder b_MoveToOutOfLine = b_MoveToScanObelisk.endTrajectory().fresh()
                .strafeTo(new Vector2d(-60, -12 * isBlueValue));


        Action MoveToScanObelisk = b_MoveToScanObelisk.build();
        Action MoveToArtifact1 = b_MoveToArtifact1.build();
        Action MoveToShootingPlace = b_MoveToShootingPlace.build();
        Action MoveToOutOfLine = b_MoveToOutOfLine.build();

        Motif[] motifHolder = new Motif[1];

        Actions.runBlocking(
                new SequentialAction(
                        MoveToScanObelisk,
                        robot.getMotifFromObeliskAction(motifHolder)
                )
        );

        opMode.telemetry.addData("motif", motifHolder[0]);

        robot.setMotif(motifHolder[0]);

        Action currentAction = null;

        boolean actionRunResult = false;
        int actionToProcess = 0;
        int iterationCount = 0;
        while (opMode.opModeIsActive()) {
            robot.update();
            iterationCount++;
            if (!actionRunResult) {
                if (actionToProcess == 0) {
                    currentAction = new SequentialAction(
                            robot.getOrientRobotForShootAction(),

                            new RaceAction(
                                    robot.drive.getHoldHeadingAction(robot),
                                    robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
                                    robot.getLaunchAllArtifactsAction()
                            ),

                            robot.drive.getStopMovingAction(),
                            robot.getStopShooterAction()

//                            robot.getStartIntakeAction(),
//                            MoveToArtifact1,
//                            robot.getStopIntakeAction(),
//
//                            MoveToShootingPlace
                    );
                    actionToProcess++;
                }
//                else if (actionToProcess == 1) {
//                    currentAction = new SequentialAction(
//                            robot.getOrientRobotForShootAction(),
//                            robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
//
//                            new RaceAction(
//                                    robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
//                                    robot.getLaunchAllArtifactsAction()
//                            )
//                    );
//                    actionToProcess++;
//                }
                else if (actionToProcess == 2) {
                    currentAction = new ParallelAction(
                            robot.getStopShooterAction(),
                            MoveToOutOfLine
                    );
                }
            }

            if (opMode.getRuntime() >= 25.0) {
                actionToProcess = 2;
                actionRunResult = false;
            }
            else {
                if (currentAction != null) {
                    actionRunResult = currentAction.run(new TelemetryPacket());
                }
            }

            if (actionToProcess >= 1 && !actionRunResult && currentAction == null) {
                robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }

            opMode.telemetry.update();
        }

        robot.setEndPose(drive.localizer.getPose());
    }
}
