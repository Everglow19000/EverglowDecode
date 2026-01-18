package org.firstinspires.ftc.teamcode.autonomous;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

public class FarAutonomous {
    LinearOpMode opMode;
    public boolean isBlue;

    public FarAutonomous(LinearOpMode opMode, boolean isBlue) {
        this.opMode = opMode;
        this.isBlue = isBlue;
    }
    public void run() throws InterruptedException {
        blackboard.put("isBlue", isBlue);
        int isBlueValue = isBlue ? 1 : -1;

        Robot robot = new Robot(opMode.hardwareMap, isBlue, true, Motif.NONE);


        while (opMode.opModeInInit() && !opMode.isStopRequested()) {
            robot.camera.start();
            robot.camera.setPipeline(2);
            opMode.telemetry.addData("current camera reading", robot.camera.status());
            opMode.telemetry.update();
        }


        opMode.waitForStart();

        Pose2d startingPlace;

        startingPlace = new Pose2d(61.1, -20.7 * isBlueValue, Math.toRadians(180));

        double[] location = new double[3];
        Motif[] motifWrapper = new Motif[1];

        Actions.runBlocking(
                new SequentialAction(
                        robot.getMotifFromObeliskAction(motifWrapper),
                        robot.getScanArtifactColorsAction()
                )
        );

        opMode.telemetry.addData("stored Artifacts", robot.getFeedingMechanismContents());

        robot.setMotif(motifWrapper[0]);
        MecanumDrive drive = robot.drive;
        drive.localizer.setPose(startingPlace);

        Actions.runBlocking(
                new SequentialAction(
                            robot.getOrientRobotForShootAction(),

                            new RaceAction(
                                    robot.drive.getHoldHeadingAction(robot),
                                    robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
                                    robot.getLaunchAllArtifactsAction()
                            ),

                            robot.drive.getStopMovingAction(),
                            robot.getStopShooterAction(),
                            drive.actionBuilder(drive.localizer.getPose()).setTangent(drive.localizer.getPose().heading).splineTo(new Vector2d(36,-24 * isBlueValue), Math.PI).build()
                )
        );

//        if (location[0] == location[1] && location[1] == location[2] && location[0] == 0.0) {
//        }
//        else {
//            startingPlace = new Pose2d(location[0], location[1], location[2]);
//        }



//        Action currentAction = null;
//
//        boolean actionRunResult = false;
//        int actionToProcess = 0;
//        int iterationCount = 0;
        while (opMode.opModeIsActive()) {
//            robot.update();
//            iterationCount++;
//            if (!actionRunResult) {
//                if (actionToProcess == 0) {
//                    currentAction = new SequentialAction(
//                            robot.getOrientRobotForShootAction(),
//
//                            new RaceAction(
//                                    robot.drive.getHoldHeadingAction(robot),
//                                    robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
//                                    robot.getLaunchAllArtifactsAction()
//                            ),
//
//                            robot.drive.getStopMovingAction(),
//                            robot.getStopShooterAction()
//
////                            robot.getStartIntakeAction(),
////                            MoveToArtifact1,
////                            robot.getStopIntakeAction(),
////
////                            MoveToShootingPlace
//                    );
//                    actionToProcess++;
//                }
////                else if (actionToProcess == 1) {
////                    currentAction = new SequentialAction(
////                            robot.getOrientRobotForShootAction(),
////                            robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
////
////                            new RaceAction(
////                                    robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
////                                    robot.getLaunchAllArtifactsAction()
////                            )
////                    );
////                    actionToProcess++;
////                }
//                else {
//                    currentAction = new ParallelAction(
//                            robot.getStopShooterAction(),
//                            drive.actionBuilder(drive.localizer.getPose()).setTangent(drive.localizer.getPose().heading).splineTo(new Vector2d(36,-24 * isBlueValue), Math.PI).build()
//                    );
//                }
//            }
//
//            if (opMode.getRuntime() >= 25.0) {
//                actionToProcess = 2;
//                actionRunResult = false;
//            }
//            else {
//                if (currentAction != null) {
//                    actionRunResult = currentAction.run(new TelemetryPacket());
//                }
//            }
//
//
//            opMode.telemetry.update();
        }

        robot.setEndPose(drive.localizer.getPose());

//
//        TrajectoryActionBuilder b_MoveToArtifact1 = drive.actionBuilder(startingPlace)
//                .splineTo(new Vector2d(36, -40 * isBlueValue), Math.toRadians(-90 * isBlueValue));
//
//        TrajectoryActionBuilder b_MoveToShootingPlace = b_MoveToArtifact1.endTrajectory().fresh()
//                .splineToSplineHeading(new Pose2d(52, -15 * isBlueValue, Math.toRadians(180)), Math.toRadians(-180));
//
//        TrajectoryActionBuilder b_MoveToArtifact2 = b_MoveToShootingPlace.endTrajectory().fresh()
//                .splineTo(new Vector2d(12, 40), Math.toRadians(90));
//
//
//
//        Action MoveToArtifact1 = b_MoveToArtifact1.build();
//        Action MoveToShootingPlace = b_MoveToShootingPlace.build();
//        Action MoveToArtifact2 = b_MoveToArtifact2.build();
//        Action MoveToOutOfLine = b_MoveToOutOfLine.build();
//        waitForStart();
//        Action currentAction = new SequentialAction(
//
//                robot.getOrientRobotForShootAction(),
//                robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
//
//                robot.getLaunchAllArtifactsAction(),
//                robot.getLaunchAllArtifactsAction(),
//                robot.getLaunchAllArtifactsAction(),
//
//                robot.getStopShooterAction(),
//
//                robot.getStartIntakeAction(),
//                MoveToArtifact1,
//                robot.getStopIntakeAction(),
//
//                MoveToShootingPlace
//        );
//        while (getRuntime() < 28 && !currentAction.run(new TelemetryPacket())) {
//            // do nothing cuz this just runs the action
//        }
//        currentAction = new SequentialAction(
//                robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
//
//                robot.getLaunchAllArtifactsAction(),
//                robot.getLaunchAllArtifactsAction(),
//                robot.getLaunchAllArtifactsAction(),
//
//                robot.getStartIntakeAction(),
//                MoveToArtifact2,
//                robot.getStopIntakeAction(),
//
//                MoveToShootingPlace
//        );
//        while (getRuntime() < 28 && !currentAction.run(new TelemetryPacket())) {
//
//        }
//        currentAction = new SequentialAction(
//                robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
//
//                robot.getLaunchAllArtifactsAction(),
//                robot.getLaunchAllArtifactsAction(),
//                robot.getLaunchAllArtifactsAction()
//        );
//        while (getRuntime() < 28 && !currentAction.run(new TelemetryPacket())) {
//
//        }
//        currentAction = new ParallelAction(
//                robot.getStopShooterAction(),
//                MoveToOutOfLine
//        );
//
//        while (!currentAction.run(new TelemetryPacket())) {
//            // do nothing cuz this just runs the action
//        }
    }
}
