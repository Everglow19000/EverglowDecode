package org.firstinspires.ftc.teamcode.autonomous;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;
import org.firstinspires.ftc.teamcode.subsystems.FeedingMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

import java.util.Arrays;

import Ori.Coval.Logging.Logger.KoalaLog;

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
        opMode.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opMode.opModeInInit() && !opMode.isStopRequested()) {
            robot.camera.start();
            robot.camera.setPipeline(1);
            opMode.telemetry.addData("current camera reading", robot.camera.status());
            opMode.telemetry.update();
        }

        robot.drive.localizer.setPose(new Pose2d(-36, -52 * isBlueValue, Math.toRadians(90 * isBlueValue)));

        opMode.waitForStart();

        KoalaLog.setup(opMode.hardwareMap);

        double[] position = new double[3];

        Actions.runBlocking(
                new ParallelAction(
//                        robot.getLocalizeWithApriltagAction(position, true),
                        robot.getScanArtifactColorsAction()
                )
        );

//        Pose2d startingPlace = new Pose2d(position[0], position[1], position[2]);
        Pose2d startingPlace = robot.drive.localizer.getPose();

        opMode.telemetry.addData("stored", robot.getFeedingMechanismContents());

        opMode.telemetry.addData("position", startingPlace.position);
        opMode.telemetry.addData("heading", startingPlace.heading.toDouble());
        opMode.telemetry.update();

        robot.drive.localizer.setPose(startingPlace);
        MecanumDrive drive = robot.drive;

        Vector2d obeliskScanPosition = new Vector2d(-36, -18 * isBlueValue);

        TrajectoryActionBuilder b_MoveToScanObelisk = drive.actionBuilder(startingPlace)
                .setTangent(startingPlace.heading)
                .splineToSplineHeading(new Pose2d(obeliskScanPosition, Math.toRadians(140 * isBlueValue)), Math.toRadians(180));

        TrajectoryActionBuilder b_TurnToGoal = b_MoveToScanObelisk.endTrajectory().fresh()
                .turnTo(robot.getOptimalAngleToShoot(obeliskScanPosition));

        TrajectoryActionBuilder b_MoveToArtifact1 = b_TurnToGoal.endTrajectory().fresh()
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-10, -30 * isBlueValue, Math.toRadians(-90 * isBlueValue)), Math.toRadians(-90 * isBlueValue))
                .splineTo(new Vector2d(-10, -50 * isBlueValue), Math.toRadians(-90 * isBlueValue), new TranslationalVelConstraint(10));

        TrajectoryActionBuilder b_MoveToShootingPlace = b_MoveToArtifact1.endTrajectory().fresh()
                .setTangent(Math.toRadians(90 * isBlueValue))
                .splineToSplineHeading(new Pose2d(-30, -28 * isBlueValue, Math.toRadians(-135*isBlueValue)), Math.toRadians(180));

        TrajectoryActionBuilder b_MoveToOutOfLine = b_MoveToShootingPlace.endTrajectory().fresh()
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(0, -42 * isBlueValue, Math.toRadians(90 * isBlueValue)), -(Math.PI/2.0) * isBlueValue);


        Action MoveToScanObelisk = b_MoveToScanObelisk.build();
        Action TurnToGoal = b_TurnToGoal.build();
        Action MoveToArtifact1 = b_MoveToArtifact1.build();
        Action MoveToShootingPlace = b_MoveToShootingPlace.build();
        Action MoveToOutOfLine = b_MoveToOutOfLine.build();

        Motif[] motifHolder = new Motif[1];

        AutonomousActions actions = new AutonomousActions(robot);


        Actions.runBlocking(
                new SequentialAction(
                        new RaceAction(
                            new SequentialAction(
                                    new RaceAction(
                                            new SequentialAction(
                                                    new ParallelAction(
                                                            MoveToScanObelisk,
                                                            robot.getMotifFromObeliskAction(motifHolder, 4000)
                                                    ),
                                                    actions.getUpdateMotifAction(motifHolder),
                                                    TurnToGoal
                                            ),
                                            robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal())
                                    ),

                                    new RaceAction(
                                            robot.drive.getHoldHeadingAction(robot),
                                            robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
                                            robot.getLaunchAllArtifactsAction()
                                    ),

                                    robot.getStopShooterAction(),

                                    new ParallelAction(
                                            robot.getIntakeThreeAction(4),
                                            MoveToArtifact1
                                    ),

                                    new RaceAction(
                                            new SequentialAction(
                                                    MoveToShootingPlace,

                                                    actions.getOrientRobotForShootAction()
                                            ),
                                            robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal())
                                    ),


                                    new RaceAction(
                                        robot.drive.getHoldHeadingAction(robot),
                                        robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
                                        robot.getLaunchAllArtifactsAction()
                                    )
                            ),
                            new SleepAction(26.5-opMode.getRuntime())
                        ),
                                new ParallelAction(
                                        MoveToOutOfLine,
                                        robot.getMoveFeedingServoAction(FeedingMechanism.FeedingServoPosition.DOWN),
                                        robot.getStopShooterAction(),
                                        robot.getStopIntakeAction()
                                )
                        )

                );
        while (opMode.opModeIsActive()) {
        }

        robot.setEndPose(drive.localizer.getPose());
    }
}
