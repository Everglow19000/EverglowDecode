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
import org.firstinspires.ftc.teamcode.everglow_library.DeferredAction;
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

        Pose2d startingPlace = robot.drive.localizer.getPose();

        opMode.telemetry.addData("stored", robot.getFeedingMechanismContents());

        opMode.telemetry.addData("position", startingPlace.position);
        opMode.telemetry.addData("heading", startingPlace.heading.toDouble());
        opMode.telemetry.update();

        robot.drive.localizer.setPose(startingPlace);
        MecanumDrive drive = robot.drive;

        Vector2d obeliskScanPosition = new Vector2d(-24, -18 * isBlueValue);

        TrajectoryActionBuilder b_MoveToScanObelisk = drive.actionBuilder(startingPlace)
                .splineToSplineHeading(new Pose2d(obeliskScanPosition, Math.toRadians(150 * isBlueValue)), Math.toRadians(0));

        TrajectoryActionBuilder b_MoveToArtifact1 = b_MoveToScanObelisk.endTrajectory().fresh()
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-11, -30 * isBlueValue, Math.toRadians(-90 * isBlueValue)), Math.toRadians(-90 * isBlueValue))
                .splineTo(new Vector2d(-11, -50 * isBlueValue), Math.toRadians(-90 * isBlueValue), new TranslationalVelConstraint(12))
                .strafeTo(new Vector2d(-18, -53 * isBlueValue), new TranslationalVelConstraint(20));

        TrajectoryActionBuilder b_MoveToCloseGate = b_MoveToArtifact1
                .splineToSplineHeading(new Pose2d(-5, -52 * isBlueValue, Math.toRadians(90 * isBlueValue)), Math.toRadians(-45 * isBlueValue))
                .strafeTo(new Vector2d(-5, -56 * isBlueValue))
                .waitSeconds(1);

        TrajectoryActionBuilder b_MoveToShootingPlace1 = b_MoveToCloseGate.endTrajectory().fresh()
                .setTangent(Math.PI/2)
                .splineToSplineHeading(new Pose2d(obeliskScanPosition, Math.toRadians(-135 * isBlueValue)), Math.toRadians(135 * isBlueValue));

        TrajectoryActionBuilder b_MoveToArtifact2 = b_MoveToShootingPlace1.endTrajectory().fresh()
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(12, -30 * isBlueValue, Math.toRadians(-90 * isBlueValue)), Math.toRadians(-90 * isBlueValue))
                .splineTo(new Vector2d(12, -50 * isBlueValue), Math.toRadians(-90 * isBlueValue), new TranslationalVelConstraint(7));

        TrajectoryActionBuilder b_MoveToShootingPlace2 = b_MoveToArtifact2
                .splineToSplineHeading(new Pose2d(obeliskScanPosition, Math.toRadians(-135 * isBlueValue)), Math.toRadians(135 * isBlueValue));


        Motif[] motifHolder = new Motif[1];
        AutonomousActions actions = new AutonomousActions(robot);


        Action MoveToScanObelisk = new ParallelAction(
                new SequentialAction(
                        b_MoveToScanObelisk.build(),
                        new DeferredAction(() -> robot.getOrientRobotForShootAction())
                ),
                new SequentialAction(
                        robot.getMotifFromObeliskAction(motifHolder, 3500),
                        actions.getUpdateMotifAction(motifHolder)
                ),
                robot.getScanArtifactColorsAction()
        );


        Action MoveToArtifact1 = new SequentialAction(
                new ParallelAction(
                        b_MoveToArtifact1.build(),
                        robot.getIntakeThreeAction(5.5)
                )
//                b_MoveToCloseGate.build()
        );


        Action MoveToShootingPlace1 = new SequentialAction(
                b_MoveToShootingPlace1.build(),
                new DeferredAction(() -> robot.getOrientRobotForShootAction())
        );

        Action MoveToArtifact2 = new ParallelAction(
                b_MoveToArtifact2.build(),
                robot.getIntakeThreeAction(4.5)
        );

        Action MoveToShootingPlace2 = new SequentialAction(
                b_MoveToShootingPlace2.build(),
                new DeferredAction(() -> robot.getOrientRobotForShootAction())
        );




        Actions.runBlocking(
                new SequentialAction(
                        new RaceAction(
                                robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
                                new SleepAction(26.5),
                                new SequentialAction(
                                        MoveToScanObelisk,

                                        new RaceAction(
                                                robot.getLaunchAllArtifactsAction(),
                                                robot.drive.getHoldHeadingAction(robot)
                                        ),

                                        MoveToArtifact1,
                                        MoveToShootingPlace1,

                                        new RaceAction(
                                                robot.getLaunchAllArtifactsAction(),
                                                robot.drive.getHoldHeadingAction(robot)
                                        ),

                                        MoveToArtifact2,
                                        MoveToShootingPlace2,

                                        new RaceAction(
                                                robot.getLaunchAllArtifactsAction(),
                                                robot.drive.getHoldHeadingAction(robot)
                                        )
                                )
                        ),
                        new ParallelAction(
                                new DeferredAction(() -> drive.actionBuilder(drive.localizer.getPose()).splineTo(new Vector2d(-0, -36 * isBlueValue), Math.toRadians(180)).build()),
                                robot.getStopShooterAction(),
                                robot.getStopIntakeAction(),
                                robot.getMoveFeedingServoAction(FeedingMechanism.FeedingServoPosition.DOWN)
                        )
                )
        );
        while (opMode.opModeIsActive()) {
        }

        robot.setEndPose(drive.localizer.getPose());
    }
}
