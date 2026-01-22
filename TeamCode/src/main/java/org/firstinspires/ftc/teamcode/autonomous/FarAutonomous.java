package org.firstinspires.ftc.teamcode.autonomous;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
                        new ParallelAction(
                                robot.getMotifFromObeliskAction(motifWrapper),
                                robot.getScanArtifactColorsAction()
                        ),
                        robot.getLocalizeWithApriltagAction(location)
                )
        );

        opMode.telemetry.addData("stored Artifacts", robot.getFeedingMechanismContents());

        robot.setMotif(motifWrapper[0]);
        MecanumDrive drive = robot.drive;
        drive.localizer.setPose(startingPlace);

//
        TrajectoryActionBuilder b_MoveToArtifact1 = drive.actionBuilder(startingPlace)
                .splineTo(new Vector2d(36, -40 * isBlueValue), Math.toRadians(-90 * isBlueValue));

        TrajectoryActionBuilder b_MoveToShootingPlace = b_MoveToArtifact1.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(52, -15 * isBlueValue, Math.toRadians(180)), Math.toRadians(-180));

        TrajectoryActionBuilder b_MoveToArtifact2 = b_MoveToShootingPlace.endTrajectory().fresh()
                .splineTo(new Vector2d(12, 40), Math.toRadians(90));

        TrajectoryActionBuilder b_MoveToOutOfLine = b_MoveToArtifact2.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(0, -48 * isBlueValue, Math.toRadians(180)), (Math.PI/2.0) * isBlueValue);


        Action MoveToArtifact1 = b_MoveToArtifact1.build();
        Action MoveToShootingPlace = b_MoveToShootingPlace.build();
        Action MoveToArtifact2 = b_MoveToArtifact2.build();
        Action MoveToOutOfLine = b_MoveToOutOfLine.build();
        opMode.waitForStart();
        Action currentAction = new SequentialAction(
                new RaceAction(
                        new SequentialAction(
                                robot.getOrientRobotForShootAction(),

                                new RaceAction(
                                        robot.drive.getHoldHeadingAction(robot),
                                        robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
                                        robot.getLaunchAllArtifactsAction()
                                ),

                                robot.getStopShooterAction(),

                                robot.getStartIntakeAction(),
                                MoveToArtifact1,
                                robot.getStopIntakeAction(),

                                MoveToShootingPlace,

                                robot.getOrientRobotForShootAction(),

                                new RaceAction(
                                        robot.drive.getHoldHeadingAction(robot),
                                        robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
                                        robot.getLaunchAllArtifactsAction()
                                ),
                                robot.getStopShooterAction(),

                                robot.getStartIntakeAction(),
                                MoveToArtifact2,
                                robot.getStopIntakeAction(),

                                MoveToShootingPlace,

                                robot.getOrientRobotForShootAction(),

                                new RaceAction(
                                        robot.drive.getHoldHeadingAction(robot),
                                        robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
                                        robot.getLaunchAllArtifactsAction())

                        ),
                        new SleepAction(25-opMode.getRuntime())
                ),
                new ParallelAction(
                        MoveToOutOfLine,
                        robot.getStopShooterAction(),
                        robot.getStopIntakeAction()
                )
        );


        while (opMode.opModeIsActive()) {
        }

        robot.setEndPose(drive.localizer.getPose());
    }
}
