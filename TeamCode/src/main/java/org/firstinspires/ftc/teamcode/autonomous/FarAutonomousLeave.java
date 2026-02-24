package org.firstinspires.ftc.teamcode.autonomous;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import android.graphics.LinearGradient;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.everglow_library.DeferredAction;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

import Ori.Coval.Logging.Logger.KoalaLog;

public class FarAutonomousLeave {
    LinearOpMode opMode;
    public boolean isBlue;

    public FarAutonomousLeave(LinearOpMode opMode, boolean isBlue) {
        this.opMode = opMode;
        this.isBlue = isBlue;
    }
    public void run() throws InterruptedException {
        blackboard.put("isBlue", isBlue);
        int isBlueValue = isBlue ? 1 : -1;

        Robot robot = new Robot(opMode.hardwareMap, isBlue, true, Motif.NONE);
        opMode.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());


        robot.drive.localizer.setPose(new Pose2d(66, -20 * isBlueValue, Math.toRadians(180)));

        opMode.waitForStart();

        Motif[] motifWrapper = new Motif[1];

        MecanumDrive drive = robot.drive;

//        TrajectoryActionBuilder b_MoveForwardABit =
//                .strafeTo(new Vector2d(63, -21 * isBlueValue));

//        TrajectoryActionBuilder b_MoveToOutOfLine = ;


//        Action MoveForwardABit = b_MoveForwardABit.build();

//        Action MoveToOutOfLine = b_MoveToOutOfLine.build();

        AutonomousActions actions = new AutonomousActions(robot);

        opMode.waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        new RaceAction(
                                robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
                                new SequentialAction(
                                        new ParallelAction(
                                                new SequentialAction(
                                                        robot.getMotifFromObeliskAction(motifWrapper),
                                                        actions.getUpdateMotifAction(motifWrapper),
//                                                        MoveForwardABit,
                                                        new DeferredAction(() -> robot.getOrientRobotForShootAction())
                                                ),
                                                robot.getScanArtifactColorsAction()
                                        ),
                                        new RaceAction(
                                                robot.getLaunchAllArtifactsAction(),
                                                robot.drive.getHoldHeadingAction(robot)
                                        ),
                                        drive.getStopMovingAction()
                                )
                        ),
                        robot.getStopShooterAction(),
                        new DeferredAction(() -> drive.actionBuilder(drive.localizer.getPose()).strafeTo(new Vector2d(63, -32 * isBlueValue)).build()),
                        drive.getStopMovingAction()
                )
        );
        while (opMode.opModeIsActive()) {
        }

        robot.calculateDrivePowers(opMode.gamepad1);

        robot.setEndPose(drive.localizer.getPose());
    }
}
