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
import org.firstinspires.ftc.teamcode.subsystems.Motif;

import Ori.Coval.Logging.Logger.KoalaLog;

public class FarAutonomousLeave {
    LinearOpMode opMode;
    public boolean isBlue;

    public FarAutonomousLeave(LinearOpMode opMode, boolean isBlue) {
        this.opMode = opMode;
        this.isBlue = isBlue;
    }
    public void runOpMode() throws InterruptedException {
        blackboard.put("isBlue", isBlue);
        int isBlueValue = isBlue ? 1 : -1;

        Robot robot = new Robot(opMode.hardwareMap, isBlue, true, Motif.NONE);
        opMode.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());


        robot.drive.localizer.setPose(new Pose2d(-66, -20 * isBlueValue, Math.toRadians(180)));

        opMode.waitForStart();

        Motif[] motifWrapper = new Motif[1];

        MecanumDrive drive = robot.drive;


        TrajectoryActionBuilder b_MoveToOutOfLine = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-60, -48 * isBlueValue));


        Action MoveToOutOfLine = b_MoveToOutOfLine.build();

        AutonomousActions actions = new AutonomousActions(robot);

        opMode.waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                MoveToOutOfLine,
                                robot.getMotifFromObeliskAction(motifWrapper)
                        ),
                        actions.getUpdateMotifAction(motifWrapper)
                )
        );
        while (opMode.opModeIsActive()) {
        }

        robot.setEndPose(drive.localizer.getPose());
    }
}
