package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.FeedingMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

public class AutonomousActions {
    public class UpdateMotifAction implements Action {
        private Motif[] motifs;

        private UpdateMotifAction(Motif[] motifs) {
            this.motifs = motifs;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.setMotif(motifs[0]);
            return false;
        }
    }
    public class OrientRobotForShootAction implements Action {
        private boolean hasStarted = false;
        Action turnAction;
        public OrientRobotForShootAction() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!hasStarted) {
                hasStarted = true;
                turnAction = robot.drive.actionBuilder(robot.drive.localizer.getPose())
                        .turnTo(robot.getOptimalAngleToShoot())
                        .build();
            }
            return turnAction.run(telemetryPacket);
        }
    }
    private Robot robot;
    public AutonomousActions(Robot robot) {
        this.robot = robot;
    }

    public Action getUpdateMotifAction(Motif[] motifs) {
        return new UpdateMotifAction(motifs);
    }
    public Action getOrientRobotForShootAction() {
        return new OrientRobotForShootAction();
    }
    public Action farLaunchAutonomous(Pose2d pose, boolean isBlue) {
        int isBlueValue = isBlue ? 1 : -1;
        return robot.drive.actionBuilder(pose).waitSeconds(8)
                .splineTo(new Vector2d(36,-40*isBlueValue), Math.toRadians(-90*isBlueValue))
                .splineToSplineHeading(new Pose2d(52,-15*isBlueValue, Math.toRadians(180)), Math.toRadians(-180))
                .waitSeconds(8)
                .splineTo(new Vector2d(12,-30*isBlueValue), Math.toRadians(-90*isBlueValue))
                .splineTo(new Vector2d(12,-40*isBlueValue), Math.toRadians(-90*isBlueValue))

                .build();
    }
}
