package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Robot;

public class AutonomousActions {
    private Robot robot;
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
