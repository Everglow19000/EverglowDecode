package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.everglow_library.SwerveDrive;

@Autonomous(name="SwerveTestingAuto")
public class SwerveTestingAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        SwerveDrive drive = new SwerveDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(30, 40), 0)
                        .splineTo(new Vector2d(20, 0), Math.PI)
                        .splineTo(new Vector2d(0, 0), Math.PI)
                        .build());

        while (opModeIsActive()) {

        }
    }
}
