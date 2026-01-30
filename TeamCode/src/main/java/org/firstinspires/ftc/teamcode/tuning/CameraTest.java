package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

import java.util.Arrays;

@TeleOp(name="CameraTest", group="Tests")
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, true);
        robot.drive.localizer.setPose(new Pose2d(0, 0, (-3*Math.PI)/4.0));

        double[] location = new double[3];

        waitForStart();

//        Actions.runBlocking(robot.getLocalizeWithApriltagAction(location));

//        robot.drive.localizer.setPose(new Pose2d(location[0], location[1], location[2]));

        while (opModeIsActive()) {
            robot.calculateDrivePowers(gamepad1);
            robot.update();

            telemetry.addData("pos", robot.camera.limelight3A.getLatestResult().getBotpose_MT2().toString());

            telemetry.addData("distance", robot.calculateDistanceFromGoal());
            telemetry.addData("location", robot.drive.localizer.getPose().position);
            telemetry.addData("heading", robot.drive.localizer.getPose().heading.toDouble());
            telemetry.update();
        }
    }
}