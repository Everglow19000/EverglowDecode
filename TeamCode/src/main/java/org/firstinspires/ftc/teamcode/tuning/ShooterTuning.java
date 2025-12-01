package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;

@TeleOp(name = "ShooterTuning")
@Config
public class ShooterTuning extends LinearOpMode {
    public static double tickPerSecond = 0;
    public static double servoPosition = 0.5;

    public static double p = 0.025;
    public static double i = 0.2;
    public static double d = 0;
    public static double f = 0.0001;
    int iterations = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Vector2d goalPose = new Vector2d(-62, -60);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Shooter shooter = new Shooter(hardwareMap);
        Camera camera = new Camera(hardwareMap);

        camera.start();


        waitForStart();

        double[] lastPose = {0.0, 0.0, 0.0};
        double[] pose = {0.0, 0.0, 0.0};
        Camera.FindLocationAction getPoseAction = camera.getFindLocationAction(pose, 1);

        Actions.runBlocking(getPoseAction);

        lastPose[0] = pose[0];
        lastPose[1] = pose[1];
        lastPose[2] = pose[2];

        OTOSLocalizer localizer = new OTOSLocalizer(hardwareMap, new Pose2d(pose[0], pose[1], pose[2]));
        localizer.setPose(new Pose2d(pose[0], pose[1], pose[2]));


//        getPoseAction = camera.getFindLocationAction(pose, 100);

        while (opModeIsActive()) {
            iterations++;
            localizer.update();
            lastPose[0] = localizer.getPose().position.x;
            lastPose[1] = localizer.getPose().position.y;
            lastPose[2] = localizer.getPose().heading.log();
//            if (!getPoseAction.run(new TelemetryPacket())) {
//                lastPose[0] = pose[0];
//                lastPose[1] = pose[1];
//                lastPose[2] = pose[2];
//                localizer.setPose(new Pose2d(lastPose[0], lastPose[1], lastPose[2]));
//                getPoseAction = camera.getFindLocationAction(pose, 100);
//            }
            shooter.flywheelPIDF.setPIDF(p, i, d, f);
            shooter.setFlywheelMotorSpeed(tickPerSecond);
            shooter.setHoodServoPosition(servoPosition);

            Vector2d diff = goalPose.minus(new Vector2d(lastPose[0], lastPose[1]));

            shooter.update(iterations);
            telemetry.addData("RR test", pose[2]);
            telemetry.addData("iterations/s", iterations/getRuntime());
            telemetry.addData("intended speed", tickPerSecond);
            telemetry.addData("position", Arrays.toString(lastPose));
            telemetry.addData("distance from goal", Math.sqrt(Math.pow(diff.x, 2) + Math.pow(diff.y, 2)));
            telemetry.addData("recorded speed", shooter.getFlywheelMotorCurrentTicksPerSecond());
            telemetry.addData("motor given power", shooter.getFlywheelPower());
            telemetry.update();
        }
    }
}