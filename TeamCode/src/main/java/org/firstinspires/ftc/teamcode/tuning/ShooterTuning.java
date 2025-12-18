package org.firstinspires.ftc.teamcode.tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;

@TeleOp(name = "ShooterTuning")
@Config
public class ShooterTuning extends LinearOpMode {
    public static double tickPerSecond = 0;
    public static double servoAngle = 10.0;
    public static boolean isTuning = true;
    public static boolean feedingServoUp = false;

    public static double p = 0.025;
    public static double i = 0.2;
    public static double d = 0;
    public static double f = 0.0001;
    int iterations = 0;
    double distance;

    MecanumDrive drive;

    public Action getOrientRobotForShootAction(Pose2d pose) {
        return drive.actionBuilder(pose)
                .turnTo(Utils.getOptimalAngleToShoot(Robot.goalEdge1, Robot.goalEdge2, pose.position))
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Shooter shooter = new Shooter(hardwareMap);
        Camera camera = new Camera(hardwareMap);
        Servo feedingServo = hardwareMap.get(Servo.class, "feedingServo");
        feedingServo.setDirection(Servo.Direction.REVERSE);

        camera.start();


        waitForStart();

        double[] pose = {0.0, 0.0, 0.0};
        Camera.FindLocationAction getPoseAction = camera.getFindLocationAction(pose, 250);

        Actions.runBlocking(getPoseAction);

        Pose2d startPose = new Pose2d(pose[0], pose[1], pose[2]);

        drive = new MecanumDrive(hardwareMap, startPose);
        drive.localizer.setPose(new Pose2d(pose[0], pose[1], pose[2]));

        Action orientRobotAction = null;

        int flag = 0;

        boolean lastIterationLeftBumper = false;
        
        while (opModeIsActive()) {
            iterations++;
            servoAngle = Math.min(Shooter.maxServoAngle, Math.max(Shooter.minServoAngle, servoAngle));
            if (gamepad1.left_bumper && !lastIterationLeftBumper) {
                Actions.runBlocking(getOrientRobotForShootAction(drive.localizer.getPose()));
                flag++;
            }
            lastIterationLeftBumper = gamepad1.left_bumper;

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y*(1.0/Math.pow(4.5, gamepad1.right_trigger)),
                            -gamepad1.left_stick_x*(1.0/Math.pow(4, gamepad1.right_trigger))
                    ),
                    -gamepad1.right_stick_x*(1.0/Math.pow(5, gamepad1.right_trigger))
            ));
            drive.updatePoseEstimate();
            pose[0] = drive.localizer.getPose().position.x;
            pose[1] = drive.localizer.getPose().position.y;
            pose[2] = drive.localizer.getPose().heading.log();
//            if (!getPoseAction.run(new TelemetryPacket())) {
//                lastPose[0] = pose[0];
//                lastPose[1] = pose[1];
//                lastPose[2] = pose[2];
//                localizer.setPose(new Pose2d(lastPose[0], lastPose[1], lastPose[2]));
//                getPoseAction = camera.getFindLocationAction(pose, 100);
//            }

            Vector2d diff = Robot.goalPoseDistance.minus(new Vector2d(pose[0], pose[1]));
            distance = Math.sqrt(Math.pow(diff.x, 2) + Math.pow(diff.y, 2));
            if (isTuning) {
                shooter.setFlywheelMotorSpeed(tickPerSecond);
                shooter.setHoodServoAngle(servoAngle);
                shooter.flywheelPIDF.setPIDF(p, i, d, f);
                if (feedingServoUp) {
                    feedingServo.setPosition(0.7);
                }
                else {
                    feedingServo.setPosition(0.5);
                }
            }
            else if (gamepad1.circle) {
                shooter.setFlywheelMotorSpeed(shooter.getFlywheelTicksPerSecondForDistanceFromGoal(distance));
                shooter.setHoodServoAngle(shooter.getServoAngleForDistanceFromGoal(distance));
                if (gamepad1.right_bumper) {
                    feedingServo.setPosition(0.7);
                }
                else {
                    feedingServo.setPosition(0.5);
                }
            }
            else {
                shooter.stopMotor();
            }


            shooter.update(iterations);
            telemetry.addData("iterations/s", iterations/getRuntime());
            telemetry.addData("intended speed", tickPerSecond);
            telemetry.addData("position", Arrays.toString(pose));
            telemetry.addData("distance from goal", distance);
            telemetry.addData("recorded speed", shooter.getFlywheelMotorCurrentTicksPerSecond());
            telemetry.addData("motor given power", shooter.getFlywheelPower());
            telemetry.update();
        }
    }
}