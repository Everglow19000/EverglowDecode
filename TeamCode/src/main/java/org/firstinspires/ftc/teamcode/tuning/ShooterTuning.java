package org.firstinspires.ftc.teamcode.tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;

@TeleOp(name = "ShooterTuning", group="Tests")
@Config
public class ShooterTuning extends LinearOpMode {
    public static double tickPerSecond = 0;
    public static double servoAngle = 10.0;
    int iterations = 0;

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(this.hardwareMap, true);

        double[] position = new double[3];

        GamepadEx gamepad = new GamepadEx(gamepad1);

        iterations = 0;

        Action shootingAction = null;

        waitForStart();

        Actions.runBlocking(robot.getLocalizeWithApriltagAction(position, false));

        robot.drive.localizer.setPose(new Pose2d(position[0], position[1], position[2]));

        while (opModeIsActive()) {
            iterations++;
            robot.update();
            gamepad.readButtons();
            if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.startIntake();
            }
            if (gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.stopIntake();
            }

            robot.shooter.setHoodServoAngle(servoAngle);
            robot.shooter.setFlywheelMotorSpeed(tickPerSecond);
            if (shootingAction == null) {
                robot.calculateDrivePowers(gamepad);
                if (gamepad.wasJustPressed(GamepadKeys.Button.CROSS)) {
                    shootingAction = new SequentialAction(
                            robot.getOrientRobotForShootAction(),
                            robot.getLaunchAllArtifactsAction(),
                            robot.getStopShooterAction()
                    );
                }
            } else {
                if (gamepad.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
                    shootingAction = null;
                    robot.stopShooterMotor();
                }
                if (shootingAction != null && !shootingAction.run(new TelemetryPacket())) {
                    shootingAction = null;
                }
            }

            telemetry.addData("current speed", robot.shooter.getFlywheelMotorCurrentTicksPerSecond());
            telemetry.addData("desired speed", robot.shooter.desiredFlywheelSpeed);

//            telemetry.addData("feeding mechanism intaking", robot.feedingMechanism.isIntaking());
//            telemetry.addData("contents", robot.getFeedingMechanismContents());
            telemetry.addData("distance", robot.calculateDistanceFromGoal());
            telemetry.addData("position", robot.drive.localizer.getPose().position);
            telemetry.addData("heading", robot.drive.localizer.getPose().heading.toDouble());
            telemetry.addData("contents", robot.getFeedingMechanismContents());
            telemetry.update();
        }
    }
}