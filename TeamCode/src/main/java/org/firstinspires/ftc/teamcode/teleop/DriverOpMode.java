package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="DriverOpMode", group="Driving")
public class DriverOpMode extends LinearOpMode {
    Robot robot;
    int iterations;

    @Override
    public void runOpMode() throws InterruptedException {
        Object isBlueObject = blackboard.get("isBlue");
        boolean isBlue = true;
        if (isBlueObject != null) {
            isBlue = (boolean) isBlueObject;
        }
        robot = new Robot(this.hardwareMap, isBlue);

        double[] position = new double[3];

        GamepadEx gamepad = new GamepadEx(gamepad1);

        iterations = 0;

        Action shootingAction = null;

        waitForStart();

        Actions.runBlocking(robot.getLocalizeWithApriltagAction(position));

        robot.drive.localizer.setPose(new Pose2d(position[0], position[1], position[2]));

        while (opModeIsActive()) {
            iterations++;
            gamepad.readButtons();
            if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.startIntake();
            }
            if (gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.stopIntake();
            }

            if (shootingAction == null) {
                robot.calculateDrivePowers(gamepad);
                if (gamepad.wasJustPressed(GamepadKeys.Button.CROSS)) {
                    shootingAction = new SequentialAction(
                            robot.getOrientRobotForShootAction(),
                            new RaceAction(
                                    robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
                                    robot.getLaunchAllArtifactsAction()
                            )
                    );
                }
            }
            else {
                if (gamepad.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
                    shootingAction = null;
                }
                if (shootingAction != null && !shootingAction.run(new TelemetryPacket())) {
                    shootingAction = null;
                }
            }

            robot.update(iterations);

            telemetry.addData("pos", robot.drive.localizer.getPose().position.toString());
            telemetry.addData("head", robot.drive.localizer.getPose().heading.toDouble());
            telemetry.update();
        }
    }
}
