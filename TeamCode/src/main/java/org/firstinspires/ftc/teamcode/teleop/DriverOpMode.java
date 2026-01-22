package org.firstinspires.ftc.teamcode.teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

import Ori.Coval.Logging.Logger.KoalaLog;

@TeleOp(name="DriverOpMode", group="Driving")
@Config
public class DriverOpMode extends LinearOpMode {
    public static double holdHeadingP = MecanumDrive.holdHeadingP;
    public static double holdHeadingD = MecanumDrive.holdHeadingD;
    Robot robot;
    int iterations;

    public class UpdateRobotPoseAction implements Action {
        double[] pose;
        Robot robot;

        public UpdateRobotPoseAction(Robot robot, double[] pose) {
            this.robot = robot;
            this.pose = pose;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            this.robot.drive.localizer.setPose(new Pose2d(pose[0], pose[1], pose[2]));
            return false;
        }
    }

    Gamepad.RumbleEffect endGenericActionRumble = new Gamepad.RumbleEffect.Builder()
            .addStep(0.75, 0.75, 125)
            .addStep(0.25, 0.25, 125)
            .addStep(0.75, 0.75, 125)
            .addStep(0.25, 0.25, 125)
            .build();
    Gamepad.RumbleEffect endShootActionRumble = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, 500)
            .build();
    Gamepad.RumbleEffect endSpindexerActionRumble = new Gamepad.RumbleEffect.Builder()
            .addStep(0.25, 0.25, 250)
            .addStep(0.5, 0.5, 250)
            .build();
    Gamepad.RumbleEffect endCameraActionRumble = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 0, 250)
            .addStep(0, 1, 250)
            .build();

    Gamepad.RumbleEffect currentRumble;

    @Override
    public void runOpMode() throws InterruptedException {
        Object isBlueObject = blackboard.get("isBlue");
        boolean isBlue = true;
        if (isBlueObject != null) {
            isBlue = (boolean) isBlueObject;
        }
        robot = new Robot(hardwareMap, isBlue, false, Motif.NONE);

        boolean driveAvailable = true;
        boolean spindexerAvailable = true;

        double[] position = new double[3];
        Motif[] motifs = new Motif[1];

        motifs[0] = Motif.NONE;

        GamepadEx gamepad = new GamepadEx(gamepad1);

        GamepadEx loggerGamepad = new GamepadEx(gamepad2);

        Action currentAction = null;

        Action spinUpShooterAction = robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal());

        boolean shouldSpinUpShooter = false;

        KoalaLog.setup(hardwareMap);

        waitForStart();

        if (!robot.usedLastPose) {
            Actions.runBlocking(robot.getLocalizeWithApriltagAction(position, false));
            robot.drive.localizer.setPose(new Pose2d(position[0], position[1], position[2]));
        }


        while (opModeIsActive()) {
            robot.update();
            gamepad.readButtons();
            loggerGamepad.readButtons();
            MecanumDrive.holdHeadingP = holdHeadingP;
            MecanumDrive.holdHeadingD = holdHeadingD;

            if (loggerGamepad.wasJustPressed(GamepadKeys.Button.CROSS)) {
                KoalaLog.logPose2d("Shoot fail - undershoot - pose", robot.drive.localizer.getPose().position.x, robot.drive.localizer.getPose().position.y, robot.drive.localizer.getPose().heading.toDouble(), false);
                KoalaLog.log("Shoot fail - undershoot - speed", robot.shooter.getFlywheelMotorCurrentTicksPerSecond(), false);
                KoalaLog.log("Shoot fail - undershoot - angle", robot.shooter.servoPositionToHoodDegrees(robot.shooter.getHoodServoPosition()), false);
            }
            else if (loggerGamepad.wasJustPressed(GamepadKeys.Button.SQUARE)) {
                KoalaLog.logPose2d("Shoot fail - overshoot - pose", robot.drive.localizer.getPose().position.x, robot.drive.localizer.getPose().position.y, robot.drive.localizer.getPose().heading.toDouble(), false);
                KoalaLog.log("Shoot fail - overshoot - speed", robot.shooter.getFlywheelMotorCurrentTicksPerSecond(), false);
                KoalaLog.log("Shoot fail - overshoot - angle", robot.shooter.servoPositionToHoodDegrees(robot.shooter.getHoodServoPosition()), false);
            }
            if (loggerGamepad.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
                KoalaLog.logPose2d("Shoot fail - incorrect heading - pose", robot.drive.localizer.getPose().position.x, robot.drive.localizer.getPose().position.y, robot.drive.localizer.getPose().heading.toDouble(), false);
                KoalaLog.log("Shoot fail - incorrect heading - speed", robot.shooter.getFlywheelMotorCurrentTicksPerSecond(), false);
                KoalaLog.log("Shoot fail - incorrect heading - angle", robot.shooter.servoPositionToHoodDegrees(robot.shooter.getHoodServoPosition()), false);
            }
            if (loggerGamepad.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
                KoalaLog.log("Shoot succeed", "yay", false);
            }

            if (robot.feedingMechanism.isNowStoppedIntaking()) {
                gamepad.gamepad.runRumbleEffect(endSpindexerActionRumble);
                shouldSpinUpShooter = true;
            }

            if (currentAction == null && gamepad.wasJustPressed(GamepadKeys.Button.CROSS)) {
                driveAvailable = false;
                spindexerAvailable = false;
                currentRumble = endShootActionRumble;
                currentAction = new SequentialAction(
                        new RaceAction(
                                robot.getOrientRobotForShootAction(),
                                robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal())
                        ),
                        new RaceAction(
                                robot.drive.getHoldHeadingAction(robot),
                                robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
                                robot.getLaunchAllArtifactsAction()
                        ),
                        robot.drive.getStopMovingAction(),
                        robot.getStopShooterAction()
                );
            }
            else if (currentAction == null && gamepad.wasJustPressed(GamepadKeys.Button.SQUARE)) {
                driveAvailable = false;
                currentRumble = endCameraActionRumble;
                currentAction = new SequentialAction(
                        robot.getLocalizeWithApriltagAction(position, false),
                        new UpdateRobotPoseAction(robot, position)
                );
            }
            else if (currentAction == null && gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                currentRumble = endCameraActionRumble;
                currentAction = robot.getMotifFromObeliskAction(motifs);
            }
            else if (currentAction == null && gamepad.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
                spindexerAvailable = false;
                currentRumble = endSpindexerActionRumble;
                currentAction = robot.getScanArtifactColorsAction();
            }

            if (currentAction == null && spindexerAvailable && gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.startIntake();
            }
            else if (currentAction == null && !spindexerAvailable || gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.stopIntake();
            }

            if (gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                shouldSpinUpShooter = !shouldSpinUpShooter;
            }

            if (driveAvailable) {
                robot.calculateDrivePowers(gamepad);
            }





            if (gamepad.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
                currentAction = null;
                driveAvailable = true;
                spindexerAvailable = true;
                gamepad.gamepad.runRumbleEffect(endGenericActionRumble);
                currentRumble = null;
                robot.stopShooterMotor();
            }
            if (currentAction != null && !currentAction.run(new TelemetryPacket())) {
                gamepad.gamepad.runRumbleEffect(currentRumble == null ? endGenericActionRumble : currentRumble);
                currentRumble = null;
                robot.setMotif(motifs[0]);
                currentAction = null;
                driveAvailable = true;
                spindexerAvailable = true;
            }

            if (shouldSpinUpShooter) {
                spinUpShooterAction.run(new TelemetryPacket());
            }

            Vector2d diff = robot.goalPoseOrientation.minus(robot.drive.localizer.getPose().position);

            telemetry.addData("desired angle", Math.atan2(diff.y, diff.x));
            telemetry.addData("feeding mechanism intaking", robot.feedingMechanism.isIntaking());
            telemetry.addData("contents", robot.getFeedingMechanismContents());
            telemetry.addData("position", robot.drive.localizer.getPose().position);
            telemetry.addData("heading", robot.drive.localizer.getPose().heading.toDouble());
            telemetry.update();
        }
    }
}
