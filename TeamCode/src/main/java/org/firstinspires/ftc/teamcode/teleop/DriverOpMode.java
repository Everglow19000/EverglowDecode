package org.firstinspires.ftc.teamcode.teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

    @Override
    public void runOpMode() throws InterruptedException {
        Object isBlueObject = blackboard.get("isBlue");
        boolean isBlue = false;
        if (isBlueObject != null) {
            isBlue = (boolean) isBlueObject;
        }
        Robot robot = new Robot(hardwareMap, isBlue, false, Motif.NONE);

        boolean driveAvailable = true;
        boolean shooterAvailable = true;
        boolean spindexerAvailable = true;

        double[] position = new double[3];
        Motif[] motifs = new Motif[1];

        motifs[0] = Motif.NONE;

        GamepadEx gamepad = new GamepadEx(gamepad1);

        Action currentAction = null;

        waitForStart();

//        if (!robot.usedLastPose) {
//            Actions.runBlocking(robot.getLocalizeWithApriltagAction(position));
//            robot.drive.localizer.setPose(new Pose2d(position[0], position[1], position[2]));
//        }


        while (opModeIsActive()) {
            robot.update();
            gamepad.readButtons();
            MecanumDrive.holdHeadingP = holdHeadingP;
            MecanumDrive.holdHeadingD = holdHeadingD;

            if (robot.feedingMechanism.isNowStoppedIntaking()) {
                gamepad.gamepad.runRumbleEffect(endSpindexerActionRumble);
            }

            if (gamepad.wasJustPressed(GamepadKeys.Button.CROSS)) {
                driveAvailable = false;
                shooterAvailable = false;
                spindexerAvailable = false;
                currentAction = new SequentialAction(
                        robot.getOrientRobotForShootAction(),
                        new RaceAction(
                                robot.drive.getHoldHeadingAction(robot),
                                robot.getSpinUpShooterAction(robot.calculateDistanceFromGoal()),
                                robot.getLaunchAllArtifactsAction()
                        ),
                        robot.drive.getStopMovingAction(),
                        robot.getStopShooterAction()
                );
            }
            else if (gamepad.wasJustPressed(GamepadKeys.Button.SQUARE)) {
                driveAvailable = false;
                currentAction = robot.getMotifFromObeliskAction(motifs);
            }
            else if (gamepad.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
                spindexerAvailable = false;
                currentAction = robot.getScanArtifactColorsAction();
            }

            if (spindexerAvailable && gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.startIntake();
            }
            else if (!spindexerAvailable || gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.stopIntake();
            }

            if (driveAvailable) {
                robot.calculateDrivePowers(gamepad);
            }





            if (gamepad.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
                currentAction = null;
                driveAvailable = true;
                shooterAvailable = true;
                spindexerAvailable = true;
                robot.stopShooterMotor();
            }
            if (currentAction != null && !currentAction.run(new TelemetryPacket())) {
                gamepad.gamepad.runRumbleEffect(endGenericActionRumble);
                robot.setMotif(motifs[0]);
                currentAction = null;
                driveAvailable = true;
                shooterAvailable = true;
                spindexerAvailable = true;
            }

            Vector2d diff = robot.goalPoseOrientation.minus(robot.drive.localizer.getPose().position);

            telemetry.addData("desired angle", Math.atan2(diff.y, diff.x));
            telemetry.addData("feeding mechanism intaking", robot.feedingMechanism.isIntaking());
            telemetry.addData("contents", robot.getFeedingMechanismContents());
            telemetry.addData("position", robot.drive.localizer.getPose().position);
            telemetry.update();
        }
    }
}
