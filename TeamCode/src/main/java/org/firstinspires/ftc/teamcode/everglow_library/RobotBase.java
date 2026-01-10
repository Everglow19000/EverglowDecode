package org.firstinspires.ftc.teamcode.everglow_library;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public abstract class RobotBase {
    // the drive
    public MecanumDrive drive;

    // all the subsystems
    public Subsystem[] subsystems;
    public int iterationCount;
    // calls the update function on all subsystems
    public void updateSubsystems() {
        for (int i = 0; i < subsystems.length; i++) {
            subsystems[i].update(iterationCount);
        }
    }

    // updates everything, including subsystems. intended for logging and the likes
    public abstract void update();

    public double squareKeepingSymbol(double num) {
        if (num > 0) {
            return num*num;
        }
        return -num*num;
    }

    // moves the robot according to the gamepad input
    public void calculateDrivePowers(Gamepad gamepad) {
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        squareKeepingSymbol(-gamepad.left_stick_y)*(1.0/Math.pow(4.5, gamepad.right_trigger)),
                        squareKeepingSymbol(-gamepad.left_stick_x)*(1.0/Math.pow(4, gamepad.right_trigger))
                ),
                squareKeepingSymbol(-gamepad.right_stick_x)*(1.0/Math.pow(5, gamepad.right_trigger))
        ));
        drive.updatePoseEstimate();
    }
    public void calculateDrivePowers(Gamepad gamepad, boolean isFieldCentric) {
        Vector2d translationVector = new Vector2d(
                squareKeepingSymbol(-gamepad.left_stick_y)*(1.0/Math.pow(4.5, gamepad.right_trigger)),
                squareKeepingSymbol(-gamepad.left_stick_x)*(1.0/Math.pow(4, gamepad.right_trigger))
        );

        if (isFieldCentric) {
            translationVector = Utils.rotateByAngle(translationVector, drive.localizer.getPose().heading.inverse().toDouble());
        }

        drive.setDrivePowers(new PoseVelocity2d(
                translationVector,
                squareKeepingSymbol(-gamepad.right_stick_x)*(1.0/Math.pow(5, gamepad.right_trigger))
        ));
        drive.updatePoseEstimate();
    }

    // moves the robot according to the gamepad input
    public void calculateDrivePowers(GamepadEx gamepad) {
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        squareKeepingSymbol(gamepad.getLeftY())*(1.0/Math.pow(4.5, gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))),
                        squareKeepingSymbol(-gamepad.getLeftX())*(1.0/Math.pow(4, gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)))
                ),
                squareKeepingSymbol(-gamepad.getRightX())*(1.0/Math.pow(5, gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)))
        ));
        drive.updatePoseEstimate();
    }

    public void calculateDrivePowers(GamepadEx gamepad, boolean isFieldCentric) {
        Vector2d translationVector = new Vector2d(
                squareKeepingSymbol(gamepad.getLeftY())*(1.0/Math.pow(4.5, gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))),
                squareKeepingSymbol(-gamepad.getLeftX())*(1.0/Math.pow(4, gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)))
        );

        if (isFieldCentric) {
            translationVector = Utils.rotateByAngle(translationVector, drive.localizer.getPose().heading.inverse().toDouble());
        }

        drive.setDrivePowers(new PoseVelocity2d(
                translationVector,
                squareKeepingSymbol(-gamepad.getRightX())*(1.0/Math.pow(5, gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)))
        ));
        drive.updatePoseEstimate();
    }
}
