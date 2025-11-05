package org.firstinspires.ftc.teamcode.everglow_library;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public abstract class RobotBase {
    // the drive
    public MecanumDrive drive;

    // all the subsystems
    public Subsystem[] subsystems;

    // calls the update function on all subsystems
    public void updateSubsystems(int iterationCount) {
        for (int i = 0; i < subsystems.length; i++) {
            subsystems[i].update(iterationCount);
        }
    }

    // updates everything, including subsystems. intended for logging and the likes
    public abstract void update(int iterationCount);

    public double squareKeepingSymbol(double num) {
        if (num > 0) {
            return num*num;
        }
        return -num*num;
    }

    // moves the robot according to the gamepad input, and moves it without considering the heading if isAbsolute is true
    public void calculateDrivePowers(Gamepad gamepad, boolean isAbsolute) {
        Vector2d movement = new Vector2d(
                squareKeepingSymbol((-gamepad.left_stick_y)*(1.0/Math.pow(4.5, gamepad.right_trigger))),
                -gamepad.left_stick_x*(1.0/Math.pow(4, gamepad.right_trigger))
        );

        if (isAbsolute) {
            movement = Utils.rotateByAngle(movement, -drive.localizer.getPose().heading.toDouble());
        }

        drive.setDrivePowers(new PoseVelocity2d(
                movement,
                -gamepad.right_stick_x*(1.0/Math.pow(5, gamepad.right_trigger))
        ));
        drive.updatePoseEstimate();
    }
}
