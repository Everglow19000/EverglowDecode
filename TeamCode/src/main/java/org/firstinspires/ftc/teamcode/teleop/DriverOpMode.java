package org.firstinspires.ftc.teamcode.teleop;

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
        robot = new Robot(this.hardwareMap);
        GamepadEx gamepad = new GamepadEx(gamepad1);

        iterations = 0;

        waitForStart();

        while (opModeIsActive()) {
            iterations++;
            gamepad.readButtons();
            if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.startIntake();
            }
            if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.stopIntake();
            }
            if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                robot.prepareToLaunch();
            }
            if (gamepad.wasJustPressed(GamepadKeys.Button.B)) {
                robot.launchSingle();
            }
            robot.calculateDrivePowers(gamepad);
            robot.update(iterations);
        }
    }
}
