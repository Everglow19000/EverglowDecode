package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@TeleOp(name="SolversLibTesting")
@Config
public class SolversLibTesting extends LinearOpMode {
    Motor testMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        testMotor = new Motor(hardwareMap, "shooterMotor", Motor.GoBILDA.BARE);

        testMotor.setRunMode(Motor.RunMode.VelocityControl);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                testMotor.set(gamepad1.left_stick_y);
            }

            telemetry.addData("left bumper", gamepad1.left_bumper);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.addData("velocity", testMotor.getCorrectedVelocity());
            telemetry.update();
        }
    }
}
