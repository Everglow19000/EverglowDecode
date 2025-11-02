package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="ShooterTuning")
public class ShooterTuning extends LinearOpMode {
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                leftMotor.setPower(gamepadEx1.getLeftY());
                rightMotor.setPower(gamepadEx1.getLeftY());
            }
            if (gamepadEx1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                leftMotor.setVelocity(150*(gamepadEx1.getRightY()), AngleUnit.DEGREES);
                rightMotor.setVelocity(150*(gamepadEx1.getRightY()), AngleUnit.DEGREES);
            }

            telemetry.addData("left velocity", leftMotor.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("right velocity", rightMotor.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("left power", leftMotor.getPower());
            telemetry.addData("right power", rightMotor.getPower());
            telemetry.update();
        }
    }
}
