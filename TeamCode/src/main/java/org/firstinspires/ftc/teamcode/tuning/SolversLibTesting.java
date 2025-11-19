package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="SolversLibTesting")
@Config
public class SolversLibTesting extends LinearOpMode {
    MotorEx testMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        testMotor = new MotorEx(hardwareMap, "shooterMotor", Motor.GoBILDA.BARE);

        testMotor.setRunMode(Motor.RunMode.VelocityControl);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                testMotor.setVelocity(gamepad1.left_stick_y*36000, AngleUnit.DEGREES);
            }

            telemetry.addData("velocity P", testMotor.getVeloCoefficients()[0]);
            telemetry.addData("velocity I", testMotor.getVeloCoefficients()[1]);
            telemetry.addData("velocity D", testMotor.getVeloCoefficients()[2]);
            telemetry.addData("feedforward s", testMotor.getFeedforwardCoefficients()[0]);
            telemetry.addData("feedforward v", testMotor.getFeedforwardCoefficients()[1]);
            telemetry.addData("feedforward a", testMotor.getFeedforwardCoefficients()[2]);
            telemetry.addData("velocity in RPM", 100*(testMotor.getCorrectedVelocity()/testMotor.getCPR()));
            telemetry.update();
        }
    }
}
