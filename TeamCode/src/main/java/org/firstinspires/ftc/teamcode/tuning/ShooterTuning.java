package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "ShooterTuningFormula")
@Config
public class ShooterTuning extends LinearOpMode {
    public static double distanceFromGoal = 3; //meters
    public static double hoodAngleRadians = 0.29;
    public static double tickPerSecond = 2000;
    int iterations = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(hardwareMap);

        GamepadEx gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            iterations++;
            gamepad.readButtons();

            shooter.setFlywheelMotorSpeed(tickPerSecond);

            telemetry.addData("is cross press", gamepad1.cross);
            telemetry.addData("required velocity", Shooter.getArtifactVelocityForDistanceAndAngle(distanceFromGoal, hoodAngleRadians));
            telemetry.addData("flywheel ticks", shooter.getFlywheelTicksPerSecondForArtifactVelocity(Shooter.getArtifactVelocityForDistanceAndAngle(distanceFromGoal, hoodAngleRadians)));
            telemetry.addData("RPM", shooter.getFlywheelMotorCurrentTicksPerSecond());
            telemetry.update();
        }
    }
}