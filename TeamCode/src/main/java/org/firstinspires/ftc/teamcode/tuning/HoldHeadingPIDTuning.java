package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name="HoldHeadingPIDTuning", group="Tests")
@Config
public class HoldHeadingPIDTuning extends LinearOpMode {
    public static double P = 0.75;
    public static double I = 0.005;
    public static double D = 0.04;
    public static double apply = 0;
    private Rotation2d targetRotation = Rotation2d.exp((3*Math.PI)/4);
    private PIDController controller = new PIDController(P, I, D);
    double lastSwitchTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            if (apply != 0) {
                apply = 0;
                controller.setPID(P, I, D);
            }
            if (getRuntime() - lastSwitchTime > 5) {
                lastSwitchTime = 0;
                targetRotation = Rotation2d.exp(targetRotation.toDouble() + Math.PI);
                controller.setSetPoint(targetRotation.toDouble());
            }

            double angVel = controller.calculate(drive.localizer.getPose().heading.toDouble());

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0, 0),
                            angVel
                    )
            );
            drive.updatePoseEstimate();

            telemetry.addData("curr head", drive.localizer.getPose().heading.toDouble());
            telemetry.addData("target head", targetRotation.toDouble());
            telemetry.addData("angVel", angVel);
            telemetry.update();
        }
    }
}
