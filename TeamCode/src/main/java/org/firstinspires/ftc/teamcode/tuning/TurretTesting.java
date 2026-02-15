package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.subsystems.Turret.ANGLE_OFFSET_TO_ROBOT;
import static org.firstinspires.ftc.teamcode.subsystems.Turret.MAX_ANGLE_RANGE;
import static org.firstinspires.ftc.teamcode.subsystems.Turret.MAX_TICK_RANGE;
import static org.firstinspires.ftc.teamcode.subsystems.Turret.RADIANS_TO_TICKS_CONSTANT;
import static org.firstinspires.ftc.teamcode.subsystems.Turret.rotation2dToTwoPIRange;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name="Turret Testing", group="Tests")
@Config
public class TurretTesting extends LinearOpMode {
    class DummyLocalizer implements Localizer {
        Pose2d pose2d = new Pose2d(0, 0, (4*Math.PI)/4);

        @Override
        public void setPose(Pose2d pose) {
            pose2d = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose2d;
        }

        @Override
        public PoseVelocity2d update() {
            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }
    }
    Camera camera;
    Turret turret;
//    public static double wantedAngle = 0;
//    DcMotorEx turretMotor;
//    public static int motorTargetPosition = 0;
//    public static double motorPower = 0;
//    public static boolean motorReversed = false;

    public Rotation2d getOptimalAngleToShoot(Vector2d pose) {
        return Utils.getOptimalAngleToShoot(Robot.goalPoseDistanceStatic, pose);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DummyLocalizer dummy = new DummyLocalizer();
        camera = new Camera(hardwareMap, dummy);
        camera.start();
        camera.setPipeline(1);
        Turret turret = new Turret(hardwareMap);

        double homeAngle = rotation2dToTwoPIRange(ANGLE_OFFSET_TO_ROBOT);
        double maxAllowableAngle = rotation2dToTwoPIRange(MAX_ANGLE_RANGE);

        double currentAngleConstant =
                (1 - ((2*Math.PI - rotation2dToTwoPIRange(MAX_ANGLE_RANGE))
                        /
                        (2*Math.PI)))
                                /
                                Turret.RADIANS_TO_TICKS_CONSTANT;

//        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
//        turretMotor.setTargetPosition(motorTargetPosition);
//        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        Actions.runBlocking(turret.getHomeTurretAction());
        while (opModeIsActive()) {
            camera.update(0);
            Rotation2d result = camera.getCameraHeadingOffsetFromAprilTag(true);
            if (result != null) {
                telemetry.addData("heading error", Math.toDegrees(result.toDouble()));
                telemetry.addData("local camera heading", Math.toDegrees(Turret.globalRotationToLocal(result, dummy.getPose().heading).toDouble()));
                telemetry.addData("global camera heading", Math.toDegrees(dummy.getPose().heading.toDouble()));
                telemetry.addData("desired turret angle from pos", Math.toDegrees(Turret.globalRotationToLocal(getOptimalAngleToShoot(dummy.getPose().position), dummy.getPose().heading).toDouble()));
                int tempTarget = turret.turretMotor.getCurrentPosition() - (int)(result.toDouble() * RADIANS_TO_TICKS_CONSTANT);
                turret.turretMotor.setTargetPosition(tempTarget < 0 ? Turret.MAX_TICK_RANGE + tempTarget : tempTarget%MAX_TICK_RANGE);
            }
            else {
                telemetry.addLine("No Apriltag found!");
            }
//            telemetry.addData("home angle", Math.toDegrees(homeAngle));
//            telemetry.addData("max angle", Math.toDegrees(maxAllowableAngle));
//            telemetry.addData("corrected angle", Math.toDegrees(rotation2dToTwoPIRange(turret.validateRotation(Rotation2d.exp(Math.toRadians(wantedAngle))))));
//            telemetry.addData("offset angle", Math.toDegrees(rotation2dToTwoPIRange(Rotation2d.exp(rotation2dToTwoPIRange(Rotation2d.exp(Math.toRadians(wantedAngle))) - homeAngle))));
            telemetry.addData("wanted ticks", turret.turretMotor.getTargetPosition());
            telemetry.addData("wanted angle", Math.toDegrees(rotation2dToTwoPIRange(ANGLE_OFFSET_TO_ROBOT) + (turret.turretMotor.getTargetPosition() * currentAngleConstant)));
            telemetry.addData("current Ticks", turret.turretMotor.getCurrentPosition());
            telemetry.addData("current angle", Math.toDegrees(rotation2dToTwoPIRange(ANGLE_OFFSET_TO_ROBOT) + turret.turretMotor.getCurrentPosition() * currentAngleConstant));
            telemetry.update();
        }
    }
}
