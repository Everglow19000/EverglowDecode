package org.firstinspires.ftc.teamcode.everglow_library;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.Controller;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RotationSquIDFController extends Controller {
    protected double kP, kI, kD, kF;
    protected AngleUnit angleUnit;
    protected double minIntegral, maxIntegral;
    protected double totalError;

    public RotationSquIDFController(double kp, double ki, double kd, double kf) {
        this(kp, ki, kd, kf, 0, 0, AngleUnit.RADIANS);
    }

    public RotationSquIDFController(double kp, double ki, double kd, double kf, double sp, double pv, AngleUnit angleUnit) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        this.angleUnit = angleUnit;

        setPoint = sp;
        measuredValue = pv;

        minIntegral = -1.0;
        maxIntegral = 1.0;

        // Use the new getAngleError helper
        errorVal_p = getAngleError(setPoint, measuredValue);
    }

    /**
     * Normalizes the error to take the shortest path around a circle.
     */
    private double getAngleError(double target, double current) {
        double error = target - current;
        double wrap = (angleUnit == AngleUnit.RADIANS) ? Math.PI : 180.0;

        while (error > wrap) error -= 2 * wrap;
        while (error <= -wrap) error += 2 * wrap;

        return error;
    }

    @Override
    protected double calculateOutput(double pv) {
        prevErrorVal = errorVal_p;

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        measuredValue = pv;
        // CRITICAL: Calculate error using the shortest path logic
        errorVal_p = getAngleError(setPoint, pv);

        if (Math.abs(period) > 1E-6) {
            // Velocity error is change in error over time
            errorVal_v = (errorVal_p - prevErrorVal) / period;
        } else {
            errorVal_v = 0;
        }

        totalError += period * errorVal_p;
        totalError = Math.max(minIntegral, Math.min(maxIntegral, totalError));

        // Returns u(t)
        // Note: kF * setPoint is usually used for static friction;
        // for rotation, ensure setPoint is normalized if using kF.
        return kP * errorVal_p + kI * totalError + kD * errorVal_v + kF * setPoint;
    }

    @Override
    public void reset() {
        totalError = 0;
        super.reset();
    }

    /**
     * @return the PIDF coefficients
     */
    public double[] getCoefficients() {
        return new double[]{kP, kI, kD, kF};
    }

    public void setPIDF(double kp, double ki, double kd, double kf) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
    }

    public void setCoefficients(PIDFCoefficients coefficients) {
        setPIDF(coefficients.p, coefficients.i, coefficients.d, coefficients.f);
    }

    public void setIntegrationBounds(double integralMin, double integralMax) {
        minIntegral = integralMin;
        maxIntegral = integralMax;
    }

    public void clearTotalError() {
        totalError = 0;
    }

    public void setP(double kp) {
        kP = kp;
    }

    public void setI(double ki) {
        kI = ki;
    }

    public void setD(double kd) {
        kD = kd;
    }

    public void setF(double kf) {
        kF = kf;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getF() {
        return kF;
    }

}