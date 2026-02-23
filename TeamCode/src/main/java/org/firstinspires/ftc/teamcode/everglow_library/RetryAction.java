package org.firstinspires.ftc.teamcode.everglow_library;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Supplier;


/**
 * <p>Runs the given action, and if the successCondition fails, retries the action from the supplier until reaching max retries or the success condition is true.</p>
 *
 * <p>Example:</p>
 * <pre>{@code
 * Action retryAction = new RetryAction(
 *      someMotorAction, // original wanted action
 *      () -> new MoveMotorAction(motor1, 45), // supplier for the retry action
 *      () -> sensor.check(), // condition to whether the action succeeded
 *      2 // how many retries are allowed
 * );
 * }</pre>
 */
public class RetryAction implements Action {
    private final Supplier<Action> retryActionSupplier;
    private final Supplier<Boolean> successCondition;
    private final int maxRetries;

    private Action currentAction;
    private int retriesCount = 0;

    public RetryAction(Action originalAction, Supplier<Action> retryActionSupplier, Supplier<Boolean> successCondition, int maxRetries) {
        this.retryActionSupplier = retryActionSupplier;
        this.successCondition = successCondition;
        this.maxRetries = maxRetries;

        currentAction = originalAction;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        boolean actionResult = currentAction.run(telemetryPacket);

        if (actionResult) {
            return true;
        }

        if (retriesCount < maxRetries && !successCondition.get()) {
            currentAction = retryActionSupplier.get();
            retriesCount++;
            return true;
        }
        else {
            return false;
        }
    }
}
