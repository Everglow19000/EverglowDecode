package org.firstinspires.ftc.teamcode.everglow_library;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import java.util.function.Supplier;

/**
 * <p>Instantiates the given action only when this action is run() for the first time.</p>
 *
 * <p>Example: <p>{@code Action lazyMove = new DeferredAction(() -> new MoveMotorAction(motor1, 45));}</p></p>
 */
public class DeferredAction implements Action {
    private final Supplier<Action> actionFactory;
    private Action wrappedAction = null;

    // The constructor takes a Supplier (the "recipe")
    public DeferredAction(Supplier<Action> actionFactory) {
        this.actionFactory = actionFactory;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        // Instantiate only on the first execution
        if (wrappedAction == null) {
            wrappedAction = actionFactory.get();
        }

        // Delegate to the instantiated action
        return wrappedAction.run(telemetryPacket);
    }
}