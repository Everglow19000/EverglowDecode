package org.firstinspires.ftc.teamcode.everglow_library;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Supplier;

/**
 * <p>When run for the first time, checks the condition and runs either the trueConditionAction or the falseConditionAction. If no falseConditionAction is provided, or if it is null, only runs the trueConditionAction if the condition is true, otherwise moves on</p>
 *
 * <p>Example 1: <p>{@code Action conditional = new ConditionalAction(trueAction, falseAction, () -> sensor.check());}</p></p>
 * <p></p>
 * <p>Example 2: <p>{@code Action conditional = new ConditionalAction(someAction, () -> sensor.check());}</p></p>
 */
public class ConditionalAction implements Action {
    private final Action trueConditionAction;
    private final Action falseConditionAction;
    private final Supplier<Boolean> condition;
    private Action chosenAction;
    private boolean hasStarted = false;

    public ConditionalAction(Action trueConditionAction, Action falseConditionAction, Supplier<Boolean> condition) {
        this.trueConditionAction = trueConditionAction;
        this.falseConditionAction = falseConditionAction;
        this.condition = condition;
    }
    public ConditionalAction(Action action, Supplier<Boolean> condition) {
        this(action, null, condition);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!hasStarted) {
            hasStarted = true;
            chosenAction = condition.get() ? trueConditionAction : falseConditionAction;
            if (chosenAction == null) {
                return false;
            }
        }

        return chosenAction.run(telemetryPacket);
    }
}
