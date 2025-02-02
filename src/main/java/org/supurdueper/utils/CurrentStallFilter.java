package org.supurdueper.utils;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CurrentStallFilter {

    private StatusSignal<Current> currentSignal;
    private LinearFilter currentFilter;
    private Debouncer currentDebouncer;
    private Current filteredCurrent;
    private Current threshold;

    public CurrentStallFilter(StatusSignal<Current> currentSignal, Current threshold) {
        this.currentSignal = currentSignal;
        this.threshold = threshold;
        currentFilter = LinearFilter.movingAverage(5);
        currentDebouncer = new Debouncer(0.33, DebounceType.kRising);
    }

    public void periodic() {
        currentSignal.refresh();
        currentFilter.calculate(currentSignal.getValueAsDouble());
    }

    public boolean isStalled() {
        return currentDebouncer.calculate(filteredCurrent.gt(threshold));
    }

    public Trigger stallTrigger() {
        return new Trigger(this::isStalled);
    }
}
