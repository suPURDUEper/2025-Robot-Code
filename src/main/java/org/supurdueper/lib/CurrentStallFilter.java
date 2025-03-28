package org.supurdueper.lib;

import static edu.wpi.first.units.Units.Amps;

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
    private Trigger stallTrigger;

    public CurrentStallFilter(StatusSignal<Current> currentSignal, Current threshold) {
        this.currentSignal = currentSignal;
        this.threshold = threshold;
        this.filteredCurrent = Amps.of(0);
        currentFilter = LinearFilter.movingAverage(7);
        currentDebouncer = new Debouncer(0.2, DebounceType.kRising);
        this.stallTrigger = new Trigger(this::isStalled);
    }

    public void periodic() {
        currentSignal.refresh();
        filteredCurrent = Amps.of(currentFilter.calculate(currentSignal.getValueAsDouble()));
    }

    public boolean isStalled() {
        return currentDebouncer.calculate(filteredCurrent.gt(threshold));
    }

    public Trigger stallTrigger() {
        return stallTrigger;
    }
}
