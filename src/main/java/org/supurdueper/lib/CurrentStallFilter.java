package org.supurdueper.lib;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CurrentStallFilter {

    private StatusSignal<Current> currentSignal;
    private LinearFilter currentFilter;
    private Current filteredCurrent;
    private Current threshold;
    private Trigger stallTrigger;

    public CurrentStallFilter(StatusSignal<Current> currentSignal, Current threshold) {
        this.currentSignal = currentSignal;
        this.threshold = threshold;
        this.filteredCurrent = Amps.of(0);
        currentFilter = LinearFilter.movingAverage(7);
        this.stallTrigger = new Trigger(this::isStalled);
    }

    public void periodic() {
        currentSignal.refresh();
        filteredCurrent = Amps.of(currentFilter.calculate(currentSignal.getValueAsDouble()));
    }

    public boolean isStalled() {
        return filteredCurrent.gt(threshold);
    }

    public Trigger stallTrigger() {
        return stallTrigger;
    }
}
