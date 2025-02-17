// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025;

import edu.wpi.first.wpilibj2.command.Command;
import org.supurdueper.robot2025.subsystems.CoralScore;
import org.supurdueper.robot2025.subsystems.Funnel;

public class IntakeCommand extends Command {

    private CoralScore coralScore;
    private Funnel funnel;
    private boolean hasCoral;
    private Command loadCoral;
    private Command runFunnel;
    private Command stopFunnel;
    private Command stopLoad;

    public IntakeCommand(CoralScore coralScore, Funnel funnel) {
        hasCoral = coralScore.hasCoral();
        loadCoral = coralScore.loadCoral();
        runFunnel = funnel.intake();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        coralScore.loadCoral().alongWith(funnel.intake());
    }

    @Override
    public void end(boolean interupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
