package org.firstinspires.ftc.teamcode;
/*
 * NextFTC: a user-friendly control library for FIRST Tech Challenge
 * Copyright (C) 2025 Rowan McAlpin
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

/**
 * This behaves like the command-form of a switch statement. You provide it with a value to
 * reference, and a list of options of outcomes. It is blocking, meaning `isDone` will not return
 * `true` until the scheduled command(s) have completed running.
 * * @param <T> the type of the value to reference
 */
public class SwitchCommand<T> extends Command {

    private final Supplier<T> valueSupplier;
    private final Map<T, Command> outcomes;
    private final Command defaultCommand;

    private Command selectedCommand;

    /**
     * Creates a SwitchCommand with all parameters specified.
     * * @param valueSupplier the supplier for the value to reference
     * @param outcomes a map of outcome values to commands
     * @param defaultCommand the command to schedule if none of the outcomes are fulfilled
     */
    public SwitchCommand(Supplier<T> valueSupplier, Map<T, Command> outcomes, Command defaultCommand) {
        this.valueSupplier = valueSupplier;
        this.outcomes = outcomes;
        this.defaultCommand = defaultCommand;
    }

    /**
     * Creates a SwitchCommand with a default NullCommand.
     * * @param valueSupplier the supplier for the value to reference
     * @param outcomes a map of outcome values to commands
     */
    public SwitchCommand(Supplier<T> valueSupplier, Map<T, Command> outcomes) {
        this(valueSupplier, outcomes, new NullCommand());
    }

    /**
     * Creates a SwitchCommand with empty outcomes and a default NullCommand.
     * * @param valueSupplier the supplier for the value to reference
     */
    public SwitchCommand(Supplier<T> valueSupplier) {
        this(valueSupplier, Collections.emptyMap(), new NullCommand());
    }

    @Override
    public boolean isDone() {
        // Kotlin 'lateinit' would throw an exception if accessed before start().
        // We replicate delegation, but users should ensure the command is started first.
        if (selectedCommand == null) {
            throw new IllegalStateException("SwitchCommand has not been started yet.");
        }
        return selectedCommand.isDone();
    }

    @Override
    public void start() {
        T val = valueSupplier.get();
        // Look up value in map, default to defaultCommand if not found (or if map returns null)
        selectedCommand = outcomes.getOrDefault(val, defaultCommand);

        // Safety check just in case the map contained a null key/value pair resulting in null
        if (selectedCommand == null) {
            selectedCommand = defaultCommand;
        }

        selectedCommand.start();
    }

    @Override
    public void update() {
        if (selectedCommand != null) {
            selectedCommand.update();
        }
    }

    @Override
    public void stop(boolean interrupted) {
        if (selectedCommand != null) {
            selectedCommand.stop(interrupted);
        }
    }

    /**
     * Returns a new SwitchCommand instance with the added case.
     * * @param caseValue the value to match
     * @param command the command to run for this case
     * @return a new SwitchCommand instance
     */
    public SwitchCommand<T> withCase(T caseValue, Command command) {
        Map<T, Command> newOutcomes = new HashMap<>(this.outcomes);
        newOutcomes.put(caseValue, command);
        return new SwitchCommand<>(this.valueSupplier, newOutcomes, this.defaultCommand);
    }

    /**
     * Returns a new SwitchCommand instance with a new default command.
     * * @param command the new default command
     * @return a new SwitchCommand instance
     */
    public SwitchCommand<T> withDefault(Command command) {
        return new SwitchCommand<>(this.valueSupplier, this.outcomes, command);
    }
}