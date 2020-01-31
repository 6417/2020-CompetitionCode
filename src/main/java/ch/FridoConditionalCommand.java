/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

/**
 * Add your docs here.
 */
public class FridoConditionalCommand extends ConditionalCommand {

    public FridoConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        super(onTrue, onFalse, condition);
    }

}
