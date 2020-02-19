/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.team6417.utils;

/**
 * Add your docs here.
 */
public class Algorithms {

    public static double scale(double x, double inMin, double inMax, double outMin, double outMax) {
        double result = (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
        result = Math.max(result, outMin);
        result = Math.min(result, outMax);
        return result;
    }

}
