// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.SystemConstants.LEDConstants;
import frc.robot.subsystems.Lights;

public class FlashLED extends Command {
    double last_time;
    double flash_time;
    boolean flash_state;
    boolean FLASHING;
    int R;
    int G;
    int B;
    double amt;
    Lights LED;
    Boolean finishWithLEDOff;

    public FlashLED(int r, int g, int b, Boolean endOff, Lights lights) {
        LED = lights;
        R = r;
        G = g;
        B = b;
        finishWithLEDOff = endOff;
    }

    @Override
    public void initialize() {
        last_time = Timer.getFPGATimestamp();
        flash_time = LEDConstants.FLASH_INTERVAL; // So it flashes first when button is pressed, not interval seconds after
        flash_state = true;
        FLASHING = true;
    }

    public void flash() {
        if (FLASHING && flash_time >= LEDConstants.FLASH_INTERVAL) {
            if (flash_state) {
                Lights.Set_Colour(R, G, B);

                flash_state = false;
            } else {
                Lights.Set_Colour(0, 0, 0);

                flash_state = true;
            }
            flash_time = 0d;
            
        } else if (FLASHING) {
            double currentTime = Timer.getFPGATimestamp();

            flash_time += currentTime - last_time;
            last_time  = currentTime;
        }
    }

    public void stopFlash() {
        FLASHING = false;
        Lights.Set_Colour(R, G, B);
    }

    @Override
    public void execute() {
        flash();
    }

    @Override
    public void end(boolean interrupted) {
        if (finishWithLEDOff) {
            Lights.Set_Colour(0, 0, 0);
        } else {
            Lights.Set_Colour(R, G, B);
        }
    }

    @Override
    public boolean isFinished() {
        return (!FLASHING);
    }
}