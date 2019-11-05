## Milestone 2 Part 3: Closing the loop
Finally, you'll create a closed loop system. To do this, 
* take the temperature being read from the 5V regulator 
* find the difference between it and the desired value (can be hard-coded into your program or received over UART at the beginning) 
* this value should then be used to determine the PWM Duty Cycle of your fan

For the final testing of your system, you need to be able to set the set point without the need to re-program your microcontroller (suggest using UART or Pot). 

You will want to capture a plot of the performance of your system over a period of about 10-30 seconds so that you can track how well your system is working. Try this with a few different set points and note how the system is reacting.

## Taking your measurement into account
Now that you get to see what happens when you feed just the difference or error signal into your control system, in what ways can we make this better. This is really the focus of this Milestone: to explore ways to close this loop. You might want to try taking this difference and subtracting it from the set point. Or maybe you try to re-characterize your system and use Look-Up tables or more complex models to track the behavior of your system. You can implement a control algorithm to attempt to modulate the fan based on certain trends in the temperature. This is totally up to you. What matters the most is that you can show a user the current temperature of the chip and your system's ability hold the desired temperature over a period of time.

## Displaying Temperature
Other than on the oscilloscope, you should be able to track the temperature of your system over a period of time. This can be done using MATLAB, the serial plotter in Code Composer, or another means that you can find. This is so that we can track your system over a period of about 2-5 minutes. It might be useful to also use an LCD display or some other method to see right away what your system is doing. You will need this data in the App Note for this Milestone so that you can describe the performance of your system.

## System Performance
For testing, you will be asked to run a few scenarios where your system will try to stabilize around a specific temperature within a period of time. You will be asked to start with your system at room temperature and bring the regulator up to a specific temperature. While your system is approaching steady state, the main thing we are going to care about is the speed at which you approach steady state as well as the ability for your system to stay stable. After this first test, you will then be asked to bring your system back to a temperature warmer than room temperature and be analyzed again. Then your system will need to bring the chip up to a mildly hot temperature, stabilize, then we will introduce "disturbances" in your system to see how it reacts. These will include increasing/decreasing the voltage across the regulator, a heavier load for your regulator to drive, and adding an additional fan.

* Room Temp -> Medium/Hot
* Medium/Hot -> Slightly Warm
* Slightly Warm -> Medium/Hot
* Add Disturbances

In summary, the testing procedure will take about 20-30 minutes in total, so you MUST come to the lab period ready to demonstrate in a group setting. If you do not come ready, your team will be penalized and we will try to accommodate an additional testing time.
