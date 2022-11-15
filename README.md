In these implementations we try to transmit data to UART in stm32 microcontroller using Arm Keil MDK and STM32Cube MX in two following ways:
1. Controlling LEDs on the borad using UART. By receiving [Base led] [Direction] [Count] [Status] as an input, it Starts from the _Base led_ number, and every 1 second, the status of one of the next _Count_ LEDs in the direction of _Direction_ will change to _Status_. (considering LED number 1 as PE8 and 8 as PE15.) 
    - example: _2 left 3 on_ which means turning on LED number 2 to 5 sequentially in three second.

2. Turning on the LEDs sequentially in a circle by pressing the blue button on the borad. By pressing it again, the LED that is on at that moment will remain constant and using UART, its number will be written in the terminal. (Image is included)

3. Controlling light intensity of LEDs on the board using UART. 
    - By receiving numbers from 0 to 4, the light intensity will change (0 for off up untill 4 for which is fully on).
    - By receiving number 5, the intensity will go from completely off to fully on within 3 seconds.
    - By receiving number 6, the intensity will go from fully on to completely off within 3 seconds. (Image is included)

4. Controlling light intensity of LEDs on the borad using UART. By receiving [Initial value] [Target value] [Time] as an input, the light intensity will change to _[Initial value]%_ and then it will reach _[Target value]%_ in _[Time]_ seconds.

    - example: _0 100 2_ which means changing the light intensity to 0% (off) at first, then changing it to 100% (fully on) in two seconds.
