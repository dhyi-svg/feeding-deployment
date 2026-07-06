Possible Error Causes

### ZED injects a frame that have value-correct-pose (confidently incorrect)

1. ZED loses track of one frame due to (low texture, reflection, motion blur) 
2. Loss of one frame might cause differential speed calculation to blow up
3. The localization is off by meters, and when Cartographer comes in, the estimated position is already way off from the actual position. Cartographer has a search radius (7m).  
4. So it either the match fails (Cartographer unable to correct) or it converges to a wrong local optimum which might furthermore increase the error.
5. The next prediction starts from a wrong pose and a wrong velocity. The next scan match initializes even further from the truth. 

Frame loss: the camera fails to deliver an image (grab timeout, USB glitch). This is the fault at the input of VIO.

### ZED Stall

1. ZED install for some reason (the odom → zed_mini_base_link drops)
2. Cartographer complains that “lookup would require extrapolation into the future”
- Possible causes: SDK hiccup, US bandwidth, frame-grab timeout.
- Questions: how long does the ZED stalled? Does the ZED stall every come back after stall? Does Cartographer recover from the extrapolation or keeps extrapolation because of that one time drop

The node’s output pauses. A fault at the output. Detected by the stamp-gap check.

### Zed Health Monitor

ZED wrapper publishes the `status` topic with `SEARCHING`, `OFF`, `FPS_TOO_LOW`, or `OK`.

- `SEARCHING`: the frame are arriving fine and publishing continues, but the VIO algorithm cannot establish confident tracking from the images it’s getting: too few trackable features, low light, motion blur from vibration.

TODO:

1. In the terminal prints, print the actual status, whether they are searching, off, fps_too_low, or ok
2. Ask Claude to read the log to understand the common errors and warnings from navigation and give me the numbers.