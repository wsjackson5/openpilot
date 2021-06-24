<b/>**Release Notes**</b>

This branch is only for GM Chevolet Bolt EV based on latest released version from comma ai

# Features

  - New panda code supports white/grey panda giraffe and comma/custom made harness for black panda
  - Update panda and DBC for Comma Pedal
  - Toggle Switch for enabling prebuilt (thanks to 양민님)
  - Add Battery Charging Logic (thanks to 양민님)
  - Add UI Recording (thanks to neokii님)
  - Add auto shut down from dragonpilot
  - Toggle Switch for selection lateral control function with LQR or INDI
  - <b>Support Comma pedal for longitudinal control but this does not gurantee fully to provide the Stop & Go </b>
    1) Only Lateral control by OP
       - Engage : main switch on
       - Disengage : brake or main switch off
    2) Only Longitudinal control by OP (comma pedal shall be installed)
       - Engage : accel(resume) button but main switch must be kept off
       - Speed control : accel or decel button (+/- 5km/h)
       - Disengage : Driver braking, cancel button or or main switch on(only lateral control enable)
    3) Both Lateral and Longitudinal control by OP (comma pedal shall be installed)
       - Engage : set(decel) button but main switch must be kept off
       - Speed control : accel or decel button (+/- 5km/h)
       - Disengage : Driver braking, cancel button or main switch on(only lateral control enable)       
