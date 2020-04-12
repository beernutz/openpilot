
This is a fork of comma's openpilot: https://github.com/commaai/openpilot, and contains tweaks for Hondas and GM vehicles.  It is open source and inherits MIT license.  By installing this software you accept all responsibility for anything that might occur while you use it.  All contributors to this fork are not liable.  <b>Use at your own risk.</b>

<b>NOTE: GM is still being worked on for 0.7.4</b> - please try untested-0.7.4-gm branch, have your Eon connected to the internet so that I can work through any errors

If you wish you can buy me a beer or 3:  https://www.patreon.com/kegman


** <b>0.7.3 and below only:</b> If you get a red screen with "Communications Mismatch" please manually reflash panda.  Instructions are here:  https://community.comma.ai/wiki/index.php/Panda_Flashing - If you are using a GM model like the Volt and you get the communications mismatch, please try the -gm branch **

  
<b>WARNING:</b>  Do NOT depend on OP to stop the car in time if you are approaching an object which is not in motion in the same direction as your car.  The radar will NOT detect the stationary object in time to slow your car enough to stop.  If you are approaching a stopped vehicle you must disengage and brake as radars ignore objects that are not in motion.

<b>NOTICE:</b>  Due to feedback I have turned on OTA updates.  You will receive updates automatically (after rebooting 2X) on your Eon so you don't have to reclone or git pull any longer to receive new features *MADE BETWEEN COMMA RELEASES*.  The reason why I separate the branches by release is because some releases can sometimes cause issues.  Features that I or others add, will continue to be updated when you are on the most current release.  If you DO NOT want OTA updates then create a file called "/data/no_ota_updates" and it will not perform OTA updates as long as that file exists.  


I will attempt to detail the changes in each of the branches here: 



<b>kegman</b> - this is the default branch

<b>kegman-BoschGasPress</b> - this branch gives Honda Bosch users the ability to press the gas without disengaging OP for more "stock ACC"-like behaviour.  

<b>kegman-gm</b> - this branch contains default untouched panda directories for GMs having trouble with communication mismatch.  Reflash panda after branch checkout.  (Seems Volt 2017 requires this)

<b>kegman-Clarity</b> - for Honda Clarity models

* Thanks to @Clarity.bru and @wirelessnet2 for restoring the dashcam in 0.7.2

Known bugs in 0.7.4:
- GM not currently working
- Honda Stock AEB re-enabled as per Comma's requirements for custom forks - will try to work on disabling at a later time

Known bugs in 0.7.3:
-Some people may get their panda stuck on blinking blue led.  Reflash Panda, disconnect power from the panda for 1 minute and reconnect power. Reboot Eon and restart car.

Known bugs in 0.7.1:
- BoschGasPress also does not work as of Panda code refactoring by Comma
- If anyone can get these things to work let me know

Known bugs in 0.7:
- UI settings menu not accessible while car is on.
- ACC icon light on dash may turn yellow - does not affect OP engage-ability.

List of changes and tweaks (latest changes at the top):
- <b> New! EPS modified config</b>: Add epsModded value in kegman.json to change the flag for having a modded EPS.

- <b> New! Timer for Nudgeless Auto Lane Change (default 2 seconds before lane change is made) </b>.  Thanks to @pjlao307 for getting this to work.  I have made the delay configurable in kegman.json
  
- <b> New! Nudgeless Auto Lane Change with configurable toggles in kegman.json. </b>  By default the behavior is like comma stock - i.e. signal above 45 mph, then nudge the steering wheel in the direction of the blinkers.  If you don't want to nudge the wheel or want ALC enabled at slower speeds, go into kegman.json and change ALCnudgeLess to "1" and ALCminSpeed to [some value] in m/s units.
  
- <b> Disabled Honda stock FCW, stock AEB because they are too sensitive on some cars. </b>

- <b> New! Dynamic Steer Ratio: </b>Some Hondas and other makes / models have been suffering from excessive ping-ponging on straights since 0.6.x.  The fix was to lower steerRatio.  However lowering steerRatio makes the car turn less aggressively on curves so you lose "turnability".  Raising the steerRatio makes you take turns with maximum force, but then you have ping ponging on straights.  Dynamic steer ratio adjusts based on the steering wheel angle to give you a low steerRatio on straights and a high steerRatio on turns.  This gives the best of both worlds.  Dynamic Steer Ratio is inactive by default, to activate, please adjust the following values using the Live Tuner or edit the kegman.json file:

"sR_BP0": "4.0", - is the steering wheel angle (degrees) when the steerRatio should begin to increase

"sR_BP1": "7.0", - is the steering wheel angle (degrees) when the steerRatio has full boost

"sR_boost": "4.5" - is the maximum boost for the steerRatio at the sR_BP1 wheel angle

"sR_time": "1", - is the amount of time (seconds) before the steerRatio is permitted to lower, after increasing


so on my Pilot which has a steerRatio setting of 12.5, with these settings I get a minimum steerRatio of 12.5 between 0 and 4 degrees on the wheel (straights) and I get an increasing steerRatio which has a 1 second "hang" time between 4+ degrees on the wheel, reaching a maximum of 17 when the wheel gets to 7 degrees (everything in between is interpolated)

- <b> New! @runchman's brake pump chatter fix for pedal users </b> - fixes brake oscillations during engagement at a stop and various other times.

- <b> New! Adjustable stopping distance, one, two, and three bar distance intervals</b>:  BE CAREFUL WITH THESE OPTIONAL SETTINGS IN kegman.json!  Add ONE_BAR_DISTANCE, TWO_BAR_DISTANCE, THREE_BAR_DISTANCE, FOUR_BAR_DISTANCE values in kegman.json to change the following distance interval.  Add STOPPING_DISTANCE to change the distance between you and the lead car at a stop.  If these values do not exist in kegman.json, they just assume default values of 0.9, 1.3, 1.8, 2.3, and 2 respectively.  Thanks to @joeljacobs for adding the optional distance intervals.

E.g.
Add this line to adjust one bar distance following distance in seconds:
"ONE_BAR_DISTANCE":"0.7"

- <b> New! Disable Auto Emergency Braking (Nidec Hondas)</b>:  Since 0.6.4 Comma passes through the AEB signal.  This causes unexpected emergency braking on 2 lane highways with oncoming traffic and is unsafe.  I have disabled this.  Ensure that you reflash you Panda if you get AEBs:  cd /data/openpilot/panda/board && pkill -f boardd && make  - after a successful flash, reboot.

- <b> New! Toyota support</b>:  Thanks to @j4z for adding distance interval support with his Arduino solution and also helping to debug the kegman.json issues to make Kegman fork work with Toyotas!

- <b> New! Added highway speed braking profile tweaks</b>.  Note that 1barHwy, 2barHwy and 3barHwy are DELTAS.  For example if One bar distance is 0.9 seconds, 1barHwy of 0.3 will add 3 seconds to the distance during braking making you brake harder.
  
- <b> New! Added kF feedforward param to live tuner.</b>

- <b> New! Enable / Disable Model based Slowdowns on turns: </b>  On tight turns, the model will slow down the car so that you can make the turn.  Some like this, some people don't.  Set slowOnCurve = "1" to enable slowdowns on curves, or "0" (default) to disable.

- <b> New! Live long tuning for city speeds < 19.44 m/s (43.5 mph, 70 km/h): </b> Execute cd /data/openpilot && ./tune.sh to access live tuner on your mobile device while driving.
  
<b>Instructions for tuning steering with live tuner:</b>
- Kp too high = the car overshoots and undershoots center
- Kp too low = the car doesn't turn enough
- Ki - dampens the overshoot / undershoot of Kp and allows the car to be centered faster, allowing for a higher Kp value
- Ki too high = it gets to center without oscillations, but it takes too long to center.  If you hit a bump or give the wheel a quick nudge, it should oscillate 3 - 5 times before coming to steady-state.  If the wheel oscillates forever (critically damped), then your Kp or Ki or both are too high.
- SteerRatio - too high, and the car ping pongs on straights and turns, too low, and the car doesn't turn enough on curves.  If you're on a turn and the wheel is oversteering and then correcting, steerRatio is too high, and it's fighting with Kp and Ki (which you don't want) - although in the past I've been able to have an oscillating oversteering tune which could do tighter turns, but the turns werent pleasant.
- Kf - lower this if your car oscillates and you've done everything else.  It can be lowered to 0
- All of these parameters interact with each other so finding the balance is a bit experimental  
  
<b>Be careful and ready to take over at any time when doing this!!!</b>  The "distance" in s is the target distance the car will try to maintain.  The default distancces are 0.9s, 1.3s, 1.8s for 1,2 and 3 bar intervals.  I manipulate this value to pass to the MPC to scale the behavior which leads to harder braking or sooner braking or softer braking.  Essentially when you are approaching a car, the distance changes depending on your approach speed.  When the lead car pulls away, the distance returns to whatever your bar setting is
  
There are 3 parameters for one two and three bar distance intervals:
xbarBP0 - is how soon it should start braking - a smaller (or negative) value means your car will brake sooner when the lead car slows, a larger value means your car will start braking later

xbarBP1 - is the approach speed in m/s at which your car gets to max distance setting (in s)

xbarMax - is the maximum distance which is reached when your approach speed reachs xbarBP1
the smaller XbarBP1 is, the sooner you get to max distance / max braking and the harder you brake
the larger xbarMax is, the harder you brake

where  X is the distance interval (bars)

Example:
1BarBP0 = -0.25 start to increase braking when approach speed is -0.25 m/s (this actually means the car is slightly pulling away)
1BarBP1 = 3 - the relative approach speed in m/s when maximum distance is applied
1BarMax = 2.5 - maximum distance in (s)  (Hint if you're slowing down way back from a slowed lead vehicle, reduce this number and reduce BP1 as well)

Everything inbetween -0.25 m/s and 3 m/s is interpolated, which adjusts the distance smoothly as you slow down depending on the lead car approach relative speed.  

- <b>Highway speed braking profiles</b>:  Added highway braking profiles so that you won't follow so closely at speeds > 70 kph. 
  
- <b>Live tuner for Kp and Ki</b>:  Tune your Kp and Ki values live using your cell phone by SSHing into the Eon and executing cd /data/openpilot && ./tune.sh

- <b>Add @pjlao307's Dashcam Recording</b>:  Sometimes you just want to record a wicked OP run on a twisty highway to show your friends.  Sometimes you want to record a big flashing red error and complain about it and show your friends.  This does a screen video capture of the Eon screen and stores the files in /sdcard/videos on your Eon when the REC button is pressed.  Thanks to @pjlao307 and @theantihero for submitting the PR.

- <b>Added primitive tuning script</b>: To invoke live tuning:  (a) turn on tethering on your Eon,  (b) install JuiceSSH or similar and connect your cellphone to the wifi of the Eon using 192.168.43.1 and import the Comma private key,  (c) in JuiceSSH in the SSH session on the Eon issue cd /data/openpilot command, then ./tune.sh.  The text UI will be shown.  (d) turn "tuneGernby" to a "1"  (e) start driving and change the values to tune your steering.  It is best to have a cell phone mount in your car.  Note:  It takes 3 seconds for any changes to take effect.  

- <b>Replaced dev UI</b> with @perpetuoviator dev UI with brake light icon by @berno22 - Thank you both!  NOTE:  There are lots of conveniences in this UI.  When the car is on, you have to press the top left corner to get to the Settings screen.  If you tap the lower right corner you can see the tmux session.  The brake light icon doesn't work properly with some cars (needs a fingerprint tweak I believe.  The wifi IP address and upload speed is printed on the screen.  The brake icon is so that you can see if OP is causing the brake lights to turn on and off and pissing the guy or gal off behind you. NOTE:  For GM vehicles, the brake icon indicates use of the friction brakes on the vehicle instead of the brake lights themselves.


- <b>Added moar JSON parameters</b>:  

"battPercOff": "25",  Turn off the Eon if the Eon battery percentage dips below this value - NOTE this only works when the Eon is NOT powered by the USB cable!

"brakeStoppingTarget": "0.25",  How much OP should mash the brakes when the car is stopped.  Increase if you live in hilly areas and need more standstill braking pressure.

"carVoltageMinEonShutdown": "11800", in mV.  Eon stops charging if car battery goes below this level - NOTE: this is the DISCHARGING voltage.  When the Eon is drawing current the voltage on the battery DROPS.  This is NOT the standing no-load voltage.  I would recommended that you unplug your Eon if you are away from your vehicle for more than a few hours and put a battery charger on your car's battery weekly to avoid wrecking your battery if your Eon stays powered when you shut off the car.

- <b>Tone down PID tuning for Pilot and Ridgline for 0.5.9</b>:  Comma changed latcontrol for 0.5.9, so I had to tone down the PID tuning, reducing steerKpV and steerKiV (to 0.45 and 0.135) because of a slow ping-pong on my 2018 Pilot.  Wheel shaking on 2017 Pilots with 0.5.9 have been reported and this change should help, but may not be sufficient for the 2017 model (and possibly 2016).  2016/7 owners may need to adjust steerKpV and steerKiV manually back to 0.38 and 0.11 in /data/openpilot/selfdrive/car/honda/interface.py to reduce the shake.

- <b>Persist some configuration data in JSON file (/data/kegman.json)</b>:  Sometimes you just want to make a tweak and persist some data that doesn't get wiped out the next time OP is updated.  Stuff like:


"battChargeMax": "70",  (Max limit % to stop charging Eon battery)

"battChargeMin": "60",  (Min limit % to start charging Eon battery)

"cameraOffset": "0.06", (CAMERA_OFFSET - distance from the center of car to Eon camera)

"lastTrMode": "2",      (last distance interval bars you used - (auto generated - do not touch this)

"wheelTouchSeconds": "180"  (time interval between wheel touches when driver facial monitoring is not on - MAX LIMIT 600 seconds)


^^^ This file is auto generated here:  <b>/data/kegman.json</b> so it will remain even when you do a fresh clone.  If you mess something up, just delete the file and it will auto generate to default values.  Use vim or nano to edit this file to your heart's content.

- <b>Interpolated (smoothed) the discontinuity of longitudinal braking profiles</b>:  Prior to this enhancement the braking profiles changed very abruptly like a step function, leading to excessive ping-ponging and late braking.  This feature reduces the ping-ponging and varies the braking strength linearly with gap closure speed (the faster the gap closes between you and the lead car, the harder the braking).

- <b>Remember last distance bar interval</b>:  On startup, the car will bring up the last distance interval used before the car was turned off.  For example:  If you were at X bars before you stopped the car or shut the Eon down, the next time you start the car, the distance setting will be X bars.  

- <b>OTA Updates turned on</b>:  Previously I had turned off OTA updates for safety reasons - I didn't want anyone to get an unexpected result when I made changes.  It appears that many more users want OTA updates for convenience so I have turned this feature back on.  IMPORTANT: If you DO NOT want OTA updates then create a file called "/data/no_ota_updates" and it will not perform OTA updates as long as that file exists.

- <b>Increase acceleration profile when lead car pulls away too quickly or no lead car</b>:  OP has two acceleration profiles, one occurs when following a lead car, and one without a lead car.  Oddly the acceleration profile when following is greater than when not following.  So sometimes a lead car will pull away so quickly, that the car goes from following to not following mode and the acceleration profile actually drops.  I've made the acceleration profiles the same so that the the car doesn't stop accelerating at the same rate when the lead car rips away quickly from a stop. 

- <b>FOUR (new) Step adjustable follow distance</b>:  The default behaviour for following distance is 1.8s of following distance.  It is not adjustable.  This typically causes, in some traffic conditions, the user to be constantly cut off by other drivers, and 1.8s of follow distance instantly becomes much shorter (like 0.2-0.5s).  I wanted to reintroduce honda 'stock-like' ACC behaviour back into the mix to prevent people from getting cutoff so often.  Here is a summary of follow distance in seconds:  <b>1 bar = 0.9s, 2 bars = 1.3s, 3 bars = 1.8, 4 bars = 2.5s of follow distance</b>. Thanks to @arne182, whose code I built upon.

- <b>Reduce speed dependent lane width to 2.85 to 3.5 (from 3.0 to 3.7) [meters]</b>:  This has the effect of making the car veer less towards a disappearing lane line because it assumes that the lane width is less.  It may also improve curb performance.

- <b>Display km/h for set speed in ACC HUD</b>:  For Nidec Hondas, Openpilot overrides Honda's global metric settings and displays mph no matter what.  This change makes the ACC HUD show km/h or mph and abides by the metric setting on the Eon.  I plan on upstreaming this change to comma in the near future.

- <b>Kill the video uploader when the car is running</b>:  Some people like to tether the Eon to a wifi hotspot on their cellphone instead of purchasing a dedicated SIM card to run on the Eon.  When this occurs default comma code will upload large video files even while you are driving chewing up your monthly data limits.  This change stops the video from uploading when the car is running.  *caution* when you stop the car, the videos will resume uploading on your cellular hotspot if you forget to disconnect it.

- <b>Increase brightness of Eon screen</b>:  After the NEOS 8 upgrade some have reported that the screen is too dim.  I have boosted the screen brightness to compensate for this.

- <b>Battery limit charging</b>:  The default comma code charges the Eon to 100% and keeps it there.  LiIon batteries such as the one in the Eon do not like being at 100% or low states of charge for extended periods (this is why when you first get something with a LiIon battery it is always near 50% - it is also why Tesla owners don't charge their cars to 100% if they can help it).  By keeping the charge between 60-70% this will prolong the life of the battery in your Eon.  *NOTE* after your battery gets to 70% the LED will turn from yellow to RED and stay there.  Rest assured that while plugged in the battery will stay between 60-70%.  You can (and should) verify this by plugging the Eon in, SSHing into the Eon and performing a 'tmux a' command to monitor what the charging does.  When you disconnect your Eon, be sure to shut it down properly to keep it in the happy zone of 60-70%.  You can also look at the battery icon to ensure the battery is approximately 60-70% by touching near the left of the eon screen.  Thanks to @csouers for the initial iteration of this.

- <b>Tuned braking at city street speeds (Nidecs only)</b>:  Some have described the default braking when slowing to a stop can be very 'late'.  I have introduced a change in MPC settings to slow the car down sooner when the radar detects deceleration in the lead car.  Different profiles are used for 1 bar and 2 bar distances, with a more aggressive braking profile applied to 1 bar distance.  Additionally lead car stopped distance is increased so that you stop a little farther away from the car in front for a greater margin of error.  Thanks to @arne182 for the MPC and TR changes which I built upon.

- <b>Fixed grinding sound when braking with Pedal (Pilots only)</b>:  Honda Pilots with pedals installed may have noticed a loud ripping / grinding noise accompanied by oscillating pressure on the brake when the brake is pressed especially at lower speeds.  This occurs because OP disengages too late when the brake is pressed and the user ends up fighting with OP for the right brake position.  This fix detects brake pressure sooner so that OP disengages sooner so that the condition is significantly reduced.  If you are on another model and this is happening this fix may also work for you so please message me on Slack or Discord @kegman.

- <b>Smoother acceration from stop (Pedal users)</b>:  The default acceleration / gas profile when pedal is installed may cause a head snapping "lurch" from a stop which can be quite jarring.  This fix smoothes out the acceleration when coming out of a stop.

- <b>Dev UI</b>:  Thanks to @zeeexaris who made this work post 0.5.7 - displays widgets with steering information and temperature as well as lead car velocity and distance.  Very useful when entering turns to know how tight the turn is and more certainty as to whether you have to intervene.  Also great when PID tuning.

- <b>LKAS button toggles steering</b>:  Stock Openpilot deactivates the LKAS button.  In some cases while driving you may have to fight the wheel for a long period of time.  By pressing the LKAS button you can toggle steering off or on so that you don't have to fight the wheel, which can get tiring and probably isn't good for the EPS motor.  When LKAS is toggled off OP still controls gas and brake so it's more like standard ACC.

- <b>Honda Pilot and Ridgeline PID</b>:  I wasn't happy with the way Honda Pilot performed on curves where the car often would hug the inside line of the turn and this was very hazardous in 2 lane highways where it got very close to the oncoming traffic.  Also, on crowned roads (where the fast lane slants to the left and where the slow lane slants to the right), the car would not overcome the gravity of the slanted road and "hug" in the direction of the slant.  After many hours of on the road testing, I have mitigated this issue.  When combined with Gernby's steering it is quite a robust setup.  This combination is found in kegman-plusPilotAwesomeness.  Apparently this branch works well with RIDGELINES too!


Enjoy everyone.


<b>Manual Instructions to flash back to v9 NEOS for downgrading back to 0.5.xx:</b>
- the boot and system image files for v9 NEOS - are in #hw-unofficial - look for the 0.5.13 - they are pinned messages (click pin icon at top)
- download android fastboot
- press and hold UP vol and Power to go into Fastboot mode (Eon Gold is hold DOWN and Power)
- connect to PC with USB cord
- put the system and boot img files in the same directory as fastboot.exe
- type in these commands (only the ones that start with fastboot): https://github.com/commaai/eon-neos/blob/master/flash.sh#L8-L19
- restart the Eon, on the setup screen enter your wifi password and SSID and SSH in - after you successfully SSH in reboot
- when your Eon boots it will ask you to enter install URL:  enter https://openpilot.comma.ai
- when the Eon reboots it will ask you to upgrade NEOS - STOP - do not say yes
- SSH into the Eon
- cd /data
- rm -rf ./openpilot
- git clone https://github.com/kegman/openpilot
- cd openpilot
- git checkout (one of the non-0.6 branches)
- reboot
- enjoy

<b>NOTE:</b> If you have upgraded at any time to v0.5.10, v0.6.x and you want to go back to a branch with v0.5.9 or v0.5.8, then you have to SSH into the Eon and edit the file /data/params/d/ControlsParams and rename "angle_model_bias" to "angle_offset" or your car will have Dash Errors and you'll be scratching your head for hours! 

<b>Pedal Users:</b> Also note that you need to flash your Pedal to go to v0.5.10.  If you want to go back to 0.5.9 or 0.5.8 you need to flash your pedal back to 0.5.9.  Instructions are here:  https://medium.com/@jfrux/comma-pedal-updating-the-firmware-over-can-fa438a3cf910.  Also. After you flash your Pedal..  All hell will break loose on your dash.  Traction control error, Power Steering Error, Trailer Error, OMFG the sky is falling error etc.  DON'T PANIC.  Just drive around a bit and it will disappear after about 2-3 restarts of the car.  Don't rush it I believe it's time dependent as well.  Just drive as normal.  They'll go away.
