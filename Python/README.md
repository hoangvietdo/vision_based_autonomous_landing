#### Running simulation
After run launch file,

```
roslaunch px4 landing.launch
```
open 2 new terminals and run each python file.
```
python autonomous_landing.py
python autonomous_landing_camera.py

```
#### Integration algorithm comparison
- blue : Rectangular integration  
- red : Trapezoidal integration  
(rostime : 270s - take-off / 350s - landing ) 






#### Simulation

Drone moves in the form of a sine wave until a marker is detected.
![Test](https://user-images.githubusercontent.com/46476876/64112409-48c91d00-cdc2-11e9-9366-cdbef56d4379.gif)



When the marker is detected, the drone moves to the center of the marker.
![Test3](https://user-images.githubusercontent.com/46476876/64112428-567ea280-cdc2-11e9-8d77-6b0a4cfa549c.gif)



Drone Landed.                                                       
![Test4](https://user-images.githubusercontent.com/46476876/64112437-5da5b080-cdc2-11e9-9057-8cb70f57f754.gif)
