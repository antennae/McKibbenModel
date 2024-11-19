# McKibbenModel
Simulation of the McKibben actuators into the SOFA framework


## To test :

You may easily choose the simulation characteristics based on parameters, on line 23 to 26 in the main file.

Then you may launch the scene using runSofa :

```console
runSofa ConstrainCylinder.py
```

## Parameters :

PLUGIN - True will required the updated version of soft robot plugin (python prefab, not shared yet)

CONSTRAIN - To choose if you add the deformation constrains

ELONGATION - False = Classic McKibben - True = McKibben Actuator that elongate

INVERSE - False = Direct control (CTRL + A = + 10 kPa ; CTRL + Q = - 10kPa) - True = Inverse Control (CTRL + G = increase height ; CTRL + B = reduce height)