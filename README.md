# Homecart
## UR Sim Instrutions:
* Navigate to ``/ur_sim`` and run ``run_ur_sim.sh`` to start docker container.
* Open UR Client GUI
* Click on Hamburger Icon in top right corner and move to settings.
* Enable remotere control
* Naviagte back to main menu and change local control to remote control 
* In a separate terminal navigate to the ``/ur_sim/scripts`` folder and run your code

*Note:*
* Test all code on the ``ur_sim`` before running it on the hardware.
* Code that will be deployed on the hardware should be contained in the ``/scripts`` folder and in its own subdirectory.
* Code that will be used in the ``ur_sim`` should be contained in the ``/ur_sim/scripts`` folder and in its own subdirectory.
