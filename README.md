# Carla-Simulator
This repository contains the projects I made with Carla Simulator.
## Way to executing
* Install carla simulator follow the official document. https://carla.org/
* Install poetry follow the official document. https://python-poetry.org/docs/
* Run the following command
```
./CarlaEu4.sh # start up Carla simulator
poetry run # run for only first time use
poetry run main
```
* After that you will see the result of following features I implemented.
## AEBS & ACC
There will be a coach car preceding our car and all we have to do is to follow its and stop our car after it stops to avoid collision. Furthermore, we should maintain an appropriate distance, that is, we should keep a distance that isn't too far but also long enough for we to take reaction after the coach car stops with the support from lidar.
demo: https://youtu.be/udK508Nedh0
## Waypoints tracking
There will be 25 waypoints set in the map. We have to follow these points to go through a curve without collision.
demo: https://youtu.be/PjX3MEytops
