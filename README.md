# Carla-Simulator
This repository contains the projects I made with Carla Simulator.
## Execution
* Install carla simulator follow the official document. https://carla.org/
* Install poetry follow the official document. https://python-poetry.org/docs/
* Run the following command
```
./CarlaEu4.sh # execute Carla simulator
poetry run # run for only first time use
poetry run main
```
* After that you will see the result of following features I implemented as demo videos I provided below.
## AEBS & ACC
There will be a coach car preceding our car and all we have to do is to follow its and stop our car after it stops to avoid collision. Furthermore, we should maintain an appropriate distance, that is, we should keep a distance that isn't too far but also long enough for we to take reaction after the coach car stops with the support from lidar.

demo: https://youtu.be/_vdIZXgd1uM
## Waypoints tracking
There will be 25 waypoints set in the map. We have to follow these points to go through a curve without collision.

demo: https://youtu.be/ZO2esmTCv9c
## Lane Finding with Sensor
The object is to go through the mountain road without collision. Different from waypoints tracking, we can only complete the task with support of the camera and lidar.

demo: https://www.youtube.com/watch?v=BcMQsvEDe_E
