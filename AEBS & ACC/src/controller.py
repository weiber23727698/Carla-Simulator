from carla import Vector3D, CityObjectLabel
import carla


events = None

def step(my_car):
    control = my_car.get_control()
    vel = my_car.get_velocity().length() # m/s
    d = on_sensor_data(events, my_car)
    if(d < 10.75):
        control.throttle = 0.0
        control.brake = 1/(20-vel*3.6)
        my_car.apply_control(control)
        return
    ttc = 100
    if(vel > 0):
        ttc = d / vel
    if(ttc<3.6):
        control.throttle = 0.4
        control.brake = 1.0
    elif(vel*3.6 < 20):
        control.throttle = 1.0
        control.brake = 0.0
    elif(vel*3.6>=20 and vel*3.6<=22):
        control.throttle = (22-vel*3.6)*0.5
        control.brake = 0.0
    elif(vel*3.6 > 22):
        control.throttle = 0.0
        control.brake = 1.0
    my_car.apply_control(control)



def on_sensor_data(event, my_car):
    ## Find points casted on vehicles
    vehicle_tag = int(CityObjectLabel.Vehicles) # coach car
    vehicle_points = filter(lambda det: det.object_tag == vehicle_tag, event) # get the lidar on the coach car
    
    global events
    events = event
    coach = 0.0
    for data in vehicle_points:
        d =  data.point.distance(carla.Location(0, 0, 0))
        coach = max(coach, d)
    return coach
