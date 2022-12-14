import carla
from .vehicle import Vehicle
from .ui import HUD
from .world import World
import pygame
from pygame.locals import (
    KMOD_CTRL,
    KMOD_SHIFT,
    K_0,
    K_9,
    K_BACKQUOTE,
    K_BACKSPACE,
    K_COMMA,
    K_DOWN,
    K_ESCAPE,
    K_F1,
    K_LEFT,
    K_PERIOD,
    K_RIGHT,
    K_SLASH,
    K_SPACE,
    K_TAB,
    K_UP,
    K_a,
    K_b,
    K_c,
    K_d,
    K_g,
    K_h,
    K_i,
    K_l,
    K_m,
    K_n,
    K_o,
    K_p,
    K_q,
    K_r,
    K_s,
    K_t,
    K_v,
    K_w,
    K_x,
    K_z,
    K_MINUS,
    K_EQUALS,
)


class KeyboardControl(object):
    """Class that handles keyboard input."""

    def __init__(self, player: Vehicle, hud: HUD, start_in_autopilot: bool):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(player.actor, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            player.set_autopilot(self._autopilot_enabled)
            player.actor.set_light_state(self._lights)
        elif isinstance(player.actor, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = player.actor.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(
        self,
        client: carla.Client,
        world: World,
        hud: HUD,
        player: Vehicle,
        clock,
        sync_mode,
    ):
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        player.set_autopilot(False)
                        world.restart()
                        player.set_autopilot(True)
                    else:
                        world.restart()
                elif event.key == K_F1:
                    hud.toggle_info()
                elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_map_layer(reverse=True)
                elif event.key == K_v:
                    world.next_map_layer()
                elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                    world.load_map_layer(unload=True)
                elif event.key == K_b:
                    world.load_map_layer()
                elif event.key == K_h or (
                    event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT
                ):
                    hud.help.toggle()
                elif event.key == K_TAB:
                    player.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                # elif event.key == K_g:
                #     world.toggle_radar()
                elif event.key == K_BACKQUOTE:
                    player.camera_manager.next_sensor()
                elif event.key == K_n:
                    player.camera_manager.next_sensor()
                elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                    if world.constant_velocity_enabled:
                        player.actor.disable_constant_velocity()
                        world.constant_velocity_enabled = False
                        hud.notification("Disabled Constant Velocity Mode")
                    else:
                        player.actor.enable_constant_velocity(carla.Vector3D(17, 0, 0))
                        world.constant_velocity_enabled = True
                        hud.notification("Enabled Constant Velocity Mode at 60 km/h")
                elif event.key == K_o:
                    try:
                        if world.doors_are_open:
                            hud.notification("Closing Doors")
                            world.doors_are_open = False
                            player.actor.close_door(carla.VehicleDoor.All)
                        else:
                            hud.notification("Opening doors")
                            world.doors_are_open = True
                            player.actor.open_door(carla.VehicleDoor.All)
                    except Exception:
                        pass
                elif event.key == K_t:
                    if world.show_vehicle_telemetry:
                        player.actor.show_debug_telemetry(False)
                        world.show_vehicle_telemetry = False
                        hud.notification("Disabled Vehicle Telemetry")
                    else:
                        try:
                            player.actor.show_debug_telemetry(True)
                            world.show_vehicle_telemetry = True
                            hud.notification("Enabled Vehicle Telemetry")
                        except Exception:
                            pass
                elif event.key > K_0 and event.key <= K_9:
                    index_ctrl = 0
                    if pygame.key.get_mods() & KMOD_CTRL:
                        index_ctrl = 9
                    player.camera_manager.set_sensor(event.key - 1 - K_0 + index_ctrl)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    player.camera_manager.toggle_recording()
                # elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                #     if world.recording_enabled:
                #         client.stop_recorder()
                #         world.recording_enabled = False
                #         hud.notification("Recorder is OFF")
                #     else:
                #         client.start_recorder("manual_recording.rec")
                #         world.recording_enabled = True
                #         hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = player.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    player.set_autopilot(self._autopilot_enabled)
                    hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file(
                        "manual_recording.rec", world.recording_start, 0, 0
                    )
                    player.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    hud.notification(
                        "Recording start time is %d" % (world.recording_start)
                    )
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    hud.notification(
                        "Recording start time is %d" % (world.recording_start)
                    )
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = (
                            not self._control.manual_gear_shift
                        )
                        self._control.gear = player.actor.get_control().gear
                        hud.notification(
                            "%s Transmission"
                            % (
                                "Manual"
                                if self._control.manual_gear_shift
                                else "Automatic"
                            )
                        )
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                        if not self._autopilot_enabled and not sync_mode:
                            print(
                                "WARNING: You are currently in asynchronous mode and could "
                                "experience some issues with the traffic simulation"
                            )
                        self._autopilot_enabled = not self._autopilot_enabled
                        player.set_autopilot(self._autopilot_enabled)
                        hud.notification(
                            "Autopilot %s"
                            % ("On" if self._autopilot_enabled else "Off")
                        )
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                    elif event.key == K_l:
                        # Use 'L' key to switch between lights:
                        # closed -> position -> low beam -> fog
                        if not self._lights & carla.VehicleLightState.Position:
                            hud.notification("Position lights")
                            current_lights |= carla.VehicleLightState.Position
                        else:
                            hud.notification("Low beam lights")
                            current_lights |= carla.VehicleLightState.LowBeam
                        if self._lights & carla.VehicleLightState.LowBeam:
                            hud.notification("Fog lights")
                            current_lights |= carla.VehicleLightState.Fog
                        if self._lights & carla.VehicleLightState.Fog:
                            hud.notification("Lights off")
                            current_lights ^= carla.VehicleLightState.Position
                            current_lights ^= carla.VehicleLightState.LowBeam
                            current_lights ^= carla.VehicleLightState.Fog
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior
                    elif event.key == K_z:
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                    elif event.key == K_x:
                        current_lights ^= carla.VehicleLightState.RightBlinker

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else:  # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else:  # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if (
                    current_lights != self._lights
                ):  # Change the light state only if necessary
                    self._lights = current_lights
                    player.actor.set_light_state(carla.VehicleLightState(self._lights))
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(
                    pygame.key.get_pressed(), clock.get_time(), world
                )
            player.actor.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.01, 1.00)
        else:
            self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.hand_brake = keys[K_SPACE]

    def _parse_walker_keys(self, keys, milliseconds, world):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = 0.01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = 0.01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = (
                player.actor_max_speed_fast
                if pygame.key.get_mods() & KMOD_SHIFT
                else player_max_speed
            )
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)
