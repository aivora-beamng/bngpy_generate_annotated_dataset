from __future__ import annotations

import json
from itertools import cycle
from pathlib import Path
from typing import Tuple

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera, Electrics, Timer
from scipy.spatial import distance
from tqdm.auto import tqdm

from utils import SPAWN_POINTS, get_metadata, unjam_traffic


def setup_and_load_scenario(fov: int, resolution: Tuple[int, int]) -> Tuple[Vehicle, Camera, cycle[str]]:
    spawnpoint = cycle(SPAWN_POINTS[curr_map])
    scenario = Scenario(curr_map, f'{curr_map}_dataset')

    ego = Vehicle('ego', model='etk800', color='White')
    pos, rot = SPAWN_POINTS[curr_map][next(spawnpoint)]
    scenario.add_vehicle(ego, pos=pos, rot_quat=rot)

    ego.sensors.attach('electrics', Electrics())
    ego.sensors.attach('timer', Timer())

    scenario.make(beamng)

    print('Loading scenario...')
    beamng.scenario.load(scenario)
    beamng.pause()
    beamng.scenario.start()

    print('Creating camera...')
    camera = Camera('camera', beamng, ego, pos=(0, -1.9, 0.9), dir=(0, -1, 0), near_far_planes=(0.01, 1000),
                    field_of_view_y=fov, resolution=resolution, is_render_colours=True, is_render_annotations=True,
                    is_using_shared_memory=False, is_render_depth=False, update_priority=1, is_visualised=False)

    print('Spawning traffic...')
    beamng.traffic.spawn()

    beamng.ui.hide_hud()
    beamng.settings.set_deterministic()
    beamng.settings.set_steps_per_second(34)
    beamng.control.step(34)

    return ego, camera, spawnpoint


if __name__ == '__main__':
    beamng = BeamNGpy('localhost', 64256)
    beamng.open()

    MAPS_TO_GENERATE = ['italy', 'east_coast_usa']
    IMGS_PER_MAP = 25000  # how many images to generate
    MINIMAL_MOVE_DISTANCE = 0.2  # minimal distance in meters to detect a jam
    # max steps without moving at least the minimal distance to reset traffic
    MAX_STEPS_WITHOUT_MOVING = 5
    STEPS_BETWEEN_CAPTURES = 60  # 1 step is 1/34 s, approximately 29.5 ms
    EGO_SPEED_KPH = 80  # the vehicle max speed

    CAMERA_FOV = 70  # field-of-view in degrees
    CAMERA_RESOLUTION = (2048, 1024)  # resolution of the camera images in pixels

    for curr_map in MAPS_TO_GENERATE:
        print(f'Generating annotated images for {curr_map}...')
        ego, camera, spawnpoint = setup_and_load_scenario(CAMERA_FOV, CAMERA_RESOLUTION)

        input('Press Enter to continue when the scenario is fully loaded...')

        ego.ai.set_mode('span')
        ego.ai.drive_in_lane(True)
        ego.ai.set_speed(EGO_SPEED_KPH / 3.6, mode='limit')

        cam_dir = Path(f'images/{curr_map}/camera')
        ann_dir = Path(f'images/{curr_map}/annotation')
        metadata_dir = Path(f'images/{curr_map}/metadata')
        cam_dir.mkdir(exist_ok=True, parents=True)
        ann_dir.mkdir(exist_ok=True, parents=True)
        metadata_dir.mkdir(exist_ok=True, parents=True)

        steps_without_moving_left = MAX_STEPS_WITHOUT_MOVING
        jammed = False
        last_pos = (0, 0, 0)

        i = 0
        with tqdm(total=IMGS_PER_MAP) as pbar:
            pbar.update(i)
            while i < IMGS_PER_MAP:
                ego.sensors.poll()
                pos = ego.sensors['state']['pos']
                cam_data = camera.poll()
                beamng.control.step(STEPS_BETWEEN_CAPTURES)
                cam_data['colour'].convert('RGB').save(Path(cam_dir, f'{curr_map}_{i:06}.png'))
                cam_data['annotation'].save(Path(ann_dir, f'{curr_map}_{i:06}_annotation.png'))
                with open(Path(metadata_dir, f'{curr_map}_{i:06}_metadata.json'), 'w') as file:
                    json.dump(get_metadata(ego), file)

                dist = distance.euclidean(last_pos, pos)
                if dist < MINIMAL_MOVE_DISTANCE:
                    steps_without_moving_left -= 1
                    print(f'Vehicle did not move (dist={dist:.2f}). {steps_without_moving_left} steps until reset...')
                else:
                    jammed = False
                    steps_without_moving_left = MAX_STEPS_WITHOUT_MOVING
                    i += 1
                    pbar.update(1)

                if steps_without_moving_left == 0:
                    print(f'Traffic jam detected at iteration {i:06}, resetting traffic vehicles...')
                    steps_without_moving_left = MAX_STEPS_WITHOUT_MOVING + 1

                    unjam_traffic(beamng, ego, curr_map, spawnpoint, jammed)
                    jammed = True
                last_pos = pos
