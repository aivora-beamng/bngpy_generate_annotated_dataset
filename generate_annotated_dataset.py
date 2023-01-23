from __future__ import annotations

import argparse
import json
import sys
from itertools import cycle
from pathlib import Path
from typing import List, Tuple

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera, Electrics, Timer
from scipy.spatial import distance
from tqdm.auto import tqdm

from utils import SPAWN_POINTS, get_metadata, unjam_traffic


class Generator:
    def __init__(self, beamng: BeamNGpy, experiment_name: str, imgs_per_map: int, jam_dist: float, jam_steps: int, steps_per_second: int,
                 capture_steps: int, ego_speed_kph: float, camera_fov: int, camera_pos: List[float], camera_res: List[int], change_tod: bool,
                 initial_time: str, **kwargs):
        self.experiment_name = experiment_name
        self.beamng = beamng
        self.imgs = imgs_per_map
        self.jam_dist = jam_dist
        self.jam_steps = jam_steps
        self.fov = camera_fov
        self.steps_per_second = steps_per_second
        self.capture_steps = capture_steps
        self.speed = ego_speed_kph
        self.camera_pos = tuple(camera_pos)
        self.camera_res = tuple(camera_res)
        self.change_tod = change_tod
        self.initial_time = initial_time

        print('Camera settings:')
        print(f'\tFOV: {self.fov} deg\n\tRES: {self.camera_res}')
        capture_period = self.capture_steps / self.steps_per_second
        print(f'\tCapture period: {capture_period:.2f} s')

        beamng.settings.set_deterministic(steps_per_second)

    @staticmethod
    def create_dir(dir: str) -> Path:
        path = Path(dir)
        path.mkdir(exist_ok=True, parents=True)
        return path

    def setup_and_load_scenario(self, map: str) -> Tuple[Vehicle, Camera, cycle[str]]:
        spawnpoint = cycle(SPAWN_POINTS[map])
        scenario = Scenario(map, f'{map}_dataset')

        ego = Vehicle('ego', model='etk800', color='White')
        pos, rot = SPAWN_POINTS[map][next(spawnpoint)]
        scenario.add_vehicle(ego, pos=pos, rot_quat=rot)

        ego.sensors.attach('electrics', Electrics())
        ego.sensors.attach('timer', Timer())

        scenario.make(beamng)

        print('Loading scenario...')
        beamng.scenario.load(scenario)
        beamng.control.pause()
        beamng.scenario.start()

        print('Creating camera...')
        camera = Camera('camera', beamng, ego, pos=self.camera_pos, dir=(0, -1, 0), near_far_planes=(0.01, 1000),
                        field_of_view_y=self.fov, resolution=self.camera_res, is_render_colours=True, is_render_annotations=True,
                        is_render_instance=False, is_using_shared_memory=True, is_render_depth=True, update_priority=1, is_visualised=False)

        print('Spawning traffic...')
        beamng.traffic.spawn()

        beamng.ui.hide_hud()
        beamng.control.step(self.steps_per_second)

        return ego, camera, spawnpoint

    def generate_images(self, map: str):
        print(f'Generating annotated images for {map}...')
        ego, camera, spawnpoint = self.setup_and_load_scenario(map)

        input('Press Enter to continue when the scenario is fully loaded...')

        print('Setting environment...')
        beamng.env.set_tod(tod=self.initial_time, play=self.change_tod)

        ego.ai.set_mode('span')
        ego.ai.drive_in_lane(True)
        ego.ai.set_speed(self.speed / 3.6, mode='limit')

        cam_dir = Generator.create_dir(f'{self.experiment_name}/images/{map}/camera')
        ann_dir = Generator.create_dir(f'{self.experiment_name}/images/{map}/annotation')
        depth_dir = Generator.create_dir(f'{self.experiment_name}/images/{map}/depth')
        metadata_dir = Generator.create_dir(f'{self.experiment_name}/images/{map}/metadata')

        steps_without_moving_left = self.jam_steps
        jammed = False
        last_pos = (0, 0, 0)

        i = 0
        with tqdm(total=self.imgs) as pbar:
            pbar.update(i)
            while i < self.imgs:
                ego.sensors.poll()
                pos = ego.state['pos']
                cam_data = camera.poll()
                metadata = get_metadata(self.beamng, ego)
                cam_data['colour'].convert('RGB').save(Path(cam_dir, f'{map}_{i:06}.png'))
                cam_data['annotation'].save(Path(ann_dir, f'{map}_{i:06}_annotation.png'))
                cam_data['depth'].convert('L').save(Path(depth_dir, f'{map}_{i:06}_depth.png'))
                with open(Path(metadata_dir, f'{map}_{i:06}_metadata.json'), 'w') as file:
                    json.dump(metadata, file)

                dist = distance.euclidean(last_pos, pos)
                if dist < self.jam_dist:
                    steps_without_moving_left -= 1
                    print(f'Vehicle did not move (dist={dist:.2f}). {steps_without_moving_left} steps until reset...')
                else:
                    jammed = False
                    steps_without_moving_left = self.jam_steps
                    i += 1
                    pbar.update(1)

                if steps_without_moving_left == 0:
                    print(f'Traffic jam detected at iteration {i:06}, resetting traffic vehicles...')
                    steps_without_moving_left = self.jam_steps + 1

                    unjam_traffic(beamng, ego, map, spawnpoint, jammed)
                    jammed = True
                last_pos = pos
                beamng.control.step(self.capture_steps)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(sys.argv[0], formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--host', type=str, default='localhost', help='BeamNG hostname.')
    parser.add_argument('--port', type=int, default=64256, help='BeamNG hostname.')

    parser.add_argument('--experiment-name', type=str, default='dataset',
                        help='The experiment name and the output folder used.')
    parser.add_argument('--maps', nargs='+', type=str,
                        default=['italy', 'east_coast_usa'], help='List of maps to generate images for.')
    parser.add_argument('--imgs-per-map', type=int, default=1000, help='Number of images generated between maps.')
    parser.add_argument('--jam-dist', type=float, default=0.2,
                        help='Distance in meters to detect a traffic jam.'
                             'If the vehicle is moving less than this value for \'--jam-steps\', then a traffic jam is detected.')
    parser.add_argument('--jam-steps', type=int, default=2,
                        help='How many steps to detect a traffic jam. These are \'camera steps\', not simulator steps.')
    parser.add_argument('--steps-per-second', type=int, default=30,
                        help='How many graphic steps per second (frames) should happen.')
    parser.add_argument('--capture-steps', type=int, default=300, help='Steps between two camera captures.')
    parser.add_argument('--ego-speed-kph', type=float, default=80, help='Maximal speed of the camera vehicle.')
    parser.add_argument('--camera-fov', type=int, default=70, help='Field-of-view of the camera, in degrees.')
    parser.add_argument('--camera-pos', type=float, nargs=3,
                        default=[0, -1.9, 0.95], help='Position of the camera, relative to the vehicle.')
    parser.add_argument('--camera-res', type=int, nargs=2,
                        default=[2048, 1024], help='The resolution of the camera (WIDTH, HEIGHT) in pixels.')
    parser.add_argument('--initial-time', type=str, default='14:00:00', help='Initial time of the simulation.')
    parser.add_argument('--change-tod', action='store_false',
                        help='Change the time of day during the simulation.')

    args = parser.parse_args()

    print('Running with args: ')
    print(args)

    experiment_dir = Generator.create_dir(args.experiment_name)
    with open(experiment_dir / 'parameters.json', 'w') as param_file:
        json.dump(vars(args), param_file, indent=4)

    beamng = BeamNGpy(args.host, args.port)
    beamng.open()

    generator = Generator(beamng, **vars(args))

    for map in args.maps:
        generator.generate_images(map)
