from beamngpy import BeamNGpy, Vehicle

SPAWN_POINTS = dict(italy={
    'village_mountain': ([262.389404, -891.666626, 246.920883], (0, 0, 0.793353388, 0.608761367)),
    'airport': ([-1045.27136, 1636.64014, 152.583511], (0, 0, 0.995607359, -0.0936268447)),
    'crossroads': ([729.62677, 763.914001, 177.753006], (0, 0, -0.220662029, 0.975350331)),
    'runway': ([-1887.9967, 1585.78821, 152.583542], (0, 0, 0.779716521, 0.626132691)),
    'port': ([487.19574, 1593.43286, 139.207413], (0, 0, 1, -5.21540749e-008)),
    'city': ([1190.64783, 1235.50586, 148.287506], (0, 0, -0.707106482, 0.707107081)),
    'town_east': ([1114.92346, -689.113403, 146.186447], (0, 0, 0.0112857523, 0.999936314)),
    'small_village': ([-690.403564, -1338.64136, 140.215942], (0, 0, -0.675715974, 0.737162073)),
    'castle_town': ([-969.635193, 953.628723, 392.483368], (0, 0, -0.608761314, 0.793353429))
}, east_coast_usa={
    'townindustrial': ([714.338013, -4.76092243, 52.0771866], (0, 0, 0.259168964, 0.965831998)),
    'highway': ([900.632019, -226.268005, 39.9494019], (0, 0, 0.0427517903, 0.999085724)),
    'gasstation': ([-792.133728, 489.444519, 23.6532993], (0, 0, 0.954438385, 0.298408058)),
    'farmhouse': ([-607.900024, -354.438995, 34.5363007], (0, 0, 0.960378802, 0.278697967)),
})


def unjam_traffic(beamng: BeamNGpy, ego: Vehicle, map: str, spawnpoint_iter, jammed):
    beamng.queue_lua_command('gameplay_traffic.forceTeleportAll()')
    data = ego.sensors['state'].data
    if not jammed:  # repair/reset ego
        beamng.teleport_vehicle(
            ego.vid, pos=data['pos'], rot_quat=data['rotation'])
    else:  # already jammed in this position, respawn
        next_spawnpoint = next(spawnpoint_iter)
        print(f'Changing spawn point to \'{next_spawnpoint}\'')
        pos, rot = SPAWN_POINTS[map][next_spawnpoint]
        beamng.teleport_vehicle(ego.vid, pos=pos, rot_quat=rot)
    ego.ai_set_mode('span')
    ego.ai_drive_in_lane(True)
    # occasionally instabilities are detected and the game engine is paused
    beamng.queue_lua_command('bullettime.pause(false)')


def get_metadata(ego: Vehicle) -> dict:
    return {
        'pos': ego.sensors['state'].data['pos'],
        'rot': ego.sensors['state'].data['rotation'],
        'time': ego.sensors['timer'].data['time'],
        'electrics': ego.sensors['electrics'].data
    }
