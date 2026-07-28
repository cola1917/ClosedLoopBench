from pathlib import Path
from ncore.impl.data.v4.components import CuboidsComponent, LidarSensorComponent, PointCloudsComponent, PosesComponent, SequenceComponentGroupsReader

manifest = Path(__import__('sys').argv[1])
reader = SequenceComponentGroupsReader([manifest])
readers = reader.open_component_readers(PointCloudsComponent.Reader)
print(type(readers).__name__, sorted(readers))
for component in (PointCloudsComponent.Reader, LidarSensorComponent.Reader, CuboidsComponent.Reader, PosesComponent.Reader):
    readers = reader.open_component_readers(component)
    print('component', component, type(readers).__name__, sorted(readers))
    for name, item in readers.items():
        print('reader', name, 'type', type(item).__name__)
        if hasattr(item, 'frames_count'):
            print('frames_count', item.frames_count, 'timestamps', item.frames_timestamps_us[:3], 'generic', item.get_generic_data_names(), 'meta', item.generic_meta_data)
            for timestamp in item.frames_timestamps_us[:1]:
                print('frame', timestamp, 'meta', item.get_frame_generic_meta_data(timestamp), 'bundle_names', item.get_frame_ray_bundle_data_names(timestamp), 'return_names', item.get_frame_ray_bundle_return_data_names(timestamp), 'return_count', item.get_frame_ray_bundle_return_count(timestamp))
                for n in item.get_frame_ray_bundle_data_names(timestamp):
                    value = item.get_frame_ray_bundle_data(timestamp, n)
                    print(' bundle', n, getattr(value, 'shape', None), getattr(value, 'dtype', None))
                for n in item.get_frame_ray_bundle_return_data_names(timestamp):
                    value = item.get_frame_ray_bundle_return_data(timestamp, n, None)
                    print(' return', n, getattr(value, 'shape', None), getattr(value, 'dtype', None))
            continue
        if hasattr(item, 'get_static_poses'):
            print('static poses', [(frames, value.tolist()) for frames, value in item.get_static_poses()])
            print('dynamic pose pairs', [(frames, values[0].shape, values[1].shape, values[0][0].tolist()) for frames, values in item.get_dynamic_poses()])
            continue
        if not hasattr(item, 'pcs_count'):
            continue
        print('pcs_count', item.pcs_count, 'timestamps', item.pc_timestamps_us[:3])
        for index in range(min(2, item.pcs_count)):
            xyz = item.get_pc_xyz(index)
            print('pc', index, 'shape', xyz.shape, 'dtype', xyz.dtype, 'ref', item.get_pc_reference_frame_id(index), item.get_pc_reference_frame_timestamp_us(index))

cuboid_readers = reader.open_component_readers(CuboidsComponent.Reader)
seen_tracks = set()
for name, item in cuboid_readers.items():
    observations = list(item.get_observations())
    print('cuboids', name, 'count', len(observations))
    for obs in observations:
        if str(obs.track_id) in {'4080c30aa7104d91ad005a50b18f6108', '56a71c208ac6472f90b6a82529a6ce61', 'e91afa15647c4c4994f19aeb302c7179', '4005437c730645c2b628dc1da999e06a'}:
            if str(obs.track_id) in seen_tracks:
                continue
            print('obs', obs.track_id, obs.timestamp_us, obs.class_id, obs.source, obs.bbox3)
            seen_tracks.add(str(obs.track_id))
