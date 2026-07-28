from pathlib import Path
from ncore.impl.data.v4.components import CuboidsComponent, LidarSensorComponent, PointCloudsComponent, SequenceComponentGroupsReader

manifest = Path(__import__('sys').argv[1])
reader = SequenceComponentGroupsReader([manifest])
readers = reader.open_component_readers(PointCloudsComponent.Reader)
print(type(readers).__name__, sorted(readers))
for component in (PointCloudsComponent.Reader, LidarSensorComponent.Reader, CuboidsComponent.Reader):
    readers = reader.open_component_readers(component)
    print('component', component, type(readers).__name__, sorted(readers))
    for name, item in readers.items():
        print('reader', name, 'type', type(item).__name__)
        if hasattr(item, 'frames_count'):
            print('frames_count', item.frames_count, 'timestamps', item.frames_timestamps_us[:3], 'generic', item.get_generic_data_names())
            for timestamp in item.frames_timestamps_us[:1]:
                print('frame', timestamp, 'bundle_names', item.get_frame_ray_bundle_data_names(timestamp), 'return_names', item.get_frame_ray_bundle_return_data_names(timestamp), 'return_count', item.get_frame_ray_bundle_return_count(timestamp))
                for n in item.get_frame_ray_bundle_data_names(timestamp):
                    value = item.get_frame_ray_bundle_data(timestamp, n)
                    print(' bundle', n, getattr(value, 'shape', None), getattr(value, 'dtype', None))
                for n in item.get_frame_ray_bundle_return_data_names(timestamp):
                    value = item.get_frame_ray_bundle_return_data(timestamp, n, None)
                    print(' return', n, getattr(value, 'shape', None), getattr(value, 'dtype', None))
            continue
        if not hasattr(item, 'pcs_count'):
            continue
        print('pcs_count', item.pcs_count, 'timestamps', item.pc_timestamps_us[:3])
        for index in range(min(2, item.pcs_count)):
            xyz = item.get_pc_xyz(index)
            print('pc', index, 'shape', xyz.shape, 'dtype', xyz.dtype, 'ref', item.get_pc_reference_frame_id(index), item.get_pc_reference_frame_timestamp_us(index))

cuboid_readers = reader.open_component_readers(CuboidsComponent.Reader)
for name, item in cuboid_readers.items():
    observations = list(item.get_observations())
    print('cuboids', name, 'count', len(observations))
    for obs in observations:
        if str(obs.track_id) in {'4080c30aa7104d91ad005a50b18f6108', '56a71c208ac6472f90b6a82529a6ce61', 'e91afa15647c4c4994f19aeb302c7179', '4005437c730645c2b628dc1da999e06a'}:
            print('obs', obs.track_id, obs.timestamp_us, obs.class_id, obs.source, obs.bbox3)
            break
