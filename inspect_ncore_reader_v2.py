from pathlib import Path
from ncore.impl.data.v4.components import PointCloudsComponent, SequenceComponentGroupsReader

manifest = Path(__import__('sys').argv[1])
reader = SequenceComponentGroupsReader([manifest])
readers = reader.open_component_readers(PointCloudsComponent.Reader)
print(type(readers).__name__, sorted(readers))
for name, item in readers.items():
    print('reader', name, 'pcs_count', item.pcs_count, 'timestamps', item.pc_timestamps_us[:3])
    for index in range(min(2, item.pcs_count)):
        xyz = item.get_pc_xyz(index)
        print('pc', index, 'shape', xyz.shape, 'dtype', xyz.dtype, 'ref', item.get_pc_reference_frame_id(index), item.get_pc_reference_frame_timestamp_us(index))
