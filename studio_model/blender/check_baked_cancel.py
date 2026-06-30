import bpy

for o in bpy.data.objects:
    m = o.matrix_world
    print(f"[{o.name}] parent={o.parent.name if o.parent else None}")
    print(f"  translation = ({m.translation.x:.4f}, {m.translation.y:.4f}, {m.translation.z:.4f})")
    e = m.to_euler('XYZ')
    print(f"  euler(deg)  = ({e.x*57.29578:.4f}, {e.y*57.29578:.4f}, {e.z*57.29578:.4f})")
