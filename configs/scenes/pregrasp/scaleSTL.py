import os
import sys
import pathlib
import trimesh

path = pathlib.Path(__file__).parent.absolute()

#######################################################################
# STEP1: Read Obj file and apply transforms
#######################################################################
# part = os.path.join(path, "meshes/mug.stl")
# folder, name_in_file = os.path.split(os.path.realpath(part))

# mesh = trimesh.load(part, skip_materials=True)
# fname = "mug_backup.stl"
# mesh.export(folder + "/" + fname)

# mesh.apply_translation(-mesh.centroid)
# mesh.apply_scale(0.5)

# fname = "mug.stl"
# mesh.export(folder + "/" + fname)

