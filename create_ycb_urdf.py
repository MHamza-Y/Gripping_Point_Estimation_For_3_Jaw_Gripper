from object2urdf import object_urdf, ObjectUrdfBuilder

object_folder = "jaw_gripper/resources/models/ycb"
builder = ObjectUrdfBuilder(object_folder)
builder.build_library(force_overwrite=True, decompose_concave=True, force_decompose=False)
