import numpy as np
import trimesh
import os


def process_mesh(dataset_name, object_name, object_mesh_folder): 
    if dataset_name == 'BigBird':
        object_mesh_path = object_mesh_folder + '/' + object_name + '/' + \
                'meshes/poisson.ply' 
        proc_mesh_path = object_mesh_folder + '/' + object_name + '/' + \
                'meshes/poisson_proc.stl' 
    elif dataset_name == 'GraspDatabase' or dataset_name == 'YCB':
        object_mesh_path = object_mesh_folder + '/' + object_name + \
                            '/' + object_name + '.stl' 
        proc_mesh_path = object_mesh_folder + '/' + object_name + \
                            '/' + object_name + '_proc.stl' 
    mesh = trimesh.load(object_mesh_path)
    # print 'Vertices #: ', mesh.vertices.shape[0]
    mesh.fix_normals()
    # if mesh.volume < 0.:
    #     # print mesh.volume
    #     mesh.invert()
    # print 'Vertices # after proc:', mesh.vertices.shape[0]
    _ = mesh.export(proc_mesh_path)


def get_object_mass_and_inertia(object_mesh_path):
    '''
        Get the mass and inertia of given object mesh.
    '''
    # Set the density of the object mesh, the default density 
    # of trimesh is 1
    # Assume the density is 100kg/m^3, tenth of the water density
    mesh = trimesh.load(object_mesh_path)
    mesh.density = 100.
    return mesh.mass, mesh.moment_inertia


def generate_urdf(dataset_name, object_name, object_pose, 
                  object_mesh_folder, save_urdf_path):
    '''
        Generate the urdf for a given object mesh with the pose, mass and inertia as
        parameters.
    '''
    # <mesh filename=\"file://""" + object_meshes_path + """/""" + object_name + """/""" + """textured_meshes/optimized_tsdf_textured_mesh.stl\" />
    #object_mesh_path = object_mesh_folder + "/" + object_name + "/" + \
    #                    "meshes/poisson_proc.stl" 

    if dataset_name == 'BigBird':
        object_mesh_path = object_mesh_folder + '/' + object_name + '/' + \
                'meshes/poisson_proc.stl' 
    elif dataset_name == 'GraspDatabase' or dataset_name == 'YCB':
        object_mesh_path = object_mesh_folder + '/' + object_name + \
                            '/' + object_name + '_proc.stl' 

    object_mass, object_inertia = get_object_mass_and_inertia(object_mesh_path)
    cur_urdf_path = save_urdf_path + '/' + object_name + '.urdf'
    f = open(cur_urdf_path, 'w')
    object_rpy = str(object_pose[0]) + ' ' + str(object_pose[1]) + ' ' + str(object_pose[2])
    object_location = str(object_pose[3]) + ' ' + str(object_pose[4]) + ' ' + str(object_pose[5])
    urdf_str = """
<robot name=\"""" + object_name + """\">
  <link name=\"""" + object_name + """_link">
    <inertial>
      <origin xyz=\"""" + str(object_location) +"""\"  rpy=\"""" + str(object_rpy) +"""\"/>
      <mass value=\"""" + str(object_mass) + """\" />
      <inertia  ixx=\"""" + str(object_inertia[0][0]) + """\" ixy=\"""" + str(object_inertia[0][1]) + """\"  ixz=\"""" + \
              str(object_inertia[0][2]) + """\"  iyy=\"""" + str(object_inertia[1][1]) + """\"  iyz=\"""" + str(object_inertia[1][2]) + \
              """\"  izz=\"""" + str(object_inertia[2][2]) + """\" />
    </inertial>
    <visual>
      <origin xyz=\"""" + str(object_location) +"""\"  rpy=\"""" + str(object_rpy) +"""\"/>
      <geometry>
        <mesh filename=\"file://""" + object_mesh_path + """\" />
      </geometry>
    </visual>
    <collision>
      <origin xyz=\"""" + str(object_location) +"""\"  rpy=\"""" + str(object_rpy) +"""\"/>
      <geometry>
        <mesh filename=\"file://""" + object_mesh_path + """\" />
      </geometry>
    </collision>
  </link>
  <gazebo reference=\"""" + object_name + """_link\">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <material>Gazebo/Red</material>
  </gazebo>
</robot>
"""
    f.write(urdf_str)
    f.close()


if __name__=='__main__':
    # dataset_name = 'BigBird'
    # dataset_name = 'GraspDatabase'
    dataset_name = 'YCB'
    object_mesh_folder = '/mnt/tars_data/sim_dataset/' + dataset_name \
                        + '/' + dataset_name + '_mesh'
    save_urdf_path = '/mnt/tars_data/sim_dataset/' + dataset_name \
                        + '/' + dataset_name + '_urdf'
    mesh_pose_array = [0., 0., 0., 0., 0., 0.]
    object_mesh_dirs = os.listdir(object_mesh_folder)

    # Create one folder for each object mesh in GraspDatabase
    # and move the object mesh into the folder
    # for object_name in object_mesh_dirs:
    #     create_folder_cmd = 'mkdir ' + object_mesh_folder + '/' +object_name[:-4]        
    #     move_mesh_cmd = 'mv ' + object_mesh_folder + '/' + object_name + ' ' + \
    #                     object_mesh_folder + '/' + object_name[:-4] + '/' + object_name
    #     print create_folder_cmd
    #     print move_mesh_cmd
    #     os.system(create_folder_cmd)
    #     os.system(move_mesh_cmd)

    # Create one folder for each object mesh in YCB
    # and move the object mesh into the folder
    # for object_name in object_mesh_dirs:
    #     create_folder_cmd = 'mkdir ' + object_mesh_folder + '/' +object_name[:-4]        
    #     move_mesh_cmd = 'mv ' + object_mesh_folder + '/' + object_name + ' ' + \
    #                     object_mesh_folder + '/' + object_name[:-4] + '/' + object_name
    #     print create_folder_cmd
    #     print move_mesh_cmd
    #     os.system(create_folder_cmd)
    #     os.system(move_mesh_cmd)

    for object_name in object_mesh_dirs:
        print object_name
        process_mesh(dataset_name, object_name, object_mesh_folder)        
        #for object_name in object_mesh_dirs:
        generate_urdf(dataset_name, object_name, mesh_pose_array, 
                      object_mesh_folder, save_urdf_path)
        #raw_input('wait')

