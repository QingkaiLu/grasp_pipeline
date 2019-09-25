import os
import trimesh

def convert_mesh_from_ply_to_stl(input_mesh_path, output_mesh_path):
    """
        Convert a mesh from ply to stl formation using trimesh library.
    """
    mesh = trimesh.io.load.load(input_mesh_path, file_type='ply')
    trimesh.io.export.export_mesh(mesh, output_mesh_path, file_type='stl')

def convert_mesh_from_ply_to_dae(input_mesh_path, output_mesh_path):
    """
        Convert a mesh from ply to dae formation using trimesh library.
    """
    mesh = trimesh.io.load.load(input_mesh_path, file_type='ply')
    trimesh.io.export.export_mesh(mesh, output_mesh_path, file_type='dae')

if __name__ == '__main__':
    dataset_dir = '/media/kai/sim_dataset/BigBird/BigBird_mesh'
    object_mesh_dirs = os.listdir(dataset_dir)
    for i, obj_dir in enumerate(object_mesh_dirs):
        #input_mesh_path = dataset_dir + '/' + obj_dir + '/textured_meshes/optimized_tsdf_textured_mesh.ply'
        #output_mesh_path = dataset_dir + '/' + obj_dir + '/textured_meshes/optimized_tsdf_textured_mesh.stl'
        input_mesh_path = dataset_dir + '/' + obj_dir + '/textured_meshes/optimized_poisson_textured_mesh.ply'
        output_mesh_path = dataset_dir + '/' + obj_dir + '/textured_meshes/optimized_poisson_textured_mesh.stl'
        convert_mesh_from_ply_to_stl(input_mesh_path, output_mesh_path)
        #output_mesh_path = dataset_dir + '/' + obj_dir + '/textured_meshes/optimized_tsdf_textured_mesh.dae'
        #convert_mesh_from_ply_to_dae(input_mesh_path, output_mesh_path)
        print obj_dir

