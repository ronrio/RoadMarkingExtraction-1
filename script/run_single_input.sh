single_file=~/Downloads/RoadMarkingExtraction-1/sample_data/Toronto_P3_1.pcd
ground_truth_file=~/Downloads/RoadMarkingExtraction-1/sample_data/Toronto_P3_1.txt
# road_type=highway
road_type=urban

# Toronto with bigger Point Cloud range
single_file=~/Downloads/Toronto-3D-Partitioning/PCD-files/Toronto_P3_09.pcd
ground_truth_file=~/Downloads/Toronto-3D-Partitioning/PCD-files/Toronto_P3_09.txt

# single_file=~/Downloads/RoadMarkingExtraction-1/sample_data/KITTI_015_3sec.pcd

# gdb --args 
./bin/roadmarking ${single_file} ${ground_truth_file} ${single_file}_out ./model_pool/models_${road_type}_example/ ./config/parameter_${road_type}_example.txt
