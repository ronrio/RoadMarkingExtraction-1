single_file=./hope.pcd
#./sample_data/aggreg_from_30.pcd

# road_type=highway
road_type=urban

# gdb --args 
./bin/roadmarking ${single_file} ${single_file}_out ./model_pool/models_${road_type}_example/ ./config/parameter_${road_type}_example.txt
