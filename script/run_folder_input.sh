#gdb --args 

#folder path 
infolder=sample_data
outfolder=pcds_out/

# road_type=highway
road_type=urban

for file in ${infolder}/*.pcd
do 
	./bin/roadmarking ${file} ${outfolder} ./model_pool/models_${road_type}_example/ ./config/parameter_${road_type}_example.txt
done

