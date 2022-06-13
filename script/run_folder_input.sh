#gdb --args 

#folder path 
infolder=/home/nabdyj/Downloads/Toronto-3D-Partitioning/PCD-files-clean
outfolder=pcds_out/

# road_type=highway
road_type=urban

for file in ${infolder}/*.pcd
do 
	ground_truth_file=$(echo $file| cut -d'.' -f 1).txt
	echo "$ground_truth_file"
	./bin/roadmarking ${file} ${ground_truth_file} ${outfolder} ./model_pool/models_${road_type}_example/ ./config/parameter_${road_type}_example.txt
done

