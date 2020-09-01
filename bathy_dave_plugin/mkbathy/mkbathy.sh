#!/bin/bash
#
# If running over ssh -X you must first set the $DISPLAY variable to whatever it is on
# the remote machine, as if you were logged into the remote machine locally (i.e. sitting
# at it.)  Most likely DISPLAY=:0 or DISPLAY=:1.  Otherwise meshlab will not run.

# NCEI 1/9 topobathy near woods hole.
PREFIX=ncei19
SRC=bathymetry_source/ncei19_n41x75_w070x75_2018v1.tif

# 500 m x 500 m, roughly.
# DLON=0.006
# DLAT=0.005
# OVERLON=0.0002
# OVERLAT=0.0002

# 1000 m x 1000 m, roughly.  Good for NCEI 1/9 arc-second
DLON=0.012
DLAT=0.010
OVERLON=0.0005
OVERLAT=0.0005

# Immediately around WH.  Takes a < 5 minutes to generate.
STARTLON=-70.699
STARTLAT=41.509
ENDLON=-70.611
ENDLAT=41.529

# All of ncei19_n41x75_w070x75_2018v1.tif - this is still just Woods Hole environs.
# STARTLON=-70.75
# STARTLAT=41.5
# ENDLON=-70.5
# ENDLAT=41.75

MLX=mkbathy_dependencies/bathy.mlx
gdal_translate -of GMT $SRC ../bathymetry/$PREFIX.grd


# GMRT ~60 m bathy MV sound and somewhat south.
# PREFIX=gmrt
# SRC=GMRTv3_7_20191219topo.grd
# DLON=0.012
# DLAT=0.010
# OVERLON=0.0005
# OVERLAT=0.0005
# STARTLON=-70.91
# STARTLAT=41.1
# ENDLON=-70.33
# ENDLAT=41.57
# MLX=gmrt_60m.mlx
# cp GMRTv3_7_20191219topo.grd ../bathymetry/$PREFIX.grd




lat=$STARTLAT
lon=$STARTLON
while [[ $(calc "($lon < $ENDLON)") -eq 1 ]]; do

    elon=$(calc -p "($lon+$DLON)")
    sslon=$(calc -p "($lon-$OVERLON)")
    eelon=$(calc -p "($elon+$OVERLON)")


    #for  number of cores, do the tasks below but background the delaunay triangulation.
    
    while [[ $(calc "($lat < $ENDLAT)") -eq 1 ]]; do

	elat=$(calc -p "($lat+$DLAT)")
	sslat=$(calc -p "($lat-$OVERLAT)")
	eelat=$(calc -p "($elat+$OVERLAT)")

	echo $lon $elon $lat $elat
	echo $sslon $eelon $sslat $eelat

	# create output filename
	fname=$(printf R_%.03f_%.03f_%0.3f_%.03f $lon $elon $lat $elat)
	
	# cut the lat/lon grid to this region.
	echo "Cut to region..."
	gmt grdcut ../bathymetry/$PREFIX.grd -G../bathymetry/$PREFIX.$fname.grd -R$sslon/$eelon/$sslat/$eelat

	# translate into a list of points for reprojection.
	echo "Translate for reprojection..."
	gmt grd2xyz ../bathymetry/$PREFIX.$fname.grd > ../bathymetry/$PREFIX.$fname.xyz
	sed -i 's#\t# #g' ../bathymetry/$PREFIX.$fname.xyz # gmt though produces xyz separated by tabs.  gdaltransform silently ignores.

	# generate approximate bounds in projected coordinate system and insert these into filename.
	#proj=($(echo $lon $lat 0 | gdaltransform -s_srs EPSG:4326 -t_srs EPSG:26987))
	#sproje=$(calc -p "round(${proj[0]})")
	#sprojn=$(calc -p "round(${proj[1]})")
	#proj=($(echo $elon $elat 0 | gdaltransform -s_srs EPSG:4326 -t_srs EPSG:26987))
	#eproje=$(calc -p "round(${proj[0]})")
	#eprojn=$(calc -p "round(${proj[1]})")
	#projfname=R$sproje.$eproje.$sprojn.$eprojn
	#echo $projfname
	# Much easier to search in regular lat/lon grid.
	projfname=$fname
	
	# reproject into a MA coordinate system.  Get the EPSG codes from http://epsg.io
	echo "Project..."
	echo "X Y Z" > ../bathymetry/$PREFIX.$projfname.epsg26987.asc  # header will be necessary in next step.
	cat  ../bathymetry/$PREFIX.$fname.xyz |  gdaltransform -s_srs EPSG:4326 -t_srs EPSG:26987 >> ../bathymetry/$PREFIX.$projfname.epsg26987.asc
	
	# 2D delaunay translation to generate a ply file to import into meshlab for simplification and texture mapping.  Requires pdal >= 1.9  (i.e. compile from source)
	# Single core...  Use blah&; blah&; wait
	# This produces walls at the edges where the mesh is concave.  convex edges work fine.
	echo "Start triangulation..."
	pdal translate --reader text -i ../bathymetry/$PREFIX.$projfname.epsg26987.asc -o ../bathymetry/$PREFIX.$projfname.epsg26987.ply --writers.ply.faces=true -f delaunay
	# greedymesh (now called greedyprojection) should be much more suited to this, but it segfaults without any useful error even with debugging turned on.  I doubt these
	# clouds are too big.  Could be the numbers are too big?  No.  small files and small numbers made no difference.  
	#pdal translate --reader text -i ../bathymetry/$PREFIX.$projfname.epsg26987.asc -o ../bathymetry/$PREFIX.$projfname.epsg26987.ply --writers.ply.faces=true -f greedyprojection  --filters.greedyprojection.multiplier=2 --filters.greedyprojection.radius=4
	echo "Done."

	# Simplify in meshlab and get rid of spurious faces at edges from triangulation.  Vertex normals need to be retained.
	#meshlabserver -i ../bathymetry/$PREFIX.$projfname.epsg26987.ply -s mkbathy_dependencies/bathy.mlx -o ../bathymetry/$PREFIX.$projfname.epsg26987.obj -om vn
	# script bathy3.mlx works in version 2020.03.  many syntax changes between versions.
	./mkbathy_dependencies/meshlab_linux_portable/meshlabserver -i ../bathymetry/$PREFIX.$projfname.epsg26987.ply -s $MLX -o ../bathymetry/$PREFIX.$projfname.epsg26987.obj -m vn wt

	# texture map?!  Not useful for underwater stuff anyway and we can use tiling in gazebo.

	# put the final product in a folder that conforms to gazebo model database structure
	#@@@ move this from ../bathymetry/ obviously.
	MODEL_URI=$PREFIX.$projfname.epsg26987
	MODEL_DIR=../bathymetry/$MODEL_URI
	mkdir -p $MODEL_DIR/meshes
	cp ../bathymetry/$PREFIX.$projfname.epsg26987.obj $MODEL_DIR/meshes/
	cp ../bathymetry/$PREFIX.$projfname.epsg26987.obj.mtl $MODEL_DIR/meshes/

	# create model.config 
	cat ./mkbathy_dependencies/templates/model.config | sed s#MODEL_NAME#$PREFIX.$projfname#g > $MODEL_DIR/model.config

	# create model.sdf
	cat ./mkbathy_dependencies/templates/model.sdf | sed s#MODEL_NAME#$PREFIX.$projfname#g | sed s#MODEL_URI#model://$MODEL_URI/meshes/$MODEL_URI.obj#g  > $MODEL_DIR/model.sdf
	
	#exit 0
	
	lat=$elat
	
    done # while lat

    lon=$elon
    lat=$STARTLAT

	echo " "
	echo " "
    
done # while lon


# delete temp files ecept model directories
rm ../bathymetry/*
