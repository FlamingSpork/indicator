.PHONY: download

download:
	wget https://download.geofabrik.de/north-america/us/massachusetts-latest.osm.bz2

massachusetts-latest.osm: massachusetts-latest.osm.bz2
	bzip2 -d massachusetts-latest.osm.bz2

greater-boston.osm: massachusetts-latest.osm
	osmium extract --bbox -71.325285,42.347803,-70.828154,42.570191 massachusetts-latest.osm -o greater-boston.osm

highways.osm: greater-boston.osm
	osmium tags-filter greater-boston.osm 'highway=*' -o highways.osm
