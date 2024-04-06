file_names = ['log_to_walmart.txt', 'log_walmart_to_eceb.txt', 'log_walmart_to_walmart.txt']

all_lat_longs = {}

for name in file_names:
    lat_long = []
    with open("Processing/logs/raw/"+name, 'r') as f:
        lines = f.readlines()

        for line in lines:
            lat_long_str = ""
            line_processed = line[2:-6]
            line_arr = [x.strip() for x in line_processed.split("  ")]

            for data in line_arr:
                if "Lat" in data or "Long" in data:
                    lat_long_str += data.split(":")[1].strip() + " "
            
            if lat_long_str.strip():
                lat_long.append(lat_long_str.strip())
    
    all_lat_longs[name] = lat_long

for name, arr in all_lat_longs.items():
    with open("Processing/logs/processed/"+name, 'w') as f:
        for line in arr:
            f.write(line)
            f.write('\n')