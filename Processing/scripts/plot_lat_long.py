import folium
file_names = ['log_to_walmart.txt', 'log_walmart_to_eceb.txt', 'log_walmart_to_walmart.txt']

all_coords = []
for file in file_names:
    with open("Processing/logs/processed/"+file) as f:
        mapit = None
        coordinates = []
        for line in f.readlines():
            coords = [float(x) for x in line.split()]
            coordinates.append(coords)
            all_coords.append(coords)
            if mapit is None:
                mapit = folium.Map(location=coords, zoom_start=10)

        folium.PolyLine(locations=coordinates, color='#ff0000').add_to(mapit)

        mapit.save(f'Processing/data/{file}_map.html')

mapit_all = folium.Map(location=all_coords[0], zoom_start=10)
folium.PolyLine(locations=all_coords, color='#ff0000').add_to(mapit_all)
mapit_all.save(f'Processing/data/all_map.html')
