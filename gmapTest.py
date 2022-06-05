import gmplot
import json, requests 
import time

DATA_COLLECT_INTERVAL = 2

lat_list = []
lon_list = []
width = 1


def getData():
   url = requests.get("http://192.168.4.1/peripherals")
   text = url.text

   data = json.loads(text)

   device = data
   lat_list.append(device["gpsLat"])
   lon_list.append(device["gpsLng"])
   width = len(lat_list)
   # gpsAlt
   # compassHeading


def create_html_map():
   gmap = gmplot.GoogleMapPlotter(lat_list[0], lon_list[0], 16) 
   gmap.scatter( lat_list, lon_list, '#FF0000',size = 2, marker = False)
   gmap.plot(lat_list, lon_list, "cornflowerblue", edge_width = 3.0)
   gmap.marker(lat_list[0], lon_list[0], color="blue")
   gmap.marker(lat_list[width - 1], lon_list[width - 1], color="red")
   gmap.draw("./map-trace.html")

def main():
   while True:
      try:
         # some code
         getData()
         time.sleep(DATA_COLLECT_INTERVAL)
      except KeyboardInterrupt:
         print("All done")
         create_html_map()
         # If you actually want the program to exit
         raise

      

if __name__ == "__main__":
    main()