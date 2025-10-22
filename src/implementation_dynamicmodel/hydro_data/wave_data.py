# Import modules
import copernicusmarine
import pandas as pd
#copernicusmarine.login()
import matplotlib.pyplot as plt
import time
start = time.time()

import numpy as np

minlon = 6
maxlon = 8
minlat = 43.4
maxlat = 44
starttime = "2024-05-1"
endtime = "2024-09-1"

# Load dataframe ! only once if you use saved copy
request_dataframe = copernicusmarine.read_dataframe(
    dataset_id = "cmems_mod_med_wav_anfc_4.2km_PT1H-i",
    minimum_longitude = minlon,
    maximum_longitude = maxlon,
    minimum_latitude = minlat,
    maximum_latitude = maxlat,
    variables = ["VHM0", "VHM0_WW", "VHM0_SW1", "VHM0_SW2", "VTPK", "VTM02", "VTM01_WW", "VTM01_SW1", "VTM01_SW2"],
    start_datetime = starttime,
    end_datetime = endtime
)
# save data
request_dataframe.to_csv(f"MEDWAVlong{minlon}-{maxlon}_lat{minlat}-{maxlat}_{starttime}-{endtime}.csv")

end1 = time.time()
print(f"Elapsed time for database request: {end1-start} s")

print(request_dataframe)

end2 = time.time()

print(f"Elapsed time for dataframe print: {end2-end1} s")

df = request_dataframe.copy()

#df = pd.read_csv(f"MEDWAVlong{minlon}-{maxlon}_lat{minlat}-{maxlat}_{starttime}-{endtime}.csv")

uniklon = df.index.get_level_values('longitude').unique()
uniklat = df.index.get_level_values('latitude').unique()

def fetch_exact(approx, numslist):
    return min(numslist, key=lambda x:abs(x-approx))
def getlon(approxlon):
    return fetch_exact(approxlon, uniklon)
def getlat(approxlat):
    return fetch_exact(approxlat, uniklat)


dataslice = df.loc[(
    slice(pd.Timestamp('2024-05-15 00:00:00'), pd.Timestamp('2024-08-15 00:00:00')),  # Time range
    slice(getlat(minlat), getlat(maxlat)),                  # Latitude range
    slice(getlon(minlon), getlon(maxlon))                   # Longitude range
), :]

print(dataslice)
result = dataslice.groupby("time").mean()

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

# Calculate omega from VTM02 and VTM01_WW
omega_vtm02 = 2 * np.pi / result['VTM02']
omega_vtm01_ww = 2 * np.pi / result['VTM01_WW']

fig, axes = plt.subplots(4, 1, figsize=(8, 4*3), dpi=200, sharex=True)

# Format time axis for better readability
locator = mdates.AutoDateLocator()
formatter = mdates.ConciseDateFormatter(locator)

# 1st row: VHM0
axes[0].plot(result.index, result['VHM0'], color='b')
axes[0].set_ylabel('VHM0 (m)')
axes[0].set_title('Significant Wave Height (Total)')
axes[0].grid(True)
axes[0].xaxis.set_major_locator(locator)
axes[0].xaxis.set_major_formatter(formatter)

# 2nd row: VHM0_WW
axes[1].plot(result.index, result['VHM0_WW'], color='g')
axes[1].set_ylabel('VHM0_WW (m)')
axes[1].set_title('Significant Wave Height (Wind Waves)')
axes[1].grid(True)
axes[1].xaxis.set_major_locator(locator)
axes[1].xaxis.set_major_formatter(formatter)

# 3rd row: omega from VTM02
axes[2].plot(result.index, omega_vtm02, color='r')
axes[2].set_ylabel('Omega VTM02 (rad/s)')
axes[2].set_title('Angular Frequency from VTM02')
axes[2].grid(True)
axes[2].xaxis.set_major_locator(locator)
axes[2].xaxis.set_major_formatter(formatter)

# 4th row: omega from VTM01_WW
axes[3].plot(result.index, omega_vtm01_ww, color='m')
axes[3].set_ylabel('Omega VTM01_WW (rad/s)')
axes[3].set_title('Angular Frequency from VTM01_WW')
axes[3].grid(True)
axes[3].set_xlabel('Time')
axes[3].xaxis.set_major_locator(locator)
axes[3].xaxis.set_major_formatter(formatter)

plt.tight_layout()


plt.savefig("sea_state_timeseries.pdf", format="pdf")
plt.close(fig)
plt.show()

import plotly.graph_objects as go

#minlon, maxlon = 7.4, 7.5
#minlat, maxlat = 43.73, 43.74
lons = [minlon, maxlon, maxlon, minlon, minlon]
lats = [minlat, minlat, maxlat, maxlat, minlat]

fig = go.Figure(go.Scattergeo(
    lon = lons,
    lat = lats,
    mode = 'lines',
    line = dict(width=3, color='red'),
    fill = "toself",
    fillcolor = "rgba(100,0,0,0.06)",
    name = "Selected Area"
))

fig.update_layout(
    geo = dict(
        projection_type = "mercator",
        showland = True,
        landcolor = "rgb(229, 229, 229)",
        showcountries = True,
        center = dict(lon=(minlon+maxlon)/2, lat=(minlat+maxlat)/2),
        lonaxis = dict(range=[minlon-0.2, maxlon+0.2]),
        lataxis = dict(range=[minlat-0.2, maxlat+0.2]),
        resolution = 250,
    ),
    title = "Selected Area for Wave Data",
    showlegend = False
)

fig.show()