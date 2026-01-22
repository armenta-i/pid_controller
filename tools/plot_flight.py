import matplotlib.pyplot as plt
import pandas as pd

# Load data (Time, Altitute, Thrust)
data = pd.read_csv('flight_data.csv', names=['Time', 'Altitude', 'Thrust'])

fig, ax1 = plt.subplots(figsize=(10,6))

# Plot altitude
ax1.set_xlabel('Time (seconds)')
ax1.set_ylabel('Altitude (m)', color='tab:blue')
ax1.plot(data['Time'], data['Altitude'], color='tab:blue', label='Altitude', linewidth=2)
ax1.axhline(y=10, color='r', linestyle='--', label='Target(10m)')
ax1.tick_params(axis='y', color='tab:blue')

# Plot Thrust on a secondary axis
ax2 = ax1.twinx()
ax2.set_ylabel('Thrust (%)', color='tab:gray')
ax2.plot(data['Time'], data['Thrust'], color='tab:grey', alpha=0.5, label='Thrust')
ax2.tick_params(axis='y', color='tab:grey')

plt.title('Drone Altitude Control - PID Step Response')
fig.tight_layout()
plt.savefig('telemetry_graph.png')
plt.show()