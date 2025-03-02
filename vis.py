import pandas as pd
import matplotlib.pyplot as plt
# import ace_tools as tools  # For displaying data in ChatGPT

# Define the file path
file_path = "your_file.csv"  # Change this to your actual file

# Read the file and filter relevant lines
filtered_data = []
with open(file_path, "r") as file:
    for line in file:
        if line.startswith("distance_sensors, Real"):
            filtered_data.append(line.strip().split(","))

# Convert to DataFrame
columns = ["category", "type", "timestamp", "x", "y", "theta"]
df = pd.DataFrame(filtered_data, columns=columns)

# Convert numeric columns to the correct data type
df["x"] = pd.to_numeric(df["x"])
df["y"] = pd.to_numeric(df["y"])

# Display filtered data
# tools.display_dataframe_to_user(name="Filtered Real Distance Sensor Coordinates", dataframe=df)

# Plot X and Y coordinates
plt.figure(figsize=(8, 8))
plt.scatter(df["x"], df["y"], c="blue", marker="o", label="Sensor Data")
plt.plot(df["x"], df["y"], linestyle="-", color="gray", alpha=0.5)  # Connect points for trajectory

plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.title("Distance Sensor Data (X-Y Plot)")
plt.legend()
plt.grid(True)

# Show the plot
plt.show()
