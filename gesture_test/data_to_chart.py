import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

# Load the Excel file
file_path = "gesture_test_table.xlsx"
data = pd.read_excel(file_path)

# Replace non-numeric values with numeric codes for visualization
mapping = {
    "[]": 0,           # Empty list or empty cell
    "stop": 1,         # Stop result
    "Thumbs up": 2     # Thumbs up result
}
data_mapped = data.replace(mapping)

# Custom colormap corresponding to the values
colors = ["grey", "red", "blue"]  # Adjust colors for 0, 1, 2
cmap = ListedColormap(colors)
bounds = [0, 0.5, 1, 1.5]  # Bounds for the discrete categories
norm = BoundaryNorm(bounds, len(colors))

# Set up the figure with a larger size
# plt.figure(figsize=(12, 12))  # Adjust the size to make it wider or taller
plt.figure(figsize=(10, 16))


# Create the heatmap with adjustments
sns.heatmap(data_mapped, cmap=cmap, cbar=True, 
            xticklabels=True, yticklabels=False,
            linewidths=0.3)  # Remove gridlines

# Flip the y-axis
plt.gca().invert_yaxis()

# Add title and labels
plt.title("Test Results Heatmap", fontsize=16)
plt.xlabel("Tests", fontsize=12)
plt.ylabel("Iterations", fontsize=12)

# # Adjust the color bar
# cbar = plt.gcf().axes[-1]
# cbar.set_yticklabels(["Empty", "Stop", "Thumbs Up"])  # Label the color bar values

# Adjust the color bar
cbar = plt.gcf().axes[-1]  # Get the color bar axis
cbar.set_yticks([0.5, 1.0, 1.5])  # Set tick positions
cbar.set_yticklabels(["Empty", "Stop", "Thumbs Up"])  # Set tick labels


# Save the plot as a PNG file
output_path = "test_results_heatmap.png"  # Specify the file path and name
plt.tight_layout()  # Adjust layout to avoid overlap
plt.savefig(output_path, dpi=300)  # Save as a high-resolution PNG
plt.show()

print(f"Graph saved to {output_path}")

# # Save processed data to a new Excel file
# processed_file_path = "processed_results.xlsx"
# data_mapped.to_excel(processed_file_path, index=False)
# print(f"Processed data saved to {processed_file_path}")
