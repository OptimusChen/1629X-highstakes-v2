import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import filedialog
from collections import defaultdict

def read_data(filename):
    data = defaultdict(lambda: {'x': [], 'y': [], 'heading': []})
    
    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split(', ')
            if len(parts) == 4:
                print_type, x, y, heading = parts
                try:
                    x, y, heading = float(x), float(y), float(heading)
                    if x != -1 and y != -1:
                        data[print_type]['x'].append(x)
                        data[print_type]['y'].append(y)
                    if heading != -1:
                        data[print_type]['heading'].append(heading)
                except ValueError:
                    print(f"Skipping invalid line: {line}")
    
    return data

def plot_data(data):
    fig_xy, axes_xy = plt.subplots(1, len(data), figsize=(6 * len(data), 6))
    fig_heading, axes_heading = plt.subplots(1, len(data), figsize=(6 * len(data), 4))
    
    if len(data) == 1:
        axes_xy = [axes_xy]
        axes_heading = [axes_heading]
    
    for ax_xy, ax_heading, (print_type, values) in zip(axes_xy, axes_heading, data.items()):
        # X-Y scatter plot
        if values['x'] and values['y']:
            ax_xy.scatter(values['x'], values['y'], label=f'{print_type} Position', marker='o')
            ax_xy.plot(values['x'], values['y'], linestyle='-', alpha=0.5)
            ax_xy.set_xlabel('X Coordinate')
            ax_xy.set_ylabel('Y Coordinate')
            ax_xy.set_title(f'X-Y Coordinates for {print_type}')
            ax_xy.legend()
            ax_xy.grid()
        
        # Heading plot
        if values['heading']:
            ax_heading.plot(values['heading'], label=f'{print_type} Heading', marker='o')
            ax_heading.set_xlabel('Index')
            ax_heading.set_ylabel('Heading (degrees)')
            ax_heading.set_title(f'Heading for {print_type}')
            ax_heading.legend()
            ax_heading.grid()
    
    plt.show()

# File selection UI
root = tk.Tk()
root.withdraw()  # Hide the main window
file_path = "data.txt"

data = read_data(file_path)
plot_data(data)
