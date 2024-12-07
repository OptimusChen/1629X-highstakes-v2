import json

# Function to load JSON data from a file
def load_json_from_file(file_path):
    with open(file_path, 'r') as file:
        content = file.read().strip()  # Strip any extra whitespace
        if not content:  # Handle empty file case
            raise ValueError("The file is empty.")
        return json.loads(content)

# Function to extract segments and control points
def extract_segments_and_controls(data):
    result = []
    for path in data['paths']:
        path_name = path.get('name', 'Unnamed Path')  # Get path name or default to 'Unnamed Path'
        segments = []
        for segment in path['segments']:
            segment_data = {
                'segment_uid': segment['uid'],
                'controls': []
            }
            for control in segment['controls']:
                control_data = {
                    'control_uid': control['uid'],
                    'x': control['x'],
                    'y': control['y'],
                    'heading': control.get('heading', None)  # Some control points may not have heading
                }
                segment_data['controls'].append(control_data)
            segments.append(segment_data)
        result.append({'path_name': path_name, 'segments': segments})
    return result

# Provide the path to the JSON file
file_path = 'path.jerryio (44) (1).txt'  # Replace with your actual file path

# Load data from the file
data = load_json_from_file(file_path)

# Extract segments and control points by path
segments_and_controls = extract_segments_and_controls(data)

# Print the extracted data grouped by path
for path in segments_and_controls:
    print(f"Path: {path['path_name']}")
    toprint = "{"
    for segment in path['segments']:
        i = 0
        s = len(segment['controls'])
        for control in segment['controls']:
            x = control["x"]
            y = control["y"]
            toprint = toprint + "{" + f"{x:.2f}" + ", " + f"{y:.2f}" + "}"
            if (i != s - 1):
                toprint = toprint + ", "
            else:
                toprint = toprint + ", \n"
            i += 1
            
    toprint = toprint + "}"
    print(toprint)