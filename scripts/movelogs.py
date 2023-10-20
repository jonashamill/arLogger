import os
import shutil
from datetime import datetime

# Define the path to the 'logs' folder where the existing log files are located
logs_folder = 'logs'  # Update this with the actual path to your 'logs' folder

# List all files in the 'logs' folder
log_files = [f for f in os.listdir(logs_folder) if os.path.isfile(os.path.join(logs_folder, f))]

# Define a function to extract the publication date from the file name
def extract_publication_date(file_name):
    try:
        # Extract date information from the file name using regular expressions
        import re
        match = re.search(r'\d{14}', file_name)
        if match:
            date_str = match.group()
            return datetime.strptime(date_str, "%Y%m%d%H%M%S")
    except ValueError:
        pass  # Ignore files with incorrect date formats
    return None

# Create subfolders for each publication date and move files accordingly
for file_name in log_files:
    publication_date = extract_publication_date(file_name)
    if publication_date:
        date_str = publication_date.strftime("%m%d")
        subfolder_path = os.path.join(logs_folder, date_str)
        
        # Create the subfolder if it doesn't exist
        os.makedirs(subfolder_path, exist_ok=True)
        
        # Move the log file to the corresponding subfolder
        source_path = os.path.join(logs_folder, file_name)
        dest_path = os.path.join(subfolder_path, file_name)
        shutil.move(source_path, dest_path)
        print(f"Moved '{file_name}' to subfolder '{date_str}'")

# print("Log files have been organized into subfolders by publication date.")


# # Define the path to the 'logs' folder where the existing subfolders are located
# logs_folder = 'logs'  # Update this with the actual path to your 'logs' folder

# # List all subfolders in the 'logs' folder
# subfolders = [f for f in os.listdir(logs_folder) if os.path.isdir(os.path.join(logs_folder, f))]

# # Define a function to convert date format from DDMM to MMDD
# def convert_date_format(ddmm_date):
#     return ddmm_date[2:] + ddmm_date[:2]

# # Rename the subfolders to MMDD format
# for subfolder in subfolders:
#     source_path = os.path.join(logs_folder, subfolder)
#     new_folder_name = convert_date_format(subfolder)
#     dest_path = os.path.join(logs_folder, new_folder_name)
#     os.rename(source_path, dest_path)
#     print(f"Renamed '{subfolder}' to '{new_folder_name}'")

# print("Subfolders have been renamed to MMDD format.")
