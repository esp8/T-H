import os

def rename_index(directory, old_filename, new_filename):
    old_path = os.path.join(directory, old_filename)
    new_path = os.path.join(directory, new_filename)
    os.rename(old_path, new_path)
    print(f"Renamed '{old_filename}' to '{new_filename}'")

# Example usage
directory = "./include"
old_filename = "index.html"
new_filename = "index.h"
rename_index(directory, old_filename, new_filename)
