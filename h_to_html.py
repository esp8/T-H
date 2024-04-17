import os
Import("env")

def after_build(source, target, env):
    # Example usage
    directory = "./include"
    old_filename = "index.h"
    new_filename = "index.html"
    rename_file(directory, old_filename, new_filename)

env.AddPostAction("checkprogsize", after_build)

def rename_file(directory, old_filename, new_filename):
    old_path = os.path.join(directory, old_filename)
    new_path = os.path.join(directory, new_filename)
    os.rename(old_path, new_path)
    print(f"Renamed '{old_filename}' to '{new_filename}'")
