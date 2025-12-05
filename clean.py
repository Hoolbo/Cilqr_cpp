import os
import shutil
import glob

def clean_directory(directory_path):
    """
    Removes all contents of a directory but keeps the directory itself.
    """
    if not os.path.exists(directory_path):
        print(f"Directory not found, skipping: {directory_path}")
        return

    print(f"Cleaning: {directory_path}")
    for item in os.listdir(directory_path):
        item_path = os.path.join(directory_path, item)
        try:
            if os.path.isfile(item_path) or os.path.islink(item_path):
                os.unlink(item_path)
            elif os.path.isdir(item_path):
                shutil.rmtree(item_path)
        except Exception as e:
            print(f"Failed to delete {item_path}. Reason: {e}")

def remove_file(file_path):
    """
    Removes a specific file.
    """
    if os.path.exists(file_path):
        try:
            os.remove(file_path)
            print(f"Removed: {file_path}")
        except Exception as e:
            print(f"Failed to remove {file_path}. Reason: {e}")
    else:
        print(f"File not found, skipping: {file_path}")

def main():
    # Define directories to clean
    dirs_to_clean = [
        "outputs/alilqr/data",
        "outputs/alilqr/images/visualizations",
        "outputs/alilqr/logs",
        "outputs/cilqr/data",
        "outputs/cilqr/images/visualizations",
        "outputs/cilqr/logs"
    ]

    # Define specific files to remove
    files_to_remove = [
        "outputs/alilqr/images/alilqr_animation.gif",
        "outputs/cilqr/images/cilqr_animation.gif"
    ]

    # Get the script's directory to ensure relative paths work correctly
    script_dir = os.path.dirname(os.path.abspath(__file__))

    print("Starting cleanup...")

    for rel_path in dirs_to_clean:
        abs_path = os.path.join(script_dir, rel_path)
        clean_directory(abs_path)

    for rel_path in files_to_remove:
        abs_path = os.path.join(script_dir, rel_path)
        remove_file(abs_path)

    print("Cleanup finished.")

if __name__ == "__main__":
    main()
