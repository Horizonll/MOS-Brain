# configuration.py
#
#   @description : Loading configurations from config.json and 
#                  config_overide.json for variant configs
#   
#   @interfaces : 
#       1. read_config(): dictionary
#           description: reading config from config.json
#                        and overide it from config_override.json
#           return: a dictionary, the config
# 
#   @note :
#       1. The .json parser allows annotation begins with '//' or
#          quoted with '/*' and '*/' (C++ flavor)
#       2. Read config.d/*.yaml 
#

import os
import sys
import copy
import yaml
import json
import re



def recursive_dict_merge(base_dict, merge_dict):
    """
    Recursively merges merge_dict into base_dict.

    When a key exists in both dictionaries:
    - If both values are dictionaries, the merge is performed recursively on them.
    - Otherwise, the value from merge_dict overrides the value in base_dict.

    Args:
        base_dict (dict): The dictionary that will be updated. A copy is made
                          to avoid modifying the original.
        merge_dict (dict): The dictionary containing values to merge into base_dict.

    Returns:
        dict: A new dictionary containing the merged result.
    """
    # Create a deep copy of the base dictionary to avoid modifying the original
    merged = copy.deepcopy(base_dict)

    for key, value in merge_dict.items():
        if key in merged:
            # Key exists in both dictionaries
            if isinstance(merged[key], dict) and isinstance(value, dict):
                # Both values are dictionaries, recurse
                merged[key] = recursive_dict_merge(merged[key], value)
            else:
                # Otherwise, value from merge_dict overrides
                merged[key] = value
        else:
            # Key only exists in merge_dict, add it to merged
            merged[key] = value

    return merged



def read_all_yaml_in_directory(directory_path):
    """
    Reads all .yaml and .yml files in a directory and returns their content
    as a dictionary where keys are filenames (without extension) and values
    are the loaded YAML data.

    Args:
        directory_path (str): The path to the directory to scan.

    Returns:
        dict: A dictionary where keys are filenames (without extension)
              and values are the loaded YAML data. Returns an empty dict
              if the directory doesn't exist or contains no YAML files.
    """
    all_yaml_data = {}

    if not os.path.isdir(directory_path):
        print(f"Error: Directory not found at {directory_path}", file=sys.stderr)
        return all_yaml_data # Return empty dict if directory doesn't exist

    print(f"Scanning directory: {directory_path}")

    # List all files and directories in the given path
    try:
        entries = os.listdir(directory_path)
    except OSError as e:
        print(f"Error listing directory {directory_path}: {e}", file=sys.stderr)
        return all_yaml_data

    yaml_files = []
    for entry in entries:
        full_path = os.path.join(directory_path, entry)
        # Check if it's a file and ends with .yaml or .yml (case-insensitive)
        if os.path.isfile(full_path) and (entry.lower().endswith('.yaml') or entry.lower().endswith('.yml')):
            yaml_files.append(full_path)

    if not yaml_files:
        print("No .yaml or .yml files found in the directory.")
        return all_yaml_data

    print(f"Found {len(yaml_files)} YAML file(s).")

    yaml_files = sorted(yaml_files)

    for yaml_file_path in yaml_files:
        file_name = os.path.basename(yaml_file_path)
        # Get filename without extension for the dictionary key
        file_key, _ = os.path.splitext(file_name)

        print(f"Reading file: {file_name}")

        try:
            with open(yaml_file_path, 'r', encoding='utf-8') as file:
                # Use safe_load for security
                data = yaml.safe_load(file)
                all_yaml_data = recursive_dict_merge(all_yaml_data, data)
                print(f"Successfully loaded {file_name}: ", yaml_file_path)

        except FileNotFoundError:
             # This case should ideally not happen because we checked isfile()
             print(f"Error: File not found during read - {file_name}", file=sys.stderr)
        except yaml.YAMLError as e:
            print(f"Error parsing YAML file {file_name}: {e}", file=sys.stderr)
            all_yaml_data[file_key] = None # Store None or an error indicator if parsing fails
        except Exception as e:
            print(f"An unexpected error occurred while reading {file_name}: {e}", file=sys.stderr)
            all_yaml_data[file_key] = None # Store None or an error indicator


    return all_yaml_data



def load_config(prefix = ""):
    config = read_all_yaml_in_directory(prefix + "config.d")
    config_override = read_all_yaml_in_directory(prefix + "config_override.d")
    return recursive_dict_merge(config, config_override)



# Example usage:
if __name__ == "__main__":
    # You can set a specific directory path here, or
    # ask the user for input, or use the current directory.
    # directory_to_read = '.' # Current directory
    # directory_to_read = '/path/to/your/folder' # Specify a path

    if len(sys.argv) > 1:
        directory_to_read = sys.argv[1]
    else:
        directory_to_read = input("Enter the directory path to read YAML files from (or press Enter for current directory): ")
        if not directory_to_read:
            directory_to_read = '.' # Default to current directory if user provides no input

    yaml_data = read_all_yaml_in_directory(directory_to_read)

    if yaml_data:
        print("\n--- Loaded YAML Data ---")
        # Print the loaded data (optional, can be large)
        # import json
        # print(json.dumps(yaml_data, indent=2))

        # Example of accessing data from a specific file (e.g., config.yaml)
        # if 'config' in yaml_data and yaml_data['config'] is not None:
        #     print("\nAccessing data from 'config.yaml':")
        #     if isinstance(yaml_data['config'], dict) and 'database' in yaml_data['config']:
        #         print(f"Database host: {yaml_data['config']['database'].get('host', 'N/A')}")
        #     else:
        #         print("config.yaml data is not a dictionary or missing 'database' key.")
        # elif 'config' in yaml_data and yaml_data['config'] is None:
        #      print("\nconfig.yaml failed to load or parse.")
        # else:
        #      print("\nNo file named 'config.yaml' was found or loaded successfully.")

        print(f"\nSuccessfully loaded data from {len(yaml_data)} YAML file(s) into the dictionary.")
        print(yaml_data)
        # You can now work with the 'yaml_data' dictionary
    else:
        print("\nNo YAML data was loaded.")
