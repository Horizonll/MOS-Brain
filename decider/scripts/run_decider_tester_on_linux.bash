#!/bin/bash

# Check and install dependencies
check_dependencies() {
    # Determine package manager
    local PKG_MANAGER
    if command -v apt &> /dev/null; then
        PKG_MANAGER="apt"
    elif command -v dnf &> /dev/null; then
        PKG_MANAGER="dnf"
    else
        echo "Error: Only APT/DNF package managers are supported" >&2
        exit 1
    fi

    # Dependency list (command package-name)
    local dependencies=(
        "gnome-terminal gnome-terminal"
        "wmctrl wmctrl"
        "python3 python3"
    )

    for dep in "${dependencies[@]}"; do
        local cmd=${dep%% *}
        local pkg=${dep#* }
        
        if ! command -v "$cmd" &> /dev/null; then
            echo "Installing missing dependency: $pkg"
            case $PKG_MANAGER in
                "apt")
                    sudo apt update -qq
                    sudo DEBIAN_FRONTEND=noninteractive apt install -yq "$pkg"
                    ;;
                "dnf")
                    sudo dnf install -y "$pkg" --quiet
                    ;;
            esac
            
            # Verify installation
            if ! command -v "$cmd" &> /dev/null; then
                echo "Failed to install dependency: $pkg" >&2
                exit 1
            fi
        fi
    done
}

# Main execution
main() {
    check_dependencies

    # Get file paths
    scripts_dir=$(dirname "$0")
    test_dir=$(dirname "$scripts_dir")/test

    # Verify Python files
    py_files=(
        "$test_dir/tcp_host_test_reciever.py"
        "$test_dir/decider_tester.py"
    )
    for file in "${py_files[@]}"; do
        if [ ! -f "$file" ]; then
            echo "Error: File not found - $file" >&2
            exit 1
        fi
    done

    # Launch applications
    gnome-terminal --title="tcp_host_test_reciever" -- bash -c "python3 '${py_files[0]}'; exec bash"
    gnome-terminal --title="decider_tester" -- bash -c "python3 '${py_files[1]}'; exec bash"

    # Window focus
    sleep 2
    wmctrl -a "decider_tester" &> /dev/null || echo "Note: Window focus failed, please switch manually" >&2
}

main